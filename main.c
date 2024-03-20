/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "string.h"
#include "register.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma location=0x30000000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x30000200
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x30000000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x30000200))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __GNUC__ ) /* GNU Compiler */
ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */

#endif

ETH_TxPacketConfig TxConfig;

ETH_HandleTypeDef heth;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi5;

TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/*
 * PACKET STRUCTURE --> |SOP|DATATYPE|SENSOR ID|TIMESTAMP|DATA|CRC|EOP|
 * 						|1B|1B|1B|4B|8B|1B|1B| = 17 Bytes
 */
typedef struct {
  uint8_t sop;       // Start of packet
  uint8_t datatype;   // Data type
  uint8_t sensorId;   // Sensor ID
  uint32_t timestamp; // Timestamp
  uint64_t data;      // Sensor data
  uint8_t crc;        // CRC for error checking
  uint8_t eop;
} SensorDataPacket;

typedef enum { // FSM States
    UART_WAIT_FOR_SOP,
    UART_DATATYPE,
    UART_SENSOR_ID,
    UART_TIMESTAMP,
    UART_DATA,
    UART_CRC,
	UART_EOP,
    UART_DONE
} UART_State_t;


UART_State_t uartState = UART_WAIT_FOR_SOP; // Starting state
SensorDataPacket sensorData; // Packet to be received and stored to memory
//uint8_t rx_data[1]; // Temp value for incoming byte
uint8_t crc_calculated = 0; // Placeholder for the calculated CRC
uint32_t dataIndex = 0; // Used for buffer indexing
uint32_t timestampBuffer;
uint64_t dataBuffer;



const uint8_t cc1200_rx_settings[50*3] =
{
	0x00, 0x01, 0x08,
	0x00, 0x03, 0x09,
	0x00, 0x08, 0x1F,
    0x00, 0x0A, 0x9F, //RX bw
	0x00, 0x0B, 0x00,
	0x00, 0x0C, 0x5D,
	0x00, 0x0D, 0x00,
	0x00, 0x0E, 0x8A,
	0x00, 0x0F, 0xCB,
	0x00, 0x10, 0xAC,
	0x00, 0x11, 0x00,
	0x00, 0x12, 0x45,
	0x00, 0x13, 0x83, //symbol rate 2 - 24kSa/s
	0x00, 0x14, 0xA9, //symbol rate 1
	0x00, 0x15, 0x2A, //symbol rate 0
	0x00, 0x16, 0x37,
	0x00, 0x17, 0xEC,
	0x00, 0x19, 0x11,
	0x00, 0x1B, 0x51,
	0x00, 0x1C, 0x87,
	0x00, 0x1D, 0x00,
	0x00, 0x20, 0x14,
	0x00, 0x26, 0x03,
	0x00, 0x27, 0x00,
	0x00, 0x28, 0x20,
	0x00, 0x2B, 0x3F,
	0x00, 0x2E, 0xFF,
	0x2F, 0x00, 0x1C,
	0x2F, 0x01, 0x02, //AFC, 0x22 - on, 0x02 - off
	0x2F, 0x05, 0x0D,
	0x2F, 0x0C, 0x57, //freq 435MHz, round((float)435000000/5000000*(1<<16))
	0x2F, 0x0D, 0x00, //freq
	0x2F, 0x0E, 0x00, //freq
	0x2F, 0x10, 0xEE,
	0x2F, 0x11, 0x10,
	0x2F, 0x12, 0x07,
	0x2F, 0x13, 0xAF,
	0x2F, 0x16, 0x40,
	0x2F, 0x17, 0x0E,
	0x2F, 0x19, 0x03,
	0x2F, 0x1B, 0x33,
	0x2F, 0x1D, 0x17,
	0x2F, 0x1F, 0x00,
	0x2F, 0x20, 0x6E,
	0x2F, 0x21, 0x1C,
	0x2F, 0x22, 0xAC,
	0x2F, 0x27, 0xB5,
	0x2F, 0x32, 0x0E,
	0x2F, 0x36, 0x03,
	0x2F, 0x91, 0x08,
};

const uint8_t cc1200_tx_settings[50*3] =
{
	0x00, 0x01, 0x08,
	0x00, 0x03, 0x09,
	0x00, 0x08, 0x1F,
	0x00, 0x0A, 0x59, //deviation
	0x00, 0x0B, 0x01, //deviation, LSB - exponent
	0x00, 0x0C, 0x5D,
	0x00, 0x0D, 0x00,
	0x00, 0x0E, 0x8A,
	0x00, 0x0F, 0xCB,
	0x00, 0x10, 0xAC,
	0x00, 0x11, 0x00,
	0x00, 0x12, 0x45,
	0x00, 0x13, 0x83, //symbol rate 2 - 24kSa/s
	0x00, 0x14, 0xA9, //symbol rate 1
	0x00, 0x15, 0x2A, //symbol rate 0
	0x00, 0x16, 0x37,
	0x00, 0x17, 0xEC,
	0x00, 0x19, 0x11,
	0x00, 0x1B, 0x51,
	0x00, 0x1C, 0x87,
	0x00, 0x1D, 0x00,
	0x00, 0x20, 0x14,
	0x00, 0x26, 0x03,
	0x00, 0x27, 0x00,
	0x00, 0x28, 0x20,
	0x00, 0x2B, 0x05, //power (0x01..0x3F)
	0x00, 0x2E, 0xFF,
	0x2F, 0x00, 0x1C,
	0x2F, 0x01, 0x22,
	0x2F, 0x05, 0x09, //16x upsampler, CFM enable
	0x2F, 0x0C, 0x57, //freq 439M = 0x57CCCD
	0x2F, 0x0D, 0xCC, //freq
	0x2F, 0x0E, 0xCD, //freq
	0x2F, 0x10, 0xEE,
	0x2F, 0x11, 0x10,
	0x2F, 0x12, 0x07,
	0x2F, 0x13, 0xAF,
	0x2F, 0x16, 0x40,
	0x2F, 0x17, 0x0E,
	0x2F, 0x19, 0x03,
	0x2F, 0x1B, 0x33,
	0x2F, 0x1D, 0x17,
	0x2F, 0x1F, 0x00,
	0x2F, 0x20, 0x6E,
	0x2F, 0x21, 0x1C,
	0x2F, 0x22, 0xAC,
	0x2F, 0x27, 0xB5,
	0x2F, 0x32, 0x0E,
	0x2F, 0x36, 0x03,
	0x2F, 0x91, 0x08,
};

uint8_t tx_data[8];
uint8_t rx_data[8]={0,0,0,0,0,0,0,0};

volatile uint8_t irq_pend=0;	//baseband sample IRQ
volatile uint8_t mode=1;		//0 - RX, 1- TX

float buff[81];			//look-back buffer for the FIR
uint8_t pushed=0;		//how many samples have we pushed to the buffer
float mac;				//multiply-accumulate result
int8_t sample=0;		//sample value, calculated

uint16_t byte=0;		//byte read from the stream
uint8_t dibit=6;		//dibit ^^ (MSB pair first)



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_HS_USB_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM16_Init(void);
static void MX_SPI5_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/////// reset pin for TX is B6   GPIOB GPIO_PIN_6
/////// reset pin for RX is E4   GPIOE, GPIO_PIN_4

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	//HAL_GPIO_TogglePin(TST_PIN_GPIO_Port, TST_PIN_Pin);
	irq_pend=1;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
//	//BTN1
//	if(GPIO_Pin==GPIO_PIN_0)
//	{
//		HAL_GPIO_TogglePin(MIC_MUTE_GPIO_Port, MIC_MUTE_Pin);
//	}
//
//	//BTN2
//	else if(GPIO_Pin==GPIO_PIN_1)
//	{
//		HAL_GPIO_TogglePin(SPK_MUTE_GPIO_Port, SPK_MUTE_Pin);
//	}
//
//	//BTN3
//	else if(GPIO_Pin==GPIO_PIN_2)
//	{
//		HAL_GPIO_TogglePin(TST_PIN_GPIO_Port, TST_PIN_Pin);
//	}
}

void CC1200_Reset(void)
{

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, 0);
	HAL_Delay(500);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, 1);
	HAL_Delay(500);





	tx_data[0]=0x30;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 0);
	HAL_SPI_Transmit_IT(&hspi1, tx_data, 1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 1);

	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, 0);
	HAL_SPI_Transmit_IT(&hspi5, tx_data, 1);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, 1);
}

void CC1200_Init(uint8_t *init_seq)
{
	for(uint8_t i=0; i<50; i++)
	{
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 0);
		  if(init_seq[i*3])
			  HAL_SPI_Transmit_IT(&hspi1, &init_seq[i*3], 3);
		  else
			  HAL_SPI_Transmit_IT(&hspi1, &init_seq[i*3+1], 2);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 1);
		  HAL_Delay(10);
	}

	for(uint8_t i=0; i<50; i++)
	{
		  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, 0);
		  if(init_seq[i*3])
			  HAL_SPI_Transmit_IT(&hspi5, &init_seq[i*3], 3);
		  else
			  HAL_SPI_Transmit_IT(&hspi5, &init_seq[i*3+1], 2);
		  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, 1);
		  HAL_Delay(10);
	}
}

void CC1200_Offset(int16_t offset)
{
	tx_data[0]=0x2F|0x40;
	tx_data[1]=0x0A;
	tx_data[2]=*((uint8_t*)&offset+1);
	tx_data[3]=*((uint8_t*)&offset);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 0);
	HAL_SPI_Transmit_IT(&hspi1, tx_data, 4);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 1);


	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, 0);
	HAL_SPI_Transmit_IT(&hspi5, tx_data, 4);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, 1);
}

void CC1200_BurstModeIncr(uint8_t enable)
{
	tx_data[0]=0x2F;
	tx_data[1]=0x06;
	tx_data[2]=enable;

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 0);
	HAL_SPI_Transmit_IT(&hspi1, tx_data, 3);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 1);


	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, 0);
	HAL_SPI_Transmit_IT(&hspi5, tx_data, 3);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, 1);
}

void CC1200_RXMode(void)
{
	tx_data[0]=0x34;
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, 0);
	HAL_SPI_Transmit(&hspi5, tx_data, 1, 100);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, 1);
}

void CC1200_TXMode(void)
{
	tx_data[0]=0x35;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 0);
	HAL_SPI_Transmit(&hspi1, tx_data, 1, 100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 1);
}

void CC1200_RXStart(void)
{
	tx_data[0]=0x2F|0xC0;
	tx_data[1]=0x7D;
	tx_data[2]=0;

	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, 0);
	HAL_SPI_TransmitReceive(&hspi5, tx_data, rx_data, 3, 10);
}
void CC1200_TXStart(void)
{
	tx_data[0]=0x2F|0x40;
	tx_data[1]=0x7E;
	tx_data[2]=0;

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 0);
	HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 3, 10);
}

void CC1200_TXRXEnd(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 1);
}

//set frequency - burst mode
void CC1200_SetFreq(uint32_t freq)
{
	uint32_t val=(float)freq/5000000*(1<<16);

	tx_data[0]=0x2F|0x40;
	tx_data[1]=0x0C;
	tx_data[2]=(val>>16)&0xFF;
	tx_data[3]=(val>>8)&0xFF;
	tx_data[4]=val&0xFF;

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 0);
	HAL_SPI_Transmit(&hspi1, tx_data, 5, 10);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 1);

	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, 0);
	HAL_SPI_Transmit(&hspi5, tx_data, 5, 10);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, 1);
}

//set power (0x01..0x3F)
void CC1200_SetPwr(uint8_t pwr)
{
	tx_data[0]=0x2B;
	tx_data[1]=pwr;

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 0);
	HAL_SPI_Transmit(&hspi1, tx_data, 2, 10);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 1);

	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, 0);
	HAL_SPI_Transmit(&hspi5, tx_data, 2, 10);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, 1);
}


SensorDataPacket test_buff[1];



void resetState(void) {
    uartState = UART_WAIT_FOR_SOP; // Starting state
    dataIndex = 0;
    timestampBuffer = 0;
    dataBuffer = 0;
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
}


void assignData(void){

	test_buff[0].sop = 0x53;         // Example start of packet value
	test_buff[0].datatype = 0x00;    // Temp
	test_buff[0].sensorId = 0x02;    // Example sensor ID
	test_buff[0].timestamp = 1234567890; // Example timestamp
	test_buff[0].data = 0x123456789ABCDEF0; // Example sensor data
	test_buff[0].crc = 0xFF;         // Example CRC value
	test_buff[0].eop = 0x45;

}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_HS_USB_Init();
  MX_SPI1_Init();
  MX_TIM16_Init();
  MX_SPI5_Init();
  /* USER CODE BEGIN 2 */

  //assignData();


  HAL_Delay(100);

  //chip reset
  CC1200_Reset();
  HAL_Delay(100);

  //TX/RX mode, 0 - RX, 1- TX
  mode=1;

  //chip config
  CC1200_Init(cc1200_tx_settings);
  CC1200_Init(cc1200_rx_settings);

  //frequency - override the setting in the init sequence
  CC1200_SetFreq(915000000);

  //power - override the setting in the init sequence
  CC1200_SetPwr(0x3C);

  //freq offset compensation
  CC1200_Offset(303);
  HAL_Delay(10);

  //mode - TX/RX
  CC1200_TXMode();
  CC1200_RXMode();

  //dont increment address in burst mode
  CC1200_BurstModeIncr(0);

  //start write/read burst - tx/rx reg
  CC1200_TXStart();
  CC1200_RXStart();

  HAL_TIM_Base_Start_IT(&htim16);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

//	 if (HAL_SPI_Transmit_IT(&hspi1, (uint8_t *)test_buff, sizeof(test_buff)) != HAL_OK){
//		  HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_1); //yellow
//	  }
//	   else {
//	      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14); // red
//	  }
//
//	 HAL_Delay(2000);


	  //if(irq_pend)
	 // {
		  if(mode)//TX
		  {
			  //do the filtering here
//			  for(uint8_t i=0; i<41-1; i++)
//				  buff[i]=buff[i+1];
//			  pushed++;
//			  pushed%=5;
//			  if(pushed==0)
//			  {
//				  uint8_t s=(m17_stream[byte]>>dibit)&3;
//
//				  if(s == 0b00)
//					  buff[40]=1.0;
//				  else if(s == 0b01)
//					  buff[40]=3.0;
//				  else if(s == 0b10)
//					  buff[40]=-1.0;
//				  else
//					  buff[40]=-3.0;
//
//				  if(dibit>0)
//					  dibit-=2;
//				  else
//				  {
//					  dibit=6;
//					  byte++;
//				  }
//
//				  if(byte==STREAM_SIZE)
//					  byte=0;
//			  }
//			  else
//			  {
//				  buff[40]=0.0;
//			  }
//
//			  mac=0.0;
//			  for(uint8_t i=0; i<41; i++)
//				  mac+=buff[i]*taps2[i];
//
//			  sample=mac*40.0;

			  uint8_t sample[8] = {'A','B','C','D','E','F','G','H'};
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 0);
			  HAL_SPI_Transmit_IT(&hspi1, (uint8_t*)&sample, 8);
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 1);
			  //HAL_SPI_Transmit_IT(&hspi5, (uint8_t*)&sample, 1);
			  //HAL_UART_Transmit_IT(&huart1, (uint8_t*)&sample, 1);
			  uint8_t test[12] = {1,2,3,4,5,6,7,8,9,10,11,12};
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, 0);
			  HAL_SPI_TransmitReceive(&hspi5, test, rx_data, 8, 10);
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, 1);
		  }
		  else //RX
		  {

			  HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 1, 10);
			  //uint16_t s=((int8_t)rx_data[0]+128);
			  //HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, s);
			  //HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
		  	 // HAL_UART_Transmit_IT(&huart1, rx_data, 1);
		  }

		  irq_pend=0;
	  //}
  }

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 275;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI5_Init(void)
{

  /* USER CODE BEGIN SPI5_Init 0 */

  /* USER CODE END SPI5_Init 0 */

  /* USER CODE BEGIN SPI5_Init 1 */

  /* USER CODE END SPI5_Init 1 */
  /* SPI5 parameter configuration*/
  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi5.Init.NSS = SPI_NSS_SOFT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi5.Init.CRCPolynomial = 0x0;
  hspi5.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi5.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi5.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi5.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi5.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi5.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi5.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi5.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi5.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi5.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI5_Init 2 */

  /* USER CODE END SPI5_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 0;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 65535;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_HS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_HS_USB_Init(void)
{

  /* USER CODE BEGIN USB_OTG_HS_Init 0 */

  /* USER CODE END USB_OTG_HS_Init 0 */

  /* USER CODE BEGIN USB_OTG_HS_Init 1 */

  /* USER CODE END USB_OTG_HS_Init 1 */
  /* USER CODE BEGIN USB_OTG_HS_Init 2 */

  /* USER CODE END USB_OTG_HS_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4|GPIO_PIN_11|GPIO_PIN_12|LED_YELLOW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_GREEN_Pin|LED_RED_Pin|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, USB_FS_PWR_EN_Pin|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE4 PE11 PE12 LED_YELLOW_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_11|GPIO_PIN_12|LED_YELLOW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PF6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_GREEN_Pin LED_RED_Pin PB6 */
  GPIO_InitStruct.Pin = LED_GREEN_Pin|LED_RED_Pin|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_FS_PWR_EN_Pin PD4 PD5 */
  GPIO_InitStruct.Pin = USB_FS_PWR_EN_Pin|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_FS_OVCR_Pin */
  GPIO_InitStruct.Pin = USB_FS_OVCR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_FS_OVCR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_FS_VBUS_Pin */
  GPIO_InitStruct.Pin = USB_FS_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_FS_VBUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_FS_ID_Pin */
  GPIO_InitStruct.Pin = USB_FS_ID_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG1_HS;
  HAL_GPIO_Init(USB_FS_ID_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef * hspi){
	__NOP();
	if(hspi->Instance == SPI1){
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0); //green
	}
	else if (hspi->Instance == SPI5){
		HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_1); //yellow
	}

}


void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef * hspi){

	if(hspi->Instance == SPI5){

		HAL_UART_Transmit_IT(&huart3, rx_data, 1);

	}

}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14); //yellow

}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
