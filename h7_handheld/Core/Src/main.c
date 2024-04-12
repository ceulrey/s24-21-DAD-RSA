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
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "Nextion.h"
#include "stdio.h"

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
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s3;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

#define TEMPERATURE 0b00
#define HUMIDITY 0b01
#define SOUND 0b10
#define VIBRATION 0b11

typedef struct {
  uint8_t sop;        // Start of packet
  uint8_t datatype;   // Data type
  uint8_t sensorId;   // Sensor ID
  uint32_t timestamp; // Timestamp
//  uint64_t data; 	  // Sensor data
  int64_t data;        // Sensor data
  uint8_t crc;        // CRC for error checking
  uint8_t eop;
} SensorDataPacket;

#define PACKET_SIZE sizeof(SensorDataPacket)

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

// Declaration for USART1
uint8_t rx_data1[PACKET_SIZE];
SensorDataPacket sensorData1;
UART_State_t uartState1 = UART_WAIT_FOR_SOP;
uint32_t timestampBuffer1;
uint64_t dataBuffer1;
uint32_t dataIndex1 = 0; // Used for buffer indexing

uint8_t spi_rx_buffer[PACKET_SIZE];
uint16_t spi_rx_count = 0;
uint8_t test_five = 0;
SensorDataPacket receivedPacket;

// DISPLAY VARIABLES
// Object for the Nextion display
Nextion nextion;
NexComp temp_C;
NexComp temp_F;
NexComp hum_RH;
NexComp vib_X;
NexComp vib_Y;
NexComp vib_Z;
NexComp sound_dB;

// Objects for the components. Button1 is for example only.
//NexComp button1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART2_UART_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
void resetState(void);
int validateCRC(const SensorDataPacket *packet);
void printData(const SensorDataPacket *packet);
void processSPIData(SPI_HandleTypeDef *SPI, SensorDataPacket *sensorData, uint8_t *rxData,
                     UART_State_t *uartState, uint32_t *timestampBuffer, uint64_t *dataBuffer, uint32_t *dataIndex);
void resetUartState(UART_State_t *uartState, uint32_t *timestampBuffer, uint64_t *dataBuffer, uint32_t *dataIndex, uint8_t *rxData);
void unpackData(uint64_t packedData, int16_t* x, int16_t* y, int16_t* z);
void printRawData(const SensorDataPacket *packet);

uint8_t test_data_count = 0;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef * hspi){
//    if(hspi->Instance == SPI1) {
//        // Process data from USART1
//    	//printData(&sensorData1);
////    	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13); //orange
////    	HAL_UART_Transmit(&huart3, rx_data1, 1, 100);
////    	HAL_SPI_Receive_IT(&hspi1, rx_data1, 1);
//    	processSPIData(hspi, &sensorData1, rx_data1, &uartState1, &timestampBuffer1, &dataBuffer1, &dataIndex1);
////    	HAL_SPI_Receive_IT(&hspi1, rx_data1, 1);
//    }
//
////    test_data_count++;
////    if(test_data_count == 1){
////    	test_data_count = 0;
//////    	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13); //orange
////
////    }
//    HAL_SPI_Receive_IT(&hspi1, rx_data1, 1);
//
//
////    HAL_SPI_Receive_IT(&hspi1, (uint8_t*)&sensorData1, sizeof(sensorData1));
//
//}

//void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef * hspi){
//    if(hspi->Instance == SPI1) {
//        char debugOutput[32];
//        sprintf(debugOutput, "Rx: 0x%X\r\n", rx_data1[0]);
//        HAL_UART_Transmit(&huart3, (uint8_t*)debugOutput, strlen(debugOutput), 100);
//        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
//        HAL_SPI_Receive_IT(&hspi1, rx_data1, 1);
//    }
//}

// This function is called when the SPI receives data
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi->Instance == SPI1) {
        // String to store the output for debugging
        char debugOutput[PACKET_SIZE * 3 + 1]; // Each byte could use up to 3 characters in text (2 hex digits and a space) + null terminator
        int offset = 0;

        // Convert the received data to hexadecimal string for UART transmission
        for (int i = 0; i < PACKET_SIZE; i++) {
            offset += snprintf(debugOutput + offset, sizeof(debugOutput) - offset, "%02X ", rx_data1[i]);
            if (offset >= sizeof(debugOutput)) break; // Safety check to prevent buffer overflow
        }

        // Transmit the formatted string over UART3
        HAL_UART_Transmit(&huart3, (uint8_t*)debugOutput, strlen(debugOutput), 100);

        // Now, process each byte of the received packet through the FSM
        for (int i = 0; i < PACKET_SIZE; i++) {
            // Pass each byte of the packet to the FSM
        	processSPIData(hspi, &sensorData1, &rx_data1[i], &uartState1, &timestampBuffer1, &dataBuffer1, &dataIndex1);
        }

        // Ready to receive the next packet
        HAL_SPI_Receive_DMA(hspi, rx_data1, PACKET_SIZE);
    }
}

// This function is called in case of an error on SPI
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
    // Handle your error here
    // After handling the error, re-arm the SPI receive interrupt
	HAL_SPI_Receive_DMA(hspi, rx_data1, PACKET_SIZE);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	NextionUpdate(huart, &nextion);
}


void processSPIData(SPI_HandleTypeDef *SPI, SensorDataPacket *sensorData, uint8_t *rxData,
                     UART_State_t *uartState, uint32_t *timestampBuffer, uint64_t *dataBuffer, uint32_t *dataIndex) {    // Your existing switch case logic here, adapted for the specific sensorData and rx_data
    // This function needs to be adapted from your existing HAL_UART_RxCpltCallback logic
	uint8_t rxByte = *rxData; // The received byte
//    sprintf(buffer, "RxByte: 0x%08lX\r\n", rxByte);
//    HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 100); // Print debug info
    switch (*uartState) {
        case UART_WAIT_FOR_SOP: // SOP Case
            if (rxByte == 0x53) { // SOP byte = 0x53 ('S')
            	sensorData->sop = rxByte; // Set the sop
            	*uartState = UART_DATATYPE;
//            	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
//                HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET); // Orange LED set when packet is complete
            }
            break;
        case UART_DATATYPE: // Data type Case
        	sensorData->datatype = rxByte; // Set th		e data type (Temp = 00, Humidity = 01, Sound = 10, Vibration = 11)
            *uartState = UART_SENSOR_ID; // Next parameter
            break;

        case UART_SENSOR_ID: // Sensor ID Case
        	sensorData->sensorId = rxByte; // Set the sensor ID (000, 001, 010, 011, 100, 101, 110, 111 (i.e. Sensor 1-8)
        	*dataIndex = 0; // Reset dataIndex for the next field
            *uartState = UART_TIMESTAMP; // Next parameter
//            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET); // Orange LED set when packet is complete
            break;

        case UART_TIMESTAMP: // Timestamp Case
            // Combine byte into timestamp assuming little endian - least significant byte first
//            	timestampBuffer |= ((uint32_t)rxByte << (24 - (dataIndex * 8)));
        	*timestampBuffer |= ((uint32_t)rxByte << ((*dataIndex-1) * 8));
//            	sprintf(buffer, "RxByte: 0x%08lX\r\n", rxByte);
//            	HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 100); // Print debug info
//            	sprintf(buffer, "Timestamp partial: 0x%08lX\r\n", timestampBuffer);
//            	HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 100); // Print debug info
            (*dataIndex)++;
            if (*dataIndex >= sizeof(sensorData->timestamp)) {
                sensorData->timestamp = *timestampBuffer; // Assign the complete timestamp
                *dataIndex = 0; // Reset dataIndex for the data field
                *timestampBuffer = 0; // Clear the buffer for the next use
                *uartState = UART_DATA; // Move to the next state
            }
            break;

        case UART_DATA: // Data Case
            // Combine byte into data assuming little endian - least significant byte first
        	*dataBuffer |= ((uint64_t)rxByte << ((*dataIndex-1) * 8));//            	sprintf(buffer, "Data partial: 0x%016llx\r\n", dataBuffer);
//            	HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 100); // Print debug info
            (*dataIndex)++;
            if (*dataIndex >= sizeof(sensorData->data)) {
                sensorData->data = *dataBuffer; // Assign the complete data
                *dataIndex = 0; // Reset dataIndex for the CRC field
                *dataBuffer = 0; // Clear the buffer for the next use
                *uartState = UART_CRC; // Move to the next state
            }
            break;

        case UART_CRC: // CRC Case
        	if(rxByte != 0){
                sensorData->crc = rxByte; // Set the CRC value based on algorithm
                *uartState = UART_EOP; // Next parameter
        	}
//                HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET); // Orange LED set when packet is complete
            break;

        case UART_EOP:
            if (rxByte == 0x45) { // EOP byte = 0x45 ('E')
                *uartState = UART_DONE; // Packet reception is complete
                sensorData->eop = rxByte; // Set the eop
                if(SPI->Instance == SPI1){
                	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET); // Red LED set when packet is complete
                }
                ///else if(SPI->Instance == USART2){
                	//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET); // Orange LED set when packet is complete
                //}
               // else if(huart->Instance == UART4){
                //	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET); // Blue LED set when packet is complete
               // }
//                HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET); // Orange LED set when packet is complete
            } else {
//                    uartState = UART_DONE; // Packet reception is complete
//                    sensorData.eop = rxByte; // Set the eop
//                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET); // Orange LED set when packet is complete
                uartState = UART_WAIT_FOR_SOP; // Invalid EOP, reset FSM
            }
            break;

        case UART_DONE:
            // Packet is complete, validate CRC and take appropriate action
//                if (validateCRC(&sensorData)) {
//                    processData(&sensorData); // Process the data
//                }
//        	test_five++;
//        	if(test_five == 5){
//        		printData(sensorData); // Process the data
//        		test_five = 0;
//        	}
//        	printRawData(sensorData);
    		printData(sensorData); // Process the data
//        	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET); // Orange LED set when packet is complete
        	resetUartState(uartState, timestampBuffer, dataBuffer, dataIndex, rxData);
            break;
    }
    // Ready to receive the next byte
    HAL_SPI_Receive_DMA(SPI, rxData, 1);
}

void resetUartState(UART_State_t *uartState, uint32_t *timestampBuffer, uint64_t *dataBuffer, uint32_t *dataIndex, uint8_t *rxData) {
    *uartState = UART_WAIT_FOR_SOP; // Reset UART state
    *timestampBuffer = 0; // Clear the timestamp buffer
    *dataBuffer = 0; // Clear the data buffer
    *dataIndex = 0;
    *rxData = 0;
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
}

//int validateCRC(const SensorDataPacket *packet) {
////     Placeholder function to validate CRC - replace with actual CRC calculation
//    return packet->crc == crc_calculated;
//}

void printData(const SensorDataPacket *packet) {
    char buffer[100]; // Ensure the buffer is large enough for all the data
    char buffer2[50];
    char x_buf[50];
    char y_buf[50];
    char z_buf[50];
    double data;
    // Assuming the data field is treated as fixed-point and needs to be converted back to float
    if(packet->datatype != VIBRATION || packet->datatype != SOUND){
        data = packet->data / 100.0;  // Convert fixed-point back to double
    }

    // Start of Packet (SOP) - Hexadecimal
    sprintf(buffer, "SOP: 0x%02X\r\n", packet->sop);
    HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 100);

    // Data Type - Binary
    sprintf(buffer, "Data Type: %u\r\n", packet->datatype);
    HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 100);

    // Sensor ID - Binary
    sprintf(buffer, "Sensor ID: %u\r\n", packet->sensorId);
    HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 100);

    // Timestamp - Decimal
    sprintf(buffer, "Timestamp: %lu\r\n", packet->timestamp);
    HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 100);

//    // Data - Decimal
//    sprintf(buffer, "Data: %lu\r\n", packet->data);
//    HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 100);

    // Data - Signed Integer
    if(packet->datatype == TEMPERATURE) {
        // For non-vibration data, print as before
    	sprintf(buffer, "Temp: %.2f C\r\n", data);
    	sprintf(buffer2, "%.2f C", data);
    	NextionSetText(&nextion, &temp_C, buffer2);
    }
    else if(packet->datatype == HUMIDITY){
    	sprintf(buffer, "Hum: %.2f %%\r\n", data);
    	sprintf(buffer2, "%.2f %%", data);
    	NextionSetText(&nextion, &hum_RH, buffer2);
    }
    else if(packet->datatype == SOUND){
    	sprintf(buffer, "Sound: %lu dB\r\n", packet->data);
    	sprintf(buffer2, "%lu dB", packet->data);
    	NextionSetText(&nextion, &sound_dB, buffer2);
    }
    else if (packet->datatype == VIBRATION) {
        int16_t x, y, z;
        unpackData(packet->data, &x, &y, &z);
        // Display the scaled values with two decimal places as floating points
        float x_float = x / 100.0f;
        float y_float = y / 100.0f;
        float z_float = z / 100.0f;
//        sprintf(buffer2, "Data: %lu\r\n", packet->data);
        sprintf(buffer, "X: %.2f G\tY: %.2f G\tZ: %.2f G\r\n", x_float, y_float, z_float);
    	sprintf(x_buf, "X: %.2f G", x_float);
    	sprintf(y_buf, "Y: %.2f G", y_float);
    	sprintf(z_buf, "Z: %.2f G", z_float);
    	NextionSetText(&nextion, &vib_X, x_buf);
    	NextionSetText(&nextion, &vib_Y, y_buf);
    	NextionSetText(&nextion, &vib_Z, z_buf);
    }
    else{
    	sprintf(buffer, "Bad Data Type", data);
    }
    HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 100);

    // CRC - Hexadecimal
    sprintf(buffer, "CRC: 0x%02X\r\n", packet->crc);
    HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 100);

    // End of Packet (EOP) - Hexadecimal
    sprintf(buffer, "EOP: 0x%02X\r\n", packet->eop);
    HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 100);

    // Separator
    HAL_UART_Transmit(&huart3, (uint8_t*)"--------\r\n", 10, 100);
}

void printRawData(const SensorDataPacket *packet) {
    char buffer[100]; // Ensure the buffer is large enough for all the data

    sprintf(buffer, "%lu\r\n", packet->data);
    HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 100);
}


void unpackData(uint64_t packedData, int16_t* x, int16_t* y, int16_t* z) {
    *x = (int16_t)((packedData >> 32) & 0xFFFF);
    *y = (int16_t)((packedData >> 16) & 0xFFFF);
    *z = (int16_t)(packedData & 0xFFFF);
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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_USB_HOST_Init();
  MX_USART3_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  NextionInit(&nextion, &huart2);
  NextionAddComp(&nextion, &temp_C, "t4", 0, 6, NULL, NULL);
  NextionAddComp(&nextion, &temp_F, "t5", 0, 7, NULL, NULL);
  NextionAddComp(&nextion, &hum_RH, "t6", 0, 8, NULL, NULL);
  NextionAddComp(&nextion, &vib_X, "t7", 0, 9, NULL, NULL);
  NextionAddComp(&nextion, &vib_Y, "t8", 0, 10, NULL, NULL);
  NextionAddComp(&nextion, &vib_Z, "t9", 0, 11, NULL, NULL);
  NextionAddComp(&nextion, &sound_dB, "t10", 0, 12, NULL, NULL);

  HAL_SPI_Receive_DMA(&hspi1, rx_data1, PACKET_SIZE);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_96K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

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
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  huart3.Init.BaudRate = 921600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
