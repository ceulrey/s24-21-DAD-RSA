/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program for reception of UART packets
  * @date			: 2/20/24
  * @author			: S24-21
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
UART_HandleTypeDef huart2;
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
uint8_t rx_data[1]; // Temp value for incoming byte
uint8_t crc_calculated = 0; // Placeholder for the calculated CRC
uint32_t dataIndex = 0; // Used for buffer indexing
uint32_t timestampBuffer;
uint64_t dataBuffer;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
void resetState(void);
int validateCRC(const SensorDataPacket *packet);
void processData(const SensorDataPacket *packet);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if(huart->Instance == USART2) { // If we are receiving on UART2
        uint8_t rxByte = rx_data[0]; // Received byte
        switch (uartState) {
            case UART_WAIT_FOR_SOP: // SOP Case
                if (rxByte == 0x53) { // SOP byte = 0x53 ('S')
                	sensorData.sop = rxByte; // Set the sop
                    uartState = UART_DATATYPE; // Next parameter
//                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET); // Orange LED set when packet is complete
                }
                break;
            case UART_DATATYPE: // Data type Case
            	sensorData.datatype = rxByte; // Set the data type (Temp = 00, Humidity = 01, Sound = 10, Vibration = 11)
                uartState = UART_SENSOR_ID; // Next parameter
                break;

            case UART_SENSOR_ID: // Sensor ID Case
            	sensorData.sensorId = rxByte; // Set the sensor ID (000, 001, 010, 011, 100, 101, 110, 111 (i.e. Sensor 1-8)
                uartState = UART_TIMESTAMP; // Next parameter
//                HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET); // Orange LED set when packet is complete
                break;

            case UART_TIMESTAMP: // Timestamp Case
                // Shift existing buffer by 8 to make room for new byte at LSB position
                timestampBuffer >>= 8;
                // Place the new byte at the LSB position
                timestampBuffer |= (uint32_t)rxByte << (8 * (sizeof(sensorData.timestamp) - 1));
                dataIndex++;
                if (dataIndex >= sizeof(sensorData.timestamp)) { // Once the timestamp is full
                    sensorData.timestamp = timestampBuffer; // Set timestamp to buffer
                    dataIndex = 0;
                    uartState = UART_DATA; // Next parameter
                }
                break;

            case UART_DATA: // Data Case
                // Same logic as timestamp for little-endian
                dataBuffer >>= 8;
                dataBuffer |= (uint64_t)rxByte << (8 * (sizeof(sensorData.data) - 1));
                dataIndex++;
                if (dataIndex >= sizeof(sensorData.data)) {
                    sensorData.data = dataBuffer;
                    dataIndex = 0;
                    uartState = UART_CRC; // Next parameter
                }
                break;

            case UART_CRC: // CRC Case
                sensorData.crc = rxByte; // Set the CRC value based on algorithm
                uartState = UART_EOP; // Next parameter
//                HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET); // Orange LED set when packet is complete
                break;

            case UART_EOP:
                if (rxByte == 0x45) { // EOP byte = 0x45 ('E')
                    uartState = UART_DONE; // Packet reception is complete
                    sensorData.eop = rxByte; // Set the eop
                    // HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET); // Orange LED set when packet is complete
                } else {
                    uartState = UART_DONE; // Packet reception is complete
                    sensorData.eop = rxByte; // Set the eop
                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET); // Orange LED set when packet is complete
//                    uartState = UART_WAIT_FOR_SOP; // Invalid EOP, reset FSM
                }
                break;

            case UART_DONE:
                // Packet is complete, validate CRC and take appropriate action
//                if (validateCRC(&sensorData)) {
//                    processData(&sensorData); // Process the data
//                }
            	processData(&sensorData); // Process the data
                resetState(); // Reset FSM and variables
                break;
        }
        // Ready to receive the next byte
        HAL_UART_Receive_IT(&huart2, rx_data, 1);
    }
}
void resetState(void) {
    uartState = UART_WAIT_FOR_SOP; // Starting state
    dataIndex = 0;
    timestampBuffer = 0;
    dataBuffer = 0;
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
}

int validateCRC(const SensorDataPacket *packet) {
    // Placeholder function to validate CRC - replace with actual CRC calculation
    return packet->crc == crc_calculated;
}

void processData(const SensorDataPacket *packet) {
    char buffer[100]; // Ensure the buffer is large enough for all the data

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

    // Data - Decimal
    sprintf(buffer, "Data: %lu\r\n", packet->data);
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

//uint8_t CalculateCRC(uint8_t *data, size_t len) {
//    uint8_t crc = 0;
//    for(size_t i = 0; i < len; ++i) {
//        crc ^= data[i];
//    }
//    return crc;
//}

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
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart2, rx_data, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart2.Init.BaudRate = 115200;
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
  huart3.Init.BaudRate = 115200;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PD13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
