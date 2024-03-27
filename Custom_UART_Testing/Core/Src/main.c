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

#define TEMPERATURE 0b00
#define HUMIDITY 0b01
#define SOUND 0b10
#define VIBRATION 0b11

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
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;

/* USER CODE BEGIN PV */

/*
 * PACKET STRUCTURE --> |SOP|DATATYPE|SENSOR ID|TIMESTAMP|DATA|CRC|EOP|
 * 						|1B|1B|1B|4B|8B|1B|1B| = 17 Bytes
 */
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
uint8_t rx_data1[1];
SensorDataPacket sensorData1;
UART_State_t uartState1 = UART_WAIT_FOR_SOP;
uint32_t timestampBuffer1;
uint64_t dataBuffer1;
uint32_t dataIndex1 = 0; // Used for buffer indexing

// Declaration for USART2
uint8_t rx_data2[1];
SensorDataPacket sensorData2;
UART_State_t uartState2 = UART_WAIT_FOR_SOP;
uint32_t timestampBuffer2;
uint64_t dataBuffer2;
uint32_t dataIndex2 = 0; // Used for buffer indexing

uint8_t crc_calculated = 0; // Placeholder for the calculated CRC
//uint32_t dataIndex = 0; // Used for buffer indexing

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
/* USER CODE BEGIN PFP */
void resetState(void);
int validateCRC(const SensorDataPacket *packet);
void printData(const SensorDataPacket *packet);
void processUartData(UART_HandleTypeDef *huart, SensorDataPacket *sensorData, uint8_t *rxData,
                     UART_State_t *uartState, uint32_t *timestampBuffer, uint64_t *dataBuffer, uint32_t *dataIndex);
void resetUartState(UART_State_t *uartState, uint32_t *timestampBuffer, uint64_t *dataBuffer);
void unpackData(int64_t packedData, int16_t* x, int16_t* y, int16_t* z);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if(huart->Instance == USART1) {
        // Process data from USART1
    	processUartData(huart, &sensorData1, rx_data1, &uartState1, &timestampBuffer1, &dataBuffer1, &dataIndex1);
    }
    if(huart->Instance == USART2) {
        // Process data from USART2
    	processUartData(huart, &sensorData2, rx_data2, &uartState2, &timestampBuffer2, &dataBuffer2, &dataIndex2);
    }
    // Re-enable UART reception interrupt correctly for each port
    if (huart->Instance == USART1) {
        HAL_UART_Receive_IT(&huart1, rx_data1, 1);
    } else if (huart->Instance == USART2) {
        HAL_UART_Receive_IT(&huart2, rx_data2, 1);
    }
}

void processUartData(UART_HandleTypeDef *huart, SensorDataPacket *sensorData, uint8_t *rxData,
                     UART_State_t *uartState, uint32_t *timestampBuffer, uint64_t *dataBuffer, uint32_t *dataIndex) {    // Your existing switch case logic here, adapted for the specific sensorData and rx_data
    // This function needs to be adapted from your existing HAL_UART_RxCpltCallback logic
	uint8_t rxByte = *rxData; // The received byte
//    	sprintf(buffer, "RxByte: 0x%08lX\r\n", rxByte);
//    	HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 100); // Print debug info
    switch (*uartState) {
        case UART_WAIT_FOR_SOP: // SOP Case
            if (rxByte == 0x53) { // SOP byte = 0x53 ('S')
            	sensorData->sop = rxByte; // Set the sop
            	*uartState = UART_DATATYPE;
//                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET); // Orange LED set when packet is complete
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
//                HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET); // Orange LED set when packet is complete
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
        	*dataBuffer |= ((uint32_t)rxByte << ((*dataIndex-1) * 8));//            	sprintf(buffer, "Data partial: 0x%016llx\r\n", dataBuffer);
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
                if(huart->Instance == USART1){
                	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET); // Orange LED set when packet is complete
                }
                else if(huart->Instance == USART2){
                	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET); // Orange LED set when packet is complete
                }
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
        	printData(sensorData); // Process the data
        	resetUartState(uartState, timestampBuffer, dataBuffer);
            break;
    }
    // Ready to receive the next byte
    HAL_UART_Receive_IT(huart, rxData, 1);
}

void resetUartState(UART_State_t *uartState, uint32_t *timestampBuffer, uint64_t *dataBuffer) {
    *uartState = UART_WAIT_FOR_SOP; // Reset UART state
    *timestampBuffer = 0; // Clear the timestamp buffer
    *dataBuffer = 0; // Clear the data buffer
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
}

int validateCRC(const SensorDataPacket *packet) {
    // Placeholder function to validate CRC - replace with actual CRC calculation
    return packet->crc == crc_calculated;
}

void printData(const SensorDataPacket *packet) {
    char buffer[100]; // Ensure the buffer is large enough for all the data

    // Assuming the data field is treated as fixed-point and needs to be converted back to float
    double data = packet->data / 100.0;  // Convert fixed-point back to double

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

    if(packet->datatype == TEMPERATURE) {
        // For non-vibration data, print as before
    	sprintf(buffer, "Temp: %.2f C\r\n", data);
    }
    else if(packet->datatype == HUMIDITY){
    	sprintf(buffer, "Hum: %.2f %%\r\n", data);
    }
    else if(packet->datatype == SOUND){
    	sprintf(buffer, "Sound: %.2f dB\r\n", data);
    }
    else if (packet->datatype == VIBRATION) {
        int16_t x, y, z;
        unpackData(packet->data, &x, &y, &z);
        // Display the scaled values with two decimal places as floating points
        float x_float = x / 100.0f;
        float y_float = y / 100.0f;
        float z_float = z / 100.0f;
        sprintf(buffer, "X: %.2f G\tY: %.2f G\tZ: %.2f G\r\n", x_float, y_float, z_float);
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

void unpackData(int64_t packedData, int16_t* x, int16_t* y, int16_t* z) {
    *x = (int16_t)((packedData >> 32) & 0xFFFF);
    *y = (int16_t)((packedData >> 16) & 0xFFFF);
    *z = (int16_t)(packedData & 0xFFFF);
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
  MX_USART1_UART_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart1, rx_data1, 1);
  HAL_UART_Receive_IT(&huart2, rx_data2, 1);

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
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD13 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_15;
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
