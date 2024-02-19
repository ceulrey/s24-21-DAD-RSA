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
uint8_t tx_buffer[27]="Hello Chris!\n\r";
uint8_t rx_indx;
uint8_t rx_data[1];
uint8_t rx_buffer[8];
uint8_t transfer_cplt;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
	  //HAL_UART_Transmit(&huart3, rx_data, strlen(rx_data), 100);
	  //HAL_UART_Transmit(&huart3, tx_buffer, 27, 10);
	  //HAL_Delay(1000);


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

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_UART_RxCpltCallback could be implemented in the user file
   */
//  HAL_UART_Transmit(&huart2, rx_data, 6, 10);
  uint8_t i;
  	  if(huart->Instance == USART2){
//  		  if(rx_indx == 0){
//  			  for(i=0; i<100; i++)
//  				  rx_buffer[i] = 0;
//  			  	  	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
//  		  }

  		  if(rx_data[0] != 32){
  			  rx_buffer[rx_indx++] = rx_data[0];
  		  }
  		  else{
  			  rx_indx = 0;
  			  transfer_cplt = 1;
  			  //HAL_UART_Transmit(&huart1, "\n\r", 2, 100);
//  			  if(!strcmp(rx_buffer, "HIGH")){
//  				  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
//  				  //HAL_UART_Transmit(&huart3, rx_buffer, strlen(rx_buffer), 100);
//  			  }
//  			  else if(!strcmp(rx_buffer, "LOW")){
//  				  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
//  				  //HAL_UART_Transmit(&huart3, rx_buffer, strlen(rx_buffer), 100);
//  			  }
  			  HAL_UART_Transmit(&huart3, "C: ", 3, 100);
  			  HAL_UART_Transmit(&huart3, rx_buffer, strlen(rx_buffer), 100);
  			  HAL_UART_Transmit(&huart3, "\n\r", 2, 100);
  			  memset(rx_buffer, 0, sizeof(rx_buffer)); // Clear the buffer after processing
  		  }

  		  HAL_UART_Receive_IT(&huart2, rx_data, 1);
  		  //HAL_UART_Transmit(&huart3, rx_data, strlen(rx_data), 100);
  	  }
}

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//    if(huart->Instance == USART2) { // Ensure we are dealing with USART2
//        if(rx_data[0] != '\n') { // Check if the received data is not a newline
//            rx_buffer[rx_indx++] = rx_data[0]; // Store received data in buffer
//        }
//        else { // Newline received, indicating end of command
//            rx_buffer[rx_indx] = '\0'; // Null-terminate the string
//            rx_indx = 0; // Reset index for next message
//
//            // Check if received command is "LED_ON"
//            if(strcmp((char*)rx_buffer, "LED_ON") == 0) {
//            	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
//            }
//            // Check if received command is "LED_OFF"
//            else if(strcmp((char*)rx_buffer, "LED_OFF") == 0) {
//            	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
//            }
//
//            memset(rx_buffer, 0, sizeof(rx_buffer)); // Clear the buffer after processing
//        }
//
//        // Prepare to receive next character
//        HAL_UART_Receive_IT(&huart2, rx_data, 1);
//    }
//}

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//    if(huart->Instance == USART2) { // Ensure we are dealing with USART2
//        if(rx_data[0] != '\n'){ // Check if the received data is not a newline
//            rx_buffer[rx_indx++] = rx_data[0]; // Store received data in buffer
//        }
//        else{
//            rx_buffer[rx_indx] = '\0'; // Null-terminate the string
//            rx_indx = 0; // Reset index for next message
//
//            // Transmit the entire line through USART1
//            // HAL_UART_Transmit(&huart1, (uint8_t*)rx_buffer, strlen((char*)rx_buffer), 100);
//            HAL_UART_Transmit(&huart1, rx_buffer, strlen(rx_buffer), 100);
//            //HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n", 2, 100); // Newline for PuTTY terminal
//
//            memset(rx_buffer, 0, sizeof(rx_buffer)); // Clear the buffer after processing
//        }
//        HAL_UART_Receive_IT(&huart2, rx_data, 1); // Prepare to receive next character
//        HAL_UART_Transmit(&huart1, rx_data, strlen(rx_data), 100);
//    }
//}

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//    if(huart->Instance == USART2) { // Ensure we are dealing with USART2
//        if(rx_data[0] != '\n'){ // Check if the received data is not a newline
//            if(rx_indx < sizeof(rx_buffer) - 1) { // Prevent buffer overflow
//                rx_buffer[rx_indx++] = rx_data[0]; // Store received data in buffer
//            }
//        }
//        else { // Newline received, indicating end of command
//            rx_buffer[rx_indx] = '\0'; // Null-terminate the string
//            HAL_UART_Transmit(&huart1, (uint8_t*)rx_buffer, strlen((char*)rx_buffer), 100); // Transmit buffer to USART1
//            HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n", 2, 100); // Newline for PuTTY terminal
//            memset(rx_buffer, 0, sizeof(rx_buffer)); // Clear the buffer after processing
//            rx_indx = 0; // Reset index for next message
//        }
//        HAL_UART_Receive_IT(&huart2, rx_data, 1); // Prepare to receive next character
//    }
//}

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
