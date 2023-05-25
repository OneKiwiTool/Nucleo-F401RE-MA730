/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
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
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t angle = 0;
uint16_t readMagAlphaAngle(void);
uint8_t readMagAlphaRegister(uint8_t address);
uint8_t writeMagAlphaRegister(uint8_t address, uint8_t value);
double readAngle();
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint8_t uartTxData[100];
	  uint32_t uartTimeoutInMs = 100;
	  uint8_t readbackRegValue;

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
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

  sprintf((char *)uartTxData, "Start Program\n");
  HAL_UART_Transmit(&huart2, uartTxData, strlen((const char *)uartTxData), uartTimeoutInMs);

  writeMagAlphaRegister(0x00, 0x00);
  readbackRegValue=readMagAlphaRegister(0x00);
  sprintf((char *)uartTxData, "0x00 = 0x%02x\n", readbackRegValue);
  HAL_UART_Transmit(&huart2, uartTxData, strlen((const char *)uartTxData), uartTimeoutInMs);

  readbackRegValue=readMagAlphaRegister(0x01);
  sprintf((char *)uartTxData, "0x01 = 0x%02x\n", readbackRegValue);
  HAL_UART_Transmit(&huart2, uartTxData, strlen((const char *)uartTxData), uartTimeoutInMs);

  readbackRegValue=readMagAlphaRegister(0x02);
  sprintf((char *)uartTxData, "0x02 = 0x%02x\n", readbackRegValue);
  HAL_UART_Transmit(&huart2, uartTxData, strlen((const char *)uartTxData), uartTimeoutInMs);

  readbackRegValue=readMagAlphaRegister(0x03);
  sprintf((char *)uartTxData, "0x03 = 0x%02x\n", readbackRegValue);
  HAL_UART_Transmit(&huart2, uartTxData, strlen((const char *)uartTxData), uartTimeoutInMs);

  readbackRegValue=readMagAlphaRegister(0x04);
  sprintf((char *)uartTxData, "0x04 = 0x%02x\n", readbackRegValue);
  HAL_UART_Transmit(&huart2, uartTxData, strlen((const char *)uartTxData), uartTimeoutInMs);

  readbackRegValue=readMagAlphaRegister(0x05);
  sprintf((char *)uartTxData, "0x05 = 0x%02x\n", readbackRegValue);
  HAL_UART_Transmit(&huart2, uartTxData, strlen((const char *)uartTxData), uartTimeoutInMs);

  readbackRegValue=readMagAlphaRegister(0x06);
  sprintf((char *)uartTxData, "0x06 = 0x%02x\n", readbackRegValue);
  HAL_UART_Transmit(&huart2, uartTxData, strlen((const char *)uartTxData), uartTimeoutInMs);

  readbackRegValue=readMagAlphaRegister(0x09);
  sprintf((char *)uartTxData, "0x09 = 0x%02x\n", readbackRegValue);
  HAL_UART_Transmit(&huart2, uartTxData, strlen((const char *)uartTxData), uartTimeoutInMs);

  readbackRegValue=readMagAlphaRegister(0x1b);
  sprintf((char *)uartTxData, "0x1b = 0x%02x\n", readbackRegValue);
  HAL_UART_Transmit(&huart2, uartTxData, strlen((const char *)uartTxData), uartTimeoutInMs);

  angle = readMagAlphaAngle();
  sprintf((char *)uartTxData, "angle = 0x%x\n", angle);
  HAL_UART_Transmit(&huart2, uartTxData, strlen((const char *)uartTxData), uartTimeoutInMs);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //readAngle();
	  angle = readMagAlphaAngle();
	    sprintf((char *)uartTxData, "%d\n", angle);
	    HAL_UART_Transmit(&huart2, uartTxData, strlen((const char *)uartTxData), uartTimeoutInMs);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
uint16_t readMagAlphaAngle(void)
{
  uint32_t timeout=10;
  uint8_t txData[2];
  uint8_t rxData[2];
  txData[1]=0;
  txData[0]=0;
  uint16_t angleSensor;
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi2, txData, rxData, 2, timeout);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
  angleSensor=rxData[0]<<8 | rxData[1];
  return angleSensor;
}

double readAngle(){
  uint16_t angle;
  double angleInDegree;
  angle = readMagAlphaAngle();
  angleInDegree = (angle*360.0)/65536.0;
  return angleInDegree;
}

uint8_t readMagAlphaRegister(uint8_t address)
{
  uint32_t timeout=10;
  uint32_t delay=1;//ms
  uint8_t txData1[2];
  uint8_t rxData1[2];
  uint8_t txData2[2];
  uint8_t rxData2[2];
  txData1[0]=(0x2<<5)|(0x1F&address);
  txData1[1]=0x00;
  txData2[0]=0x00;
  txData2[1]=0x00;
  uint8_t registerReadbackValue;
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi2, txData1, rxData1, 2, timeout);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
  HAL_Delay(delay);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi2, txData2, rxData2, 2, timeout);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
  registerReadbackValue=rxData2[0];
  return registerReadbackValue;
}

uint8_t writeMagAlphaRegister(uint8_t address, uint8_t value)
{
  uint32_t timeout=10;
  uint32_t delay=20;//ms
  uint8_t txData1[2];
  uint8_t rxData1[2];
  uint8_t txData2[2];
  uint8_t rxData2[2];
  txData1[0]=(0x4<<5)|(0x1F&address);
  txData1[1]=value;
  txData2[0]=0x00;
  txData2[1]=0x00;
  uint8_t registerReadbackValue;
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi2, txData1, rxData1, 2, timeout);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
  HAL_Delay(delay);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi2, txData2, rxData2, 2, timeout);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
  registerReadbackValue=rxData2[0];
  return registerReadbackValue;
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
