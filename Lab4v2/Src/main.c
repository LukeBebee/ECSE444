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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "stm32l4s5i_iot01_accelero.h"
#include "stm32l4s5i_iot01_gyro.h"
#include "stm32l4s5i_iot01_hsensor.h"
#include "stm32l4s5i_iot01_magneto.h"
#include "stm32l4s5i_iot01_psensor.h"
#include "stm32l4s5i_iot01_tsensor.h"
#include "stm32l4s5i_iot01.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// =================
//  Global constant
// =================

#define MAX_BUF 50

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart1;

osThreadId readSensorsHandle;
osThreadId handleButtonHandle;
osThreadId printSerialHandle;
osMutexId bufMutexHandle;
osMutexId sensorMutexHandle;
/* USER CODE BEGIN PV */

// ==================
//  Global variables
// ==================

// 0: Temp
// 1: Magnet
// 2: Gyro
// 3: Pressure
int sensor = 0;

float temp_read;
uint16_t mag_read[3];
float gyro_read[3];
float bar_read;

uint8_t buf[MAX_BUF];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
void StartReadSensors(void const * argument);
void StartHandleButton(void const * argument);
void StartPrintSerial(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	// Only used with interrupt

//	if (GPIO_Pin == GPIO_PIN_13) {
//		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//
//		// Cycle through sensors
//		sensor = (sensor + 1) % 4;
//	}
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

  // ====================
  //  Initialize sensors
  // ====================

  BSP_TSENSOR_Init(); 	// Temperature sensor
  BSP_MAGNETO_Init(); 	// Magnetometer
  BSP_GYRO_Init();		// Gyroscope
  BSP_PSENSOR_Init();	// Barometer

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of bufMutex */
  osMutexDef(bufMutex);
  bufMutexHandle = osMutexCreate(osMutex(bufMutex));

  /* definition and creation of sensorMutex */
  osMutexDef(sensorMutex);
  sensorMutexHandle = osMutexCreate(osMutex(sensorMutex));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of readSensors */
  osThreadDef(readSensors, StartReadSensors, osPriorityNormal, 0, 128);
  readSensorsHandle = osThreadCreate(osThread(readSensors), NULL);

  /* definition and creation of handleButton */
  osThreadDef(handleButton, StartHandleButton, osPriorityIdle, 0, 128);
  handleButtonHandle = osThreadCreate(osThread(handleButton), NULL);

  /* definition and creation of printSerial */
  osThreadDef(printSerial, StartPrintSerial, osPriorityIdle, 0, 128);
  printSerialHandle = osThreadCreate(osThread(printSerial), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  // Clear character buffer
	  for (int i = 0; i < MAX_BUF; i++) {
		  buf[i] = '\0';
	  }

	  // Get new readings
	  if (sensor == 0) {
		  temp_read = BSP_TSENSOR_ReadTemp();
		  sprintf(buf, "Temperature: %d\r\n", temp_read);
	  } else if (sensor == 1) {
		  BSP_MAGNETO_GetXYZ(&mag_read);
		  sprintf(buf, "Magneto: [%d, %d, %d]\r\n", mag_read[0], mag_read[1], mag_read[2]);
	  } else if (sensor == 2) {
		  BSP_GYRO_GetXYZ(&gyro_read);
		  sprintf(buf, "Gyro: [%d, %d, %d]\r\n", gyro_read[0], gyro_read[1], gyro_read[2]);
	  } else {
		  bar_read = BSP_PSENSOR_ReadPressure();
		  sprintf(buf, "Pressure: %d\r\n", bar_read);
	  }

	  // Send to serial terminal
	  HAL_UART_Transmit(&huart1, buf, sizeof(buf), 10);
	  HAL_Delay(100);

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x307075B1;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartReadSensors */
/**
  * @brief  Function implementing the readSensors thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartReadSensors */
void StartReadSensors(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  // 10 Hz sample rate
	  osDelay(100);

	  // Assert mutual exclusion
	  osMutexWait(sensorMutexHandle, osWaitForever);
	  osMutexWait(bufMutexHandle, osWaitForever);

	  // Clear character buffer
	  for (int i = 0; i < MAX_BUF; i++) {
		  buf[i] = '\0';
	  }

	  // Get new readings
	  if (sensor == 0) {
		  temp_read = BSP_TSENSOR_ReadTemp();
		  sprintf(buf, "Temperature: %d\r\n", temp_read);
	  } else if (sensor == 1) {
		  BSP_MAGNETO_GetXYZ(&mag_read);
		  sprintf(buf, "Magneto: [%d, %d, %d]\r\n", mag_read[0], mag_read[1], mag_read[2]);
	  } else if (sensor == 2) {
		  BSP_GYRO_GetXYZ(&gyro_read);
		  sprintf(buf, "Gyro: [%d, %d, %d]\r\n", gyro_read[0], gyro_read[1], gyro_read[2]);
	  } else {
		  bar_read = BSP_PSENSOR_ReadPressure();
		  sprintf(buf, "Pressure: %d\r\n", bar_read);
	  }

	  // Release mutual exclusion
	  osMutexRelease(sensorMutexHandle);
	  osMutexRelease(bufMutexHandle);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartHandleButton */
/**
* @brief Function implementing the handleButton thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartHandleButton */
void StartHandleButton(void const * argument)
{
  /* USER CODE BEGIN StartHandleButton */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);

    // Read button status
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == 0) {
    	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

    	// Cycle through sensors
    	osMutexWait(sensorMutexHandle, osWaitForever);
    	sensor = (sensor + 1) % 4;
    	osMutexRelease(sensorMutexHandle);
    }

    // Wait while button is still pressed
    while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == 0);

  }
  /* USER CODE END StartHandleButton */
}

/* USER CODE BEGIN Header_StartPrintSerial */
/**
* @brief Function implementing the printSerial thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPrintSerial */
void StartPrintSerial(void const * argument)
{
  /* USER CODE BEGIN StartPrintSerial */
  /* Infinite loop */
  for(;;)
  {
    osDelay(100);

	// Send to serial terminal
    osMutexWait(bufMutexHandle, osWaitForever);
	HAL_UART_Transmit(&huart1, buf, sizeof(buf), 10);
	osMutexRelease(bufMutexHandle);
  }
  /* USER CODE END StartPrintSerial */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
