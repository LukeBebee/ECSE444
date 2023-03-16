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
#include <stdio.h>
// included so we can call CMSIS functions
#define ARM_MATH_CM4
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// UART stuff for printf


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define PI 3.14159 // already defined in arm_math or C
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
DAC_ChannelConfTypeDef sConfigGlobal;

/*
 * Part 2 variables
 * 44.1 kHz sample rate (done with 1814 counter period bc of
 */
int note_selector; // [0, 2] to indicate if the note that should be played is C6 E6 or G6 respectively
uint16_t note_data[924]; // array to hold DAC output of whichever note is currently being played (924 is LCM of each note's number of samples for cicularity)
int note_data_index; // index of above array to indicate which piece of data we are on (from step 2 of part 2)
int C6_size = 44; // C6 is 1kHz, so 44 samples per period
int E6_size = 33; // E6 is 1.3kHz, so 33 samples per period
int G6_size = 28; // G6 is 1.57Hz, so 28 samples per period
uint16_t C6_data[44];
uint16_t E6_data[33];
uint16_t G6_data[28];

char change_note = 0;


/*
 * Part 1 variables
 */
int sine_wave_index; //[0, 43] 44.1 kHz sample rate, 1kHz desired sine wave, so one period every 44 samples
float sine_wave_values[44];


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Define our interrupt handlers
// Handler for button interrupt
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) { // page 391 HAL driver manual
	if (GPIO_Pin == userButton_Pin) { // verify that only the pin we want is starting this interrupt (good coding practice)
		note_selector = (note_selector + 1)%3; // cycle through three notes
		change_note = 1; // raise flag so we know to change the note
		HAL_GPIO_TogglePin(myLed_GPIO_Port, myLed_Pin); // toggle LED as user feedback for a successful button press
		printf("Need to change note\n");

	}
}

// Handler for timer interrupt (from step 2 in part 2)
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
//	if (htim == &htim2) {
//		sine_wave_index = (sine_wave_index + 1)%44;
//
//		HAL_DAC_SetValue(&hdac1, DAC1_CHANNEL_2, DAC_ALIGN_12B_R, (uint32_t) sine_wave_values[sine_wave_index]);
//
//		note_data_index = (note_data_index + 1) % 15;
//	}
//}

// DMA
void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac) {
	printf("Interrupt\n");
	if (change_note) {
		if (note_selector == 0) { // C6
			for (int i = 0; i < 924; i++) {
				note_data[i] = C6_data[i%C6_size];
			}
		} else if (note_selector == 1) { // E6
			for (int i = 0; i < 924; i++) {
				note_data[i] = E6_data[i%E6_size];
			}
		} else if (note_selector == 2) { // G6
			for (int i = 0; i < 924; i++) {
				note_data[i] = G6_data[i%G6_size];
			}
		}
		change_note = 0; // reset this flag so we know we don't need to change the note
		printf("Note Changed\n");
	}
}

// for printf
int _write(int file, char *ptr, int len) {
	int DataIdx;
	for (DataIdx = 0; DataIdx < len; DataIdx++) {
		ITM_SendChar(*ptr++);
	}
	return len;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	note_selector = 0; // [0, 2] to indicate if the note that should be played is C6 E6 or G6 respectively

	note_data_index = 0; // index of above array to indicate which piece of data we are on (for step 2 of part 2)

	sine_wave_index = 0; //[0, 43] 44.1 kHz sample rate, 1kHz desired sine wave, so one period every 44 samples (for part 1)

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
  MX_DAC1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  // Initialize DAC
  HAL_DACEx_SelfCalibrate(&hdac1, &sConfigGlobal, DAC1_CHANNEL_1);
  HAL_DACEx_SelfCalibrate(&hdac1, &sConfigGlobal, DAC1_CHANNEL_2);//from part 1


  // Start DAC and timer
  //HAL_DAC_Start(&hdac1, DAC1_CHANNEL_1);
  //HAL_DAC_Start(&hdac1, DAC1_CHANNEL_2); //from part 1
  HAL_TIM_Base_Start_IT(&htim2); // Start the timer in interrupt mode (for step 2 of part 2)


  // ----------------------------------------------------------------------------------------------------------------------------------------
  // Part 1 Code below
  // ----------------------------------------------------------------------------------------------------------------------------------------
  /*
   * 16 discrete values for saw wave (as in lab doc example)
   * 8 discrete values for triangle wave (as in lab doc example)
   */
  // Variables for triangle wave
//  int triangle_wave_value = 0;
//  int triangle_wave_increment = -2; // start negative because sign will flip in first
//  uint32_t triangle_signal;
//
//  // Variables for saw wave
//  int saw_wave_value = 0;
//  int saw_wave_increment = 1;
//  uint32_t saw_signal;
//
//  int saw_tri_samples_per_period = 8;
//
//  // Variables for sine wave
//  float sine_wave_value;
//  uint32_t sine_wave_signal;
//  int sine_radians = 0; // value [0,sine_cylces] which when multiplied by 2*PI/sine_samples_per_period will return radians for sine output
//  int sine_samples_per_period = 8;
//
//  // Variables for input/output
//  char status = 1; // status 1 is saw, 0 is triangle
//  int saw_multiplier = 585; // signal multiplier (should make signal = 4095 for max voltage @ 12-bit resolution, max value for saw and tri is 7 and 7*585 = 4095)
//  int tri_multiplier = 512; // makes signal 4096, will subtract 1 from end value
//  int sine_multiplier = 2048; // max signal output 2, so this makes max DAC value 4096 (will subtract one from signal for 4095)
  // ----------------------------------------------------------------------------------------------------------------------------------------
  // Part 1 Code above
  // ----------------------------------------------------------------------------------------------------------------------------------------
  // ----------------------------------------------------------------------------------------------------------------------------------------
  // Part 2 Code below
  // ----------------------------------------------------------------------------------------------------------------------------------------


  // make sine array (from step 2 of part 2)
//  for (sine_wave_index = 0; sine_wave_index < 44; sine_wave_index++) {
//	  sine_wave_values[sine_wave_index] = (arm_sin_f32(2*PI*sine_wave_index/44)+1)*(1365); //1365 multiplier as 4095 max output, max sine output of 2, scale down to 2/3 to reduce distortion (4095/2)*(2/3)
//
//  }
//  printf("Sine array made.\n");



  // Make notes
  for (int i = 0; i < C6_size; i++) {
	  C6_data[i] = (arm_sin_f32(2*PI*i/C6_size)+1)*(1365); // 1365 multiplier as 4095 max output, max sine output of 2, scale down to 2/3 to reduce distortion (4095/2)*(2/3)
  }
  for (int i = 0; i < E6_size; i++) {
  	  E6_data[i] = (arm_sin_f32(2*PI*i/E6_size)+1)*(1365);
  }
  for (int i = 0; i < G6_size; i++) {
  	  G6_data[i] = (arm_sin_f32(2*PI*i/G6_size)+1)*(1365);
  }



  // initialize note_data for C6 to start
    for (int i = 0; i < 924; i++) {
  	  note_data[i] = C6_data[i%C6_size];
    }
  // Start DAC with DMA
  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*)note_data, (uint32_t) 924, DAC_ALIGN_12B_R);



  printf("Program started\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  // ----------------------------------------------------------------------------------------------------------------------------------------
	  	  // Part 1 Code below
	  	  // ----------------------------------------------------------------------------------------------------------------------------------------
	  	  // Increment triangle wave
//	  	  if (triangle_wave_value == saw_tri_samples_per_period || triangle_wave_value == 0) { // flip increment when at max and min
//	  		  triangle_wave_increment *= -1;
//	  	  }
//	  	  triangle_wave_value  += triangle_wave_increment;
//
//	  	  // Increment saw wave
//	  	  saw_wave_value += saw_wave_increment;
//	  	  saw_wave_value = saw_wave_value % saw_tri_samples_per_period;
//
//	  	  // Increment sine wave
//	  	  sine_radians += 1;
//	  	  sine_radians = sine_radians % sine_samples_per_period;
//
//	  	  // Get signal to pass to DAC (scaled up for larger voltage)
//	  	  triangle_signal = triangle_wave_value*tri_multiplier;
//	  	  saw_signal = saw_wave_value*saw_multiplier;
//
//	  	  // Output to DAC (this currently switches the signals to the output ports, but one of the ports is not working)
//	  	  // Comment out to show sine signal
//	  	  if (status = 1) {
//
//	  	  	  HAL_DAC_SetValue(&hdac1, DAC1_CHANNEL_2, DAC_ALIGN_12B_R, saw_signal);
//	  		//HAL_DAC_SetValue(&hdac1, DAC1_CHANNEL_1, DAC_ALIGN_12B_R, triangle_signal); // second output pin not working
//	  	  } else {
//	  		  if (triangle_signal == 4096) {triangle_signal -= 1;}
//	  		  HAL_DAC_SetValue(&hdac1, DAC1_CHANNEL_2, DAC_ALIGN_12B_R, triangle_signal);
//	  		  //HAL_DAC_SetValue(&hdac1, DAC1_CHANNEL_1, DAC_ALIGN_12B_R, saw_signal); // second output pin not working
//	  	  }
//	  	  HAL_Delay(1); // delay 2ms for triangle and saw

	  	  /*
	  	   * Sine output
	  	   * 14 cycles (~14ms period)
	  	   * Commented out to show saw and triangle signals
	  	   */
//	  	  sine_wave_value = arm_sin_f32(sine_radians*PI*2/(sine_samples_per_period)) + 1; // potentially add 1 for positive DAC output
//	  	  sine_wave_signal = sine_wave_value * sine_multiplier; // make max value 4095 (max for 12 bit DAC resolution)
//	  	  if (sine_wave_signal == 4096) {sine_wave_signal -= 1;}
//	  	  HAL_DAC_SetValue(&hdac1, DAC1_CHANNEL_2, DAC_ALIGN_12B_R, sine_wave_signal);
//	  	  HAL_Delay(1); // delay  2ms for sine wave (sample frequency is more than twice the signal frequency, so we meet nyquist criterion)
	  	  // ----------------------------------------------------------------------------------------------------------------------------------------
	  	  // Part 1 Code above
	  	  // ----------------------------------------------------------------------------------------------------------------------------------------








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
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_ABOVE_80MHZ;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1814;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(myLed_GPIO_Port, myLed_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : userButton_Pin */
  GPIO_InitStruct.Pin = userButton_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(userButton_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_LPUART1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : myLed_Pin */
  GPIO_InitStruct.Pin = myLed_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(myLed_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
