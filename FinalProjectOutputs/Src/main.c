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
#include <string.h>
#define ARM_MATH_CM4
#include "arm_math.h"

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
DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
// printf & scanf stuff
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#define GETCHAR_PROTOTYPE int __io_getchar(void)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#define GETCHAR_PROTOTYPE int fgetc(FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

GETCHAR_PROTOTYPE
{
  uint8_t ch = 0;

  /* Clear the Overrun flag just before receiving the first character */
  __HAL_UART_CLEAR_OREFLAG(&huart1);

  /* Wait for reception of a character on the USART RX line and echo this
   * character on console */
  HAL_UART_Receive(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}


// for mode 0
char inputChar;
int morseLetterSize;
char morseLetter[4];
char mode;

// for mode 1
char morseInputArray[4] = {'.', '\0', '\0', '\0'};
int morseInputArraySize = 1;

// for speaker
int dotArraySize = 44;
uint16_t dotArray[44];
int dashArraySize = 44;
uint16_t dashArray[44];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) { // page 391 HAL driver manual
	printf("Interrupt \n\r");
	if (GPIO_Pin == userButton_Pin) { // verify that only the pin we want is starting this interrupt (good coding practice)
		printf("Button Pressed. \n\r");
		HAL_GPIO_TogglePin(led1_GPIO_Port, led1_Pin);
		mode = (mode+1)%2;
		if (mode == 1) {
			printf("Taking input Morse input (array), displaying letter to terminal. \n\r");
		} else {
			printf("Taking letter input from terminal, outputting Morse. \n\r");
		}
	}
}

void updateMorseLetter(char letter){

	// \0 for space
	morseLetterSize = 4;
	morseLetter[0] = '\0'; morseLetter[1] = '\0'; morseLetter[2] = '\0'; morseLetter[3] = '\0';
	switch (letter)
	{
		case 'a':
			morseLetter[0] = '.'; morseLetter[1] = '-';
			morseLetterSize = 2;
			break;
		case 'b':
			morseLetter[0] = '-'; morseLetter[1] = '.'; morseLetter[2] = '.'; morseLetter[3] = '.';
			morseLetterSize = 4;
			break;
		case 'c':
			morseLetter[0] = '-'; morseLetter[1] = '.'; morseLetter[2] = '-'; morseLetter[3] = '.';
			morseLetterSize = 4;
			break;
		case 'd':
			morseLetter[0] = '-'; morseLetter[1] = '.'; morseLetter[2] = '.';
			morseLetterSize = 3;
			break;
		case 'e':
			morseLetter[0] = '.';
			morseLetterSize = 1;
			break;
		case 'f':
			morseLetter[0] = '.'; morseLetter[1] = '.'; morseLetter[2] = '-'; morseLetter[3] = '.';
			morseLetterSize = 4;
			break;
		case 'g':
			morseLetter[0] = '-'; morseLetter[1] = '-'; morseLetter[2] = '.';
			morseLetterSize = 3;
			break;
		case 'h':
			morseLetter[0] = '.'; morseLetter[1] = '.'; morseLetter[2] = '.'; morseLetter[3] = '.';
			morseLetterSize = 4;
			break;
		case 'i':
			morseLetter[0] = '.'; morseLetter[1] = '.';
			morseLetterSize = 2;
			break;
		case 'j':
			morseLetter[0] = '.'; morseLetter[1] = '-'; morseLetter[2] = '-'; morseLetter[3] = '-';
			morseLetterSize = 4;
			break;
		case 'k':
			morseLetter[0] = '-'; morseLetter[1] = '.'; morseLetter[2] = '-';
			morseLetterSize = 3;
			break;
		case 'l':
			morseLetter[0] = '.'; morseLetter[1] = '-'; morseLetter[2] = '.'; morseLetter[3] = '.';
			morseLetterSize = 4;
			break;
		case 'm':
			morseLetter[0] = '-'; morseLetter[1] = '-';
			morseLetterSize = 2;
			break;
		case 'n':
			morseLetter[0] = '-'; morseLetter[1] = '.';
			morseLetterSize = 2;
			break;
		case 'o':
			morseLetter[0] = '-'; morseLetter[1] = '-'; morseLetter[2] = '-';
			morseLetterSize = 3;
			break;
		case 'p':
			morseLetter[0] = '.'; morseLetter[1] = '-'; morseLetter[2] = '-'; morseLetter[3] = '.';
			morseLetterSize = 4;
			break;
		case 'q':
			morseLetter[0] = '-'; morseLetter[1] = '-'; morseLetter[2] = '.'; morseLetter[3] = '-';
			morseLetterSize = 4;
			break;
		case 'r':
			morseLetter[0] = '.'; morseLetter[1] = '-'; morseLetter[2] = '.';
			morseLetterSize = 3;
			break;
		case 's':
			morseLetter[0] = '.'; morseLetter[1] = '.'; morseLetter[2] = '.';
			morseLetterSize = 3;
			break;
		case 't':
			morseLetter[0] = '-';
			morseLetterSize = 1;
			break;
		case 'u':
			morseLetter[0] = '.'; morseLetter[1] = '.'; morseLetter[2] = '-';
			morseLetterSize = 3;
			break;
		case 'v':
			morseLetter[0] = '.'; morseLetter[1] = '.'; morseLetter[2] = '.'; morseLetter[3] = '-';
			morseLetterSize = 4;
			break;
		case 'w':
			morseLetter[0] = '.'; morseLetter[1] = '-'; morseLetter[2] = '-';
			morseLetterSize = 3;
			break;
		case 'x':
			morseLetter[0] = '-'; morseLetter[1] = '.'; morseLetter[2] = '.'; morseLetter[3] = '-';
			morseLetterSize = 4;
			break;
		case 'y':
			morseLetter[0] = '-'; morseLetter[1] = '.'; morseLetter[2] = '-'; morseLetter[3] = '-';
			morseLetterSize = 4;
			break;
		case 'z':
			morseLetter[0] = '-'; morseLetter[1] = '-'; morseLetter[2] = '.'; morseLetter[3] = '.';
			morseLetterSize = 4;
	}
}

void printMorseLetter() {
	if (morseLetter[0] == '\0') {
		printf(" *space* ");
	} else {
		for (int i = 0; i < morseLetterSize; i++){
			printf("%c", morseLetter[i]);
		}
	}
}

void playMorseToSpeaker(char *morseArray, int morseArraySize) {
	for (int i = 0; i < morseArraySize; i++) {
		if (morseArray[i] == '.') {
			HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*) *dotArray, (uint32_t) dotArraySize, DAC_ALIGN_12B_R);
			HAL_Delay(300);
			HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
			HAL_Delay(300);
		} else if (morseArray[i] == '-') {
			HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*) *dashArray, (uint32_t) dashArraySize, DAC_ALIGN_12B_R);
			HAL_Delay(600);
			HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
			HAL_Delay(300);
		} else {
			return;
		}
		printf("\n\r");
	}

}

char getLetterFromMorse(char *morseArray, int morseArraySize) {
	char nullChar = '\0';
	switch (morseArraySize)
	{
		case 1: // single char Morse Codes
			; // weird c thing  but we technically need this semicolon
			char morse1 = morseArray[0];

			switch (morse1) {
			case '.':
				return 'E';
			case '-':
				return 'T';
			default:
				return nullChar;
			}

		case 2: // double char Morse Codes


		case 3: // triple char Morse Codes


		case 4: // quad char Morse Codes


		default:
			return nullChar;
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	/*
	 * mode 0 for taking input from terminal, outputting Morse code
	 * mode 1 for taking input of array for Morse letter, displaying letter to terminal
	 */
mode = 0;


morseInputArray[0] = '.';
morseInputArray[1] = '\0';
morseInputArray[2] = '\0';
morseInputArray[3] = '\0';
morseInputArraySize = 1;
char letterToPrint;







  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  // for scanf
  setvbuf(stdin, NULL, _IONBF, 0);

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_DAC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  //HAL_DACEx_SelfCalibrate(&hdac1, &sConfigGlobal, DAC1_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim2);

  /*
   * Initialize beeps
   */
  for (int i = 0; i < dotArraySize; i++) {
	  dotArray[i] = (arm_sin_f32(2*PI*i/dotArraySize)+1)*(1365); // 1365 multiplier as 4095 max output, max sine output of 2, scale down to 2/3 to reduce distortion (4095/2)*(2/3)
  }
  for (int i = 0; i < dashArraySize; i++) {
  	  dashArray[i] = (arm_sin_f32(2*PI*i/dashArraySize)+1)*(1365); // 1365 multiplier as 4095 max output, max sine output of 2, scale down to 2/3 to reduce distortion (4095/2)*(2/3)
    }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (mode == 0) { // taking input from terminal, outputting Morse code
		  printf("Input a character: ");
		  scanf("%c", &inputChar);
		  printf("\n\rYou entered: %c \n\r", inputChar); // print character
		  //printf("ASCII Character: %d \n\r", inputChar); // print ASCII character

		  updateMorseLetter(inputChar);
		  printf("Morse Translation: ");
		  printMorseLetter();
		  printf("\n\r");

		  playMorseToSpeaker(morseLetter, morseLetterSize);
		  HAL_Delay(1000);

	  }

	  if (mode == 1) { // taking input of array for Morse letter, displaying letter to terminal

		  letterToPrint = getLetterFromMorse(morseInputArray, morseInputArraySize);

		  printf("Letter from  array: %c \n\r", letterToPrint);
		  HAL_Delay(5000);
	  }



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
  htim2.Init.Period = 2721;
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(led2_GPIO_Port, led2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(led1_GPIO_Port, led1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : userButton_Pin */
  GPIO_InitStruct.Pin = userButton_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(userButton_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : led2_Pin */
  GPIO_InitStruct.Pin = led2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(led2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : led1_Pin */
  GPIO_InitStruct.Pin = led1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(led1_GPIO_Port, &GPIO_InitStruct);

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
