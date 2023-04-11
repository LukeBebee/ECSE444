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
#include <stm32l4s5i_iot01_gyro.h>

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
ADC_HandleTypeDef hadc1;

DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim5;

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
int millis;
int start;
int delays[5];
char code[5];
int res = 0;

	// for speaker
int beepArraySize = 44;
uint16_t beepArray[44];


char letterToPrint;
char notSpace; // 1 is true

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM5_Init(void);
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
			printf("Press one more letter to end current translation. \n\r");
		} else {
			printf("Taking letter input from terminal, outputting Morse. \n\r");
			printf("Press the spacebar to end current translation. \n\r");
		}
	}
}

/**
 * Interrupt method to increment the counter at every millisecond
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim5) {
		millis++;
	}
}

/**
 * Update global variables for the Morse array based on the inputted letter
 */
void updateMorseLetter(char letter){

	// \0s for space
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

/**
 * print the current Morse array to the serial terminal
 */
void printMorseLetter() {
	if (morseLetter[0] == '\0') {
		printf(" *space* ");
	} else {
		for (int i = 0; i < morseLetterSize; i++){
			printf("%c", morseLetter[i]);
		}
	}
}

/**
 * Play the current Morse array to the DAC speaker (and display on LED)
 */
void playMorseToSpeaker(char *morseArray, int morseArraySize) {
	for (int i = 0; i < morseArraySize; i++) {
		if (morseArray[i] == '.') {
			HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*) *beepArray, (uint32_t) beepArraySize, DAC_ALIGN_12B_R);
			HAL_GPIO_TogglePin(led2_GPIO_Port, led2_Pin);
			HAL_Delay(300);
			HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
			HAL_GPIO_TogglePin(led2_GPIO_Port, led2_Pin);
			HAL_Delay(300);
		} else if (morseArray[i] == '-') {
			HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*) *beepArray, (uint32_t) beepArraySize, DAC_ALIGN_12B_R);
			HAL_GPIO_TogglePin(led2_GPIO_Port, led2_Pin);
			HAL_Delay(600);
			HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
			HAL_GPIO_TogglePin(led2_GPIO_Port, led2_Pin);
			HAL_Delay(300);
		} else {
			return;
		}
	}
	printf("\n\r");
}

/**
 * Get a letter from the inputted Morse array
 */
char getLetterFromMorse(char *morseArray, int morseArraySize) {
	char nullChar = '\0';
	if (morseArraySize == 0) {return nullChar;}
	switch (morseArray[0]) {
	case '.':
		if (morseArraySize == 1) {return 'E';}
		switch (morseArray[1]) {
		case '.':
			if (morseArraySize == 2) {return 'I';}
			switch (morseArray[2]) {
			case '.':
				if (morseArraySize == 3) {return 'S';}
				switch(morseArray[3]) {
				case '.':
					return 'H';
				case '-':
					return 'V';
				}
			case '-':
				if (morseArraySize == 3) {return 'U';}
				switch(morseArray[3]) {
				case '.':
					return 'F';
				case '-':
					;
				}
			}
		case '-':
			if (morseArraySize == 2) {return 'A';}
			switch (morseArray[2]) {
			case '.':
				if (morseArraySize == 3) {return 'R';}
				switch(morseArray[3]) {
				case '.':
					return 'L';
				case '-':
					;
				}
			case '-':
				if (morseArraySize == 3) {return 'W';}
				switch(morseArray[3]) {
				case '.':
					return 'P';
				case '-':
					return 'J';
				}
			}
		}
	case '-':
		if (morseArraySize == 1) {return 'T';}
		switch (morseArray[1]) {
		case '.':
			if (morseArraySize == 2) {return 'N';}
			switch (morseArray[2]) {
			case '.':
				if (morseArraySize == 3) {return 'D';}
				switch(morseArray[3]) {
				case '.':
					return 'B';
				case '-':
					return 'X';
				}
			case '-':
				if (morseArraySize == 3) {return 'K';}
				switch(morseArray[3]) {
				case '.':
					return 'C';
				case '-':
					return 'Y';
				}
			}
		case '-':
			if (morseArraySize == 2) {return 'M';}
			switch (morseArray[2]) {
			case '.':
				if (morseArraySize == 3) {return 'G';}
				switch(morseArray[3]) {
				case '.':
					return 'Z';
				case '-':
					return 'Q';
				}
			case '-':
				if (morseArraySize == 3) {return 'O';}
				switch(morseArray[3]) {
				case '.':
					;
				case '-':
					;
				}
			}
		}
	}
	return nullChar;
}

/**
 * Calculate the corresponding letter associated with the code
 */
void calcMorseArray() {
	// Clear the code array
	for (int i = 0; i < 5; i++) code[i] = '\000';

	// Translate the delays into the appropriate symbols (i.e. '.', '-')
	for (int i = 0; i < 5; i++) {
		if (delays[i] == 0) break;

		if (delays[i] >= 300) code[i] = '-';
		if (delays[i] < 300) code[i] = '.';
	}
}

/**
 * Wait for the ADC input to be pressed returns 0 when pressed or 1 when timeouts
 */
int waitForADCPress(int i) {
	while (1) {
		HAL_ADC_Start(&hadc1);
		if (HAL_ADC_PollForConversion(&hadc1, 1000) == HAL_OK) {
			if (HAL_ADC_GetValue(&hadc1) <= 1000) {
				HAL_ADC_Stop(&hadc1);
				return 0;
			}
			if (i > 0 && millis - (start + delays[i - 1]) >= 2000) {
				HAL_ADC_Stop(&hadc1);
				return 1;
			}
		}
		HAL_ADC_Stop(&hadc1);
	}
}

/**
 * Wait for the ADC input to be released
 */
void waitForADCRelease() {
	while (1) {
		HAL_ADC_Start(&hadc1);
		if (HAL_ADC_PollForConversion(&hadc1, 1000) == HAL_OK) {
			if (HAL_ADC_GetValue(&hadc1) > 1000) {
				HAL_ADC_Stop(&hadc1);
				return;
			}
		}
		HAL_ADC_Stop(&hadc1);
	}
}

/**
 * Wait for the user to input up to 5 signals
 * returns the size of the input Morse
 */
int getMorseInput() {
	// Start the timer at 0 ms
	millis = 0;
	HAL_TIM_Base_Start_IT(&htim5);

	// Clear the delay array
	for (int i = 0; i < 5; i++) delays[i] = 0;

	// Ask for up to 5 signals (i < 6 because return is in the next iteration)
	for (int i = 0; i < 6; i++) {
		// Wait while the button is not pressed
		if (waitForADCPress(i) == 1) {
			calcMorseArray();
			return i;
		}
		HAL_Delay(10);
		start = millis;

		// Wait until the button is released
		waitForADCRelease();
		HAL_Delay(10);
		delays[i] = millis - start;
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
char ADC = 0;


morseInputArray[0] = '\0';
morseInputArray[1] = '\0';
morseInputArray[2] = '\0';
morseInputArray[3] = '\0';
morseInputArraySize = 0;


notSpace = 1; // 1 is true





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
  MX_ADC1_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */

  //HAL_DACEx_SelfCalibrate(&hdac1, &sConfigGlobal, DAC1_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim2);

  /*
   * Initialize beeps
   */
  for (int i = 0; i < beepArraySize; i++) {
	  beepArray[i] = (arm_sin_f32(2*PI*i/beepArraySize)+1)*(1365); // 1365 multiplier as 4095 max output, max sine output of 2, scale down to 2/3 to reduce distortion (4095/2)*(2/3)
  }


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (mode == 0) { // taking input from terminal, outputting Morse code
		  // get character from user
		  printf("Input a character: ");
		  scanf("%c", &inputChar);
		  printf("\n\rYou entered: %c \n\r", inputChar); // print character
		  //printf("ASCII Character: %d \n\r", inputChar); // print ASCII value

		  // Update, display, and play the Morse letter
		  updateMorseLetter(inputChar);
		  printf("Morse Translation: ");
		  printMorseLetter();
		  printf("\n\r");
		  playMorseToSpeaker(morseLetter, morseLetterSize);
		  HAL_Delay(1000);

	  }

	  if (mode == 1) { // taking input of array for Morse letter, displaying letter to terminal
		  if (ADC == 0) {
		  	  // reset input stuff
			  notSpace = 1;
			  morseInputArraySize = 0;
			  morseInputArray[0] = '\0'; morseInputArray[1] = '\0'; morseInputArray[2] = '\0'; morseInputArray[3] = '\0';
			  // get user input until space input
			  printf("\n\rInput Morse (. and - with a space at the end)\n\r");
			  while (notSpace == 1) {
				  scanf("%c", &inputChar);
				  if (inputChar == 32) {
					  printf("\n\rSpace Inputed\n\r");
					  notSpace = 0;
					  break;
				  }
				  printf(" You entered: %c\n\r", inputChar);
				  morseInputArray[morseInputArraySize] = inputChar;
				  morseInputArraySize = morseInputArraySize+1;
			  }
			  // get morse input from ADC
			  letterToPrint = getLetterFromMorse(morseInputArray, morseInputArraySize);
			  printf("Letter from  input: %c \n\r", letterToPrint);
			  HAL_Delay(500);

		  }
		  if (ADC == 1)  {
			  printf("\n\rInput Morse using the analog stick (wait 2 seconds when done)\n\r");
			  morseInputArraySize = getMorseInput();
			  printf("\n\rYou entered: %c%c%c%c%c\n\r", code[0], code[1], code[2], code[3], code[4]);

			  // display letter corresponding to input
			  letterToPrint = getLetterFromMorse(code, morseInputArraySize);
			  printf("Letter from  input: %c \n\r", letterToPrint);
			  HAL_Delay(500);
		  }
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.DFSDMConfig = ADC_DFSDM_MODE_ENABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 120000;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

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
