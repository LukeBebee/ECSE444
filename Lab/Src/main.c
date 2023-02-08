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
#include "main.h" // header for main.c

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
// included so we can call CMSIS functions
#define ARM_MATH_CM4
#include "arm_math.h"
// including our Assembly file that contains the kalman function
#include "kalman.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ITM_Port32(n) (*((volatile unsigned long *) (0xE0000000+4*n))) // for serial wire viewing (SWV) debugging
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

typedef struct kalman_state{
	float q; // process noise variance, i.e., E(w^2)
	float r; // measurement noise variance, i.e., E(v^2)
	float x; // value
	float p; // estimation error co-variance
	float k; // Kalman gain

}kalman_state;



/**
 *
 */
int Kalmanfilter_assembly(float* InputArray, float* OutputArray, kalman_state* kstate, int Length);
/**
 *
 */
int Kalmanfilter_C(float* InputArray, float* OutputArray, kalman_state* kstate, int Length);
/**
 *
 */
int Kalmanfilter_DSP(float* InputArray, float* OutputArray, kalman_state* kstate, int Length);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * Uses the assembly implementation of kalman to run update the kalman_state for an array of measurement inputs
 * @Param float* InputArray address of array of measurements
 * @Param float* OutputArray address of array to load with x values from updatting the kalman_state
 * @Param kalman_state* kstate is initial state
 * @Param int Length is the length of InputArray
 * @Return int 0 if function ran as expected, -1 if error
 */
int Kalmanfilter_assembly(float* InputArray, float* OutputArray, kalman_state* kstate, int Length)  {
	for (int i = 0; i < Length; i++){ // loop through input array of measurements
		float measurement = InputArray[i];
		if (kalman(kstate, measurement) == -1) { // run kalman function, check for overflow, underflow, and division by zero
			return -1; // return -1 if kalman returns -1 (happens if arithmetic condition in computations)
		}
		OutputArray[i] = kstate->x; // Add x to output array
	}
	return 0; // return 0 if all goes well
}

/**
 * Uses the C implementation of kalman to run update the kalman_state for an array of measurement inputs
 * @Param float* InputArray address of array of measurements
 * @Param float* OutputArray address of array to load with x values from updatting the kalman_state
 * @Param kalman_state* kstate is initial state
 * @Param int Length is the length of InputArray
 * @Return int 0 if function ran as expected, -1 if error
 */
int Kalmanfilter_C(float* InputArray, float* OutputArray, kalman_state* kstate, int Length)  {
	for (int i = 0; i < Length; i++){  // loop through input array of measurements
		float measurement = InputArray[i];

		// kalman arithmetic
		kstate->p = kstate->p + kstate->q;
		kstate->k = kstate->p / (kstate->p +kstate->r);
		kstate->x = kstate->x + (kstate->k * (measurement - kstate->x));
		kstate->p = (1.0 - kstate->k) * kstate->p;

		OutputArray[i] = kstate->x; // Add x to output array
	}
		//Checking for overflow, underflow, and division by 0 via checking for inf or NaN
		if (isnan(kstate->p) || isnan(kstate->k) || isnan(kstate->q) || isnan(kstate->x) || isnan(kstate->r)) {
			return -1;
		}
		if (isinf(kstate->p) || isinf(kstate->k) || isinf(kstate->q) || isinf(kstate->x) || isinf(kstate->r)) {
			return -1;
		return 0;
}

/**
 * Uses the CMSIS-DSP implementation of kalman to run update the kalman_state for an array of measurement inputs
 * @Param float* InputArray address of array of measurements
 * @Param float* OutputArray address of array to load with x values from updatting the kalman_state
 * @Param kalman_state* kstate is initial state
 * @Param int Length is the length of InputArray
 * @Return int 0 if function ran as expected, -1 if error
 */
int Kalmanfilter_CMSIS(float* InputArray, float* OutputArray, kalman_state* kstate, int Length)  {
// TODO see if this works at all
	int result = 0;
	float temp = 0;
	for (int i = 0; i < Length; i++){
		float measurement = InputArray[i];

		arm_add_f32(kstate->p, kstate->q, kstate->p, 1);

		arm_add_f32(kstate->p, kstate->r, kstate->k, 1);
		arm_div_f32(kstate->p, kstate->k, kstate->k, 1);

		arm_sub_f32(measurement, kstate->x, temp, 1);
		arm_mult_f32(kstate->k, temp, temp, 1);
		arm_add_f32(kstate->x, temp, kstate->x, 1);

		arm_sub_f32(1.0, kstate->k, temp, 1);
		arm_mult_f32(temp, kstate->p, kstate->p, 1);

		OutputArray[i] = kstate->x; // add x to output array
	}
	uint32_t flags = 0;
	arm_and_u32(__get_FPSCR(), 15, flags, 1); // check FPSCR with 0b1111
	if(flags != 0) {return -1;} // if any flags, return -1
	return 0; // return 0 if no FPSCR issues
}




// C implementation for processing filtered data
// TODO Subtraction of original and data obtained by Kalman filter tracking.
// TODO Calculation of the standard deviation and the average of the difference obtained in a).
// TODO Calculation of the correlation between the original and tracked vectors.
// TODO Calculation of the convolution between the two vectors.



// CMSIS-DSP implementation for processing filtered data
// TODO Subtraction of original and data obtained by Kalman filter tracking.
// TODO Calculation of the standard deviation and the average of the difference obtained in a).
// TODO Calculation of the correlation between the original and tracked vectors.
// TODO Calculation of the convolution between the two vectors.



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */



















// Below is code from part 1 ---------------------------------------------------------------------------------
//	// Check that calculations work (using table from lab doc)
//	struct kalman_state kstate = {0.1, 0.1, 5.0, 0.1, 0.0};
//	float measurement = 0.0;
//	for (int i = 0; i < 5; i++){
//		kalman(&kstate, measurement); // calling kalman
//		measurement++;
//	}
//
//	// Overflow check by adding two max floats (p and q)
//	struct kalman_state kstate_of = {FLT_MAX, 0.1, 5.0, FLT_MAX, 0.0};
//	kalman(&kstate_of, measurement); // kstate_of should remain unchanged
//
//	// Underflow check (p+q+r = max float)
//	struct kalman_state kstate_uf = {0.5, (FLT_MAX - 1.0), 0.5, 0.1, 0.0};
//	kalman(&kstate_uf, measurement); // kstate_uf should remain unchanged
//
//	// Division by 0 check (w/ p+q+r = 0)
//	struct kalman_state kstate_dz = {1.0, -2.0, 5.0, 1.0, 0.0};
//	kalman(&kstate_dz, measurement); // kstate_dz should remain unchanged
// Above is code from part 1 ---------------------------------------------------------------------------------


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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	//Below is for SWV debugging
	  ITM_Port32(31) = 1;
	  //put your code in here for monitoring execution time
	  ITM_Port32(31) = 2;






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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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
