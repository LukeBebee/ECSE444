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

typedef struct data_processed{
//	int length; // length of original measurements and obtained data  -- assume 100 for now
	float difference[25]; // to hold difference between measurement and x values
	float standard_deviation[1]; // standard deviation of difference array
	float average[1]; // average of difference array
	float convolution[49]; // convolution between measurements and x values
	float correlation[49]; // correlation between measurements and x values
}data_processed;

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
		if (kalman(kstate, measurement) == 1) { // run kalman function, check for overflow, underflow, and division by zero
			return -1; // return 1 if kalman returns -1 (happens if arithmetic condition in computations)
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
// TODO fix this somehow
	float temp = 0;
	for (int i = 0; i < Length; i++){
		// get vectors to use CMSIS library
		float measurement[1] = {InputArray[i]};

		float pvector[1] = {kstate->p};
		float qvector[1] = {kstate->q};
		float kvector[1] = {kstate->k};
		float xvector[1] = {kstate->x};
		float rvector[1] = {kstate->r};

		float onevector[1] = {1.0};

		float temp[1];

		// arithmetic
		arm_add_f32(pvector, qvector, pvector, 1);

		arm_add_f32(pvector, rvector, kvector, 1);
		arm_div_f32(pvector, kvector, kvector, 1);

		arm_sub_f32(measurement, xvector, temp, 1);
		arm_mult_f32(kvector, temp, temp, 1);
		arm_add_f32(xvector, temp, xvector, 1);

		arm_sub_f32(onevector, kvector, temp, 1);
		arm_mult_f32(temp, pvector, pvector, 1);

		kstate->p = pvector[1];
		kstate->q = qvector[1];
		kstate->k = kvector[1];
		kstate->x = xvector[1];
		kstate->r = rvector[1];

		OutputArray[i] = kstate->x; // add x to output array
	}

	if((__get_FPSCR() & 15) == 0) {return 0;} // if no flags, return 0
	return -1; // return -1 if  FPSCR issues
}





/**
 * Process data from use of kalman filter using C
 * @Param data_processed* data structure to put values into
 * @Param float* original is original measurements
 * @Param float* x_values  are x values found by kalman
 * @Param int length is the length of the original and x_values arrays
 */
void data_processing_C(data_processed* data, float* original, float* x_values, int length) {
	// Subtraction of original and data obtained by Kalman filter tracking.
	for (int i = 0; i<length; i++) {
		data->difference[i] = original[i] - x_values[i];
	}

	// Calculation of the standard deviation and the average of the difference.
	int sum = 0;
	for (int i = 0; i<length; i++) {
			sum += data->difference[i];
	}
	data->average[1] = (sum / length);

	sum = 0;
	for (int i = 0; i<length; i++) {
				sum += (data->difference[i] - data->average[1])*(data->difference[i] - data->average[1]);
	}
	sum /= length;
	data->standard_deviation[1] = sqrt(sum);

	// TODO Calculation of the correlation between the original and tracked vectors.


	// TODO Calculation of the convolution between the two vectors.



}


/**
 * Process data from use of kalman filter using CMSIS functions
 * @Param data_processed* data structure to put values into
 * @Param float* original is original measurements
 * @Param float* x_values  are x values found by kalman
 * @Param int length is the length of the original and x_values arrays
 */
void data_processing_CMSIS(data_processed* data, float* original, float* x_values, int length) {
	// Subtraction of original and data obtained by Kalman filter tracking.
	arm_sub_f32(original, x_values, data->difference, length);

	// Calculation of the standard deviation and the average of the difference.
	arm_std_f32(data->difference, length, data->standard_deviation);
	arm_mean_f32(data->difference, length, data->average);

	// Calculation of the correlation between the original and tracked vectors.
	arm_correlate_f32(original, length, x_values, length, data->correlation);

	// Calculation of the convolution between the two vectors.
	arm_conv_f32(original, length, x_values, length, data->convolution);
}


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


	  // Below is code for part 2 ---------------------------------------------------------------------------------
	  // -----------------------------------------------------------------------------------------------------------
	  	// TODO add timers

	  	int length = 25;
	  	float measurements[25] =
	  	{
	  	-0.665365,
	  	-0.329988,
	  	0.164465,
	  	0.043962,
	  	0.295885,
	  	-0.643138,
	  	0.615203,
	  	-0.254512,
	  	0.261842,
	  	0.014163,
	  	0.045181,
	  	0.554502,
	  	0.198915,
	  	0.120703,
	  	-0.547104,
	  	0.103219,
	  	-0.204776,
	  	0.107782,
	  	-0.105263,
	  	0.356157,
	  	0.172390,
	  	-0.154121,
	  	0.134996,
	  	0.392204,
	  	0.204622};


	  	// Try all 3 implementations of kalman filter (assembly, C, CMSIS) ------------------------------------------------
	  	float x_values_asm[25];
	  	struct kalman_state kstate_asm = {0.1, 0.1, 5.0, 0.1, 0.0};
	  	Kalmanfilter_assembly(measurements, x_values_asm, &kstate_asm, length);

	  	float x_values_C[25];
	  	struct kalman_state kstate_C = {0.1, 0.1, 5.0, 0.1, 0.0};
	  	Kalmanfilter_C(measurements, x_values_C, &kstate_C, length);

	  	float x_values_CMSIS[25];
	  	struct kalman_state kstate_CMSIS = {0.1, 0.1, 5.0, 0.1, 0.0};
	  	Kalmanfilter_CMSIS(measurements, x_values_CMSIS, &kstate_CMSIS, length);


	  	// Try processing data with CMSIS implementation ------------------------------------------------------------------
	  	struct data_processed data_asm_CMSIS;
	  	data_processing_CMSIS(&data_asm_CMSIS, measurements, x_values_asm, length);

	  	struct data_processed data_C_CMSIS;
	  	data_processing_CMSIS(&data_C_CMSIS, measurements, x_values_C, length);

	  	struct data_processed data_CMSIS_CMSIS;
	  	data_processing_CMSIS(&data_CMSIS_CMSIS, measurements, x_values_CMSIS, length);


	  	// Try processing data with C implementation ----------------------------------------------------------------------
	  	struct data_processed data_asm_C;
	  	data_processing_CMSIS(&data_asm_C, measurements, x_values_asm, length);

	  	struct data_processed data_C_C;
	  	data_processing_CMSIS(&data_C_C, measurements, x_values_C, length);

	  	struct data_processed data_CMSIS_C;
	  	data_processing_CMSIS(&data_CMSIS_C, measurements, x_values_CMSIS, length);



	  // Above is code for part 2 ----------------------------------------------------------------------------------------
	  // ------------------------------------------------------------------------------------------------------------------


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
