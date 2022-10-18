/* USER CODE BEGIN Header */
/**
 *******************************************************
 Info:		STM32 DMA and PWM with HAL
 Author:		Amaan Vally
 *******************************************************
 In this practical you will to use PWM using DMA on the STM32 using the HAL.
 We also set up an interrupt to switch the waveform between various LUTs.

 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//TO DO:
//TASK 2
//Assign values to NS, TIM2CLK and F_SIGNAL
#define NS 200
#define TIM2CLK 48000000
#define F_SIGNAL 5000

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim2_ch1;

/* USER CODE BEGIN PV */

//TO DO:
//TASK 1
//Create global variables for LUTs
uint32_t sin_LUT[NS] = { };
uint32_t saw_LUT[NS] = { };
uint32_t triangle_LUT[NS] = { };

//TO DO:
//TASK 3
//Calculate TIM2_Ticks
uint32_t TIM2_Ticks = TIM2CLK / (NS * F_SIGNAL); // Fc/ (NS *Fs)
uint32_t sin_LUT[] = { 527, 543, 559, 575, 591, 607, 623, 638, 654, 669, 684,
		699, 714, 729, 743, 757, 771, 785, 799, 812, 825, 837, 849, 861, 873,
		884, 895, 905, 915, 925, 934, 943, 951, 959, 967, 974, 980, 987, 992,
		997, 1002, 1006, 1010, 1013, 1016, 1018, 1020, 1021, 1022, 1023, 1022,
		1021, 1020, 1018, 1016, 1013, 1010, 1006, 1002, 997, 992, 987, 980, 974,
		967, 959, 951, 943, 934, 925, 915, 905, 895, 884, 873, 861, 849, 837,
		825, 812, 799, 785, 771, 757, 743, 729, 714, 699, 684, 669, 654, 638,
		623, 607, 591, 575, 559, 543, 527, 511, 495, 479, 463, 447, 431, 415,
		399, 384, 368, 353, 338, 323, 308, 293, 279, 265, 251, 237, 223, 210,
		197, 185, 173, 161, 149, 138, 127, 117, 107, 97, 88, 79, 71, 63, 55, 48,
		42, 35, 30, 25, 20, 16, 12, 9, 6, 4, 2, 1, 0, 0, 0, 1, 2, 4, 6, 9, 12,
		16, 20, 25, 30, 35, 42, 48, 55, 63, 71, 79, 88, 97, 107, 117, 127, 138,
		149, 161, 173, 185, 197, 210, 223, 237, 251, 265, 279, 293, 308, 323,
		338, 353, 368, 384, 399, 415, 431, 447, 463, 479, 495, 511 };
uint32_t sawtooth_LUT[] = { 0, 10, 21, 31, 41, 52, 62, 72, 83, 93, 103, 114,
		124, 134, 145, 155, 165, 176, 186, 196, 207, 217, 227, 238, 248, 258,
		269, 279, 289, 300, 310, 320, 331, 341, 351, 362, 372, 382, 393, 403,
		413, 424, 434, 444, 455, 465, 475, 486, 496, 506, 517, 527, 537, 548,
		558, 568, 579, 589, 599, 610, 620, 630, 641, 651, 661, 672, 682, 692,
		703, 713, 723, 734, 744, 754, 765, 775, 785, 796, 806, 816, 827, 837,
		847, 858, 868, 878, 889, 899, 909, 920, 930, 940, 951, 961, 971, 982,
		992, 1002, 1013, 1023, 0, 10, 21, 31, 41, 52, 62, 72, 83, 93, 103, 114,
		124, 134, 145, 155, 165, 176, 186, 196, 207, 217, 227, 238, 248, 258,
		269, 279, 289, 300, 310, 320, 331, 341, 351, 362, 372, 382, 393, 403,
		413, 424, 434, 444, 455, 465, 475, 486, 496, 506, 517, 527, 537, 548,
		558, 568, 579, 589, 599, 610, 620, 630, 641, 651, 661, 672, 682, 692,
		703, 713, 723, 734, 744, 754, 765, 775, 785, 796, 806, 816, 827, 837,
		847, 858, 868, 878, 889, 899, 909, 920, 930, 940, 951, 961, 971, 982,
		992, 1002, 1013, 1023 };
uint32_t triangular_LUT = { 0, 20, 41, 61, 82, 102, 123, 143, 164, 184, 205,
		225, 246, 266, 286, 307, 327, 348, 368, 389, 409, 430, 450, 471, 491,
		512, 532, 552, 573, 593, 614, 634, 655, 675, 696, 716, 737, 757, 777,
		798, 818, 839, 859, 880, 900, 921, 941, 962, 982, 1003, 1023, 1003, 982,
		962, 941, 921, 900, 880, 859, 839, 818, 798, 777, 757, 737, 716, 696,
		675, 655, 634, 614, 593, 573, 552, 532, 512, 491, 471, 450, 430, 409,
		389, 368, 348, 327, 307, 286, 266, 246, 225, 205, 184, 164, 143, 123,
		102, 82, 61, 41, 20, 0, 20, 41, 61, 82, 102, 123, 143, 164, 184, 205,
		225, 246, 266, 286, 307, 327, 348, 368, 389, 409, 430, 450, 471, 491,
		512, 532, 552, 573, 593, 614, 634, 655, 675, 696, 716, 737, 757, 777,
		798, 818, 839, 859, 880, 900, 921, 941, 962, 982, 1003, 1023, 1003, 982,
		962, 941, 921, 900, 880, 859, 839, 818, 798, 777, 757, 737, 716, 696,
		675, 655, 634, 614, 593, 573, 552, 532, 512, 491, 471, 450, 430, 409,
		389, 368, 348, 327, 307, 286, 266, 246, 225, 205, 184, 164, 143, 123,
		102, 82, 61, 41, 20 };

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void EXTI0_1_IRQHandler(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_DMA_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	/* USER CODE BEGIN 2 */

	//TO DO:
	//TASK 4
	//Start TIM3 in PWM mode on channel 1
	HAL_TIM_PWM_Start(&htim1, 1);
	//Start TIM2 in Output Compare (OC) mode on channel 1.
	HAL_TIM_PWM_Start(&htim2, 2);

	//Start the DMA in interrupt (IT) mode.
	uint32_t DestAddress = (uint32_t) &(TIM3->CCR1);

	//Start the DMA transfer
	__HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_CC1);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		//No need to do anything in the main loop for this practical

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
	RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = TIM2_Ticks - 1; //To make the frequency what we want it to be
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_OC_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_TIMING;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 1023;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel4_5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel4_5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel4_5_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, LD4_Pin | LD3_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : PA0 */
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : LD4_Pin LD3_Pin */
	GPIO_InitStruct.Pin = LD4_Pin | LD3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

}

/* USER CODE BEGIN 4 */
void EXTI0_1_IRQHandler(void) {
	//TO DO:
	//TASK 5
	//Disable DMA transfer, start DMA in IT mode with new source and re enable transfer
	//Remember to debounce using HAL_GetTick()

	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0); // Clear interrupt flags
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
