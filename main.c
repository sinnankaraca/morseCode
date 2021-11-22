/* USER CODE BEGIN Header */
/*******************************************************************************
 * File Name          : main.c
 * Description        : The program to indicate initials of the user input.
 * It will look for 2 Capital character which will be typed by user and indicate
 * through the PB3 - morseLD output of STM32L432.
 *
 * Author:              Sinan KARACA
 * Date:                13.06.2021
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// RULES OF MORSE CODE : 													  //
// The length of a dot is 1 time unit.										  //
// A dash is 3 time units.													  //
// The space between 2 initials are 3 time units.							  //
// 1 unit time presented by UNIT_TIME 			  						      //
#define UNIT_TIME 250

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;

//DMA defined for user input as receive interrupt
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
//uart_bfr 1 character buffer from serial communication
//cmd 1 character container from serial communication
uint8_t uart_bfr, cmd;


uint8_t cmdstate = 0; // cmdstate checking the user input it is written.

uint8_t init1; //containers of first and second initials
uint8_t init2;

uint8_t inits[2]; //array container of 2 initials

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

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
	MX_USART2_UART_Init();
	MX_TIM1_Init();
	/* USER CODE BEGIN 2 */
	HAL_UART_Receive_DMA(&huart2, &uart_bfr, 1);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		// The codes for user interface
		HAL_UART_Transmit(&huart2, "\r\n", 2, 1000);
		HAL_UART_Transmit(&huart2,"----------------------------------------\r\n", 42, 1000);
		HAL_UART_Transmit(&huart2, "Please type your first initial\r\n", 32,1000);
		HAL_UART_Transmit(&huart2,"Use only capital letter and 1 character:\r\n", 42, 1000);

		checkLetter(); // Check the first initial if it is between [A-Z]

		init1 = cmd; // Store the character if it is between [A-Z]

		HAL_UART_Transmit(&huart2,"----------------------------------------\r\n", 42, 1000);
		HAL_UART_Transmit(&huart2, "Please type your second initial\r\n", 33,1000);
		HAL_UART_Transmit(&huart2,"Use only capital letter and 1 character:\r\n", 42, 1000);

		checkLetter(); // Check the second initial if it is between [A-Z]

		init2 = cmd; // Store the character if it is between [A-Z]

		start(); // Indicate 2 stored initials with LED on board

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
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Configure LSE Drive Capability
	 */
	HAL_PWR_EnableBkUpAccess();
	__HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE
			| RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = 0;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 16;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV16;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
	/** Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1)
			!= HAL_OK) {
		Error_Handler();
	}
	/** Enable MSI Auto calibration
	 */
	HAL_RCCEx_EnableMSIPLLMode();
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 65535;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.BreakFilter = 0;
	sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
	sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
	sBreakDeadTimeConfig.Break2Filter = 0;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit(&htim1);

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

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
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel6_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(morseLD_GPIO_Port, morseLD_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : morseLD_Pin */
	GPIO_InitStruct.Pin = morseLD_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(morseLD_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

// FUNCTION      : HAL_UART_RxCpltCallback
// DESCRIPTION   :
//  Function captures the user input over DMA interrupt
//  Waits for the user to make cmdstate high.
//  Quotation from github:
//  https://github.com/u2man/controlling-LED-with-UART-DMA-STM32
// PARAMETERS    :
//   UART_HandleTypeDef *huart
//
// RETURNS       :
//   void

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

	if ((uart_bfr != 0x0D) && (uart_bfr != 0x0A)) {
		cmd = uart_bfr;
	}

	cmdstate = 1;
	HAL_UART_Receive_DMA(&huart2, &uart_bfr, 1);

}

// FUNCTION      : start
// DESCRIPTION   :
//   The function stores initials to array
// 	 Indicates the Led with stores initals
//	 Show the user if it is in progress.
// PARAMETERS    :
//                 Void
// RETURNS       :
//				   Void
void start() {
	inits[0] = init1;
	inits[1] = init2;

	HAL_UART_Transmit(&huart2, "----------------------------------------\r\n", 42, 1000);
	HAL_UART_Transmit(&huart2, "Your initials - ", 16, 1000);
	HAL_UART_Transmit(&huart2, inits, 2, 1000);
	HAL_UART_Transmit(&huart2, " - are in progress..\r\n", 22, 1000);

	ledMorseAlphabet(init1); // Match the first character between A-Z and make the morse indication.

	space(); // 3 morse unit time space between 2 initials
	space();
	space();

	ledMorseAlphabet(init2); // Match the second character between A-Z and make the morse indication.

	space(); // 3 morse unit time space between 2 initials
	space();
	space();
	HAL_UART_Transmit(&huart2, "\r\n", 2, 1000);
	HAL_UART_Transmit(&huart2, "COMPLETED!\r\n", 12, 1000);

	return;
}

// FUNCTION      : dot
// DESCRIPTION   :
//    "dot" indication of morse alphabet.
//    On/Off time of dot is one unit time. (250 ms.)
// PARAMETERS    :
//                 Void
// RETURNS       :
//				   Void

void dot() {
	HAL_UART_Transmit(&huart2, " . ", 3, 1000);//Serial output to show user, current state
	ledTiming(UNIT_TIME, UNIT_TIME);
	return;

}

// FUNCTION      : dash
// DESCRIPTION   :
//    "dash" indication of morse alphabet.
//    On time of dash is three unit time. (750 ms.)
//	  Off time of dash is one time. (250 ms.)
// PARAMETERS    :
//                 Void
// RETURNS       :
//				   Void

void dash() {
	HAL_UART_Transmit(&huart2, " - ", 3, 1000);	//Serial output to show user, current state
	ledTiming(3 * UNIT_TIME, UNIT_TIME);
	return;
}

// FUNCTION      : ledTiming
// DESCRIPTION   :
//    The actual function which controls the LED on/off
//    The indication LED is PB3 ( embedded led on stm32)
// PARAMETERS    :
//                 ontime: for LED high timing.
//				   offtime: for LED low timing
//
// RETURNS       :
//				   Void

void ledTiming(int onTime, int offTime) {

	HAL_GPIO_WritePin(morseLD_GPIO_Port, morseLD_Pin, GPIO_PIN_SET);
	HAL_Delay(onTime);
	HAL_GPIO_WritePin(morseLD_GPIO_Port, morseLD_Pin, GPIO_PIN_RESET);
	HAL_Delay(offTime);
	return;
}

// FUNCTION      : space
// DESCRIPTION   :
//    "space" indication of morse alphabet.
//	  space time is determined as 1 unit time.
// PARAMETERS    :
//                 Void
// RETURNS       :
//				   Void

void space() {
	HAL_Delay(UNIT_TIME);
	HAL_UART_Transmit(&huart2, "   ", 3, 1000); //Serial output to show user, current state
	return;
}

// FUNCTION      : ledMorseAlphabet
// DESCRIPTION   :
//    Match the character if it is between A-Z
//    It is suppose to match between A-Z. Because we ensure it is in A-Z
//	  by using checkletter function.
// PARAMETERS    :
//                 input : the character passing through to match.
// RETURNS       :
//				   Void

void ledMorseAlphabet(uint8_t input) {

	switch (input) {
	case 'A':
		dot();space();dash();
		break;
	case 'B':
		dash();space();dot();space();dot();space();dot();
		break;
	case 'C':
		dash();space();dot();space();dash();space();dot();
		break;
	case 'D':
		dash();space();dot();space();dot();space();
		break;
	case 'E':
		dot();
		break;
	case 'F':
		dot();space();dot();space();dash();space();dot();
		break;
	case 'G':
		dash();space();dash();space();dot();
		break;
	case 'H':
		dot();space();dot();space();dot();space();dot();
		break;
	case 'I':
		dot();space();dot();
		break;
	case 'J':
		dot();space();dash();space();dash();space();dash();
		break;
	case 'K':
		dash();space();dot();space();dash();
		break;
	case 'L':
		dot();space();dash();space();dot();space();dot();
		break;
	case 'M':
		dash();space();dash();
	case 'N':
		dash();space();dot();
	case 'O':
		dash();space();dash();space();dash();space();
		break;
	case 'P':
		dot();space();dash();space();dash();space();dot();
		break;
	case 'Q':
		dash();space();dash();space();dot();space();dash();
		break;
	case 'R':
		dot();space();dash();space();dot();
		break;
	case 'S':
		dot();space();dot();space();dot();
		break;
	case 'T':
		dash();
		break;
	case 'U':
		dot();space();dot();space();dash();
		break;
	case 'V':
		dot();space();dot();space();dot();space();dash();
		break;
	case 'W':
		dot();space();dash();space();dash();
		break;
	case 'X':
		dash();space();dot();space();dot();space();dash();
		break;
	case 'Y':
		dash();space();dot();space();dash();space();dash();
		break;
	case 'Z':
		dash();space();dash();space();dot();space();dot();
		break;
	default:
		// Error state
		dash();space();dash();space();dash();space();dash();space();dash();space();dash();space();
	}
}

// FUNCTION      : checkLetter
// DESCRIPTION   :
//    Match the character if it is between A-Z
//    It is suppose to match between A-Z. Because we ensure it is in A-Z
//	  by using checkletter function.
// PARAMETERS    :
//                 input : the character passing through to match.
// RETURNS       :
//				   Void

void checkLetter() {
	while (1) {
		char c;
		if (cmdstate == 1) {
			for (c = 'A'; c <= 'Z'; ++c) {
				if (cmd == c) {
					HAL_UART_Transmit(&huart2, " is one of your initials\r\n", 26, 1000);

					//cmdstate is like control bit of if user typed a input.
					//if it is not this function will be wait inside while loop for user to type input.
					cmdstate = 0;
					return 0;
				}
			}
			HAL_UART_Transmit(&huart2," is not valid. Please select between [A-Z]\r\n", 44, 1000);
			HAL_UART_Transmit(&huart2, "----------------------------------------\r\n", 42, 1000);
			cmdstate = 0;

		}
	}

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
