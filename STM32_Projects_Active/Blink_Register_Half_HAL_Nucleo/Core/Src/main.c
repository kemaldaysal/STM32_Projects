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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
//static void MX_GPIO_Init(void);
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
	//MX_GPIO_Init();
	/* USER CODE BEGIN 2 */
	// CUSTOM CODES
	// Configure the internal LED on PA5
	RCC->AHBENR |= (1 << 17); // Enable GPIOA port

	// Configure PA5 as output by making bits 10 and 11 to 01.
	GPIOA->MODER &= ~(1 << 11);
	GPIOA->MODER |= (1 << 10);

	// Configure PA5 as push-pull by changing bit 5 to 0 (it was like this at default).
	GPIOA->OTYPER &= ~(1 << 5);

	// Configure PA5's speed as low (00) by making bit 10 and 11 to 00.
	GPIOA->OSPEEDR &= ~((1 << 11) | (1 << 10));

	// Enable PA5's pull down resistor by making making bit 10 and 11 to 10.
	GPIOA->PUPDR |= (1 << 11);
	GPIOA->PUPDR &= ~(1 << 10);

// Configure the internal button in PC13

	// Enable Clock Register AHB1 for PC13 by making 19. bit to 1:
	RCC->AHBENR |= (1 << 19);

	// Configure PC13 as INPUT by making bits 26 and 27 to 00.
	GPIOC->MODER &= ~((1 << 27) | (1 << 26));

	// Configure PC13 as push-pull by changing bit 13 to 0 (it was like this at default).
	GPIOC->OTYPER &= ~(1 << 13);

	// Configure PC13's speed as low (00) by making bit 26 and 27 to 00.
	GPIOC->OSPEEDR &= ~((1 << 26) | (1 << 27));

	// Enable PC13's pull down resistor by making making bit 27 and 26 to 10.
	GPIOC->PUPDR |= (1 << 27);
	GPIOC->PUPDR &= ~(1 << 26);

	/*	Bit işlemlerinin konu anlatımı ve açıklamalarını not aldın, gerektiğinde tekrar oku !! */

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */
		// Custom code while
		// Blink test using registers
		/*
		 GPIOA_ODR |= (1<<5);
		 HAL_Delay(500);
		 GPIOA_ODR &= ~(1<<5);
		 HAL_Delay(500);
		 */
		// veya XOR ile de yapılabilir, 5. bitin değilini alarak PA5'i toggle yapar.
		/*
		GPIOA->ODR ^= (1 << 5);
		HAL_Delay(500);
		 */

		if (!(GPIOC->IDR & (1<<13))) // PC13'teki butona basıldıysa 13 biti 1 oluyor. (Ancak buton ters çalışıyor, basılınca 0 oluyor!)
		{
			GPIOA->ODR |= (1 << 5);
		}
		else
		{
			GPIOA->ODR &= ~(1 << 5);
		}


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
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
//static void MX_GPIO_Init(void) {
//	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
//	/* USER CODE BEGIN MX_GPIO_Init_1 */
//	/* USER CODE END MX_GPIO_Init_1 */
//
//	/* GPIO Ports Clock Enable */
//	__HAL_RCC_GPIOC_CLK_ENABLE();
//	__HAL_RCC_GPIOF_CLK_ENABLE();
//	__HAL_RCC_GPIOA_CLK_ENABLE();
//
//	/*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
//	GPIO_InitStruct.Pin = USART_TX_Pin | USART_RX_Pin;
//	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//	GPIO_InitStruct.Pull = GPIO_NOPULL;
//	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//	GPIO_InitStruct.Alternate = GPIO_AF1_USART2;
//	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//
//	/* USER CODE BEGIN MX_GPIO_Init_2 */
//	/* USER CODE END MX_GPIO_Init_2 */
//}

/* USER CODE BEGIN 4 */

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
