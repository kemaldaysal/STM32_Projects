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

uint8_t buttonCounter = 0;
uint8_t buttonState = 0;

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

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

	  if ((buttonState = (HAL_GPIO_ReadPin(Button_External_GPIO_Port, Button_External_Pin))) == 1)
	  {
		  while ((buttonState = (HAL_GPIO_ReadPin(Button_External_GPIO_Port, Button_External_Pin))) == 1);
		  HAL_Delay(300);

		  buttonCounter = buttonCounter + 1;
		  //HAL_Delay(300);

	  }

	  if (buttonCounter == 0) // Display "1"
	  {
		  HAL_GPIO_WritePin(Segment_A_GPIO_Port, Segment_A_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(Segment_B_GPIO_Port, Segment_B_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(Segment_C_GPIO_Port, Segment_C_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(Segment_D_GPIO_Port, Segment_D_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(Segment_E_GPIO_Port, Segment_E_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(Segment_F_GPIO_Port, Segment_F_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(Segment_G_GPIO_Port, Segment_G_Pin, GPIO_PIN_RESET);

	  }

	  else if (buttonCounter == 1) // Display "1"
	  {
		  HAL_GPIO_WritePin(Segment_A_GPIO_Port, Segment_A_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Segment_B_GPIO_Port, Segment_B_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(Segment_C_GPIO_Port, Segment_C_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(Segment_D_GPIO_Port, Segment_D_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Segment_E_GPIO_Port, Segment_E_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Segment_F_GPIO_Port, Segment_F_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Segment_G_GPIO_Port, Segment_G_Pin, GPIO_PIN_RESET);

	  }

	  else if (buttonCounter == 2){ // Display "2"

		  HAL_GPIO_WritePin(Segment_A_GPIO_Port, Segment_A_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(Segment_B_GPIO_Port, Segment_B_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(Segment_C_GPIO_Port, Segment_C_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Segment_D_GPIO_Port, Segment_D_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(Segment_E_GPIO_Port, Segment_E_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(Segment_F_GPIO_Port, Segment_F_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Segment_G_GPIO_Port, Segment_G_Pin, GPIO_PIN_SET);

	  }

	  else if (buttonCounter == 3){ // Display "3"

		  HAL_GPIO_WritePin(Segment_A_GPIO_Port, Segment_A_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(Segment_B_GPIO_Port, Segment_B_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(Segment_C_GPIO_Port, Segment_C_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(Segment_D_GPIO_Port, Segment_D_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(Segment_E_GPIO_Port, Segment_E_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Segment_F_GPIO_Port, Segment_F_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Segment_G_GPIO_Port, Segment_G_Pin, GPIO_PIN_SET);

	  }

	  else if (buttonCounter == 4){ // Display "4"

		  HAL_GPIO_WritePin(Segment_A_GPIO_Port, Segment_A_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Segment_B_GPIO_Port, Segment_B_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(Segment_C_GPIO_Port, Segment_C_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(Segment_D_GPIO_Port, Segment_D_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Segment_E_GPIO_Port, Segment_E_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Segment_F_GPIO_Port, Segment_F_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(Segment_G_GPIO_Port, Segment_G_Pin, GPIO_PIN_SET);

	  }

	  else if (buttonCounter == 5){ // Display "5"

		  HAL_GPIO_WritePin(Segment_A_GPIO_Port, Segment_A_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(Segment_B_GPIO_Port, Segment_B_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Segment_C_GPIO_Port, Segment_C_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(Segment_D_GPIO_Port, Segment_D_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(Segment_E_GPIO_Port, Segment_E_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Segment_F_GPIO_Port, Segment_F_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(Segment_G_GPIO_Port, Segment_G_Pin, GPIO_PIN_SET);

	  }

	  else if (buttonCounter == 6){ // Display "6"

		  HAL_GPIO_WritePin(Segment_A_GPIO_Port, Segment_A_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(Segment_B_GPIO_Port, Segment_B_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Segment_C_GPIO_Port, Segment_C_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(Segment_D_GPIO_Port, Segment_D_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(Segment_E_GPIO_Port, Segment_E_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(Segment_F_GPIO_Port, Segment_F_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(Segment_G_GPIO_Port, Segment_G_Pin, GPIO_PIN_SET);

	  }

	  else if (buttonCounter == 7){ // Display "7"

		  HAL_GPIO_WritePin(Segment_A_GPIO_Port, Segment_A_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(Segment_B_GPIO_Port, Segment_B_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(Segment_C_GPIO_Port, Segment_C_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(Segment_D_GPIO_Port, Segment_D_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Segment_E_GPIO_Port, Segment_E_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Segment_F_GPIO_Port, Segment_F_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Segment_G_GPIO_Port, Segment_G_Pin, GPIO_PIN_RESET);

	  }

	  else if (buttonCounter == 8){ // Display "8"

		  HAL_GPIO_WritePin(Segment_A_GPIO_Port, Segment_A_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(Segment_B_GPIO_Port, Segment_B_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(Segment_C_GPIO_Port, Segment_C_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(Segment_D_GPIO_Port, Segment_D_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(Segment_E_GPIO_Port, Segment_E_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(Segment_F_GPIO_Port, Segment_F_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(Segment_G_GPIO_Port, Segment_G_Pin, GPIO_PIN_SET);

	  }

	  else if (buttonCounter == 9){ // Display "9"

		  HAL_GPIO_WritePin(Segment_A_GPIO_Port, Segment_A_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(Segment_B_GPIO_Port, Segment_B_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(Segment_C_GPIO_Port, Segment_C_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(Segment_D_GPIO_Port, Segment_D_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(Segment_E_GPIO_Port, Segment_E_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Segment_F_GPIO_Port, Segment_F_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(Segment_G_GPIO_Port, Segment_G_Pin, GPIO_PIN_SET);

	  }

	  else { // Reset counter and display "0"

		  buttonCounter = 0;

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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Segment_A_Pin|Segment_B_Pin|Segment_E_Pin|Segment_F_Pin
                          |Segment_G_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Segment_D_Pin|Segment_C_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Segment_A_Pin Segment_B_Pin Segment_E_Pin Segment_F_Pin
                           Segment_G_Pin */
  GPIO_InitStruct.Pin = Segment_A_Pin|Segment_B_Pin|Segment_E_Pin|Segment_F_Pin
                          |Segment_G_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Button_External_Pin */
  GPIO_InitStruct.Pin = Button_External_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(Button_External_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Segment_D_Pin Segment_C_Pin */
  GPIO_InitStruct.Pin = Segment_D_Pin|Segment_C_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
