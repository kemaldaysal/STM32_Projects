/*
 * joystick_adc_driver.c
 *
 *  Created on: Feb 6, 2024
 *      Author: Kemal
 */

#include "joystick_adc_driver.h"
#include "stm32f0xx_hal.h"
#include "pwm_driver.h"

uint16_t ADC_read_value;
int motor_speed;
int16_t x_axis_upper_deadzone = 2220;
int16_t x_axis_bottom_deadzone = 1790;

ADC_HandleTypeDef hadc;

static void MX_ADC_Init(void);
extern void Error_Handler(void);

void joystick_init(void) {

	MX_ADC_Init();
	HAL_ADC_Start(&hadc);

}

uint16_t read_joystick_value(void) {
	if (HAL_ADC_PollForConversion(&hadc, 500) == HAL_OK) {
		ADC_read_value = HAL_ADC_GetValue(&hadc);
		return ADC_read_value;
	} else {
		return -1;
	}
}

int command_motors(uint16_t motor_commander_value) {

	if (motor_commander_value < x_axis_bottom_deadzone) {

		// Make B9 (IN1) high, B8 (IN2) low
		GPIOB->ODR |= (1<<9);
		GPIOB->ODR &= ~(1<<8);
		motor_speed = ((motor_commander_value - x_axis_bottom_deadzone) * (-1) / 3);

	}	else if (motor_commander_value > x_axis_upper_deadzone) {

		// Make B9 (IN1) low, B8 (IN2) high
		GPIOB->ODR &= ~(1<<9);
		GPIOB->ODR |= (1<<8);
		motor_speed = ((motor_commander_value - x_axis_upper_deadzone) / 3);

	} else {
		motor_speed = 0;
	}

	pwm_set_duty_cycle_dynamically(motor_speed);
	HAL_Delay(50);

	return motor_speed;


}

static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}


