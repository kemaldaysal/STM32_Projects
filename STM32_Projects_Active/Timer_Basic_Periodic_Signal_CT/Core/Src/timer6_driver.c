/*
 * timer6_driver.c
 *
 *  Created on: Dec 4, 2023
 *      Author: Kemal
 */

#include "stm32f0xx_hal.h"
#include "timer6_driver.h"
#include "led_driver.h"

void timer6_init(void) {

	__HAL_RCC_TIM6_CLK_ENABLE();

	TIM6->PSC = 48000-1; // 48 Mhz / 48000 = 1000 Hz -> 1 / 1 kHz = 1 ms period

	TIM6->ARR = 5000-1;

	TIM6->DIER |= TIM_DIER_UIE;

	NVIC_EnableIRQ(TIM6_IRQn);
	NVIC_SetPriority(TIM6_IRQn, 2);

}

void timer6_set_interrupt_period(uint16_t period) {
	TIM6->ARR = period-1;
}


uint16_t timer6_get_counter_value(void) {
	uint16_t cnt = TIM6->CNT;
	return cnt;
}

void timer6_enable(void) {
	TIM6->CR1 |= TIM_CR1_CEN;
}

void timer6_disable(void) {
	TIM6->CR1 &= ~(TIM_CR1_CEN);
}

void TIM6_IRQHandler(void) {

	TIM6->SR &= ~(TIM_SR_UIF); // interrupt bayrağını sıfırlama

	// GPIOA->ODR ^= GPIO_PIN_5; // led toggle, aşağıdakini de kullanabiliriz.
	user_led_toggle();
}




