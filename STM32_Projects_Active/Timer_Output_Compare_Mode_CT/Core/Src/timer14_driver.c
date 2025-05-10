/*
 * timer14_driver.c
 *
 *  Created on: Dec 5, 2023
 *      Author: Kemal
 */

#include "stm32f0xx_hal.h"
#include "timer14_driver.h"

static void oc_led_init(void);

void timer14_init(void) {

	oc_led_init();

	__HAL_RCC_TIM14_CLK_ENABLE();

	// SET TIMER TO OUTPUT COMPARE MODE;
	// CC1S, CCMR1's 0 and 1 bits must be 00.

	TIM14->CCMR1 &= ~(1 << 0);
	TIM14->CCMR1 &= ~(1 << 1);

	// SET Output Compare Mod to work as Toggle Mode

	TIM14->CCMR1 &= ~(1 << 6);
	TIM14->CCMR1 |= (1 << 5);
	TIM14->CCMR1 |= (1 << 4);

	// Enable OC1REF Output

	TIM14-> CCER |= TIM_CCER_CC1E;

	// Output Polarity: Active High

	TIM14->CCER &= ~TIM_CCER_CC1P;

	// Timer clock = 48 Mhz / 48000 = 1000 Hz (1 ms period)
	TIM14->PSC = 48000 - 1;

	// Set Period to 100 ms;

	TIM14->ARR = 100-1;

	// Set the value which'll be compared to timer on each step.

	TIM14->CCR1 = 100-1;


}


void oc_led_init() {


	// OC Output ==> GPIOA-7 (AF4)

	// Enable Clock

	__HAL_RCC_GPIOA_CLK_ENABLE();

	// Select Mode: Alternate Function
	// Bit 14: 0 ; Bit 15: 1

	GPIOA->MODER &= ~(1<<14); // AF
	GPIOA->MODER |= (1<<15); // AF

	GPIOA->OTYPER &= ~(1<<7);

	GPIOA->OSPEEDR |= (1<<14);
	GPIOA->OSPEEDR &= ~(1<<15);

	GPIOA->PUPDR &= ~(1<<14);
	GPIOA->PUPDR |= (1<<15);

	// Set AF for AF4
	// for AF4, 31, 30, 29, 28. bits of GPIOx_AFRH (AFR[0]) register must be 0100 in order;

	GPIOA->AFR[0] &= ~(1 << 31);
	GPIOA->AFR[0] |= (1 << 30);
	GPIOA->AFR[0] &= ~(1 << 29);
	GPIOA->AFR[0] &= ~(1 << 28);


}


void timer14_enable(void) {

	TIM14->CR1 |= TIM_CR1_CEN;
}

void timer14_disable(void) {

	TIM14->CR1 &= ~TIM_CR1_CEN;
}

void timer14_capture_set_period(uint32_t ms) {

	TIM14->ARR = ms-1;
	TIM14->CCR1 = ms-1;
}
