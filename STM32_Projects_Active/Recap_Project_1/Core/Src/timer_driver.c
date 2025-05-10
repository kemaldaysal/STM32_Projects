/*
 * timer_driver.c
 *
 *  Created on: Dec 19, 2023
 *      Author: Kemal
 */

#include "timer_driver.h"
#include "stm32f0xx_hal.h"
#include "stm32f070xb.h"
#include "led_driver.h"

uint16_t timer_interrupt_triggered_count = 0;

extern uint8_t button_pressed;

void timer_init(void) {

	// We'll use this timer to generate a 1000 ms delay.

	__HAL_RCC_TIM6_CLK_ENABLE();

	//TIM6->CR1 |= (1<<2); // commented on 291223 1623, test it later

	// The clock source is 48 MHz.
	// 48.000.000 / 48000 = 1000 Hz -> 1 / 1kHz = 1 ms timer period

	TIM6->PSC = 48000-1;

	// We had a 1ms timer period, let it count 1000 times, which is equal to 1 second
	// and generate a event caused by overflow.

	TIM6->ARR = 3000-1;

	// Update interrupt (UIE) enable, in DIER register, bit 0 must be 1

	TIM6->DIER |= (1<<0);
	//TIM6->DIER |= TIM_DIER_UIE;

	NVIC_EnableIRQ(TIM6_IRQn);
	NVIC_SetPriority(TIM6_IRQn, 2);

}

void TIM6_IRQHandler(void) {

	TIM6->SR &= ~(TIM_SR_UIF); // interrupt bayrağını sıfırlama

	timer_interrupt_triggered_count++;

	if (button_pressed == 1) {

		TIM6->SR &= ~(TIM_SR_UIF);

		led_on();

		// timer_disable();

		button_pressed = 0;

	}

}



void timer_set_delay_time(uint16_t time) {
	TIM6->ARR = time-1;
}


void timer_enable(void) {
	// In order to enable the timer, bit 0 of CR1 register must be 1

	TIM6->CR1 |= (1<<0);

}

void timer_disable(void) {

	// In order to disable the timer, bit 0 of CR1 register must be 0

	TIM6->CR1 &= ~(1<<0);
}

/*
void clear_sr_flag_at_interrupt_startup(void) {

	// Clear the SR flag just after starting the IRQHandler function for timer.
	// In order to do so, bit 0 of SR register must be 0.

	TIM6->SR &= ~(1<<0);

}*/


/*

static void gpio_init_for_timer(void);

void gp_timer_init(void) {

	gpio_init_for_timer();

	// We'll use this timer to generate a 1000 ms delay.

	__HAL_RCC_TIM6_CLK_ENABLE();


	// We don't need them now, try it later.
	// Set the timer for Output Compare Mode

	// CC1 channel must be configured as output, so
	// CC1S bits (0 and 1) of CCMR1 register must be 00 for output mode

	TIM14->CCMR1 &= ~(1<<1);
	TIM14->CCMR1 &= ~(1<<0);

	// Detailed Output Compare 1 Mode Selection, we'll select toggle mode.
	// To do so, bits 6,5,4 must be 011

	TIM14->CCMR1 &= ~(1<<6);
	TIM14->CCMR1 |= (1<<5);
	TIM14->CCMR1 |= (1<<4);

	// The clock source is 48 MHz.
	// 48.000.000 / 48000 = 1000 Hz -> 1 / 1kHz = 1 ms timer period

	TIM14->PSC = 48000-1;

	// We had a 1ms timer period, let it count 1000 times, which is equal to 1 second
	// and generate a event caused by overflow.

	TIM14->ARR = 1000-1;

	// The codes below may be unnecessary, research later.
	// Update interrupt enable, in DIER register, bit 0 must be 1


} */



/* Enable it when compare mode will be used
void gpio_init_for_timer_output_compare_mode(void) {

	// Set PA7 for TIM14, AF4 mode will be used.

	// For AF mode, bits 15 and 14 must be 10
	GPIOA->MODER |= (1<<15);
	GPIOA->MODER &= ~(1<<14);

	// For push pull, bit 7 must be 0.
	GPIOA->OTYPER &= ~(1<<7);

	// For pull-down, bit 15 and 14 must be 10

	GPIOA->PUPDR |= (1<<15);
	GPIOA->PUPDR &= ~(1<<14);

	// Set it for AF4, we should use AFRL (AFR[0]),
	// bits 31, 30, 29, 28 must be 0100 for AF4 mode.

	GPIOA->AFR[0] &= ~(1<<31);
	GPIOA->AFR[0] |= (1<<30);
	GPIOA->AFR[0] &= ~(1<<29);
	GPIOA->AFR[0] &= ~(1<<28);

} */

/*
void enable_tim_14(void) {

	// In CR1 register, bit 0 must be 1 to enable the timer.

	TIM14->CR1 |= (1<<0);

}

void disable_tim_14(void){

	TIM14->CR1 &= ~(1<<0);

}*/



// Don't forget to enable the timer.
// U CAN USE THIS LATER, WHEN STARTING THE DELAY. TIM14->CNT = 0; instead of this, you can just enable and disable the timer when some conditions met.
// DON'T FORGET TO USE TIM14_IRQHandler function after ARR event generation.
// Also, don't forget to reset Status Register, which guided us to this IRQHandler function after interrupt.
//


