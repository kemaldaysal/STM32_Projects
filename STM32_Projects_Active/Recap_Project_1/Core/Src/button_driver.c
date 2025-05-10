/*
 * button_driver.c
 *
 *  Created on: Dec 16, 2023
 *      Author: Kemal
 */

#include "button_driver.h"
#include "stm32f0xx_hal.h"
#include "stm32f070xb.h"
#include "led_driver.h"
#include "timer_driver.h"

uint8_t button_state = 0;
uint8_t button_interrupt_triggered_count = 0;
uint8_t button_pressed = 0;

void button_init(void) {

	// Internal button is on PC13

	__HAL_RCC_GPIOC_CLK_ENABLE();

	// For input mode, bit 27 = 0, bit 26 = 0

	GPIOC->MODER &= ~(1<<27);
	GPIOC->MODER &= ~(1<<26);

	// For push-pull mode, bit 13 = 0

	GPIOC->OTYPER &= ~(1<<13);

	// For low-speed, bit 27 = x, bit 26 = 0

	GPIOC->OSPEEDR |= (1<<27);
	GPIOC->OSPEEDR |= (1<<26);

	// For pull-down, bit 27 = 1, bit 26 = 0

	//GPIOC->PUPDR |= (1<<27);
	//GPIOC->PUPDR &= ~(1<<26);

	// For pull-up, bit 27 = 0, bit 26 = 1

	GPIOC->PUPDR &= ~(1<<27);
	GPIOC->PUPDR |= (1<<26);

	// For no pull, bit 27 = 0, bit 26 = 0

	//GPIOC->PUPDR &= ~(1<<27);
	//GPIOC->PUPDR &= ~(1<<26);
}

void button_interrupt_init(void) {

	// Enable interrupts for PC13

	// Enable the SYSCFG clock to manage desired pins will be related to EXTI unit
	__HAL_RCC_SYSCFG_CLK_ENABLE();

	// Pick PC13 pin from there to enable interrupts for it.
	// For setting for PC13, we'll use SYSCFG_EXTICR4 (same as EXTICR[3])
	// and it's bits 7, 6, 5, 4 should be x 0 1 0

	SYSCFG->EXTICR[3] |= (1<<5);

	// Enable the desired interrupt line by unmasking that related bit for PC13.
	// For PC13, bit 13 must be 1 for unmasked interrupt.

	EXTI->IMR |= (1<<13);

	// Set interrupt trigger mechanism by rising edge (after releasing the button)
	// For PC13, bit 13 = 1 for rising edge trigger

	// EXTI->RTSR |= (1<<13);

	// For PC13, bit 13 in FTSR must be 1 for falling edge trigger

	EXTI->FTSR |= (1<<13);

	// Give permission to this interrupt event in NVIC unit.

	NVIC_SetPriority(EXTI4_15_IRQn, 1);
	NVIC_EnableIRQ(EXTI4_15_IRQn);

	// After that, when this interrupt occurs, it'll branch to an adress defined in vector addresses for EXTI.
	// That address book is in startup file with .s extension.

}

void  EXTI4_15_IRQHandler(void) {
	// Check if interrupt comes from PC13, if so, then do the below functions, if not, ignore.

	if ((EXTI->PR & EXTI_PR_PR13) == EXTI_PR_PR13) {

		button_interrupt_triggered_count++;

		// Reset the PR register's flag for PC13 by making it 1. (Making it 1 resets it.)
		EXTI->PR |= EXTI_PR_PR13;

		// read_and_use_button();
		//read_and_use_button_with_timer_delay();

		button_pressed = 1;

	}
}



void read_and_use_button(void) {

	button_state = (!(GPIOC->IDR & (1<<13)));

	if (button_state == 1) {
		led_on();
	}
	else if (button_state == 0) {
		led_off();
	}

}








