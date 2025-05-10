/*
 * led_driver.c
 *
 *  Created on: Dec 16, 2023
 *      Author: Kemal
 */

#include "led_driver.h"
#include "stm32f0xx_hal.h"
#include "stm32f070xb.h"


void led_init(void) {

	__HAL_RCC_GPIOA_CLK_ENABLE();

	// For general purpose output mode, bit 11 = 0, bit 10 = 1
	GPIOA->MODER &= ~(1<<11);
	GPIOA->MODER |= (1<<10);

	// For output push-pull mode, bit 5 = 0;

	GPIOA->OTYPER &= ~(1<<5);

	// For low speed, bit 11 = x , bit 10 = 0 .

	GPIOA->OSPEEDR &= ~(1<<10);

	// For pull-down, bit 11 = 1, bit 10 = 0

	GPIOA->PUPDR |= (1<<11);
	GPIOA->PUPDR &= ~(1<<10);





}

void led_on(void) {

	GPIOA->ODR |= (1<<5);

}

void led_off(void) {

	GPIOA->ODR &= ~(1<<5);

}

void led_toggle(void) {

	GPIOA->ODR ^= (1<<5);



}



