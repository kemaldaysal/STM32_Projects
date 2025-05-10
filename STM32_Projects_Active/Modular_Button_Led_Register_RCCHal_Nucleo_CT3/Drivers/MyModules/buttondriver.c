/*
 * buttondriver.c
 *
 *  Created on: Oct 31, 2023
 *      Author: Kemal
 */

#include "stm32f0xx_hal.h"

void buttondriver_init() {
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

}

int buttondriver_get_state() {

	if (!(GPIOC->IDR & (1<<13))){
		return 1;
	}
	else
	{
		return 0;
	}

}
