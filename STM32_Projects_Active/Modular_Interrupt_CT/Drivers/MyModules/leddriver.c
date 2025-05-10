/*
 * leddriver.c
 *
 *  Created on: Oct 31, 2023
 *      Author: Kemal
 */

#include "stm32f0xx_hal.h"

void LedDriver_Init(void)
{

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

}

void LedDriver_On(void)
{
	GPIOA->ODR |= (1 << 5);
}

void LedDriver_Off(void)
{
	GPIOA->ODR &= ~(1 << 5);
}
