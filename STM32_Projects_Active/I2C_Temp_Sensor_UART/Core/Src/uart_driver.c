/*
 * uart_driver.c
 *
 *  Created on: Dec 29, 2023
 *      Author: Kemal
 */

#include "uart_driver.h"
#include "stm32f0xx_hal.h"
#include "stm32f070xb.h"
#include <string.h>
#include <stdint.h>

UART_HandleTypeDef UartHandle;

#define BUFFER_SIZE 256

typedef struct UART_Buffer_Type {
	uint32_t buffer[BUFFER_SIZE];
	uint32_t head_pointer;
	uint32_t tail_pointer;
} UART_Buffer_t;

volatile UART_Buffer_t UART_BufferRX;
volatile UART_Buffer_t UART_BufferTX;

static void Init_GPIO_for_UART(void);
static void UART_Error_Handler(void); // Optional for error handling
static int32_t UART_is_buffer_empty(volatile UART_Buffer_t *buffer);

void UART_Init(void) {

	Init_GPIO_for_UART();

	__HAL_RCC_USART2_CLK_ENABLE();

	/*##-3- Configure the UART peripheral ######################################*/
	/* Put the USART peripheral in the Asynchronous mode (UART Mode) */
	/* UART configured as follows:
	 - Word Length = 8 Bits
	 - Stop Bit = One Stop bit
	 - Parity = None
	 - BaudRate = 115200 baud
	 - Hardware flow control disabled (RTS and CTS signals) */
	UartHandle.Instance = USART2;

	UartHandle.Init.BaudRate = 115200; // was 9600 before
	UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
	UartHandle.Init.StopBits = UART_STOPBITS_1;
	UartHandle.Init.Parity = UART_PARITY_NONE;
	UartHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	UartHandle.Init.Mode = UART_MODE_TX_RX;
	UartHandle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

	if (HAL_UART_DeInit(&UartHandle) != HAL_OK) {

		UART_Error_Handler();
	}

	if (HAL_UART_Init(&UartHandle) != HAL_OK) {

		UART_Error_Handler();
	}

	/* 4- Enable UART Receive Data Register Not Empty */
	SET_BIT(USART2->CR1, USART_CR1_RXNEIE);

	// Enable UART Interrupts in NVIC and set high priority

	HAL_NVIC_SetPriority(USART2_IRQn, 0, 1);
	HAL_NVIC_EnableIRQ(USART2_IRQn);

}

void Init_GPIO_for_UART(void) {

	/*##-1- Enable peripherals and GPIO Clocks #################################*/
	/* Enable GPIO TX/RX clock for PA2 and PA3 */
	__HAL_RCC_GPIOA_CLK_ENABLE();

	// Let's set it for PA2 (TX) first

	// Moder should be AF mode (pins 5-4 must be 10

	GPIOA->MODER |= (1 << 5);
	GPIOA->MODER &= ~(1 << 4);

	// OTYPER must be push-pull, bit 2 must be 0

	GPIOA->OTYPER &= ~(1 << 2);

	// OSPEEDR must be high, bits 5-4 must be 11

	GPIOA->OSPEEDR |= (1 << 5);
	GPIOA->OSPEEDR |= (1 << 4);

	// PUPDR should be Pull Up, bits 5-4 must be 01

	GPIOA->PUPDR &= ~(1 << 5);
	GPIOA->PUPDR |= (1 << 4);

	// Let's set it for PA3 (RX) now.

	// Moder should be AF mode (pins 7-6 must be 10

	GPIOA->MODER |= (1 << 7);
	GPIOA->MODER &= ~(1 << 6);

	// OTYPER must be push-pull, bit 3 must be 0

	GPIOA->OTYPER &= ~(1 << 3);

	// OSPEEDR must be high, bits 7-6 must be 11

	GPIOA->OSPEEDR |= (1 << 7);
	GPIOA->OSPEEDR |= (1 << 6);

	// PUPDR should be Pull Up, bits 7-6 must be 01

	GPIOA->PUPDR &= ~(1 << 7);
	GPIOA->PUPDR |= (1 << 6);

}

/**
 * @brief  Retargets the C library printf function to the USART.
 * @param  None
 * @retval None
 */
int __io_putchar(int ch) {
	UART_send_byte(ch);
	return ch;
}

void USART2_IRQHandler(void) {

	/* UART in mode Receiver */
	if (((USART2->ISR & USART_ISR_RXNE) != 0)
			&& ((USART2->CR1 & USART_CR1_RXNEIE) != 0)) {

		UART_BufferRX.buffer[UART_BufferRX.head_pointer] = USART2->RDR;
		UART_BufferRX.head_pointer++;


		if (UART_BufferRX.head_pointer == BUFFER_SIZE) {
			UART_BufferRX.head_pointer = 0;
		}

		return;

	}

	if (((USART2->ISR & USART_ISR_TXE) != 0)
			&& ((USART2->CR1 & USART_CR1_TXEIE) != 0)) {

		if (UART_BufferTX.head_pointer != UART_BufferTX.tail_pointer) {
			// Send one byte from Transmit buffer
			USART2->TDR = UART_BufferTX.buffer[UART_BufferTX.tail_pointer];
			UART_BufferTX.tail_pointer++;

			if (UART_BufferTX.tail_pointer == BUFFER_SIZE) {
				UART_BufferTX.tail_pointer = 0;
			}
		} else {
			/* Disable the UART Transmit Data Register Empty Interrupt */
			CLEAR_BIT(USART2->CR1, USART_CR1_TXEIE);
		}

		return;
	}

}

void UART_send_byte(uint8_t data) {
	UART_BufferTX.buffer[UART_BufferTX.head_pointer] = data;
	UART_BufferTX.head_pointer++;

	if (UART_BufferTX.head_pointer == BUFFER_SIZE) {
		UART_BufferTX.head_pointer = 0;
	}
	/* Enable the UART Transmit Data Register Empty Interrupt */
	SET_BIT(USART2->CR1, USART_CR1_TXEIE);
}

void UART_send_byte_array(uint8_t *buffer, uint32_t size) {
	int i;

	for (i = 0; i < size; i++) {
		UART_send_byte(buffer[i]);
	}
}

void UART_send_string(const char *str) {
    while (*str != '\0') {
        UART_send_byte((uint8_t)*str++);
    }
}

int32_t UART_is_buffer_empty(volatile UART_Buffer_t *buffer) {
	return (buffer->head_pointer == buffer->tail_pointer ? 1 : 0);
}

int32_t UART_read_byte(void) {
	int kar = 0;

	if (UART_is_buffer_empty(&UART_BufferRX) == 1) {
		kar = -1;
	} else {
		kar = UART_BufferRX.buffer[UART_BufferRX.tail_pointer];
		UART_BufferRX.tail_pointer++;

		if (UART_BufferRX.tail_pointer == BUFFER_SIZE) {
			UART_BufferRX.tail_pointer = 0;
		}
	}

	return kar;
}

uint32_t UART_bytes_to_read(void) {
	if (UART_BufferRX.head_pointer >= UART_BufferRX.tail_pointer) {
		return UART_BufferRX.head_pointer - UART_BufferRX.tail_pointer;
	} else {
		return (BUFFER_SIZE + UART_BufferRX.head_pointer
				- UART_BufferRX.tail_pointer);
	}
}

static void UART_Error_Handler(void) {
	while (1) {

	}
}

