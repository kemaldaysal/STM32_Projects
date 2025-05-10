/*
 * uart_driver.h
 *
 *  Created on: Dec 29, 2023
 *      Author: Kemal
 */

#ifndef INC_UART_DRIVER_H_
#define INC_UART_DRIVER_H_

#include "stm32f0xx_hal.h"

/**
  * @brief UART Initialization Function:
  * 	   Initializes UART with the following settings:
  * 	   Word Length = 8 Bits
      	   Stop Bit = One Stop bit
      	   Parity = None
           BaudRate = 115200 baud
           Hardware flow control disabled (RTS and CTS signals)

  * @param void
  * @retval void
  */
void UART_Init(void);

/**
  * @brief UART Transmitter Function:
  * 	     Sends one byte through UART

  * @param data
  * @retval void
  */
void UART_send_byte(uint8_t data);

/**
  * @brief UART Transmitter Function:
  * 	     Sends an array through UART

  * @param buffer to send
	* @param size
  * @retval void
  */

void UART_send_byte_array(uint8_t* buffer, uint32_t size);

/**
  * @brief UART Receiver Function:
  * 	     Reads one byte from UART buffer

  * @param None
  * @retval byte from the UART receiver buffer
	          -1 : if there's no data to read
  */

void UART_send_string(const char *str);

int32_t UART_read_byte(void);

/**
  * @brief Returns the number of bytes
					 waiting in the UART receive buffer
  * @param  None
  * @retval uint32_t
  */
uint32_t UART_bytes_to_read(void);

uint32_t UART_password_check(void);

#endif /* INC_UART_DRIVER_H_ */
