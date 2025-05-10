/*
 * timer6_driver.h
 *
 *  Created on: Dec 4, 2023
 *      Author: Kemal
 */

#ifndef INC_TIMER6_DRIVER_H_
#define INC_TIMER6_DRIVER_H_

#include <stdint.h>

void timer6_init(void);
void timer6_set_interrupt_period(uint16_t period);
uint16_t timer6_get_counter_value(void);
void timer6_enable(void);
void timer6_disable(void);

#endif /* INC_TIMER6_DRIVER_H_ */
