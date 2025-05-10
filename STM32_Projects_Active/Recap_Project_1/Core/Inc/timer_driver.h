/*
 * timer_driver.h
 *
 *  Created on: Dec 19, 2023
 *      Author: Kemal
 */

#ifndef INC_TIMER_DRIVER_H_
#define INC_TIMER_DRIVER_H_

#include <stdint.h>

void timer_init(void);
void timer_set_delay_time(uint16_t time);
void timer_enable(void);
void timer_disable(void);

#endif /* INC_TIMER_DRIVER_H_ */
