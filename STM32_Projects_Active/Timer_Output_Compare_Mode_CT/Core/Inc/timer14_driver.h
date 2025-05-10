/*
 * timer14_driver.h
 *
 *  Created on: Dec 5, 2023
 *      Author: Kemal
 */

#ifndef INC_TIMER14_DRIVER_H_
#define INC_TIMER14_DRIVER_H_

void timer14_init(void);

void timer14_enable(void);
void timer14_disable(void);

void timer14_capture_set_period(uint32_t ms);


#endif /* INC_TIMER14_DRIVER_H_ */
