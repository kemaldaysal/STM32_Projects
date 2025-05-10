/*
 * pwm_driver.h
 *
 *  Created on: Dec 10, 2023
 *      Author: Kemal
 */

#ifndef INC_PWM_DRIVER_H_
#define INC_PWM_DRIVER_H_

#include <stdint.h>

typedef enum CHANNELS{
	CHANNEL1,
	CHANNEL2,
	CHANNEL3,
	CHANNEL4
}Channels_e;

void pwm_init(void);
void pwm_enable (void);
void pwm_disable (void);
void pwm_set_duty_cycle_dynamically (uint32_t duty, Channels_e channel);


#endif /* INC_PWM_DRIVER_H_ */
