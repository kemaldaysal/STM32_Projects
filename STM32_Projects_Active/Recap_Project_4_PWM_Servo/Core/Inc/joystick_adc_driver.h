/*
 * joystick_adc_driver.h
 *
 *  Created on: Feb 6, 2024
 *      Author: Kemal
 */

#ifndef INC_JOYSTICK_ADC_DRIVER_H_
#define INC_JOYSTICK_ADC_DRIVER_H_

#include <stdint.h>

void joystick_init(void);
uint16_t read_joystick_value(void);
int command_motors(uint16_t motor_commander_value);

#endif /* INC_JOYSTICK_ADC_DRIVER_H_ */
