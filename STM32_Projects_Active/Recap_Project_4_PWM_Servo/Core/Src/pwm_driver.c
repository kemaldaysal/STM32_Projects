/*
 * pwm_driver.c
 *
 *  Created on: Dec 10, 2023
 *      Author: Kemal
 */

#include "stm32f0xx_hal.h"
#include "pwm_driver.h"

static void pwm_pin_configure();
static void motor_pin_configure();

void pwm_init(void) {

	pwm_pin_configure();
	motor_pin_configure();

	TIM3->PSC = 48-1; // Timer clock = 48 MHz / 48 = 1 MHz timer clock
	TIM3->ARR = 1000-1; // Period must be 20 ms for this RC Servo. 1/20ms = 50 Hz. So we need to obtain 50 Hz result from 1 MHz timer clock. Then we must divide 1 MHz timer clock after prescaler by x to get 50 Hz frequency. Then it's 20.000
	// While servos are generally driven with 50 Hz, DC motors are driwen with > 500 Hz. So we'll use 1000 Hz now.

	// Initial duty cycle is set to 1000 instead of 0 because this RC servo starts at 0 degres when a pwm signal whose duty cycle equal to 1ms is applied.
    //	If we make it 2000 (2ms), then the motors will go to 180 degrees position.
	TIM3->CCR1 = 1000; // Duty Cycle Initial Setting to 1000. Reason is above.
	TIM3->CCR2 = 1000;
	TIM3->CCR3 = 1000;
	TIM3->CCR4 = 1000;

	// Set CH-1 for PWM Mode 1 (OC1M's 3 bits should be 110) Check reference manual for details.

	TIM3->CCMR1 |= TIM_CCMR1_OC1M_2; // Set bit 2 (actually 3rd) to 1
	TIM3->CCMR1 |= TIM_CCMR1_OC1M_1; // Set bit 1 (actually 2nd) to 1
	TIM3->CCMR1 |= TIM_CCMR1_OC1PE; // Enable Output Compare 1 Preload

	// Enable OC1REF and OC2REF Outputs by enabling CC1E bits of CCER register.

	TIM3->CCER |= TIM_CCER_CC1E;
}

void pwm_enable (void) {

// Enable Timer
	TIM3->CR1 |= TIM_CR1_CEN;

	// Enable Timer Event Generation Register
	TIM3->EGR |= TIM_EGR_UG; // UG: Update Generation -> 0: No action, 1: Re-initialize the counter and generates an update of the registers.


}

void pwm_disable (void) {

// Disable Timer
	TIM3->CR1 &= ~(TIM_CR1_CEN);


}

// Channels_e is an enumeration here. Declared on pwmdriver.h
void pwm_set_duty_cycle_dynamically (uint32_t duty) {

	/* ** duty değeri aslında sayısal bir değer ancak yüzdelik cinsten olduğu şöyle anlaşılabilir,
		ARR registerı 99 (100-1) olduğundan, duty değeri de 0 ile 99 (100-1) arasında olmalıdır.
    	Başka bir örnek olarak, ARR = 500 olsaydı ancak biz duty'e 100 atasaydık,Duty Cycle'ı %20 ye ayarlamış olurduk.
    	Burada duty cycle, tamamen girilen duty değerinin ARR'ye oranına bağlı. ARR de clock frekansına bağlı.
    	100 ARR için 100 duty cycle atarsak, sürekli 1 veren bir PWM sinyali elde ederiz.
    */

	// motor için yarı hızda 500 verilecek.

		TIM3->CCR1 = duty;

}






static void pwm_pin_configure() {

	GPIO_InitTypeDef GPIO_InitStruct;

	__HAL_RCC_TIM3_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	// PB4 - TIM3 CH1
	// B4 pin's AF1 can work as TIM3's PWM output as CH1 (can be found in "Datasheet").

	GPIO_InitStruct.Pin = GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF1_TIM3;

	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	// The other options stayed same, so we didn't change them.
}

static void motor_pin_configure() {

	// For controlling 6V 250 rpm DC motor with L298N, we'll use IN1, IN2 and EN pins of L298N.
	// The CH1 PWM generation pin from STM32 (B4 in above config.), goes to EN pin on L298N.
	// The IN1 pin controls the motor in one-way, the IN2 pin controls the motor in other way.
	// So we'll use PB9 for IN1, PB8 for IN2 in output mode.

	// First, let's set PB9 pin using registers

	// Bits 19 and 18 of MODER must be 01 for GP Output mode.
	GPIOB->MODER &= ~(1<<19);
	GPIOB->MODER |= (1<<18);

	// Pin 9 must be 0 for push-pull
	GPIOB->OTYPER &= ~(1<<9);

	// For high speed, pin 19-18 must be 11
	GPIOB->OSPEEDR |= (1<<19);
	GPIOB->OSPEEDR |= (1<<18);

	// For pull-down, pin 19-18 must be 10

	GPIOB->PUPDR |= (1<<19);
	GPIOB->PUPDR &= ~(1<<18);

	// Second, let's set PB8 pin using registers

	// For GP Output mode, bits 17-16 must be 01
	GPIOB->MODER &= ~(1<<17);
	GPIOB->MODER |= (1<<16);

	// Pin 9 must be 0 for push-pull
	GPIOB->OTYPER &= ~(1<<8);

	// For high speed, pin 19-18 must be 11
	GPIOB->OSPEEDR |= (1<<17);
	GPIOB->OSPEEDR |= (1<<16);

	// For pull-down, pin 19-18 must be 10

	GPIOB->PUPDR |= (1<<17);
	GPIOB->PUPDR &= ~(1<<16);

}

void set_motor_direction() {

	// Let's try IN1 (PB9) as HIGH, IN2 (PB8) as LOW first.

	// Make B9 high
	GPIOB->ODR |= (1<<9);

	// Make B8 low
	GPIOB->ODR &= ~(1<<8);


}


