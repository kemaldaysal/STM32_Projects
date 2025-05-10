/*
 * pwm_driver.c
 *
 *  Created on: Dec 10, 2023
 *      Author: Kemal
 */

#include "stm32f0xx_hal.h"
#include "pwm_driver.h"

static void pwm_pin_configure();

void pwm_init(void) {

	pwm_pin_configure();

	TIM3->PSC = 24-1; // Timer clock = 48 MHz / 24 = 2 MHz timer clock
	TIM3->ARR = 100-1; // Freq. & Period = (2 MHz / 100) = 20 KHz

	TIM3->CCR1 = 0; // Duty Cycle Initial Setting to 0
	TIM3->CCR2 = 0;
	TIM3->CCR3 = 0;
	TIM3->CCR4 = 0;

	// Set CH-1 for PWM Mode 1 (OC1M's 3 bits should be 110) Check reference manual for details.

	TIM3->CCMR1 |= TIM_CCMR1_OC1M_2; // Set bit 2 (actually 3rd) to 1
	TIM3->CCMR1 |= TIM_CCMR1_OC1M_1; // Set bit 1 (actually 2nd) to 1
	TIM3->CCMR1 |= TIM_CCMR1_OC1PE; // Enable Output Compare 1 Preload

	// Set CH-2 for PWM Mode 1

	TIM3->CCMR1 |= TIM_CCMR1_OC2M_2;
	TIM3->CCMR1 |= TIM_CCMR1_OC2M_1;
	TIM3->CCMR1 |= TIM_CCMR1_OC2PE;

	// Set CH-3 for PWM Mode 1

	TIM3->CCMR2 |= TIM_CCMR2_OC3M_2;
	TIM3->CCMR2 |= TIM_CCMR2_OC3M_1;
	TIM3->CCMR2 |= TIM_CCMR2_OC3PE;

	// Set CH-4 for PWM Mode 1

	TIM3->CCMR2 |= TIM_CCMR2_OC4M_2;
	TIM3->CCMR2 |= TIM_CCMR2_OC4M_1;
	TIM3->CCMR2 |= TIM_CCMR2_OC4PE;

	// Enable OC1REF and OC2REF Outputs by enabling CC1E bits of CCER register.

	TIM3->CCER |= TIM_CCER_CC1E;
	TIM3->CCER |= TIM_CCER_CC2E;
	TIM3->CCER |= TIM_CCER_CC3E;
	TIM3->CCER |= TIM_CCER_CC4E;
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
void pwm_set_duty_cycle_dynamically (uint32_t duty, Channels_e channel) {

	/* ** duty değeri aslında sayısal bir değer ancak yüzdelik cinsten olduğu şöyle anlaşılabilir,
		ARR registerı 99 (100-1) olduğundan, duty değeri de 0 ile 99 (100-1) arasında olmalıdır.
    	Başka bir örnek olarak, ARR = 500 olsaydı ancak biz duty'e 100 atasaydık,Duty Cycle'ı %20 ye ayarlamış olurduk.
    	Burada duty cycle, tamamen girilen duty değerinin ARR'ye oranına bağlı. ARR de clock frekansına bağlı.
    	100 ARR için 100 duty cycle atarsak, sürekli 1 veren bir PWM sinyali elde ederiz.
    */
	switch (channel) {

	case CHANNEL1:
		TIM3->CCR1 = duty;
		break;

	case CHANNEL2:
		TIM3->CCR2 = duty;
		break;

	case CHANNEL3:
		TIM3->CCR3 = duty;
		break;

	case CHANNEL4:
		TIM3->CCR4 = duty;
		break;

	}
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

	// PB5 - TIM3 CH2
	// B5 pin's AF1 can work as TIM3's PWM output as CH2 (can be found in "Datasheet").

	GPIO_InitStruct.Pin = GPIO_PIN_5;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	// PB0 - TIM3 CH3
	// B5 pin's AF1 can work as TIM3's PWM output as CH2 (can be found in "Datasheet").

	GPIO_InitStruct.Pin = GPIO_PIN_0;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	// PB1 - TIM3 CH4
	// B5 pin's AF1 can work as TIM3's PWM output as CH2 (can be found in "Datasheet").

	GPIO_InitStruct.Pin = GPIO_PIN_1;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	// The other options stayed same, so we didn't change them.


}

