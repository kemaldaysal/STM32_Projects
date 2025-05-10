/*
 * buttondriver.c
 *
 *  Created on: Oct 31, 2023
 *      Author: Kemal
 */

#include "stm32f0xx_hal.h"

void ButtonDriver_Interrupt_Init() {

	// 1) ENABLE THE CLOCKS FIRST

	// Configure the internal button in PC13
	// Enable Clock Register AHB1 for PC13 by making 19. bit to 1:
	//RCC->AHBENR |= (1 << 19);

	// 1.a) C portunu aktif etmeliyiz. Bu sefer bunun yerine HAL kütüphanesiyle çağıralım. Sebebi, CT Kesmeler -4- videosunda, Errata sheets'teki
	// çakışma ve ST firmasının kasıtlı bir gecikme koyması. Ayrıntılar için videonun 3. dakikasını tekrar izleyebilirsin.

	__HAL_RCC_GPIOC_CLK_ENABLE(); // Bunlar aslında birer makro, aslında bizim daha önce registerla yaptığımız işlemlerin aynısını yapıyor ama kısaltılmış hali.

	// 1.b) SYSCFG External Interrupt Config Registerları yöneten clocku aktif et.
	// Bu birim, hangi pinlerin EXTI'ye yönlendirileceğini belirler.

	__HAL_RCC_SYSCFG_CLK_ENABLE();

	// 2) GPIO Settings

	// Configure PC13 as INPUT by making bits 26 and 27 to 00.
	// GPIOC->MODER &= ~((1 << 27) | (1 << 26));

	GPIOC->MODER &= ~(3UL << 26);

	// Yukarıdaki gösterim, 3'ü ifade eden sayıyı (0011) 26 defa sola kaydır ve registerla and'leyip oraları 0 yap anlamına gelir.
	// Yani hem 26 hem de 27'yi 0 yapmanın kolay yoludur. 7UL yapsaydık 0111'i sola kaydıracaktı. 26, 27, 28. biti değiştirmiş olacaktık.


	// Configure PC13 as push-pull by changing bit 13 to 0 (it was like this at default).
	GPIOC->OTYPER &= ~(1 << 13);

	// Configure PC13's speed as low (00) by making bit 26 and 27 to 00.
	//GPIOC->OSPEEDR &= ~((1 << 26) | (1 << 27));
	GPIOC->OSPEEDR &= ~(3UL << 26);

	// Enable PC13's pull down resistor by making making bit 27 and 26 to 10.
	GPIOC->PUPDR |= (1 << 27);
	GPIOC->PUPDR &= ~(1 << 26);

	// 3) EXTI Settings

	// Select EXTI_13 ==> GPIOC
	// 13. pini de kontrol eden EXTICR4'ün 5. pinini 1 yaparsak, 010 inputuyla 13. pini interrupta duyarlı hale getirdik.
	// 4. REGİSTER AMA İSİMLENDİRMEYLE ALAKALI BİR SORUN VAR, BİR YERDE DİZİ, DİĞER YERDE İNDEXLİ TANIMLANMIŞ, BUNUN İÇİN 3 KULLANDIK

	SYSCFG->EXTICR[3] |= (1<<5);
	// SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI13_PC; // Hazır yapmak için bunu da kullanabilirsin.

	EXTI->IMR |= (1 << 13);
	// EXTI->IMR |= EXTI_IMR_IM13; // Alternative

	EXTI->RTSR |= (1 << 13);
	// EXTI->RTSR |= EXTI_RTSR_RT13; // Alternative

	// 4) NVIC Settings

	// NVIC Registerlarıyla da yapabilirdik. Ancak CMSIS kütüphanelerini tercih ettik.

	NVIC_SetPriority(EXTI4_15_IRQn, 1);

	NVIC_EnableIRQ(EXTI4_15_IRQn);

	// Artık EXTI'den gelen kesme NVIC'e, oradan onaylandıysa CPU'ya ulaşacak. CPU'dan da bir adrese dallanacak.
	// Dallanacağı adres de startup adlı, .s uzantılı vektör adreslerinde.


}


void EXTI4_15_IRQHandler(void) // Bu özel kesme hizmet fonksiyonları, input veya output alamadığı/veremediği için void-void şeklinde tanımladık. Bu fonksiyonu da startup.s scriptinden aldık. Oradaki tanımlaması weak şeklindeydi. Buraya alarak öncelikli yaptık.
{

	if ((EXTI->PR & EXTI_PR_PR13) == EXTI_PR_PR13) // Sadece 13. pinden geldiyse işlem yap. Çünkü EXTI4_15 arası ortak hat olduğundan 13 olduğunu belirtmeliyiz. Diğerlerinden de gelmiş olabilir.
	{
		EXTI->PR |= EXTI_PR_PR13; // 13. pinden gelen "Interrupt var" flagini sıfırlamak için PR registerını 1 yap. (ters mantık ama normal, kafan karışmasın.)

		GPIOA->ODR ^= (1 << 5); // XOR ile Toggle led, leddriver'ı şimdilik kullanmadık çünkü o da ayrı bir fonksiyonda tanımlı.

	}


}


int ButtonDriver_Get_State() {

	if (!(GPIOC->IDR & (1<<13))){
		return 1;
	}
	else
	{
		return 0;
	}

}
