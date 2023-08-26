/*
 * stm32f429xx_timer_driver.c
 *
 *  Created on: 02-Aug-2023
 *      Author: rakshit
 */

#include "stm32f429xx_timer_driver.h"

volatile uint32_t* tim2Cr1 = (uint32_t*) TIM2_CR1;
volatile uint32_t* tim2Die1 = (uint32_t*) TIM2_DIER;
volatile uint32_t* tim2Sr = (uint32_t*) TIM2_SR;
volatile uint32_t* tim2Egr = (uint32_t*) TIM2_EGR;
volatile uint32_t* tim2Cnt = (uint32_t*) TIM2_CNT;
volatile uint32_t* tim2Psc = (uint32_t*) TIM2_PSC;
volatile uint32_t* tim2Arr = (uint32_t*) TIM2_ARR;
volatile uint32_t* tim2Ccr1 = (uint32_t*) TIM2_CCR1;
volatile uint32_t* tim2Ccmr1 = (uint32_t*) TIM2_CCMR1;
volatile uint32_t* tim2Ccer = (uint32_t*) TIM2_CCER;

uint32_t getTimerClkFreq()
{
	uint32_t fclk = 16000000;  // HSI, no AHB or APB prescalar
	// TODO(add support for timer prescalar)

	return fclk;
}

void init_tim2_pclk()
{
	// enable tim2 clock
	RCC->APB1ENR |= (1 << 0);
}

void tim2_delay_ms_IT(uint32_t delay_ms)
{
	uint32_t reload_val = delay_ms * (getTimerClkFreq() / 1000);  // no of cycles to count
	// Enable timer interrupt
	*tim2Die1 |= (1 << 0);  // DIER.UIE

	// program tim2 ARR;
	*tim2Arr = reload_val;

	// enable tim2
	*tim2Cr1 |= (1 << 0);
}

void tim2_delay_ms(uint32_t delay_ms)
{
	uint32_t reload_val = delay_ms * (getTimerClkFreq() / 1000);  // no of cycles to count

	// program tim2 ARR;
	*tim2Arr = reload_val;

	*tim2Cnt = 0;
	// enable tim2
	*tim2Cr1 |= (1 << 0);

	while(! (*tim2Sr & (1 << 0)));

	// SW needs to clear the status flag
	*tim2Sr &= ~(1 << 0);

}

void tim2_init_input_capture()
{
	 // Map TIM2 Channel 1 to TI1
	*tim2Ccmr1 &= ~(0b11 << 0);  // CCMR1.CC1S
	*tim2Ccmr1 |= (0b01 << 0);  // CC1 as input, connected to TI1
	// No filter, no prescalar

	// Program CC1 polarity
	// 0b00 means rising trigger
	// bit 0 needs to be programmed in CCER.CC1P
	// bit 1 needs to be programmed in CCER.CNP
	*tim2Ccer &= ~(1 << 1);  // CC1P
	*tim2Ccer &= ~(1 << 3);  // CC1NP

	// Enable input capture
	*tim2Ccer |= (1 << 0);  // CC1E

	*tim2Cr1 |= (1 << 0);
}
