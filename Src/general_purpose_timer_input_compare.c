/*
 * general_purpose_timer_interrupt.c
 *
 *  Created on: Aug 1, 2023
 *      Author: rakshit
 */


#include "stm32f429xx.h"
#include <string.h>
#include <stdio.h>
#include <inttypes.h>

extern volatile uint32_t* tim2Cr1;
extern volatile uint32_t* tim2Die1;
extern volatile uint32_t* tim2Sr;
extern volatile uint32_t* tim2Egr;
extern volatile uint32_t* tim2Cnt;
extern volatile uint32_t* tim2Psc;
extern volatile uint32_t* tim2Arr;
extern volatile uint32_t* tim2Ccr1;

GPIO_Handle_t GpioButton;
GPIO_Handle_t GpioLed;
uint8_t timEvent = 0;

void GPIO_LED_and_Button_Init()
{
	GpioLed.pGPIOx = GPIOG;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTP;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;

	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&GpioLed);
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&GpioLed);
}

void Gpio_Tim2_Ch1_init()
{
	// In this case, it is the same as GpioButton
	GpioButton.pGPIOx = GPIOA;
	GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	GpioButton.GPIO_PinConfig.GPIO_PinAltFunMode = 1;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PD;
	GPIO_Init(&GpioButton);
}

int main()
{
	uint32_t count1, count2, duration;

	// Misc peripheral init
	GPIO_LED_and_Button_Init();
	GPIO_WriteToOutputPin(GpioLed.pGPIOx, 13, 0);
	GPIO_WriteToOutputPin(GpioLed.pGPIOx, 14, 1);

	// enable tim2 clock
	init_tim2_pclk();

	// TIM2 is at position 28 in vector table
	// NVIC_IRQ_EnDi(28, ENABLE);

	// PA0 is TIM2_CH1 in AF1
	Gpio_Tim2_Ch1_init();

	tim2_init_input_capture();

	while(1)
	{
		// get the first count
		while(! (*tim2Sr & (1 << 1)));  // wait for TIM2_SR.CC1IF to be set
		count1 = *tim2Ccr1;  // Reading CCR1 also clears TIM2_SR.CC1IF

		// get the second count
		while(! (*tim2Sr & (1 << 1)));  // wait for TIM2_SR.CC1IF to be set
		count2 = *tim2Ccr1;

		*tim2Cr1 &= ~(1 << 0);

		// duration  = (count2 - count1) * timer time period
		duration = (count2 - count1) / getTimerClkFreq();

		tim2_delay_ms(duration * 1000);

		GPIO_ToggleOutputPin(GpioLed.pGPIOx, 13);
		GPIO_ToggleOutputPin(GpioLed.pGPIOx, 14);
	}
}


void TIM2_IRQHandler(void)
{
	// SE needs to clear SR
	*tim2Sr &= ~(1 << 0);

	// disable tim2
	*tim2Cr1 &= ~(1 << 0);

	timEvent = 1;
}
