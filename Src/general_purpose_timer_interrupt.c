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

GPIO_Handle_t GpioButton;
GPIO_Handle_t GpioLed;
uint8_t timEvent = 0;

void GPIO_LED_and_Button_Init()
{
	GpioButton.pGPIOx = GPIOA;
	GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INP;
	GpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PD;
	GPIO_Init(&GpioButton);

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


int main()
{
	// Misc peripheral init
	GPIO_LED_and_Button_Init();
	GPIO_WriteToOutputPin(GpioLed.pGPIOx, 13, 0);
	GPIO_WriteToOutputPin(GpioLed.pGPIOx, 14, 1);

	// enable tim2 clock
	RCC->APB1ENR |= (1 << 0);

	// TIM2 is at position 28 in vector table
	NVIC_IRQ_EnDi(28, ENABLE);

	while(1)
	{
		tim2_delay_ms_IT(100);
		while(! timEvent);
		timEvent = 0;
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
