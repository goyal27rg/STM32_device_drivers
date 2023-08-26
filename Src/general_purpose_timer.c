/*
 * general_purpose_timer.c
 *
 *  Created on: Jul 31, 2023
 *      Author: rakshit
 */

#include "stm32f429xx.h"
#include <string.h>
#include <stdio.h>
#include <inttypes.h>

#define TIM2_BASEADDR    0x40000000
#define TIM2_CR1         (TIM2_BASEADDR)
#define TIM2_DIER        (TIM2_BASEADDR + 0x0c)
#define TIM2_SR          (TIM2_BASEADDR + 0x10)
#define TIM2_CNT         (TIM2_BASEADDR + 0x24)
#define TIM2_PSC         (TIM2_BASEADDR + 0x28)
#define TIM2_ARR         (TIM2_BASEADDR + 0x2c)

volatile uint32_t* tim2Cr1 = (uint32_t*) TIM2_CR1;
volatile uint32_t* tim2Die1 = (uint32_t*) TIM2_DIER;
volatile uint32_t* tim2Sr = (uint32_t*) TIM2_SR;
volatile uint32_t* tim2Cnt = (uint32_t*) TIM2_CNT;
volatile uint32_t* tim2Psc = (uint32_t*) TIM2_PSC;
volatile uint32_t* tim2Arr = (uint32_t*) TIM2_ARR;

GPIO_Handle_t GpioButton;
GPIO_Handle_t GpioLed;


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



	// prgroam tim2 ARR;
	*tim2Arr = 0x100000;

	// enable tim2
	*tim2Cr1 |= (1 << 0);

	while(1)
	{
		while(! (*tim2Sr & (1 << 0)));
		*tim2Sr &= ~(1 << 0);

		GPIO_ToggleOutputPin(GpioLed.pGPIOx, 13);
	}
}
