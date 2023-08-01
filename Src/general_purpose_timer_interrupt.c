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

#define TIM2_BASEADDR    0x40000000
#define TIM2_CR1         (TIM2_BASEADDR)
#define TIM2_DIER        (TIM2_BASEADDR + 0x0c)
#define TIM2_SR          (TIM2_BASEADDR + 0x10)
#define TIM2_EGR         (TIM2_BASEADDR + 0x14)
#define TIM2_CNT         (TIM2_BASEADDR + 0x24)
#define TIM2_PSC         (TIM2_BASEADDR + 0x28)
#define TIM2_ARR         (TIM2_BASEADDR + 0x2c)

volatile uint32_t* tim2Cr1 = (uint32_t*) TIM2_CR1;
volatile uint32_t* tim2Die1 = (uint32_t*) TIM2_DIER;
volatile uint32_t* tim2Sr = (uint32_t*) TIM2_SR;
volatile uint32_t* tim2Egr = (uint32_t*) TIM2_EGR;
volatile uint32_t* tim2Cnt = (uint32_t*) TIM2_CNT;
volatile uint32_t* tim2Psc = (uint32_t*) TIM2_PSC;
volatile uint32_t* tim2Arr = (uint32_t*) TIM2_ARR;

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


void tim2_delay_ms_IT(uint32_t delay_ms)
{
	uint32_t fclk = 16000000;  // HSI, no AHB or APB prescalar
	// TODO(add support for timer prescalar)

	uint32_t reload_val = delay_ms * (fclk / 1000);  // no of cycles to count
	// Enable timer interrupt
	*tim2Die1 |= (1 << 0);  // DIER.UIE

	// program tim2 ARR;
	*tim2Arr = reload_val;

	// enable tim2
	*tim2Cr1 |= (1 << 0);
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
