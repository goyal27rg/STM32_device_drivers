/*
 * 005ButtonInterrupt.c
 *
 *  Created on: 26-Jun-2022
 *      Author: rakshit
 */


#include <stdint.h>
#include "stm32f429xx.h"

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

void sw_delay(void)
{
	int i=0;
	for (i=0; i<700000; i++)
	{

	}
}

int main(void)
{
	// Configure the pin connected to LED: PG14
	GPIO_Handle_t GpioLed;
	GpioLed.pGPIOx = GPIOG;  // Select Port: G
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;  // Select Pin: 14
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTP;  // Select Pin mode: Output
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;  // Select Pin Output Type: Push-pull
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;  // Select Pin Speed: Low
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;  // Select Pull-up / Pull-down: No PU-PU

	GPIO_PeriClockControl(GpioLed.pGPIOx, ENABLE);  // Enable clock for GPIO peripheral
	GPIO_Init(&GpioLed);  // Initialise the GPIO peripheral

	// Configure button connected to Pin: PA0 in interrupt mode
	GPIO_Handle_t GpioButton;
	GpioButton.pGPIOx = GPIOA;
	GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_RT;
	GpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl =  GPIO_NO_PUPD;

	GPIO_PeriClockControl(GpioButton.pGPIOx, ENABLE);
	GPIO_Init(&GpioButton);


	// IRQ configurations for EXTI0
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI0, NVIC_IRQ_PRIO6);
	GPIO_IRQITConfig(IRQ_NO_EXTI0, ENABLE);

	GPIO_WriteToOutputPin(GPIOG, 14, 1);


	while(1)
	{

	}
}

void EXTI0_IRQHandler(void)
{
	// call the required Handler
	GPIO_IRQHandling(0);
	GPIO_ToggleOutputPin(GPIOG, GPIO_PIN_NO_14);
}

