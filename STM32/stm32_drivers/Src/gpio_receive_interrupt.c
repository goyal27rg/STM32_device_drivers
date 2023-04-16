/*
 * gpio_receive_interrupt.c
 *
 *  Created on: 30-Jul-2022
 *      Author: rakshit
 */


#include "stm32f429xx.h"

int main(void)
{
	GPIO_Handle_t GPIOLed , GPIOInt;

	GPIOLed.pGPIOx = GPIOG;
	GPIOLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIOLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTP;
	GPIOLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PD;

	GPIO_Init(&GPIOLed);

	GPIOInt.pGPIOx = GPIOD;
	GPIOInt.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;
	GPIOInt.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_RT;
	GPIOInt.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GPIOInt);
	GPIO_IRQITConfig(IRQ_NO_EXTI4, ENABLE);

}

void EXTI4_IRQHandler(void)
{
	GPIO_WriteToOutputPin(GPIOG, 14, 1);
	sw_delay_ms(500);
	GPIO_WriteToOutputPin(GPIOG, 14, 0);
	GPIO_IRQHandling(GPIO_PIN_NO_4);
}
