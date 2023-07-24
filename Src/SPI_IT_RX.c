/*
 * SPI_IT_RX.c
 *
 *  Created on: 28-Aug-2022
 *      Author: rakshit
 */


#include "stm32f429xx.h"
#include <string.h>

#define ACK                 0xF5


volatile uint8_t spi_tx_cmplt = 0, spi_rx_cmplt = 0;
volatile uint8_t dataAvailable = 0;

SPI_Handle_t SPITx;
uint8_t RxData;
uint8_t dummy;

GPIO_Handle_t GpioLed;

void SPI_GPIO_Init(void)
{
	// Tx
	// Use PORTA SPI Pins in AF5
	// PA4 -> SP1_NSS
	// PA5 -> SPI1_SCK
	// PA6 -> SPI1_MISO
	// PA7 -> SPI1_MOSI

	GPIO_Handle_t SPI_GPIO;
	SPI_GPIO.pGPIOx = GPIOA;
	SPI_GPIO.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPI_GPIO.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPI_GPIO.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPI_GPIO.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPI_GPIO.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;

	// Now configure for each pin

	// NSS
	SPI_GPIO.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;
	GPIO_Init(&SPI_GPIO);

	// SCK
	SPI_GPIO.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GPIO_Init(&SPI_GPIO);

	// MISO
	SPI_GPIO.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&SPI_GPIO);

	// MOSI
	SPI_GPIO.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&SPI_GPIO);
}


int main()
{
	GPIO_Handle_t GpioInt;
	GpioInt.pGPIOx = GPIOD;
	GpioInt.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;
	GpioInt.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_RT;
	GpioInt.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_Init(&GpioInt);

	GPIO_Handle_t GpioButton;
	GpioButton.pGPIOx = GPIOA;
	GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INP;
	GpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PD;
	GPIO_Init(&GpioButton);

	GpioLed.pGPIOx = GPIOG;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;  // Green LED
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTP;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;
	GPIO_Init(&GpioLed);

	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;  // Red LED
	GPIO_Init(&GpioLed);


	// 1. Init GPIO Pins needed for SPI
	SPI_GPIO_Init();

	// 2. Init SPI

	SPITx.pSPIx = SPI1 ;
	SPITx.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPITx.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPITx.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
	SPITx.SPI_Config.SPI_DFF = SPIDFF_8BITS;
	SPITx.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPITx.SPI_Config.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV32;
	SPITx.SPI_Config.SPI_SSM = SPI_SSM_DI;

	SPI_Init(&SPITx);

	SPI_SSOE_Config(SPITx.pSPIx, ENABLE);

	SPI_IRQITConfig(IRQ_NO_SPI1, ENABLE);

	GPIO_IRQITConfig(IRQ_NO_EXTI4, ENABLE);

	while(1)
	{
		char end_byte = '\0';

		while(!dataAvailable);

		while(SPI_ReceiveDataIT(&SPITx, (uint8_t *) &RxData, 1) == SPI_BUSY_IN_RX);
		SPI_SendData(SPITx.pSPIx, (uint8_t *) &end_byte, 1);
		dataAvailable = 0;

		GPIO_IRQITConfig(IRQ_NO_EXTI4, ENABLE);


	}
	return 0;
}

void SPI1_IRQHandler()
{
	SPI_IRQHandling(&SPITx);
	sw_delay_ms(100);
	if (RxData == 0xf5)
	{
		GPIO_WriteToOutputPin(GpioLed.pGPIOx, GPIO_PIN_NO_13, 1);  // Turn LED ON
		sw_delay_ms(500);
		GPIO_WriteToOutputPin(GpioLed.pGPIOx, GPIO_PIN_NO_13, 0);  // Turn LED ON
		sw_delay_ms(500);
	}
}

void EXTI4_IRQHandler(void)
{
	GPIO_IRQITConfig(IRQ_NO_EXTI4, DISABLE);

	GPIO_WriteToOutputPin(GPIOG, 14, 1);
	dataAvailable = 1;
	sw_delay_ms(500);
	GPIO_WriteToOutputPin(GPIOG, 14, 0);
	sw_delay_ms(500);

	//GPIO_IRQITConfig(IRQ_NO_EXTI4, ENABLE);
}


void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t event)
{
	if(event == SPI_EVENT_TX_CMPLT)
		spi_tx_cmplt = 1;
	if(event == SPI_EVENT_RX_CMPLT)
		spi_rx_cmplt = 1;
}



