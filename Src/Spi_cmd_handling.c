/*
 * Spi_TX_only_with_arduino_slave
 *
 * Created on: 03-Jul-2022
 * Author: rakshit
 *
 * This module uses STM32 as the master and an external device (arduino) as slave
 * Master is only sending the data. MISO pin is not configured
 *
 */


#include "stm32f429xx.h"
#include <string.h>

#define CMD_LED_CTRL        0x50

#define LED_ON              1
#define LED_OFF             0

#define ACK                 0xF5

void sw_delay(int delay)
{
	int i=0;
	for (; i < delay*1000000; i++);
}

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

	GPIO_Handle_t GpioButton;
	GpioButton.pGPIOx = GPIOA;
	GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INP;
	GpioButton.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GpioButton);


	GPIO_Handle_t GpioLed;
	GpioLed.pGPIOx = GPIOG;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTP;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;

	GPIO_Init(&GpioLed);


	// 1. Init GPIO Pins needed for SPI
	SPI_GPIO_Init();

	// 2. Init SPI
	SPI_Handle_t SPITx;

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

	while(1)
	{
		uint8_t dummy_byte = 0xff;
		uint8_t ackbyte;  // To store ack received form the slave

		GPIO_WriteToOutputPin(GpioLed.pGPIOx, GPIO_PIN_NO_13, 1);  // Turn LED ON
		sw_delay(1);
		while(GPIO_ReadFromInputPin(GpioButton.pGPIOx, GPIO_PIN_NO_0) == 0);
		while(GPIO_ReadFromInputPin(GpioButton.pGPIOx, GPIO_PIN_NO_0));

		GPIO_WriteToOutputPin(GpioLed.pGPIOx, GPIO_PIN_NO_13, 0);  // Turn LED OFF to indicate that communication is going on

		// 1. Send command: CMD_LED_CTRL  <pin_no>  <value>
		uint8_t cmd_code = CMD_LED_CTRL;
		sw_delay(1);
		SPI_SendData(SPITx.pSPIx, &cmd_code, 1);
		// Can do a dummy read here to clear RXNE (not being done currently)
		sw_delay(1);
		SPI_ReceiveData(SPITx.pSPIx, &ackbyte, 1);

		// 2. Send a dummy byte to fetch the acknowledgement from slave (since slave can't initiate transfer)
		sw_delay(1);
		SPI_SendData(SPITx.pSPIx, &dummy_byte, 1);
		sw_delay(1);
		SPI_ReceiveData(SPITx.pSPIx, &ackbyte, 1);

		if (ackbyte == ACK)
		{
			// send pin number and data
			sw_delay(1);
			uint8_t pin = 9;
			SPI_SendData(SPITx.pSPIx, &pin, 1);

			sw_delay(1);
			uint8_t data = 1;
			SPI_SendData(SPITx.pSPIx, &data, 1);
		}
		else
		{}


		while(SPITx.pSPIx->SPI_SR & (1 << SPI_SR_BSY));;  //Wait while SPI is busy
	}

	return 0;
}
