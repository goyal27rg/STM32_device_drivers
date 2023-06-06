/*
 * I2C_MasterSendData.c
 */

#include "stm32f429xx.h"
#include <string.h>
#include <stdio.h>

ADC_Handle_t ADC;
USART_Handle_t USARTTx;

GPIO_Handle_t GpioButton;
GPIO_Handle_t GpioLed;

uint8_t Tx_buff[100];
uint8_t ADC_NumChannels = 2;


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

void USART_GPIO_Config(GPIO_Handle_t* GPIOx_Handle)
{
	GPIOx_Handle->GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	GPIOx_Handle->GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIOx_Handle->GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;
	GPIOx_Handle->GPIO_PinConfig.GPIO_PinAltFunMode = 7;
	GPIOx_Handle->GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
}

void USART_GPIO_Init()
{
	GPIO_Handle_t GPIOTx;

	GPIOTx.pGPIOx = GPIOB;

	USART_GPIO_Config(&GPIOTx);

	GPIOTx.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;  // PA9: USART1_Tx
	GPIO_Init(&GPIOTx);

	GPIOTx.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;  // PA10: USART1_Rx
	GPIO_Init(&GPIOTx);
}

void USART_Inits()
{
	USARTTx.pUSARTx = USART1;
	USARTTx.USART_Config.USART_Baud = USART_STD_BAUD_115200;
	USARTTx.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	USARTTx.USART_Config.USART_Mode = USART_MODE_TXRX;
	USARTTx.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
	USARTTx.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
	USARTTx.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
	USART_Init(&USARTTx);
}

void ADC_GPIO_Init()
{
	GPIO_Handle_t GPIOAdc;
	GPIOAdc.pGPIOx = GPIOF;
	GPIOAdc.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ANALOG;

	// Using PF4 (mapped to ADC3_IN14) as Vy for Joystick
	GPIOAdc.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;
	GPIO_Init(&GPIOAdc);

	// Using PF5 (mapped to ADC3_IN15) as Vx for Joystick
	GPIOAdc.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GPIO_Init(&GPIOAdc);
}

void ADC_Inits()
{
	ADC.pADCx = ADC3;
	ADC.ADC_Config.ADC_ConversionMode = ADC_CONVERSION_MODE_SINGLE;
	ADC.ADC_Config.ADC_ScanEnOrDi = ENABLE;
	ADC.ADC_Config.ADC_DataAlignment = ADC_DATA_ALIGNMENT_RIGHT_ALIGN;
	ADC.ADC_Config.ADC_NumChannels = ADC_NumChannels;
	uint8_t ConversionSequence[] = {14, 15};
	ADC.ADC_Config.ADC_ConversionSequence = ConversionSequence;
	ADC_Init(&ADC);
}

int main()
{
	USART_GPIO_Init();
	USART_Inits();
	USART_PeriClockControl(USARTTx.pUSARTx, ENABLE);

	ADC_GPIO_Init();
	ADC_Inits();

	GPIO_LED_and_Button_Init();
	GPIO_WriteToOutputPin(GpioLed.pGPIOx, GPIO_PIN_NO_13, 0);

	uint32_t Vx, Vy;
	char str[10];

	while(1)
	{
		//while(GPIO_ReadFromInputPin(GpioButton.pGPIOx, GPIO_PIN_NO_0) == 0);
		//while(GPIO_ReadFromInputPin(GpioButton.pGPIOx, GPIO_PIN_NO_0));
		GPIO_WriteToOutputPin(GpioLed.pGPIOx, GPIO_PIN_NO_13, 1);
		sw_delay_ms(500);

		Vx = ADC_ConvertChannel(&ADC, 15);
		Vy = ADC_ConvertChannel(&ADC, 14);

		sprintf(str, "Vx: %d, Vy: %d\n", (int)Vx, (int)Vy);
		USART_MasterSendData(&USARTTx, (uint8_t*)str, strlen((char*)str));

		GPIO_WriteToOutputPin(GpioLed.pGPIOx, GPIO_PIN_NO_13, 0);
	}
}
