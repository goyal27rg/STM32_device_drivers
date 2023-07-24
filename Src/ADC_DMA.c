/*
 * ADC_DMA.c
 *
 * Write ADC contents to memory using DMA
 *
 *  Created on: Jul 17, 2023
 *      Author: rakshit
 */


#include "stm32f429xx.h"
#include <string.h>
#include <stdio.h>
#include <inttypes.h>

USART_Handle_t USARTTx;

GPIO_Handle_t GpioButton;
GPIO_Handle_t GpioLed;

ADC_Handle_t ADC;
DMA_Handle_t DMA;

uint8_t Tx_buff[100];
uint32_t ADC_DMA_buff[10];
uint8_t ADC_NumChannels = 2;
volatile uint8_t adc_it = 0, dma_it = 0;


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
	USART_GPIO_Init();

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
	ADC_GPIO_Init();

	ADC.pADCx = ADC3;
	ADC.ADC_Config.ADC_ConversionMode = ADC_CONVERSION_MODE_CONTINUOUS;
	ADC.ADC_Config.ADC_ScanEnOrDi = ENABLE;
	ADC.ADC_Config.ADC_ConfigEOC = ADC_EOC_SET_AT_END_OF_SEQUENCE_CONVERSION;
	ADC.ADC_Config.ADC_DMAModeEnOrDi = DISABLE;
	ADC.ADC_Config.ADC_DataAlignment = ADC_DATA_ALIGNMENT_RIGHT_ALIGN;
	ADC.ADC_Config.ADC_NumChannels = ADC_NumChannels;
	uint8_t ConversionSequence[] = {14, 15};
	ADC.ADC_Config.ADC_ConversionSequence = ConversionSequence;
	ADC_Init(&ADC);
}

void DMA_Inits()
{
	DMA.pDMAx = DMA2;
	// ADC3 is connected to Channel 2 of both Stream 0 and 1
	DMA.DMA_Config.DMA_StreamNum = 0;
	DMA.DMA_Config.DMA_ChannelNum = 2;
	DMA.DMA_Config.DMA_FlowController = DMA_FLOW_CONTROLLER_DMA;
	DMA.DMA_Config.DMA_TransferDirection = DMA_TRANSFER_DIR_PERIPH_TO_MEM;
	DMA.DMA_Config.DMA_CircularModeEnorDi = ENABLE;  // read again and again from peripheral
	DMA.DMA_Config.DMA_MemIncrementEnorDi = ENABLE;
	DMA.DMA_Config.DMA_Mem0Addr = (uint32_t) ADC_DMA_buff;
	DMA.DMA_Config.DMA_MemDataWidth = 0b10;  // taking ADC_DR reg size as 4;
	DMA.DMA_Config.DMA_MemIncrementEnorDi = ENABLE;
	DMA.DMA_Config.DMA_NumDataItems = ADC_NumChannels;
	DMA.DMA_Config.DMA_PeriphAddr = (uint32_t)ADC.pADCx + offsetof(ADC_RegDef_t, ADC_DR);
	DMA.DMA_Config.DMA_PeriphDataWidth = 0b10;

	DMA_Init(&DMA);
}

int main()
{
	uint32_t Vx=2702, Vy=1997;
	uint32_t counter = 0;
	char str[50];  // = "Hello World\r\n";

	// Misc peripheral init
	GPIO_LED_and_Button_Init();
	USART_Inits();
	USART_PeriClockControl(USARTTx.pUSARTx, ENABLE);


	ADC_Inits();

	DMA_Inits();
	//DMA_IRQEnDi(&DMA, 56, ENABLE);  // Enable DMA2 stream 0 interrupts in NVIC
	DMA_Enable(&DMA);

	ADC_IRQITConfig(18, ENABLE);
	ADC_StartConversionIT(&ADC);

	while(1)
	{
		//GPIO_WriteToOutputPin(GpioLed.pGPIOx, GPIO_PIN_NO_13, 1);
		//sw_delay_ms(500);

		//if (adc_it && dma_it)
		{
			adc_it = dma_it = 0;
			Vx = ADC_DMA_buff[0];
			Vy = ADC_DMA_buff[1];
			//DMA_IRQEnDi(&DMA, 56, ENABLE);
			//DMA_Enable(&DMA);
			//ADC_StartConversionIT(&ADC);
		}

		if (counter % 100000 == 0)  // print every 100th sample
		{
			sprintf(str, "Vx: %"PRIu32", Vy: %"PRIu32"\n", Vx, Vy);
			USART_MasterSendData(&USARTTx, (uint8_t*)str, strlen((char*)str));
		}

		//GPIO_WriteToOutputPin(GpioLed.pGPIOx, GPIO_PIN_NO_13, 0);
		counter++;
	}
}


void DMA2_Stream0_IRQHandler(void)
{
	DMA_IRQEnDi(&DMA, 56, DISABLE);
	DMA_Disable(&DMA);

	dma_it = 1;

	GPIO_WriteToOutputPin(GpioLed.pGPIOx, GPIO_PIN_NO_13, 1);
	sw_delay_ms(200);
	GPIO_WriteToOutputPin(GpioLed.pGPIOx, GPIO_PIN_NO_13, 0);
}

void ADC_IRQHandler(void)
{
	//DMA_Disable(&DMA);
	//ADC_StopConversionIT(&ADC);

	adc_it = 1;

	uint32_t adc_sr = ADC.pADCx->ADC_SR;
	if (adc_sr & (1 << ADC_SR_EOC))
	{
		//char str[] = "EOC interrupt\r\n";
		//USART_MasterSendData(&USARTTx, (uint8_t*)str, strlen((char*)str));
		GPIO_WriteToOutputPin(GpioLed.pGPIOx, GPIO_PIN_NO_13, 1 ^ (GPIO_ReadFromInputPin(GpioLed.pGPIOx, GPIO_PIN_NO_13)));

	}
	if (adc_sr & (1 << ADC_SR_OVR))
	{
		//char str[] = "OVR interrupt\r\n";
		//USART_MasterSendData(&USARTTx, (uint8_t*)str, strlen((char*)str))
		GPIO_WriteToOutputPin(GpioLed.pGPIOx, GPIO_PIN_NO_14, 1 ^ (GPIO_ReadFromInputPin(GpioLed.pGPIOx, GPIO_PIN_NO_14)));

	}



	//ADC_StartConversionIT(&ADC);
}
