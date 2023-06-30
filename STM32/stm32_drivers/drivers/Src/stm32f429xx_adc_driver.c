/*
 * stm32f429xx_adc_driver.c
 *
 *  Created on: 26-May-2023
 *      Author: rakshit
 */


#include "stm32f429xx.h"
#include "stm32f429xx_adc_driver.h"


/*
 * ADCx Peripheral clock setup
 */
void ADC_PeriClockControl(ADC_RegDef_t* pADCx, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		if (pADCx == ADC1) {
			ADC1_PCLK_EN();
		}
		else if (pADCx == ADC2) {
			ADC2_PCLK_EN();
		}
		else if (pADCx == ADC3) {
			ADC3_PCLK_EN();
		}
	}
	else {
		if (pADCx == ADC1) {
			ADC1_PCLK_DI();
		}
		else if (pADCx == ADC2) {
			ADC2_PCLK_DI();
		}
		else if (pADCx == ADC3) {
			ADC3_PCLK_DI();
		}
	}
}

/*
 * ADCx Init and DeInit
 */
void ADC_Init(ADC_Handle_t *pADCHandle) {

	// 1. Enable peripheral clock
	ADC_PeriClockControl(pADCHandle->pADCx, ENABLE);

	// 2. Program ADC_CR2
	uint32_t temp = 0;
	temp |= (1 << ADC_CR2_ADON);  // Turn ADC ON

	// Program Single OR Continuous mode
	if (pADCHandle->ADC_Config.ADC_ConversionMode == ADC_CONVERSION_MODE_SINGLE) {
		temp &= ~(1 << ADC_CR2_CONT);
	}
	else if (pADCHandle->ADC_Config.ADC_ConversionMode == ADC_CONVERSION_MODE_CONTINUOUS) {
			temp |= (1 << ADC_CR2_CONT);
	}

	// Program Alignment
	if (pADCHandle->ADC_Config.ADC_DataAlignment == ADC_DATA_ALIGNMENT_LEFT_ALIGN) {
		temp |= (1 << ADC_CR2_ALIGN);
	}
	else {
		temp &= ~(1 << ADC_CR2_ALIGN);
	}

	// TODO(Add another field in ADC_Config for EOCS)
	temp |= (1 << ADC_CR2_EOCS); // tmp: enable EOCS
	pADCHandle->pADCx->ADC_CR2 = temp;

	// 3. Program number of regular channels in ADC_SRQ1.L[3:0]
	pADCHandle->pADCx->ADC_SQR1 |= (pADCHandle->ADC_Config.ADC_NumChannels << ADC_SQR1_L);

	temp = 0;
	uint8_t AdcSqrFieldWidth = 5;
	// map Input 0 to conversion number 0 (ADC_SQR3.SQ1 = 0)
	for (int i = 0; i < pADCHandle->ADC_Config.ADC_NumChannels; i++) {
		// TODO (Handle the cases where NumChannels > 6)
		temp |= (pADCHandle->ADC_Config.ADC_ConversionSequence[i] << (i * AdcSqrFieldWidth));
	}
	pADCHandle->pADCx->ADC_SQR3 = temp;

	// 4. Program SCAN mode Enable/Disable
	if (pADCHandle->ADC_Config.ADC_ScanEnOrDi == ENABLE) {
		pADCHandle->pADCx->ADC_CR1 |= (1 << ADC_CR1_SCAN);
	}
	else {
		pADCHandle->pADCx->ADC_CR1 &= ~(1 << ADC_CR1_SCAN);
	}
}

void ADC_StartConversion(ADC_Handle_t *pADCHandle) {
	// Set ADC_CR2.SWSTART
	pADCHandle->pADCx->ADC_CR2 |= (1 << ADC_CR2_SWSTART);
}

void ADC_StartConversionIT(ADC_Handle_t *pADCHandle) {
	// Enable End of conversion interrupt
	pADCHandle->pADCx->ADC_CR1 |= (1 << ADC_CR1_EOCIE) | (1 << ADC_CR1_OVRIE);

	// Set ADC_CR2.SWSTART
	pADCHandle->pADCx->ADC_CR2 |= (1 << ADC_CR2_SWSTART);
}

void ADC_StopConversion(ADC_Handle_t *pADCHandle) {
	// Clear ADC_CR2.SWSTART
	pADCHandle->pADCx->ADC_CR2 &= ~(1 << ADC_CR2_SWSTART);
}

void ADC_StopConversionIT(ADC_Handle_t *pADCHandle) {

	// Disable End of conversion interrupt
	pADCHandle->pADCx->ADC_CR1 &= ~(1 << ADC_CR1_EOCIE) & ~(1 << ADC_CR1_OVRIE);

	// Clear ADC_CR2.SWSTART
	pADCHandle->pADCx->ADC_CR2 &= ~(1 << ADC_CR2_SWSTART);
}

uint32_t ADC_ConvertChannel(ADC_Handle_t *pADCHandle, uint8_t ChannelNum) {
	uint32_t data;

	// save ADC_SQR3
	uint32_t tempAdc_Sqr3 = pADCHandle->pADCx->ADC_SQR3;

	// Program Channel number in ADC_SQR3
	pADCHandle->pADCx->ADC_SQR3 = ChannelNum;

	// Make the conversion
	ADC_StartConversion(pADCHandle);
	while(!(pADCHandle->pADCx->ADC_SR & (1 << ADC_SR_EOC)));
	data = pADCHandle->pADCx->ADC_DR;
	ADC_StopConversion(pADCHandle);


	// reprogram ADC_SQR3 to original value
	pADCHandle->pADCx->ADC_SQR3 = tempAdc_Sqr3;

	return data;
}


void ADC_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	// ADC1, ADC2 and ADC3 IRQs are mapped at position 18 in NVIC

	uint8_t IserRegNo = IRQNumber / 32;
	uint8_t IserRegOffset = IRQNumber % 32;

	if (EnorDi == ENABLE)
	{
		switch(IserRegNo)
		{
	 	case 0: *NVIC_ISER0 |= (1 << IserRegOffset); break;
		}
	}
	else
	{
		switch(IserRegNo)
		{
		case 0: *NVIC_ICER0 |= (1 << IserRegOffset); break;
		}
	}
}
