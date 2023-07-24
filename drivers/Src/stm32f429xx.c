/*
 * stm32f429xx.c
 *
 *  Created on: 23-Jul-2023
 *      Author: rakshit
 */


#include "stm32f429xx.h"

void NVIC_IRQ_EnDi(uint8_t IRQNumber, uint8_t EnOrDi)
{
	uint8_t NVICRegNo = IRQNumber / 32;
	uint8_t NVICRegBitOffset = IRQNumber % 32;

	if (EnOrDi == ENABLE)
	{
		uint32_t* IserReg = (uint32_t *) NVIC_ISER_BASEADDR + NVICRegNo;
		*IserReg |= (1 << NVICRegBitOffset);
	}
	else
	{
		uint32_t* IcerReg = (uint32_t *) NVIC_ICER_BASEADDR + NVICRegNo;
		*IcerReg |= (1 << NVICRegBitOffset);
	}
}
