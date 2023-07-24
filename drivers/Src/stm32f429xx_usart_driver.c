/*
 * stm32f429xx_usart_driver.c
 *
 *  Created on: 10-May-2023
 *      Author: rakshit
 */

#include "stm32f429xx_usart_driver.h"

static uint32_t nearestInteger(float num)
{
	float fraction = num - (int)num;
	if (fraction >= 0.5)
		return(uint32_t)num +1;
	else
		return(uint32_t)num;
}


static void USART_InitBRR(USART_Handle_t *pUSARTHandle)
{
	/*
	 * USARTDIV = fclk / (8 * (2 - OVER8) * BaudRate)
	 */

	uint32_t baud = pUSARTHandle->USART_Config.USART_Baud;
	uint8_t over8 = (pUSARTHandle->pUSARTx->USART_CR1 >> USART_CR1_OVER8) & 1;
	uint32_t fclk = 16000000;
	float USARTDIV = (float)fclk / (8 * (2 - over8) * baud);
	uint8_t divFractionBitSize = (over8 == 0) ? 4 : 3;
	uint8_t base = (1 << divFractionBitSize);

	uint32_t DIV_Fraction = nearestInteger((USARTDIV - (int)USARTDIV) * base);

	// Mantissa is the integer part of USARTDIV + the overflow of DIV_Fraction
	uint32_t DIV_Mantissa = (int)USARTDIV + (DIV_Fraction / base);

	// Fraction needs to be in the bit width limit
	DIV_Fraction = DIV_Fraction % base;

	uint32_t temp = 0;
	temp |= DIV_Fraction << USART_BRR_DIV_Fraction;
	temp |= DIV_Mantissa << USART_BRR_DIV_Mantissa;

	pUSARTHandle->pUSARTx->USART_BRR = temp;
}

/*
 * USARTx Peripheral clock setup
 */
void USART_PeriClockControl(USART_RegDef_t* pUSARTx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (pUSARTx == USART1)
			USART1_PCLK_EN();
		else if (pUSARTx == USART2)
			USART2_PCLK_EN();
		else if (pUSARTx == USART3)
			USART3_PCLK_EN();
		else if (pUSARTx == UART4)
			UART4_PCLK_EN();
		else if (pUSARTx == UART5)
			UART5_PCLK_EN();
		else if (pUSARTx == USART6)
			USART6_PCLK_EN();
	}
	else
	{
		if (pUSARTx == USART1)
			USART1_PCLK_DI();
		else if (pUSARTx == USART2)
			USART2_PCLK_DI();
		else if (pUSARTx == USART3)
			USART3_PCLK_DI();
		else if (pUSARTx == UART4)
			UART4_PCLK_DI();
		else if (pUSARTx == UART5)
			UART5_PCLK_DI();
		else if (pUSARTx == USART6)
			USART6_PCLK_DI();
	}
}


/*
 * USARTx Init and DeInit
 */
void USART_Init(USART_Handle_t *pUSARTHandle)
{
	// Enable peripheral clock
	USART_PeriClockControl(pUSARTHandle->pUSARTx, ENABLE);

	// Program Baud Rate
	USART_InitBRR(pUSARTHandle);

	// Program Rx / Tx mode
	if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX)
	{
		pUSARTHandle->pUSARTx->USART_CR1 |= (1 << USART_CR1_TE);
		pUSARTHandle->pUSARTx->USART_CR1 &= ~(1 << USART_CR1_RE);
	}
	else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX)
	{
		pUSARTHandle->pUSARTx->USART_CR1 &= ~(1 << USART_CR1_TE);
		pUSARTHandle->pUSARTx->USART_CR1 |= (1 << USART_CR1_RE);
	}
	else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX)
	{
		pUSARTHandle->pUSARTx->USART_CR1 |= (1 << USART_CR1_TE);
		pUSARTHandle->pUSARTx->USART_CR1 |= (1 << USART_CR1_RE);
	}

	// Program number of stop bits
	// First make USART_CR2.STOP field 0
	pUSARTHandle->pUSARTx->USART_CR2 &= ~(0b11 << USART_CR2_STOP);
	pUSARTHandle->pUSARTx->USART_CR2 |= (pUSARTHandle->USART_Config.USART_NoOfStopBits << USART_CR2_STOP);

	// Program Parity control bits
	if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
	{
		// Disable Parity control
		pUSARTHandle->pUSARTx->USART_CR1 &= ~(1 << USART_CR1_PCE);
	}
	else
	{
		// Enable Parity control
		pUSARTHandle->pUSARTx->USART_CR1 |= (1 << USART_CR1_PCE);

		// Now, program Parity selection
		if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN)
		{
			// EVEN Parity. i.e USART_CR1.PS = 0
			pUSARTHandle->pUSARTx->USART_CR1 &= ~(1 << USART_CR1_PS);
		}
		else
		{
			// only remaining option is ODD Parity. i.e USART_CR1.PS = 1
			pUSARTHandle->pUSARTx->USART_CR1 |= (1 << USART_CR1_PS);
		}
	}

	// Program Word Length (8 or 9 data bits)
	switch (pUSARTHandle->USART_Config.USART_WordLength)
	{
	case USART_WORDLEN_8BITS: pUSARTHandle->pUSARTx->USART_CR1 &= ~(1 << USART_CR1_M);
							  break;
	case USART_WORDLEN_9BITS: pUSARTHandle->pUSARTx->USART_CR1 |= (1 << USART_CR1_M);
							  break;
	}

	// Enable CTS / RTS
	if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS || pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
	{
		// Enable CTS
		pUSARTHandle->pUSARTx->USART_CR3 |= (1 << USART_CR3_CTSE);
	}
	if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS || pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
	{
		// Enable RTS
		pUSARTHandle->pUSARTx->USART_CR3 |= (1 << USART_CR3_RTSE);
	}
}


// TODO(implement USART_DeInit
void USART_DeInit(USART_Handle_t *pUSARTHandle);


/*
 * USARTx Send and Receive
 */

void USART_MasterSendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	// Enable USART
	pUSARTHandle->pUSARTx->USART_CR1 |= (1 << USART_CR1_UE);

	while (Len > 0)
	{
		/*
		 * USART_CR1.{M, PCE} bits decide the actual number of data bits in a UART frame as follows:
		 * ----------------------------------
		 * | M PCE | Data size in UART frame|
		 * |-------|------------------------|
		 * | 0  0  | 8 bit data             |
		 * | 0  1  | 7-bit data             |
		 * | 1  0  | 9-bit data             |
		 * | 1  1  | 8-bit data             |
		 * ----------------------------------
		 */
		// uint32_t tempUSARTCR1 = pUSARTHandle->pUSARTx->USART_CR1;
		uint8_t tempUSARTCR1_M = (pUSARTHandle->pUSARTx->USART_CR1 & (1 << USART_CR1_M)) >> USART_CR1_M;
		uint8_t tempUSARTCR1_PCE = (pUSARTHandle->pUSARTx->USART_CR1 & (1 << USART_CR1_PCE)) >> USART_CR1_PCE;
		while (! USART_GetFlagStatus(pUSARTHandle->pUSARTx, (1 << USART_SR_TXE)));

		if ((0 == tempUSARTCR1_M && 0 == tempUSARTCR1_PCE)
				|| (1 == tempUSARTCR1_M && 1 == tempUSARTCR1_PCE))
		{
			//(M==0 && PCE=0) || (M==1 && PCE==1)
			// => 8-bit data
			pUSARTHandle->pUSARTx->USART_DR = *pTxBuffer;
			pTxBuffer++;
			Len--;
		}
		else if (tempUSARTCR1_M)
		{
			// 9-bit data
			uint16_t *pdata = (uint16_t*) pTxBuffer;
			pUSARTHandle->pUSARTx->USART_DR = *pdata & ((1 << 9) - 1);
			pTxBuffer++;
			pTxBuffer++;
			Len--;
			Len--;
		}
		else
		{
			// 7-bit data
			pUSARTHandle->pUSARTx->USART_DR = *pTxBuffer & ((1 << 7) - 1);
			pTxBuffer++;
			Len--;
		}

	}

	// wait for transmission to complete
	while (! USART_GetFlagStatus(pUSARTHandle->pUSARTx, (1 << USART_SR_TC)));

	// Disable USART
	pUSARTHandle->pUSARTx->USART_CR1 &= ~(1 << USART_CR1_UE);
}


void USART_MasterReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	// Enable USART
	pUSARTHandle->pUSARTx->USART_CR1 |= (1 << USART_CR1_UE);

	while (Len > 0)
	{
		while (! USART_GetFlagStatus(pUSARTHandle->pUSARTx, (1 << USART_SR_RXNE)));
		*pRxBuffer = pUSARTHandle->pUSARTx->USART_DR;
		pRxBuffer++;
		Len--;
	}

	// Disable USART
	pUSARTHandle->pUSARTx->USART_CR1 &= ~(1 << USART_CR1_UE);
}


//Misc functions
uint8_t USART_GetFlagStatus(USART_RegDef_t* pUSARTx, uint8_t flag)
{
	return pUSARTx->USART_SR & flag;
}
