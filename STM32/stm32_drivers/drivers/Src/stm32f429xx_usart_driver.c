/*
 * stm32f429xx_usart_driver.c
 *
 *  Created on: 10-May-2023
 *      Author: rakshit
 */

#include "stm32f429xx_usart_driver.h"

static void USART_InitBRR(USART_Handle_t *pUSARTHandle)
{
	/*
	 * USARTDIV = fclk / (8 * (2 - OVER8) * BaudRate)
	 */
	/*
	uint32_t baud = pUSARTHandle->USART_Config->USART_Baud;
	uint8_t over8 = (pUSARTHandle->pUSARTx->USART_CR1 >> USART_CR1_OVER8) & 1;
	uint32_t fclk = 16000000;
	float USARTDIV = (float)fclk / (8 * (2 - over8) * baud);
	uint8_t carry = 0;
	if (over8 == 0)
	{
		// 4-bit DIV_Fraction

	}
	*/
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
}


void USART_DeInit(USART_Handle_t *pUSARTHandle);
