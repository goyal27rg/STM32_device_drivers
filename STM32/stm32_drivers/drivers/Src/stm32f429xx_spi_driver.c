/*
 * stm32f429xx_spi_driver.c
 *
 *  Created on: 02-Jul-2022
 *      Author: rakshit
 */


#include "stm32f429xx_spi_driver.h"


// Helper functions
// "static" keyword makes the scope limited to this file
// Application should not be able to access these
static void SPI_txe_interrupt_handler(SPI_Handle_t *pSPIHandle);
static void SPI_rxne_interrupt_handler(SPI_Handle_t *pSPIHandle);
static void SPI_ovr_interrupt_handler(SPI_Handle_t *pSPIHandle);


void sw_delay_ms(int delay)
{
	int i=0;
	for (; i < delay*1000; i++);
}


/*********************************************************************
 * @fn      		  - SPI_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given SPI peripheral
 *
 * @param[in]         - base address of the SPI peripheral
 * @param[in]         - ENABLE or DISABLE macros
 *
 * @return            -  none
 *
 * @Note              -  none
 */

void SPI_PeriClockControl(SPI_RegDef_t* pSPIx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}
		else if (pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}
		else if (pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}
		else if (pSPIx == SPI4)
		{
			SPI4_PCLK_EN();
		}
		else if (pSPIx == SPI5)
		{
			SPI5_PCLK_EN();
		}
		else if (pSPIx == SPI6)
		{
			SPI6_PCLK_EN();
		}
	}
	else
	{
		if (pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}
		else if (pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}
		else if (pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}
		else if (pSPIx == SPI4)
		{
			SPI4_PCLK_DI();
		}
		else if (pSPIx == SPI5)
		{
			SPI5_PCLK_DI();
		}
		else if (pSPIx == SPI6)
		{
			SPI6_PCLK_DI();
		}
	}
}


/*
 * SPIx Init and DeInit
 */
void SPI_Init(SPI_Handle_t* pSPIHandle)
{
	// Enable SPI Peripheral Clock
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	uint32_t temp_spi_cr1 = 0;

	temp_spi_cr1 |= pSPIHandle->SPI_Config.SPI_CPHA << SPI_CR1_CPHA;
	temp_spi_cr1 |= pSPIHandle->SPI_Config.SPI_CPOL << SPI_CR1_CPOL;
	temp_spi_cr1 |= pSPIHandle->SPI_Config.SPI_DeviceMode << SPI_CR1_MSTR;
	temp_spi_cr1 |= pSPIHandle->SPI_Config.SPI_SclkSpeed << SPI_CR1_BR;
	temp_spi_cr1 |= pSPIHandle->SPI_Config.SPI_SSM << SPI_CR1_SSM;
	temp_spi_cr1 |= pSPIHandle->SPI_Config.SPI_DFF << SPI_CR1_DFF;

	if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		temp_spi_cr1 &= ~(1 << SPI_CR1_BIDIMODE);  // Clear SPI_CR1.BIDIMODE bit
	}
	else if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		temp_spi_cr1 |= (1 << SPI_CR1_BIDIMODE);  // Set SPI_CR1.BIDI pin
	}
	else if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		temp_spi_cr1 &= ~(1 << SPI_CR1_BIDIMODE);  // Clear SPI_CR1.BIDI bit
		temp_spi_cr1 |= (1 << SPI_CR1_RXONLY);  // Set SPI_CR1.RX_ONLY bit
	}

	// Program the register
	pSPIHandle->pSPIx->SPI_CR1 = temp_spi_cr1;
	pSPIHandle->pSPIx->SPI_CR1 |= (1 << 6);  // Enable SPI
}


void SPI_DeInit(SPI_Handle_t* pSPIHandle);


/*
 * SPIx Send and Receive
 */

void SPI_SendData(SPI_RegDef_t* pSPIx, uint8_t* pTxBuffer, uint32_t Len)
{
	// Blocking API: will wait until all the bytes are transmitted

	// SPI_SR.TXE is set when the Tx buffer is empty
	// 1. Wait until buffer is empty
	while(! (pSPIx->SPI_SR & (1 << SPI_SR_TXE)));

	while (Len > 0)
	{
		// 2. Check DFF and Load the data
		if (pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF))
		{
			// 16 bit DFF
			pSPIx->SPI_DR = *((uint16_t*)pTxBuffer);
			Len = Len - 2;
			(uint16_t*)pTxBuffer++;
		}
		else
		{
			// 8 bit DFF
			pSPIx->SPI_DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}

		// 3. Wait for Tx to complete
		while(! (pSPIx->SPI_SR & (1 << SPI_SR_TXE)));
	}
}

void SPI_ReceiveData (SPI_RegDef_t* pSPIx, uint8_t* pRxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		// 1. Wait for Rx Buffer not empty
		while(!(pSPIx->SPI_SR & (1 << SPI_SR_RXNE)));

		// 2. Check DFF and Load the data from Rx Buffer
		if (pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF))
		{
			// 16-bit DFF
			*((uint16_t*)pRxBuffer) = pSPIx->SPI_DR;
			Len -= 2;
			(uint16_t*)pRxBuffer++;
		}
		else
		{
			// 8-bit DFF
			*pRxBuffer = pSPIx->SPI_DR;
			Len--;
			pRxBuffer++;
		}
	}
}


/*
 * Interriupt based SPIx Send and Receive
 */

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t* pTxBuffer, uint32_t Len)
{
	// Non-Blocking API
	uint8_t state = pSPIHandle->TxState;
	if (state != SPI_BUSY_IN_TX)
	{
		// 1. Send Tx Buffer and Len information in global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;

		// 2. Mark the state of current SPI peripheral as SPI_BUSY_IN_TX to prevent
		//    other application from taking over
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		// 3. Enable TXEIE to get an interrupt when Tx completes (TXE flag is set in SPI SR)
		pSPIHandle->pSPIx->SPI_CR2 |= (1 << SPI_CR2_TXEIE);

		// 4. Handle the data Tx in ISR
	}
	return state;
}

uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t* pRxBuffer, uint32_t Len)
{
	// Non-Blocking API
	uint8_t state = pSPIHandle->RxState;
	if (state != SPI_BUSY_IN_RX)
	{
		// 1. Send Rx Buffer and Len information in global variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;

		// 2. Mark the state of current SPI peripheral as SPI_BUSY_IN_RX to prevent
		//    other application from taking over
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		// 3. Enable RXNEIE to get an interrupt when Rx completes (RXNE flag is set in SPI SR)
		pSPIHandle->pSPIx->SPI_CR2 |= (1 << SPI_CR2_RXNEIE);

		// 4. Handle the data Rx in ISR
	}
	return state;
}



/*
 * IRQ Handling
 */

void SPI_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		__vo uint32_t* pNvicIserReg = NVIC_ISER0 + (IRQNumber/32);
		*(pNvicIserReg) |= (1 << IRQNumber % 32);
	}
	else
	{
		__vo uint32_t* pNvicIcerReg = NVIC_ICER0 + (IRQNumber/32);
		*(pNvicIcerReg) &= ~(1 << IRQNumber % 32);
	}
}


void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	__vo uint32_t* pNvicIprReg = NVIC_IPR_BASEADDR + (IRQPriority / 4);

	//clear the field
	*(pNvicIprReg) &= ~(0xf << IRQNumber % 4);
	*(pNvicIprReg) |= (IRQPriority << IRQNumber % 4);
}


void SPI_IRQHandling(SPI_Handle_t* pSPIHandle)
{
	// 1. Detect the cause of Interrupt: TXE, RXNE or Error

	// Check for TXE Flag set
	if(pSPIHandle->pSPIx->SPI_SR & (1 << SPI_SR_TXE) && pSPIHandle->pSPIx->SPI_CR2 & (1 << SPI_CR2_TXEIE))
	{
		SPI_txe_interrupt_handler(pSPIHandle);
	}

	// Check for RXNE Flag set
	if (pSPIHandle->pSPIx->SPI_SR & (1 << SPI_SR_RXNE) && pSPIHandle->pSPIx->SPI_CR2 & (1 << SPI_CR2_RXNEIE))
	{
		SPI_rxne_interrupt_handler(pSPIHandle);
	}

	// Check for overrun, OVR Flag
	if (pSPIHandle->pSPIx->SPI_SR & (1 << SPI_SR_OVR) && pSPIHandle->pSPIx->SPI_CR2 & (1 << SPI_CR2_ERRIE))
	{
		SPI_ovr_interrupt_handler(pSPIHandle);
	}
}


void SPI_txe_interrupt_handler(SPI_Handle_t *pSPIHandle)
{
	if (pSPIHandle->pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF))  // 16-bit DFF
	{
		pSPIHandle->pSPIx->SPI_DR = *((uint16_t*) pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen -= 2;
		(uint16_t*) pSPIHandle->pTxBuffer++;
	}
	else  // 8-bit DFF
	{
		pSPIHandle->pSPIx->SPI_DR = *(pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}
	if (!pSPIHandle->TxLen)
	{
		// Len=0, i.e. SPI Tx completed
		// Reset the SPI_CR2.TXEIE
		pSPIHandle->pSPIx->SPI_CR2 &= ~(1 << SPI_CR2_TXEIE);

		pSPIHandle->TxLen = 0;
		pSPIHandle->pTxBuffer = NULL;
		pSPIHandle->TxState = SPI_READY;

		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}
}


void SPI_rxne_interrupt_handler(SPI_Handle_t *pSPIHandle)
{

	if (pSPIHandle->pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF))  // 16-bit DFF
	{
		*((uint16_t*) pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->SPI_DR;
		pSPIHandle->RxLen -= 2;
		(uint16_t*) pSPIHandle->pRxBuffer++;
	}
	else  // 8-bit DFF
	{
		*(pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->SPI_DR;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer++;
	}

	if (!pSPIHandle->RxLen)
	{
		// Len=0, i.e. SPI Rx completed
		// Reset the SPI_CR2.RXNEIE
		pSPIHandle->pSPIx->SPI_CR2 &= ~(1 << SPI_CR2_RXNEIE);

		pSPIHandle->RxLen = 0;
		pSPIHandle->pRxBuffer = NULL;
		pSPIHandle->RxState = SPI_READY;

		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}
}


void SPI_ovr_interrupt_handler(SPI_Handle_t *pSPIHandle)
{
	// 1. Clear the OVR
	// OVR cleared by read access to SPI_DR followed by read access to SPI_SR
	// Clear if SPI Periph is not busy in Tx otherwise the data in the buffer maybe needed
	uint8_t dummy;
	if (pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		dummy = pSPIHandle->pSPIx->SPI_DR;
		dummy = pSPIHandle->pSPIx->SPI_SR;
	}

	(void) dummy;  // Just to avoid 'unsued-variable' warning

	// 2. Inform the Application
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}


 __attribute__((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t event)
{
	// Weak implementation
}


/*
 * Other functions
 */
void SPI_SSOE_Config(SPI_RegDef_t* pSPIx, uint8_t EnoOrDi)
{
	if (EnoOrDi == ENABLE)
	{
		pSPIx->SPI_CR2 |= (1 << SPI_CR2_SSOE);
	}
	else
	{
		pSPIx->SPI_CR2 &= ~(1 << SPI_CR2_SSOE);
	}

}
