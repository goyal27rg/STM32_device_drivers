/*
 * stm32f429xx_spi_driver.c
 *
 *  Created on: 02-Jul-2022
 *      Author: rakshit
 */


#include "stm32f429xx_spi_driver.h"


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
		if (0 & pSPIx->SPI_CR1 & ~(1 << SPI_CR1_DFF))
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
 * IRQ Handling
 */

void SPI_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t* pSPIHandle);




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
