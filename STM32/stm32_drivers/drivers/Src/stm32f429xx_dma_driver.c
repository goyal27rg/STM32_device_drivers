/*
 * stm32f429xx_dma_driver.c
 *
 *  Created on: 30-Jun-2023
 *      Author: rakshit
 */


#include "stm32f429xx.h"
#include "stm32f429xx_dma_driver.h"


void DMA_PeriClockControl(DMA_RegDef_t* pDMAx, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		if (pDMAx == DMA1) {
			DMA1_PCLK_EN();
		}
		else if (pDMAx == DMA2) {
			DMA2_PCLK_EN();
		}
	}
	else {
		if (pDMAx == DMA1) {
			DMA1_PCLK_DI();
		}
		else if (pDMAx == DMA2) {
			DMA2_PCLK_DI();
		}
	}
}

static uint32_t DMA_GetSxCR(DMA_Handle_t *pDMAHandle) {
	switch  (pDMAHandle->DMA_Config.DMA_StreamNum) {
	case 0: return pDMAHandle->pDMAx->DMA_S0CR;
	case 1: return pDMAHandle->pDMAx->DMA_S1CR;
	case 2: return pDMAHandle->pDMAx->DMA_S2CR;
	case 3: return pDMAHandle->pDMAx->DMA_S3CR;
	case 4: return pDMAHandle->pDMAx->DMA_S4CR;
	case 5: return pDMAHandle->pDMAx->DMA_S5CR;
	case 6: return pDMAHandle->pDMAx->DMA_S6CR;
	case 7: return pDMAHandle->pDMAx->DMA_S7CR;
	default: return 0;
	}
}

static void DMA_SetSxCR(DMA_Handle_t *pDMAHandle, uint32_t value) {
	switch  (pDMAHandle->DMA_Config.DMA_StreamNum) {
	case 0: pDMAHandle->pDMAx->DMA_S0CR = value; return;
	case 1: pDMAHandle->pDMAx->DMA_S1CR = value; return;
	case 2: pDMAHandle->pDMAx->DMA_S2CR = value; return;
	case 3: pDMAHandle->pDMAx->DMA_S3CR = value; return;
	case 4: pDMAHandle->pDMAx->DMA_S4CR = value; return;
	case 5: pDMAHandle->pDMAx->DMA_S5CR = value; return;
	case 6: pDMAHandle->pDMAx->DMA_S6CR = value; return;
	case 7: pDMAHandle->pDMAx->DMA_S7CR = value; return;
	default: return;
	}
}


static void DMA_SetSxNDTR(DMA_Handle_t *pDMAHandle) {
	uint32_t value = pDMAHandle->DMA_Config.DMA_NumDataItems;
	switch  (pDMAHandle->DMA_Config.DMA_StreamNum) {
	case 0: pDMAHandle->pDMAx->DMA_S0NDTR = value; return;
	case 1: pDMAHandle->pDMAx->DMA_S1NDTR = value; return;
	case 2: pDMAHandle->pDMAx->DMA_S2NDTR = value; return;
	case 3: pDMAHandle->pDMAx->DMA_S3NDTR = value; return;
	case 4: pDMAHandle->pDMAx->DMA_S4NDTR = value; return;
	case 5: pDMAHandle->pDMAx->DMA_S5NDTR = value; return;
	case 6: pDMAHandle->pDMAx->DMA_S6NDTR = value; return;
	case 7: pDMAHandle->pDMAx->DMA_S7NDTR = value; return;
	default: return;
	}
}

static void DMA_SetSxPAR(DMA_Handle_t *pDMAHandle) {
	switch  (pDMAHandle->DMA_Config.DMA_StreamNum) {
	case 0: pDMAHandle->pDMAx->DMA_S0PAR = pDMAHandle->DMA_Config.DMA_PeriphAddr; return;
	case 1: pDMAHandle->pDMAx->DMA_S1PAR = pDMAHandle->DMA_Config.DMA_PeriphAddr; return;
	case 2: pDMAHandle->pDMAx->DMA_S2PAR = pDMAHandle->DMA_Config.DMA_PeriphAddr; return;
	case 3: pDMAHandle->pDMAx->DMA_S3PAR = pDMAHandle->DMA_Config.DMA_PeriphAddr; return;
	case 4: pDMAHandle->pDMAx->DMA_S4PAR = pDMAHandle->DMA_Config.DMA_PeriphAddr; return;
	case 5: pDMAHandle->pDMAx->DMA_S5PAR = pDMAHandle->DMA_Config.DMA_PeriphAddr; return;
	case 6: pDMAHandle->pDMAx->DMA_S6PAR = pDMAHandle->DMA_Config.DMA_PeriphAddr; return;
	case 7: pDMAHandle->pDMAx->DMA_S7PAR = pDMAHandle->DMA_Config.DMA_PeriphAddr; return;
	default: return;
	}
}

static void DMA_SetSxM0AR(DMA_Handle_t *pDMAHandle) {
	//TODO(Handle Double Buffer mode, later)
	switch  (pDMAHandle->DMA_Config.DMA_StreamNum) {
	case 0: pDMAHandle->pDMAx->DMA_S0M0AR = pDMAHandle->DMA_Config.DMA_Mem0Addr; return;
	case 1: pDMAHandle->pDMAx->DMA_S1M0AR = pDMAHandle->DMA_Config.DMA_Mem0Addr; return;
	case 2: pDMAHandle->pDMAx->DMA_S2M0AR = pDMAHandle->DMA_Config.DMA_Mem0Addr; return;
	case 3: pDMAHandle->pDMAx->DMA_S3M0AR = pDMAHandle->DMA_Config.DMA_Mem0Addr; return;
	case 4: pDMAHandle->pDMAx->DMA_S4M0AR = pDMAHandle->DMA_Config.DMA_Mem0Addr; return;
	case 5: pDMAHandle->pDMAx->DMA_S5M0AR = pDMAHandle->DMA_Config.DMA_Mem0Addr; return;
	case 6: pDMAHandle->pDMAx->DMA_S6M0AR = pDMAHandle->DMA_Config.DMA_Mem0Addr; return;
	case 7: pDMAHandle->pDMAx->DMA_S7M0AR = pDMAHandle->DMA_Config.DMA_Mem0Addr; return;
	default: return;
	}
}

void DMA_Init(DMA_Handle_t *pDMAHandle) {

	// Enable the clock
	DMA_PeriClockControl(pDMAHandle->pDMAx, ENABLE);

	// First, disable the stream
	DMA_SetSxCR(pDMAHandle, DMA_GetSxCR(pDMAHandle) & ~(1 << DMA_SxCR_EN));

	// wait for DMA_SxCR.EN to become 0
	while(DMA_GetSxCR(pDMAHandle) & (1 << DMA_SxCR_EN));

	DMA_SetSxPAR(pDMAHandle);
	DMA_SetSxM0AR(pDMAHandle);

	DMA_SetSxNDTR(pDMAHandle);

	uint32_t tempDmaSxcr = 0;
	tempDmaSxcr |= pDMAHandle->DMA_Config.DMA_ChannelNum << DMA_SxCR_CHSEL;

	if (pDMAHandle->DMA_Config.DMA_FlowController == DMA_FLOW_CONTROLLER_PERIPH) {
		tempDmaSxcr |=  1 << DMA_SxCR_PFCTRL;
	}

	tempDmaSxcr |= pDMAHandle->DMA_Config.DMA_StreamPriorityLevel << DMA_SxCR_PL;

	tempDmaSxcr |= pDMAHandle->DMA_Config.DMA_TransferDirection << DMA_SxCR_DIR;

	tempDmaSxcr |= pDMAHandle->DMA_Config.DMA_CircularModeEnorDi << DMA_SxCR_CIRC;

	tempDmaSxcr |= pDMAHandle->DMA_Config.DMA_PeriphIncrementEnorDi << DMA_SxCR_PINC;
	tempDmaSxcr |= pDMAHandle->DMA_Config.DMA_MemIncrementEnorDi << DMA_SxCR_MINC;

	tempDmaSxcr |= pDMAHandle->DMA_Config.DMA_PeriphDataWidth << DMA_SxCR_PSIZE;
	tempDmaSxcr |= pDMAHandle->DMA_Config.DMA_MemDataWidth << DMA_SxCR_MSIZE;

	//TODO(Program PBURST, MBURST, PINCOS and other DMA_SxCR fields, later)
	DMA_SetSxCR(pDMAHandle, tempDmaSxcr);

	//TODO(Program DMA_SxFCR, later)
}

void DMA_Enable(DMA_Handle_t *pDMAHandle) {
	DMA_SetSxCR(pDMAHandle, DMA_GetSxCR(pDMAHandle) | (1 << DMA_SxCR_EN));
}











