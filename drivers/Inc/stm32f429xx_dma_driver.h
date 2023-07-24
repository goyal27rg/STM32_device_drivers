/*
 * stm32f429xx_dma_driver.h
 *
 *  Created on: 30-Jun-2023
 *      Author: rakshit
 */

#ifndef INC_STM32F429XX_DMA_DRIVER_H_
#define INC_STM32F429XX_DMA_DRIVER_H_


/*
 * Configuration structure for DMA peripheral
 */
typedef struct {
	uint8_t DMA_StreamNum;
	uint8_t DMA_ChannelNum;
	uint8_t DMA_StreamPriorityLevel;
	uint8_t DMA_TransferDirection;
	uint8_t DMA_PeriphDataWidth;
	uint8_t DMA_MemDataWidth;
	uint8_t DMA_NumDataItems;
	uint8_t DMA_PeriphIncrementEnorDi;
	uint8_t DMA_MemIncrementEnorDi;
	uint8_t DMA_CircularModeEnorDi;
	uint8_t DMA_FlowController;
	uint32_t DMA_PeriphAddr;
	uint32_t DMA_Mem0Addr;
	uint32_t DMA_Mem1Addr;
} DMA_Config_t;


/*
 * Configuration structure for DMA peripheral
 */
typedef struct {
	DMA_RegDef_t *pDMAx;
	DMA_Config_t DMA_Config;

} DMA_Handle_t;


#define DMA_TRANSFER_DIR_PERIPH_TO_MEM        0
#define DMA_TRANSFER_DIR_MEM_TO_PERIPH        1
#define DMA_TRANSFER_DIR_MEM_TO_MEM           2

#define DMA_FLOW_CONTROLLER_DMA               0
#define DMA_FLOW_CONTROLLER_PERIPH            1

#define DMA_MEM_PERIPH_DATA_WIDTH_BYTE        0
#define DMA_MEM_PERIPH_DATA_WIDTH_HALFWORD    1
#define DMA_MEM_PERIPH_DATA_WIDTH_WORD        2

/*
 * DMAx Peripheral clock setup
 */
void DMA_PeriClockControl(DMA_RegDef_t* pDMAx, uint8_t EnorDi);


/*
 * DMAx Init and DeInit
 */
void DMA_Init(DMA_Handle_t *pDMAHandle);
void DMA_DeInit(DMA_Handle_t *pDMAHandle);


/*
 * IRQ Handling
 */

void DMA_IRQEnDi(DMA_Handle_t *pDMAHandle, uint8_t IRQNumber, uint8_t EnorDi);
void DMA_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);


/*
 * Control functions
 */

void DMA_Enable(DMA_Handle_t *pDMAHandle);
void DMA_Disable(DMA_Handle_t *pDMAHandle);

#endif /* INC_STM32F429XX_DMA_DRIVER_H_ */
