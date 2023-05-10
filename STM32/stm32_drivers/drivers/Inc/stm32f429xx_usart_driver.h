/*
 * stm32f429xx_usart_driver.h
 *
 *  Created on: 09-May-2023
 *      Author: rakshit
 */

#ifndef INC_STM32F429XX_USART_DRIVER_H_
#define INC_STM32F429XX_USART_DRIVER_H_

#include "stm32f429xx.h"


/*
 * Configuration structure for USART peripheral
 */
typedef struct {
	uint8_t USART_Mode;
	uint32_t USART_Baud;
	uint8_t USART_NoOfStopBits;
	uint8_t USART_WordLength;
	uint8_t USART_ParityControl;
	uint8_t USART_HWFlowControl;
} USART_Config_t;


/*
 * Configuration structure for USART peripheral
 */
typedef struct {
	USART_RegDef_t *pUSARTx;
	USART_Config_t USART_Config;
} USART_Handle_t;


/*
 *@USART_Mode
 *Possible options for USART_Mode
 */
#define USART_MODE_ONLY_TX 0
#define USART_MODE_ONLY_RX 1
#define USART_MODE_TXRX  2

/*
 *@USART_Baud
 *Possible options for USART_Baud
 */
#define USART_STD_BAUD_1200					1200
#define USART_STD_BAUD_2400					400
#define USART_STD_BAUD_9600					9600
#define USART_STD_BAUD_19200 				19200
#define USART_STD_BAUD_38400 				38400
#define USART_STD_BAUD_57600 				57600
#define USART_STD_BAUD_115200 				115200
#define USART_STD_BAUD_230400 				230400
#define USART_STD_BAUD_460800 				460800
#define USART_STD_BAUD_921600 				921600
#define USART_STD_BAUD_2M 					2000000
#define SUART_STD_BAUD_3M 					3000000


/*
 *@USART_ParityControl
 *Possible options for USART_ParityControl
 */
#define USART_PARITY_EN_ODD   2
#define USART_PARITY_EN_EVEN  1
#define USART_PARITY_DISABLE   0

/*
 *@USART_WordLength
 *Possible options for USART_WordLength
 */
#define USART_WORDLEN_8BITS  0
#define USART_WORDLEN_9BITS  1

/*
 *@USART_NoOfStopBits
 *Possible options for USART_NoOfStopBits
 */
#define USART_STOPBITS_1     0
#define USART_STOPBITS_0_5   1
#define USART_STOPBITS_2     2
#define USART_STOPBITS_1_5   3

/*
 *@USART_HWFlowControl
 *Possible options for USART_HWFlowControl
 */
#define USART_HW_FLOW_CTRL_NONE    	0
#define USART_HW_FLOW_CTRL_CTS    	1
#define USART_HW_FLOW_CTRL_RTS    	2
#define USART_HW_FLOW_CTRL_CTS_RTS	3



/*
 * USARTx Peripheral clock setup
 */
void USART_PeriClockControl(USART_RegDef_t* pUSARTx, uint8_t EnorDi);


/*
 * USARTx Init and DeInit
 */
void USART_Init(USART_Handle_t *pUSARTHandle);
void USART_DeInit(USART_Handle_t *pUSARTHandle);


/*
 * USARTx Send and Receive
 */

void USART_MasterSendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t Slaveddr);
void USART_MasterReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t Slaveddr);

uint8_t USART_MasterSendDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t Slaveddr);
uint8_t USART_MasterReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t Slaveddr);

void USART_SlaveSendData(USART_RegDef_t *pUSARTx, uint8_t data);
uint8_t USART_SlaveReceiveData(USART_RegDef_t *pUSARTx);


/*
 * IRQ Handling
 */

void USART_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi);
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);

/*
 * Control functions
 */
void USART_PeripheralControl(USART_RegDef_t* pUSARTx, uint8_t EnoOrDi);
void USART_ManageAcking(USART_RegDef_t *pUSARTx, uint8_t EnorDi);
void USART_SlaveEnableDisableCallbackEvents(USART_RegDef_t *pUSARTx, uint8_t EnorDi);

//Misc functions
uint8_t USART_GetFlagStatus(USART_RegDef_t* pUSARTx, uint8_t flag);

// Event Callback
void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t event);

#endif /* INC_STM32F429XX_USART_DRIVER_H_ */
