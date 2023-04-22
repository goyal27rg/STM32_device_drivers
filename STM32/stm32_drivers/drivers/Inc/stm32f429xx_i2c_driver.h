/*
 * stm32f429xx_i2c_driver.h
 *
 *  Created on: 04-Sep-2022
 *      Author: rakshit
 */

#ifndef INC_STM32F429XX_I2C_DRIVER_H_
#define INC_STM32F429XX_I2C_DRIVER_H_

#include "stm32f429xx.h"

/*
 * I2C status flag related definitions
 */
#define I2C_FLAG_SB               (1 << I2C_SR1_SB)
#define I2C_FLAG_ADDR             (1 << I2C_SR1_ADDR)
#define I2C_FLAG_BTF              (1 << I2C_SR1_BTF)
#define I2C_FLAG_STOPF            (1 << I2C_SR1_STOPF)
#define I2C_FLAG_RXNE             (1 << I2C_SR1_RXNE)
#define I2C_FLAG_TXE              (1 << I2C_SR1_TXE)
#define I2C_FLAG_ARLO             (1 << I2C_SR1_ARLO)
#define I2C_FLAG_AF               (1 << I2C_SR1_AF)
#define I2C_FLAG_OVR              (1 << I2C_SR1_OVR)
#define I2C_FLAG_TIMEOUT          (1 << I2C_SR1_TIMEOUT)

#define I2C_DISABLE_SR            RESET
#define I2C_ENABLE_SR             SET


/*
 * Configuration struct for I2Cx peripheral
 */
typedef struct {
	uint32_t I2C_SCLSpeed;
	uint8_t  I2C_DeviceAddress;  // Own Address when acting as slave
	uint8_t  I2C_ACKControl;  // Automatic ACKing is off by default
	uint16_t I2C_FMDutyCycle;  // Fast-mode duty cycle
}I2C_Config_t;

/*
 * Handle structure for I2C peripherals
 */
typedef struct {
	I2C_RegDef_t *pI2Cx;
	I2C_Config_t I2C_Config;

	//Following are necessary for interrupt handling
	uint8_t      *pTxBuffer;  // To store application Transmit buffer address
	uint8_t      *pRxBuffer;  // To store application Receive buffer address
	uint32_t     TxLen;
	uint32_t     RxLen;
	uint8_t      TxRxState;   // No separate states for Tx and Rx because I2C is a simplex protocol
	uint8_t      DevAddr;     // Slave / Device address
	uint32_t     RxSize;
	uint8_t      Sr;          // Repeated Start
}I2C_Handle_t;


/*
 * I2C application states
 */
#define I2C_READY 					0
#define I2C_BUSY_IN_RX 				1
#define I2C_BUSY_IN_TX 				2

/*
 * @I2C_SCLSpeed
 */

#define I2C_SCL_SPEED_SM          100000  // Standard Mode, 100 KHz
#define I2C_SCL_SPEED_FM2K        200000  // Fast Mode, 200 KHz
#define I2C_SCL_SPEED_FM4K        400000  // Fast Mode, 400 KHz

/*
 * @I2C_ACKControl
 */

#define I2C_ACK_ENABLE            1
#define I2C_ACK_DISABLE           0  // Default is 0

/*
 * @I2C_FMDutyCycle
 */

#define I2C_FM_DUTY_2             0  // t_low/t_high = 2Z
/*
 * I2C master Read/Write
 */

#define I2C_MASTER_ADDR_FLAG_READ   1
#define I2C_MASTER_ADDR_FLAG_WRITE  0


/**************************************************************************************
 *                              APIs Supported by this driver
 *                    Check function definitions for more information
 **************************************************************************************/


/*
 * I2Cx Peripheral clock setup
 */
void I2C_PeriClockControl(I2C_RegDef_t* pI2Cx, uint8_t EnorDi);


/*
 * I2Cx Init and DeInit
 */
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_Handle_t *pI2CHandle);


/*
 * I2Cx Send and Receive
 */

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t Slaveddr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t Slaveddr);

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t Slaveddr);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t Slaveddr);

/*
 * IRQ Handling
 */

void I2C_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void I2C_IRQHandling(I2C_Handle_t *pI2CHandle);

/*
 * Control functions
 */
void I2C_PeripheralControl(I2C_RegDef_t* pI2Cx, uint8_t EnoOrDi);
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

//Misc functions
uint8_t I2C_GetFlagStatus(I2C_RegDef_t* pI2Cx, uint8_t flag);

// Event Callback
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t event);

#endif /* INC_STM32F429XX_I2C_DRIVER_H_ */