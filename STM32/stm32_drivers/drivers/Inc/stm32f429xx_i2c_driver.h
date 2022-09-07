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
}I2C_handle_t;


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

#define I2C_FM_DUTY_2             0  // t_low/t_high = 2
#define I2C_FM_DUTY_16_9          1  // t_low/t_high = 16/9



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
void I2C_Init(I2C_Handle_t* pI2CHandle);
void I2C_DeInit(I2C_Handle_t* pI2CHandle);


/*
 * I2Cx Send and Receive
 */
void I2C_SendData(I2C_RegDef_t* pI2Cx, uint8_t* pTxBuffer, uint32_t Len);
void I2C_ReceiveData (I2C_RegDef_t* pI2Cx, uint8_t* pRxBuffer, uint32_t Len);
uint8_t I2C_SendDataIT (I2C_Handle_t * pI2CHandle, uint8_t* pTxBuffer, uint32_t Len);
uint8_t I2C_ReceiveDataIT (I2C_Handle_t* pI2CHandle, uint8_t* pRxBuffer, uint32_t Len);

/*
 * IRQ Handling
 */

void I2C_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void I2C_IRQHandling(I2C_Handle_t* pI2CHandle);


void I2C_SSOE_Config(I2C_RegDef_t* pI2Cx, uint8_t EnoOrDi);


// Event Callback
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t event);

#endif /* INC_STM32F429XX_I2C_DRIVER_H_ */
