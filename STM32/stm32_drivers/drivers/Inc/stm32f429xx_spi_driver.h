/*
 * stm32f429xx_spi_driver.h
 *
 *  Created on: 10-Jun-2022
 *      Author: rakshit
 */

#ifndef INC_STM32F429XX_SPI_DRIVER_H_
#define INC_STM32F429XX_SPI_DRIVER_H_

#include "stm32f429xx.h"

#define SPI_READY           0
#define SPI_BUSY_IN_RX      1
#define SPI_BUSY_IN_TX      2

/*
 * Configuration struct for SPIx peripheral
 */
typedef struct {
	uint8_t SPI_DeviceMode;             // <Possible values from @GPIO_PIN_NO>
	uint8_t SPI_BusConfig;              // <Possible values from @GPIO_PIN_MODES>
	uint8_t SPI_SclkSpeed;              // <Possible values from @GPIO_OUPUT_SPEED>
	uint8_t SPI_DFF;                    // <Possible values from @GPIO_PUPD>
	uint8_t SPI_CPOL;                   // <Possible values from @GPIO_OUTPUT_TYPE>
	uint8_t SPI_CPHA;                   // <Possible values from @GPIO_ALTFN_MODE>
	uint8_t SPI_SSM;                    // <Possible values from @GPIO_ALTFN_MODE>
}SPI_Config_t;


/*
 * Handle structure for SPIx Peripheral
 */

typedef struct {

	// Pointer to SPIx base addr
	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPI_Config;
	uint8_t      *pTxBuffer;
	uint8_t      *pRxBuffer;
	uint32_t     TxLen;
	uint32_t     RxLen;
	uint8_t      TxState;
	uint8_t      RxState;

}SPI_Handle_t;


/*
 * @SPI_DeviceMode
 */
#define SPI_DEVICE_MODE_MASTER        1
#define SPI_DEVICE_MODE_SLAVE         0

/*
 * @SPI_BusConfig
 */
#define SPI_BUS_CONFIG_FD                   1    // Full Duplex
#define SPI_BUS_CONFIG_HD                   2    // Half Duplex
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY       4    // Simplex Rx Only
// Simplex Tx only id not required as it just means that MISO pin is not configured


/*
 * @SPI_SclkSpeed
 */
#define SPI_SCLK_SPEED_DIV2          0
#define SPI_SCLK_SPEED_DIV4          1
#define SPI_SCLK_SPEED_DIV8          2
#define SPI_SCLK_SPEED_DIV16         3
#define SPI_SCLK_SPEED_DIV32         4
#define SPI_SCLK_SPEED_DIV64         5
#define SPI_SCLK_SPEED_DIV128        6
#define SPI_SCLK_SPEED_DIV256        7


/*
 * @SPI_DFF
 */
#define SPIDFF_8BITS         0
#define SPIDFF_16BITS        1


/*
 * @SPI_CPOL
 */
#define SPI_CPOL_LOW         0
#define SPI_CPOL_HIGH        1


/*
 * @SPI_CPHA
 */
#define SPI_CPHA_LOW         0
#define SPI_CPHA_HIGH        1


/*
 * @SPI_SSM
 */
#define SPI_SSM_DI           0
#define SPI_SSM_EN            1


/**************************************************************************************
 *                              APIs Supported by this driver
 *                    Check function definitions for more information
 **************************************************************************************/


/*
 * SPIx Peripheral clock setup
 */
void SPI_PeriClockControl(SPI_RegDef_t* pSPIx, uint8_t EnorDi);


/*
 * SPIx Init and DeInit
 */
void SPI_Init(SPI_Handle_t* pSPIHandle);
void SPI_DeInit(SPI_Handle_t* pSPIHandle);


/*
 * SPIx Send and Receive
 */
void SPI_SendData(SPI_RegDef_t* pSPIx, uint8_t* pTxBuffer, uint32_t Len);
void SPI_ReceiveData (SPI_RegDef_t* pSPIx, uint8_t* pRxBuffer, uint32_t Len);


/*
 * IRQ Handling
 */

void SPI_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t* pSPIHandle);


void SPI_SSOE_Config(SPI_RegDef_t* pSPIx, uint8_t EnoOrDi);

#endif /* INC_STM32F429XX_SPI_DRIVER_H_ */
