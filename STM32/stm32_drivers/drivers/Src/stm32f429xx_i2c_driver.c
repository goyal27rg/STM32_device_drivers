/*
 * stm32f429xx_i2c_driver.c
 *
 *  Created on: 04-Sep-2022
 *      Author: rakshit
 */

/*
 * 1. I2C Init
 * 2. I2C Master Tx
 * 3. I2C Master Rx
 * 4. I2C Slave Tx
 * 5. I2C Slave Rx
 * 6. I2C Error Interrupt handling
 * 7. I2C Event Interrupt handling
 */

#include "stm32f429xx_i2c_driver.h"

uint16_t AHB_Prescalar_DivFactor[8] = {2, 4, 8, 16, 64, 128, 256, 512};  // AHB Prescalar values, indexed as: (RCC_CFGR.HPRE - 8)
uint16_t APBx_Prescalar_DivFactor[4] = {2, 4, 8, 16};  // APBx Prescalar values, indexed as: (RCC_CFGR.PPREx - 4)

static void I2C_GenerateStartCondition(I2C_RegDef_t* pI2Cx);
static void I2C_GenerateStopCondition(I2C_RegDef_t* pI2Cx);
static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t Slaveddr);
static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx);


uint32_t Rcc_GetPLLOutputCLOCK() { return 0;}  // Dummy Implementation

uint8_t I2C_GetFlagStatus(I2C_RegDef_t* pI2Cx, uint8_t flag)
{
	return (pI2Cx->I2C_SR1 & flag);
}

uint32_t Rcc_GetPCLK1Value(void)
{
	// Refer the Clock-tree
	//  ___________________       ___________      ___________      ______
	// |      CLK SRC      |  -> |    AHB    | -> |    APB1   | -> | APB1 |
	// | (HSI / HSE / PLL) |     | Prescaler |    | Prescaler |    |      |
	// |___________________|     |___________|    |___________|    |______|

	uint32_t SystemCLK, ahbp, apb1p;

	// 1. Find the CLK SRC: RCC_CFGR.SWS, bits[3:2]

	__vo uint32_t clksrc = (RCC->CFGR >> 2) & 0x3;

	if (clksrc == 0) {  // HSI
		SystemCLK = 16000000;  // 16 MHz
	}
	else if (clksrc == 1) {  // HSE
		SystemCLK = 8000000;  // 8 MHz
	}
	else if (clksrc == 1) {  // PLL
		SystemCLK = Rcc_GetPLLOutputCLOCK();
	}

	// 2. Find AHB Prescalar: RCC_CFGR.HPRE, bits[7:4]

	__vo uint32_t hpre = (RCC->CFGR >> 4) & 0xF;

	if (hpre < 8) {  // Not divided for less than 8
		ahbp = 1;
	} else {
		ahbp = AHB_Prescalar_DivFactor[hpre - 8];
	}

	// 3. Find APB1 Prescalar: RCC_CFGR.PPRE1, bits[12:10]

	__vo uint32_t ppre1 = (RCC->CFGR >> 10) & 0x7;

	if (ppre1 < 4) {  // Not divided for less than 4
		apb1p = 1;
	} else {
		apb1p = APBx_Prescalar_DivFactor[ppre1 - 4];
	}

	return (SystemCLK / ahbp / apb1p);
}


void I2C_PeriClockControl(I2C_RegDef_t* pI2Cx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}
		else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}
		else if (pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		}
	}
	else
	{
		if (pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}
		else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}
		else if (pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		}
	}
}


void I2C_Init(I2C_Handle_t* pI2CHandle)
{
	// Init the peripheral clock
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	uint32_t temp_reg = 0;

	// Program I2C_CR1.ACK
	temp_reg |= (pI2CHandle->I2C_Config.I2C_ACKControl << I2C_CR1_ACK);
	pI2CHandle->pI2Cx->I2C_CR1 = temp_reg;


	// Program I2C_CR2.FREQ
	temp_reg = 0;
	uint32_t pClk1 = Rcc_GetPCLK1Value();
	pClk1 = pClk1 / 1000000;  // Only the number in MHz in required

	temp_reg |= ((pClk1 & 0x3F) << I2C_CR2_FREQ);  // applying 5-bit mask to pClk1
	pI2CHandle->pI2Cx->I2C_CR2 = temp_reg;


	// Program own address: I2C_OAR1.ADD[7:1]
	// OAR1.ADDMODE[15] decides 7-bit or 10-bit mode. By default, ADDMODE = 0 => 7-bit address
	temp_reg = 0;
	temp_reg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	temp_reg |= (1 << 14);  // No reason, reference manual asks that software keeps bit[14] as 1
	pI2CHandle->pI2Cx->I2C_OAR1 = temp_reg;


	// Program I2C_CCR
	temp_reg = 0;
	uint16_t ccr_value;

	if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)  // Standard Mode
	{
		ccr_value = Rcc_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
	}
	else  // Fast Mode
	{
		temp_reg |= (1 << 15);  // Fm Mode I2C selection
		if (pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_value = Rcc_GetPCLK1Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		}
		else
		{
			temp_reg |= (1 << 14);  // Duty:  16/9
			ccr_value = (9 * Rcc_GetPCLK1Value()) / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		}
	}
	temp_reg |= (ccr_value & 0xFFF);
	pI2CHandle->pI2Cx->I2C_CCR = temp_reg;

	// TODO: Configure I2C_TRISE
}


void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t Slaveddr)
{
	// 1. Generate START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	// 2. When start condition is generated, I2C_SR1.SB is set. Wait for it to be set
	//    SCL remains pulled to LOW (stretched) until SB is cleared
	//    SB is cleared by reading the SR followed by writing to DR
	//    The last call to I2C_GetFlagStatus() will perform the read part
	//    Next step (3) will do the write part
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	// 3. Send the slave address along with R/nW bit set to W (i.e. 0)
	I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, Slaveddr);

	// 4. Confirm the completion of Address phase by reading I2C_SR.ADDR
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	// 5. Clear I2C_SR1.ADDR flag by reading SR1 followed by SR2
	//    SCL remains pulled to LOW (stretched) until ADDR is cleared
	I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

	// 6. Data phase: Keep sending Data till Len becomes 0
	while(Len > 0)
	{
		while( !I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));
		pI2CHandle->pI2Cx->I2C_DR =  *pTxBuffer;
		pTxBuffer++;
		Len--;
	}

	// 7. When Len becomes 0, we exit the loop
	//    At this instant, I2C_DR is empty (TxE=1) and the transfer of last byte has also finished (BTF=1)
	//    This is the condition (TxE==1 && BTF==1) to generate the stop condition
	//    When BTF=1, SCL is streched LOW
	while( !I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));  // Wait till TxE is set
	while( !I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF));  // Wait till BTF is set

	// 8. Generate STOP condition
	//    BTF is cleared by Hardware after generating START/STOP condition in transmission
	I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

}


static void I2C_GenerateStartCondition(I2C_RegDef_t* pI2Cx)
{
	// Set I2C_CR1.START = 1 (repeated start)
	pI2Cx->I2C_CR1 |= (1 << I2C_CR1_START);
}

static void I2C_GenerateStopCondition(I2C_RegDef_t* pI2Cx)
{
	// Set I2C_CR1.STOP = 1
	pI2Cx->I2C_CR1 |= (1 << I2C_CR1_STOP);
}

static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t Slaveddr)
{
	Slaveddr = (Slaveddr << 1) & ~(0x1);  // Bit[0] used for R/nW = 0
	pI2Cx->I2C_DR = Slaveddr;
}

static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx)
{
	uint32_t dummy = pI2Cx->I2C_SR1;
	dummy = pI2Cx->I2C_SR1;

	(void) dummy;  // Just to avoid unused-variable warning
}
