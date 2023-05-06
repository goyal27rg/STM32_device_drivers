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
static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t Slaveddr, uint8_t ReadNotWrite);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);
static void I2C_CloseI2C(I2C_Handle_t *pI2CHandle);


uint32_t Rcc_GetPLLOutputCLOCK() { return 0;}  // Dummy Implementation

uint8_t I2C_GetFlagStatus(I2C_RegDef_t* pI2Cx, uint8_t flag)
{
	if (pI2Cx->I2C_SR1 & flag)
	{
		return 1;
	}
	return 0;
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

	return ((SystemCLK / ahbp) / apb1p);
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


void I2C_PeripheralControl(I2C_RegDef_t* pI2Cx, uint8_t EnoOrDi)
{
	if(EnoOrDi == ENABLE)
	{
		pI2Cx->I2C_CR1 |= (1 << I2C_CR1_PE);
	}
	else
	{
		pI2Cx->I2C_CR1 &= ~(1 << I2C_CR1_PE);
	}
}

void I2C_Init(I2C_Handle_t* pI2CHandle)
{
	// Init the peripheral clock
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	uint32_t temp_reg = 0;

	// Program I2C_CR1.ACK
	temp_reg |= (pI2CHandle->I2C_Config.I2C_ACKControl << I2C_CR1_ACK);
	pI2CHandle->pI2Cx->I2C_CR1 = (1 << 10);//temp_reg;


	// Program I2C_CR2.FREQ
	temp_reg = 0;
	uint32_t pClk1 = Rcc_GetPCLK1Value();
	pClk1 = pClk1 / 1000000U;  // Only the number in MHz in required

	temp_reg = 0;
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

	// Configure I2C_TRISE
	temp_reg = 0;
	if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		/*
		 * Standard mode:
		 * As per I2C Specification, maximum rise time(tr_max)  = 1000ns
		 * TRISE = (tr_max / Tpclk) + 1
		 * 		 = (1000 * 10^(-9) * Fpclk) + 1
		 * 		 = (10^(-6) * Fpclk) + 1
		 * 		 = (Rcc_GetPCLK1Value() / 1_000_000U) + 1
		 */
		temp_reg = (Rcc_GetPCLK1Value() / 1000000U) + 1;
	}
	else
	{
		/*
		 * Fast mode:
		 * As per I2C Specification, maximum rise time(tr_max)  = 300ns
		 * TRISE = (3 * Rcc_GetPCLK1Value() / 10_000_000) + 1
		 */
		temp_reg = (3 * Rcc_GetPCLK1Value() / 1000000000U) + 1;
	}
	temp_reg = temp_reg & 0x3F;
	pI2CHandle->pI2Cx->I2C_TRISE = temp_reg;
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
	I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, Slaveddr, I2C_MASTER_ADDR_FLAG_WRITE);

	// 4. Confirm the completion of Address phase by reading I2C_SR.ADDR
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	// 5. Clear I2C_SR1.ADDR flag by reading SR1 followed by SR2
	//    SCL remains pulled to LOW (stretched) until ADDR is cleared
	I2C_ClearADDRFlag(pI2CHandle);

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
	//if (Sr == I2C_DISABLE_SR)
	I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

}


void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t Slaveddr)
{
	// 1. Generate start condition

	// 2. Occurs EV5: SB=1, clear by reading SR1 register, then write Slave address to DR register
	//	  (SCL is stretched low until SW sequence completes)

	// 3. Occurs EV6: ADDR=1, clear by reading SR1 followed by reading SR2 (SCL stretched low)
	//	  (SCL is stretched low until SW sequence completes)
	//    Data will arrive after SW sequence is completed

	// 4. Occurs EV7: RxNE=1, Data has arrived, hence Rx buffer is not empty. Cleared by reading DR register
	//    (SCL is stretched to Low if the SW sequence not complete before the next byte reception completes,
	//     one more byte can be received after [4], after which SCL remains low if DR is not read in [4])

	// So, every time EV7 occurs, master would have received one more byte after the byte causing EV7
	// Hence, if we want to receive 'Len' number of bytes, we need to disable ACK when we receive the second-last byte
	// i.e., in the EV7 after second-last byte, we will set ACK=0 and program STOP request. Master would have already received last byte and since we just now set ACK=0,
	// NACK would be sent after EV7 of second-last byte, and slave would stop sending data

	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, Slaveddr, I2C_MASTER_ADDR_FLAG_READ);
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	if (Len == 1)  // For only 1 byte reception, disable ACKing before clearing ADDR flag, since data arrives as soon as ADDR is cleared
	{
		// First disable ACKing so that ACK is not sent after first byte is received
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

		// clear ADDR flag. SCL is stretched to low until ADDR is cleared
		I2C_ClearADDRFlag(pI2CHandle);

		// Wait for RxNE to be set and then read the byte
		while( !I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		*pRxBuffer = pI2CHandle->pI2Cx->I2C_DR;
		Len--;
	}
	else
	{
		I2C_ClearADDRFlag(pI2CHandle);
		while(Len > 0)
		{
			while( !I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

			if (Len == 2)  // Disable ACKing
			{
				I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);
				I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
			}

			*pRxBuffer = pI2CHandle->pI2Cx->I2C_DR;
			pRxBuffer ++;
			Len--;
		}
	}

	//re-enable ACKing if needed
	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
	}
}


uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t Slaveddr)
{
	__vo uint8_t state = pI2CHandle->TxRxState;
	if(state == I2C_READY)
	{
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		state = I2C_BUSY_IN_TX;
		pI2CHandle->TxLen = Len;
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->DevAddr = Slaveddr;

		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		pI2CHandle->pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITBUFEN);  // Enable ITBUFEN
		pI2CHandle->pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITEVTEN);  // Enable ITEVTEN
	}
	return state;
}

uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t Slaveddr)
{
	__vo uint8_t state = pI2CHandle->TxRxState;
	if(state == I2C_READY)
	{
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		state = I2C_BUSY_IN_RX;
		pI2CHandle->RxLen = Len;
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->DevAddr = Slaveddr;

		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		pI2CHandle->pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITBUFEN);  // Enable ITBUFEN
		pI2CHandle->pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITEVTEN);  // Enable ITEVTEN
	}
	return state;
}

void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	// check the reason for interrupt
	uint32_t TempI2cSr1 = pI2CHandle->pI2Cx->I2C_SR1;

	if(TempI2cSr1 & I2C_FLAG_SB)
	{
		// Start bit set. I2C_SR1 read at the beginning of this function cleared the SB flag
		// Address Phase can begin

		// select if the transaction is 'read' or 'write'
		uint8_t I2cRWFlag = (pI2CHandle->TxRxState == I2C_BUSY_IN_TX) ? I2C_MASTER_ADDR_FLAG_WRITE : I2C_MASTER_ADDR_FLAG_READ;
		I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, pI2CHandle->DevAddr, I2cRWFlag);
	}

	if(TempI2cSr1 & I2C_FLAG_ADDR)
	{
		if ((pI2CHandle->TxRxState == I2C_BUSY_IN_RX) && pI2CHandle->RxLen == 1)
		{
			I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);
		}
		// ADDR bit set. Clear the flag.
		I2C_ClearADDRFlag(pI2CHandle);
	}

	// Handle the setting of TxE bit
	if(TempI2cSr1 & I2C_FLAG_TXE)
	{
		// Check Device mode
		if(pI2CHandle->pI2Cx->I2C_SR2 && (1 << I2C_SR2_MSL))
		{
			// Master mode
			if(pI2CHandle->TxLen > 0)
			{
				pI2CHandle->pI2Cx->I2C_DR = *pI2CHandle->pTxBuffer;
				pI2CHandle->TxLen--;
				pI2CHandle->pTxBuffer++;
			}

			if (TempI2cSr1 & I2C_FLAG_BTF)
			{
				// Transfer complete, STOP the communication.
				I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
				I2C_CloseI2C(pI2CHandle);
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
			}
		}
		else
		{
			// Slave mode
			// Confirm slave mode by checking I2C_SR2.TRA
			if(pI2CHandle->pI2Cx->I2C_SR2 & (1 << I2C_SR2_TRA))
			{
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
			}
		}
	}

	// Handle the setting of RxNE bit
	if (TempI2cSr1 & I2C_FLAG_RXNE)
	{
		if(pI2CHandle->pI2Cx->I2C_SR2 && (1 << I2C_SR2_MSL))
		{
			// Handle the special case of single byte reception
			if (pI2CHandle->RxLen == 1)
			{
				*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->I2C_DR;
				pI2CHandle->RxLen--;
				pI2CHandle->pRxBuffer++;
			}

			if (pI2CHandle->RxLen > 1)
			{
				if (pI2CHandle->RxLen == 2)  // Disable ACKing
				{
					I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);
				}

				*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->I2C_DR;
				pI2CHandle->RxLen--;
				pI2CHandle->pRxBuffer++;
			}

			if(pI2CHandle->RxLen == 0)
			{
				I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
				I2C_CloseI2C(pI2CHandle);
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
			}
		}
		else
		{
			// Slave mode
			// Confirm slave mode by checking I2C_SR2.TRA
			if(pI2CHandle->pI2Cx->I2C_SR2 & (1 << I2C_SR2_TRA))
			{
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
			}
		}
	}

	// Handle STOPF
	// I2C_SR1.STOPF bit is set only when I2C peripheral is in Slave Receiver mode
	// It is set after last byte of data is received and stop condition is generated by master
	if (TempI2cSr1 & I2C_FLAG_STOPF)
	{
		// STOPF is cleared by reading SR1 followed by writing to CR1.
		// SR1 has already been read to get this status.
		// Dummy write to CR1
		pI2CHandle->pI2Cx->I2C_CR1 |= 0;
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
	}

}

void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data)
{
	pI2Cx->I2C_DR = data;
}

uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx)
{
	return pI2Cx->I2C_DR;
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

static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t Slaveddr, uint8_t ReadNotWrite)
{
	Slaveddr = (Slaveddr << 1) | (ReadNotWrite & 0x1);  // Bit[0] used for R/nW = 0
	pI2Cx->I2C_DR = Slaveddr;
}

void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == I2C_ACK_ENABLE)
	{
		//enable the ack
		pI2Cx->I2C_CR1 |= ( 1 << I2C_CR1_ACK);
	}else
	{
		//disable the ack
		pI2Cx->I2C_CR1 &= ~( 1 << I2C_CR1_ACK);
	}
}

void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle )
{
	uint32_t dummy_read;
	dummy_read = pI2CHandle->pI2Cx->I2C_SR1;
	dummy_read = pI2CHandle->pI2Cx->I2C_SR2;
	(void)dummy_read;  // To avoid unused variable warnings
}

void I2C_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	uint8_t IserRegNo = IRQNumber / 32;
	uint8_t IserRegOffset = IRQNumber % 32;

	if (EnorDi == ENABLE)
	{
		switch(IserRegNo)
		{
		case 0: *NVIC_ISER0 |= (1 << IserRegOffset);
		case 1: *NVIC_ISER1 |= (1 << IserRegOffset);
		case 2: *NVIC_ISER2 |= (1 << IserRegOffset);
		case 3: *NVIC_ISER3 |= (1 << IserRegOffset);
		}
	}
	else
	{
		switch(IserRegNo)
		{
		case 0: *NVIC_ICER0 |= (1 << IserRegOffset);
		case 1: *NVIC_ICER1 |= (1 << IserRegOffset);
		case 2: *NVIC_ICER2 |= (1 << IserRegOffset);
		case 3: *NVIC_ICER3 |= (1 << IserRegOffset);
		}
	}
}

void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	uint8_t IprRegNo = IRQNumber / 4;
	uint8_t IprRegOffset = IRQNumber % 4;
	uint8_t shift = IprRegOffset * 8 + 8 - NO_PR_BITS_IMPLEMENTED;

	__vo uint32_t *IprReg = (NVIC_IPR_BASEADDR + IprRegNo * 4);
	*IprReg &= ~(0xF << shift);
	*IprReg |= (IRQPriority << shift);

}

void I2C_CloseI2C(I2C_Handle_t *pI2CHandle)
{
	// Disable interrupts
	pI2CHandle->pI2Cx->I2C_CR2 &= ~(1 << I2C_CR2_ITBUFEN);  // Disable Buffer interrupt
	pI2CHandle->pI2Cx->I2C_CR2 &= ~(1 << I2C_CR2_ITEVTEN);  // Disable Event interrupt

	// Clear the application's context
	pI2CHandle->RxLen = 0;
	pI2CHandle->TxLen = 0;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->DevAddr = 0;
	pI2CHandle->TxRxState = I2C_READY;

	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);

	}

}
