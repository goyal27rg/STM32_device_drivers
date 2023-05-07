/*
 * I2C_MasterSendData.c
 */

#include "stm32f429xx.h"
#include <string.h>

#define SLAVE_ADDR 0x68

I2C_Handle_t I2CTx;
GPIO_Handle_t GpioButton;
GPIO_Handle_t GpioLed;

uint8_t Tx_buff[100] = "Hello World!";


void GPIO_LED_and_Button_Init()
{
	GpioButton.pGPIOx = GPIOA;
	GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INP;
	GpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PD;
	GPIO_Init(&GpioButton);

	GpioLed.pGPIOx = GPIOG;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTP;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;

	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&GpioLed);
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&GpioLed);
}

void I2C_GPIO_Config(GPIO_Handle_t* GPIOx_Handle)
{
	GPIOx_Handle->GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	GPIOx_Handle->GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;  // Open-drain for I2C
	GPIOx_Handle->GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;
	GPIOx_Handle->GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	GPIOx_Handle->GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
}

void I2C1_GPIO_Init()
{
	GPIO_Handle_t GPIOTx;

	GPIOTx.pGPIOx = GPIOB;

	I2C_GPIO_Config(&GPIOTx);

	GPIOTx.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;  // PB6: SCL
	GPIO_Init(&GPIOTx);

	GPIOTx.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;  // PB7: SDA
	GPIO_Init(&GPIOTx);
}

void I2C_Inits()
{

	I2CTx.pI2Cx = I2C1;
	I2CTx.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	I2CTx.I2C_Config.I2C_DeviceAddress = SLAVE_ADDR;
	I2CTx.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2CTx.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_Init(&I2CTx);
}


int main()
{
	I2C1_GPIO_Init();
	I2C_Inits();

	GPIO_LED_and_Button_Init();

	GPIO_WriteToOutputPin(GpioLed.pGPIOx, GPIO_PIN_NO_13, 0);

	I2C_IRQITConfig(IRQ_NO_I2C1_EV, ENABLE);

	// For Master mode, interrupts are enabled in I2C_MasterSendDataIT and I2C_MasterReceiveDataIT functions
	// Need to enable separately for Slave mode

	I2C_SlaveEnableDisableCallbackEvents(I2CTx.pI2Cx, ENABLE);

	//enable the i2c peripheral
	I2C_PeripheralControl(I2C1,ENABLE);
	I2C_ManageAcking(I2C1,I2C_ACK_ENABLE);

	while(1)
	{
	}
}


void I2C1_EV_IRQHandler()
{
	I2C_EV_IRQHandling(&I2CTx);
}


void I2C1_ER_IRQHandler()
{
	I2C_ER_IRQHandling(&I2CTx);
}


void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t event)
{
	static uint8_t cmd_code = 0;
	static uint8_t count = 0;

	if (event == I2C_EV_DATA_REQ)
	{
		// Master requested data, Slave needs to send
		if (cmd_code == 0x51)
		{
			// Length of message requested by master
			I2C_SlaveSendData(pI2CHandle->pI2Cx, strlen((char*) Tx_buff));
		}
		else if (cmd_code == 0x52)
		{
			// Send the message
			I2C_SlaveSendData(pI2CHandle->pI2Cx, Tx_buff[count++]);
		}
	}
	else if (event == I2C_EV_DATA_RCV)
	{
		// Data is waiting to be received by the Slave
		cmd_code = I2C_SlaveReceiveData(pI2CHandle->pI2Cx);
	}
	else if (event == I2C_EV_STOP)
	{
		// Happens only during Slave reception
		// Master has ended communication
	}
	else if (event == I2C_ERROR_AF)
	{
		// ACK Failure. Happens only in Slave transmission mode
		// Master has sent NACK. Slave shouldn't send any more data
		cmd_code = 0xff;  // Invalidate command code
		count = 0;
	}
}
