/*
 * I2C_MasterSendData.c
 */

#include "stm32f429xx.h"
#include <string.h>

#define DUMMY_ADDR 0x61  // Make
#define SLAVE_ADDR 0x68

I2C_Handle_t I2CTx;
GPIO_Handle_t GpioButton;
GPIO_Handle_t GpioLed;

uint8_t data[100] = "Hello how are khana kha ke jana\n";


void GPIO_Button_Init()
{
	GpioButton.pGPIOx = GPIOA;
	GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INP;
	GpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PD;
	GPIO_Init(&GpioButton);

	GpioLed.pGPIOx = GPIOG;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTP;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;

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
	I2CTx.I2C_Config.I2C_DeviceAddress = DUMMY_ADDR;
	I2CTx.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2CTx.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_Init(&I2CTx);
}


int main()
{
	I2C1_GPIO_Init();
	I2C_Inits();

	GPIO_Button_Init();

	GPIO_WriteToOutputPin(GpioLed.pGPIOx, GPIO_PIN_NO_13, 0);

	I2C_IRQITConfig(IRQ_NO_I2C1_EV, ENABLE);

	//enable the i2c peripheral
	I2C_PeripheralControl(I2C1,ENABLE);
	I2C_ManageAcking(I2C1,I2C_ACK_ENABLE);

	while(1)
	{
		GPIO_WriteToOutputPin(GpioLed.pGPIOx, GPIO_PIN_NO_13, 0);

		while(GPIO_ReadFromInputPin(GpioButton.pGPIOx, GPIO_PIN_NO_0) == 0);
		while(GPIO_ReadFromInputPin(GpioButton.pGPIOx, GPIO_PIN_NO_0));

		GPIO_WriteToOutputPin(GpioLed.pGPIOx, GPIO_PIN_NO_13, 1);
		sw_delay_ms(250);
/*
		// send command code 0x51 to read length from slave
		data[0] = 0x51;
		while(I2C_MasterSendDataIT(&I2CTx, data, 1,  SLAVE_ADDR) == I2C_BUSY_IN_TX);

		// save the length in data[0]
		I2C_MasterReceiveData(&I2CTx, data, 1, SLAVE_ADDR);

		uint8_t Len = data[0];

		// send command code 0x52 to ask for data
		data[0] = 0x52;
		while(I2C_MasterSendDataIT(&I2CTx, data, 1,  SLAVE_ADDR) == I2C_BUSY_IN_TX);

		I2C_MasterReceiveData(&I2CTx, data, Len, SLAVE_ADDR);
*/
		I2C_MasterSendDataIT(&I2CTx, data, strlen((char*)data), SLAVE_ADDR);
		while(I2CTx.TxRxState == I2C_BUSY_IN_TX);

		//if (!strcmp((char*) data, "x"))
	//		while(1);
	}
}


void I2C1_EV_IRQHandler()
{
	I2C_IRQHandling(&I2CTx);
}
