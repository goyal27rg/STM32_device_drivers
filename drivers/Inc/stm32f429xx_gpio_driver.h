/*
 * stm32f429xx_gpio_driver.h
 *
 *  Created on: 10-Jun-2022
 *      Author: rakshit
 */

#ifndef INC_STM32F429XX_GPIO_DRIVER_H_
#define INC_STM32F429XX_GPIO_DRIVER_H_

#include "stm32f429xx.h"

/*
 * Configuration struct for GPIO pins
 */
typedef struct {
	uint8_t GPIO_PinNumber;             // <Possible values from @GPIO_PIN_NO>
	uint8_t GPIO_PinMode;               // <Possible values from @GPIO_PIN_MODES>
	uint8_t GPIO_PinSpeed;              // <Possible values from @GPIO_OUPUT_SPEED>
	uint8_t GPIO_PinPuPdControl;        // <Possible values from @GPIO_PUPD>
	uint8_t GPIO_PinOPType;             // <Possible values from @GPIO_OUTPUT_TYPE>
	uint8_t GPIO_PinAltFunMode;         // <Possible values from @GPIO_ALTFN_MODE>
}GPIO_PinConfig_t;


/*
 * Handle structure for GPIO
 */

typedef struct {

	// Pointer to GPIO base addr
	GPIO_RegDef_t *pGPIOx;
	GPIO_PinConfig_t GPIO_PinConfig;


}GPIO_Handle_t;


/*
 * @GPIO_PIN_NO
 * GPIO Pin numbers
 */
#define GPIO_PIN_NO_0       0
#define GPIO_PIN_NO_1       1
#define GPIO_PIN_NO_2       2
#define GPIO_PIN_NO_3       3
#define GPIO_PIN_NO_4       4
#define GPIO_PIN_NO_5       5
#define GPIO_PIN_NO_6       6
#define GPIO_PIN_NO_7       7
#define GPIO_PIN_NO_8       8
#define GPIO_PIN_NO_9       9
#define GPIO_PIN_NO_10      10
#define GPIO_PIN_NO_11      11
#define GPIO_PIN_NO_12      12
#define GPIO_PIN_NO_13      13
#define GPIO_PIN_NO_14      14
#define GPIO_PIN_NO_15      15



/*
 * @GPIO_PIN_MODES
 * GPIO Pin - possible modes
 */

#define GPIO_MODE_INP       0  // These four modes are defined in the reference manual
#define GPIO_MODE_OUTP      1
#define GPIO_MODE_ALTFN     2
#define GPIO_MODE_ANALOG    3
#define GPIO_MODE_IT_FT     4  // This and the following modes are user defined
#define GPIO_MODE_IT_RT     5  // IT: Interrupt, F: Falling, R: Rising, T: Trigger
#define GPIO_MODE_IT_RFT    6


/*
 * @GPIO_OUTPUT_TYPES
 * GPIO Pin - output types
 */

#define GPIO_OP_TYPE_PP     0  // Output push-pull (reset state)
#define GPIO_OP_TYPE_OD     1  // Output open-drain


/*
 * @GPIO_OUTPUT_SPEED
 * GPIO Pin - output speed
 */

#define GPIO_SPEED_LOW      0
#define GPIO_SPEED_MEDIUM   1
#define GPIO_SPEED_FAST     2
#define GPIO_SPEED_HIGH     3


/*
 * @GPIO_PUPD
 * GPIO Pin - pull-up / pull-down
 */

#define GPIO_NO_PUPD        0
#define GPIO_PU             1
#define GPIO_PD             2




/**************************************************************************************
 *                              APIs Supported by this driver
 *                    Check function definitions for more information
 **************************************************************************************/

/*
 * GPIO Periph Clock Setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*
 * Init and De-Init
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);  // Use peripheral reset register: RCC_AHB1RSTR

/*
 * Data read and write
 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx );  // 16 pins per port
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * IRQ Handling
 */

void GPIO_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);


#endif /* INC_STM32F429XX_GPIO_DRIVER_H_ */
