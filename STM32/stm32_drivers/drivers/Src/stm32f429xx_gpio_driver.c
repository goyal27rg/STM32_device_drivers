/*
 * stm32f429xx_gpio.c
 *
 *  Created on: 10-Jun-2022
 *      Author: rakshit
 */


#include "stm32f429xx_gpio_driver.h"


/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 *
 * @return            -  none
 *
 * @Note              -  none
 */

#include <stddef.h>

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		} else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		} else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		} else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		} else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		} else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		} else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		} else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		} else if (pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		} else if (pGPIOx == GPIOJ)
		{
			GPIOJ_PCLK_EN();
		} else if (pGPIOx == GPIOK)
		{
			GPIOK_PCLK_EN();
		}
	}
}



/*********************************************************************
 * @fn      		  - GPIO_Init
 *
 * @brief             - This function initialises the given GPIO pin
 *
 * @param[in]         - address of GPIO_Handle struct
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	// Enable GPIO Peripheral Clock
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	// 1. Configure the mode of GPIO
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		// Non-interrupt mode
		pGPIOHandle->pGPIOx->MODER &= ~(0b11 <<  pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);  // clear the required bits
		pGPIOHandle->pGPIOx->MODER |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode) << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}
	else
	{
		// GPIO Interrupt modes:

		// For interrupt mode, the GPIO pin must be in Input mode
		pGPIOHandle->pGPIOx->MODER &= ~(0b00 << 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		// 1. Now set the triggering action for the given pin on EXTI line
		// For very GPIOx port, Pin y is connected to EXTIy line
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);  // Enable RT
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);  // Disable FT
		}

		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);  // Enable FT
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);  // Disable RT
		}

		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);  // Enable RT
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);  // Enable FT
		}

		// 2. Enable the peripheral clock
		SYSCFG_PCLK_EN();

		// 3. Configure GPIO port selection in SYSCFG_EXTICRx register
		int ExtiCrNo = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;  // Get the SYSCFG_EXTICRx register
		int ExtiCrOffset = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;  // Get the offset within the SYSCFG_EXTICRx register
		SYSCFG->EXTICR[ExtiCrNo] &= ~(0b0000 << (4 *ExtiCrOffset));
		SYSCFG->EXTICR[ExtiCrNo] |= (GPIO_BASEADDR_TO_PORTCODE(pGPIOHandle->pGPIOx) << (4 *ExtiCrOffset));

		// 4. Unmask interrupt request from the line
		EXTI->IMR |= 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
	}

	// 2. Configure the speed
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0b11 <<  pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed) << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

	// 3. Configure pull-up pull-down
	pGPIOHandle->pGPIOx->PUPDR &= ~(0b11 <<  pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl) << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

	// 4. Configure the output type
	pGPIOHandle->pGPIOx->OTYPER &= ~(0b1 <<  pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;

	// 5. Configure the Alternate Functionality
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		pGPIOHandle->pGPIOx->AFR[pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8] &= ~(0b1111 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8));
		pGPIOHandle->pGPIOx->AFR[pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8] |=  ((pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode) << (4 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8)));
	}

}


/*********************************************************************
 * @fn      		  - GPIO_DeInit
 *
 * @brief             - This function De-initialises the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		} else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		} else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		} else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		} else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		} else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		} else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		} else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		} else if (pGPIOx == GPIOI)
		{
			GPIOI_PCLK_DI();
		} else if (pGPIOx == GPIOJ)
		{
			GPIOJ_PCLK_DI();
		} else if (pGPIOx == GPIOK)
		{
			GPIOK_PCLK_DI();
		}
}


/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPin
 *
 * @brief             - This function reads a given pin from a GPIOx port
 *
 * @param[in]         - base address of the gpio peripheral
 *
 * @param[in]         - pin number to be read
 *
 * @return            - pin value, 0 or 1
 *
 * @Note              -  none
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	return (uint8_t) ((pGPIOx->IDR >>  PinNumber) & 0b1);
}


/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPort
 *
 * @brief             - This function reads a given GPIOx port
 *
 * @param[in]         - base address of the gpio peripheral
 *
 * @return            - Value of the port
 *
 * @Note              -  none
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	return (uint16_t) pGPIOx->IDR;
}


/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPin
 *
 * @brief             - This function writes to a pin of the given GPIOx port
 *
 * @param[in]         - base address of the gpio peripheral
 *
 * @param[in]         - pin number to be written
 *
 * @param[in]         - Value to be written to the pin, 0 or 1
 *
 * @return            - none
 *
 * @Note              - none
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if (Value == GPIO_PIN_RESET)
		pGPIOx->ODR &= ~(1 << PinNumber);
	else
		pGPIOx->ODR |= (1 << PinNumber);

}


/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPort
 *
 * @brief             - This function writes a value to the given GPIOx port
 *
 * @param[in]         - base address of the gpio peripheral
 *
 * @param[in]         - Value to be written
 *
 * @return            - none
 *
 * @Note              -   none
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}


/*********************************************************************
 * @fn      		  - GPIO_ToggleOutputPin
 *
 * @brief             - This function a pin on a GPIOx port
 *
 * @param[in]         - base address of the gpio peripheral
 *
 * @param[in]         - Pin to be toggled
 *
 * @return            - none
 *
 * @Note              - none
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}


/*********************************************************************
 * @fn      		  - GPIO_IRQITConfig
 *
 * @brief             - Used to Enable/Disable given IRQNumber
 *
 * @param[in]         - IRQNumber
 *
 * @param[in]         - Enable / Disable
 *
 * @return            - none
 *
 * @Note              - none
 */
void GPIO_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	/*
	 * Configure the GPIO pinmode to Input
	 * Configure edge trigger to RT/FT/RFT
	 * Enable periph interrupt delivery to processor
	 * Identify IRQ number
	 * Configure IRQ priority
	 * Enable interrupt reception on that IRQ number
	 * Implement the IRQ number
	 */

	/* From Cortex-M4 documentation: Interrupt Set-Enable (NVIC_ISER) and Inerrupt Clear-Enable (ISEER_ICER)
	 * registers used to enable or disable a given IRQ number. There are 8 sets of each register (NVIC_I*ER0 - NVIC_I*ER7)
	 */

	// Identify the correct register based on the IRQ number and Enable pr Disable IRQ
	if (EnorDi == ENABLE)
	{
		if (IRQNumber <= 31)
		{
			// program NVIC_ISER0
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if (IRQNumber > 31 && IRQNumber <= 63)
		{
			// program NVIC_ISER1
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		else if (IRQNumber > 63 && IRQNumber <= 90)
		{
			// program NVIC_ISER2
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	}
	else
	{
		if (IRQNumber <= 31)
		{
			// program NVIC_ICER0
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if (IRQNumber > 31 && IRQNumber <= 63)
		{
			// program NVIC_ICER1
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}
		else if (IRQNumber > 63 && IRQNumber <= 90)
		{
			// program NVIC_ICER2
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		}
	}

}


/*********************************************************************
 * @fn      		  - GPIO_IRQPriorityConfig
 *
 * @brief             - Used to set priority of an IRQNumber
 *
 * @param[in]         - IRQNumber
 *
 * @param[in]         - IRQPriority
 *
 * @return            - none
 *
 * @Note              - none
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	// Configure IRQ Priority using NVIC_IPR0-59 register.
	// There is a 8-bit field per IRQ in the register
	int IprRegNo = IRQNumber / 4;

	// Wihtin a register, there are 4 sections. section_offset = IRQNumber % 4
	// Each section is 8-bit wide. shift_amount = section_offset * 8
	// Only the top NO_PR_BITS_IMPLEMENTED are implemented per section. shift_amount += (8 - NO_PR_BITS_IMPLEMENTED)
	int shift_amount = ((IRQNumber % 4) * 8) + (8 - NO_PR_BITS_IMPLEMENTED);

	__vo uint32_t *pIprReg = NVIC_IPR_BASEADDR + IprRegNo * 4;
	*pIprReg &= ~(0b0000 << shift_amount);
	*pIprReg |= (IRQPriority << shift_amount);
}


/*********************************************************************
 * @fn      		  - GPIO_IRQHandling
 *
 * @brief             - IRQ Handler for a given pin number
 *
 * @param[in]         - IRQNumber
 *
 * @param[in]         - IRQPriority
 *
 * @return            - none
 *
 * @Note              - none
 */
void GPIO_IRQHandling(uint8_t PinNumber)
{
	// First clear the EXTI Pending register
	if (EXTI->PR & (1 << PinNumber))
	{
		EXTI->PR |= (1 << PinNumber);
	}
}





