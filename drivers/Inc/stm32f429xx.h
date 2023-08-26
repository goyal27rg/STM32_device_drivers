 /*
 * stm32f429xx.h
 *
 *  Created on: Jun 5, 2022
 *      Author: rakshit
 */

#ifndef INC_STM32F429XX_H_
#define INC_STM32F429XX_H_

#include<stdint.h>
#include<stddef.h>
#include<stdlib.h>

#define __vo volatile

/***************************** PROCESSOR SPECIFIC DETAILS *******************************/
/*
 * PROCESSOR: ARM Cortex M4 (based on ARMv7E-M architecture)
 */


// NVIC Base Addresses

// NVIC_ISER* addresses
#define NVIC_ISER_BASEADDR   ((__vo uint32_t*) 0xE000E100)
#define NVIC_ISER0           NVIC_ISER_BASEADDR
#define NVIC_ISER1           ((__vo uint32_t*) 0xE000E104)
#define NVIC_ISER2           ((__vo uint32_t*) 0xE000E108)
#define NVIC_ISER3           ((__vo uint32_t*) 0xE000E10C)
// There are 4 more ISER registers but the current implementation needs only 4 registers

// NVIC_ICER* addresses
#define NVIC_ICER_BASEADDR   ((__vo uint32_t*) 0xE000E180)
#define NVIC_ICER0           NVIC_ICER_BASEADDR
#define NVIC_ICER1           ((__vo uint32_t*) 0xE000E184)
#define NVIC_ICER2           ((__vo uint32_t*) 0xE000E188)
#define NVIC_ICER3           ((__vo uint32_t*) 0xE000E18C)

// NVIC_IPR base address
#define NVIC_IPR_BASEADDR            ((__vo uint32_t*) 0xE000E400)

// No. of priority bits implemented per section in the priority register
#define NO_PR_BITS_IMPLEMENTED       4
#define NVIC_REG_SIZE_BYTE		     4

/*
 * Flash and Main memory base addresses
 */

#define FLASH_BASEADDR          0x80000000U  // Flash is also called Main Memory
#define SRAM1_BASEADDR          0x20000000U
#define SRAM1_SIZE              0x1C000  // 112KB
#define SRAM2_BASEADDR          (SRAM1_BASEADDR + SRAM1_SIZE)
#define ROM_BASEADDR            0x1FFF0000 // ROM is also called System Memory

#define SRAM                    SRAM1_BASEADDR

/***************************** BUS ADDRESSES *******************************/

// Bus domain base addresses
#define PERIPH_BASE             0x40000000U
#define APB1PERIPH_BASEADDR     PERIPH_BASE
#define APB2PERIPH_BASEADDR     0x40010000U
#define AHB1PERIPH_BASEADDR     0x40020000U
#define AHB2PERIPH_BASEADDR     0x50000000U
#define AHB3PERIPH_BASEADDR     0xA0000000U

// Peripheral base addresses on AHB1 bus
#define GPIOA_BASEADDR          (AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR          (AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR          (AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR          (AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR          (AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR          (AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR          (AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR          (AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR          (AHB1PERIPH_BASEADDR + 0x2000)
#define GPIOJ_BASEADDR          (AHB1PERIPH_BASEADDR + 0x2400)
#define GPIOK_BASEADDR          (AHB1PERIPH_BASEADDR + 0x2800)
#define DMA1_BASEADDR           (AHB1PERIPH_BASEADDR + 0x6000)
#define DMA2_BASEADDR           (AHB1PERIPH_BASEADDR + 0x6400)

// Peripheral base addresses on APB1 bus
#define SPI2_BASEADDR           (APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR           (APB1PERIPH_BASEADDR + 0x3C00)
#define USART2_BASEADDR         (APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR         (APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR          (APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR          (APB1PERIPH_BASEADDR + 0x5000)
#define I2C1_BASEADDR           (APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR           (APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR           (APB1PERIPH_BASEADDR + 0x5C00)

// Peripheral base addresses on APB2 bus
#define USART1_BASEADDR         (APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR         (APB2PERIPH_BASEADDR + 0x1400)
#define SPI1_BASEADDR           (APB2PERIPH_BASEADDR + 0x3000)
#define SPI4_BASEADDR           (APB2PERIPH_BASEADDR + 0x3400)
#define SYSCFG_BASEADDR         (APB2PERIPH_BASEADDR + 0x3800)
#define EXTI_BASEADDR           (APB2PERIPH_BASEADDR + 0x3C00)
#define SPI5_BASEADDR           (APB2PERIPH_BASEADDR + 0x5000)
#define SPI6_BASEADDR           (APB2PERIPH_BASEADDR + 0x5400)


// RCC base address
#define RCC_BASEADDR            (AHB1PERIPH_BASEADDR + 0x3800)

// ADC base address
#define ADCPERIPH_BASEADDR      (APB2PERIPH_BASEADDR + 0x2000)

#define ADC1_BASEADDR           (ADCPERIPH_BASEADDR)
#define ADC2_BASEADDR           (ADCPERIPH_BASEADDR + 0x100)
#define ADC3_BASEADDR           (ADCPERIPH_BASEADDR + 0x200)
#define ADCCOMMON_BASEADDR      (ADCPERIPH_BASEADDR + 0x300)


/***************************** Peripheral Register Definition Structures *******************************/
typedef struct {
	__vo uint32_t MODER;
	__vo uint32_t OTYPER;
	__vo uint32_t OSPEEDR;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFR[2];  // Create array for AFRL / AFRH regs

}GPIO_RegDef_t;


typedef struct {
	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	__vo uint32_t AHB3RSTR;
	uint32_t RESERVED0;
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	uint32_t RESERVED1[2];
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	__vo uint32_t AHB3ENR;
	uint32_t RESERVED1_1;
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	uint32_t RESERVED2[2];
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	__vo uint32_t AHB3LPENR;
	uint32_t RESERVED3;
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	uint32_t RESERVED4[2];
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	uint32_t RESERVED5[2];
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;
	__vo uint32_t PLLSAICFGR;
	__vo uint32_t DCKCFGR;
}RCC_RegDef_t;


typedef struct {
	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;
}EXTI_RegDef_t;


typedef struct {
	__vo uint32_t MEMRMP;
	__vo uint32_t PMC;
	__vo uint32_t EXTICR[4];
	uint32_t RESERVED[2];
	__vo uint32_t CMPCR;
}SYSCFG_RegDef_t;


typedef struct {
	__vo uint32_t SPI_CR1;
	__vo uint32_t SPI_CR2;
	__vo uint32_t SPI_SR;
	__vo uint32_t SPI_DR;
	__vo uint32_t SPI_CRCPR;
	__vo uint32_t SPI_RXCRCR;
	__vo uint32_t SPI_TXCRCR;
	__vo uint32_t SPI_I2SCFGR;
	__vo uint32_t SPI_I2SPR;
}SPI_RegDef_t;


typedef struct {
	__vo uint32_t I2C_CR1;
	__vo uint32_t I2C_CR2;
	__vo uint32_t I2C_OAR1;
	__vo uint32_t I2C_OAR2;
	__vo uint32_t I2C_DR;
	__vo uint32_t I2C_SR1;
	__vo uint32_t I2C_SR2;
	__vo uint32_t I2C_CCR;
	__vo uint32_t I2C_TRISE;
	__vo uint32_t I2C_FLTR;

}I2C_RegDef_t;


typedef struct {
	__vo uint32_t USART_SR;
	__vo uint32_t USART_DR;
	__vo uint32_t USART_BRR;
	__vo uint32_t USART_CR1;
	__vo uint32_t USART_CR2;
	__vo uint32_t USART_CR3;
	__vo uint32_t USART_GTPR;
}USART_RegDef_t;

typedef struct {
	__vo uint32_t ADC_SR;
	__vo uint32_t ADC_CR1;
	__vo uint32_t ADC_CR2;
	__vo uint32_t ADC_SMPR1;
	__vo uint32_t ADC_SMPR2;
	__vo uint32_t ADC_JOFR1;
	__vo uint32_t ADC_JOFR2;
	__vo uint32_t ADC_JOFR3;
	__vo uint32_t ADC_JOFR4;
	__vo uint32_t ADC_HTR;
	__vo uint32_t ADC_LTR;
	__vo uint32_t ADC_SQR1;
	__vo uint32_t ADC_SQR2;
	__vo uint32_t ADC_SQR3;
	__vo uint32_t ADC_JSQR;
	__vo uint32_t ADC_JDR1;
	__vo uint32_t ADC_JDR2;
	__vo uint32_t ADC_JDR3;
	__vo uint32_t ADC_JDR4;
	__vo uint32_t ADC_DR;
	__vo uint32_t ADC_CSR;
	__vo uint32_t ADC_CCR;
	__vo uint32_t ADC_CDR;
}ADC_RegDef_t;

typedef struct {
    __vo uint32_t DMA_LISR;
    __vo uint32_t DMA_HISR;
    __vo uint32_t DMA_LIFCR;
    __vo uint32_t DMA_HIFCR;

    __vo uint32_t DMA_S0CR;
    __vo uint32_t DMA_S0NDTR;
    __vo uint32_t DMA_S0PAR;
    __vo uint32_t DMA_S0M0AR;
    __vo uint32_t DMA_S0M1AR;
    __vo uint32_t DMA_S0FCR;

    __vo uint32_t DMA_S1CR;
    __vo uint32_t DMA_S1NDTR;
    __vo uint32_t DMA_S1PAR;
    __vo uint32_t DMA_S1M0AR;
    __vo uint32_t DMA_S1M1AR;
    __vo uint32_t DMA_S1FCR;

    __vo uint32_t DMA_S2CR;
    __vo uint32_t DMA_S2NDTR;
    __vo uint32_t DMA_S2PAR;
    __vo uint32_t DMA_S2M0AR;
    __vo uint32_t DMA_S2M1AR;
    __vo uint32_t DMA_S2FCR;

    __vo uint32_t DMA_S3CR;
    __vo uint32_t DMA_S3NDTR;
    __vo uint32_t DMA_S3PAR;
    __vo uint32_t DMA_S3M0AR;
    __vo uint32_t DMA_S3M1AR;
    __vo uint32_t DMA_S3FCR;

    __vo uint32_t DMA_S4CR;
    __vo uint32_t DMA_S4NDTR;
    __vo uint32_t DMA_S4PAR;
    __vo uint32_t DMA_S4M0AR;
    __vo uint32_t DMA_S4M1AR;
    __vo uint32_t DMA_S4FCR;

    __vo uint32_t DMA_S5CR;
    __vo uint32_t DMA_S5NDTR;
    __vo uint32_t DMA_S5PAR;
    __vo uint32_t DMA_S5M0AR;
    __vo uint32_t DMA_S5M1AR;
    __vo uint32_t DMA_S5FCR;

    __vo uint32_t DMA_S6CR;
    __vo uint32_t DMA_S6NDTR;
    __vo uint32_t DMA_S6PAR;
    __vo uint32_t DMA_S6M0AR;
    __vo uint32_t DMA_S6M1AR;
    __vo uint32_t DMA_S6FCR;

    __vo uint32_t DMA_S7CR;
    __vo uint32_t DMA_S7NDTR;
    __vo uint32_t DMA_S7PAR;
    __vo uint32_t DMA_S7M0AR;
    __vo uint32_t DMA_S7M1AR;
    __vo uint32_t DMA_S7FCR;
} DMA_RegDef_t;

/*
 * Peripheral definitions (Peripheral base addresses typecasted to xxx_RegDef_t )
 */

#define GPIOA        ((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB        ((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC        ((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD        ((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE        ((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOF        ((GPIO_RegDef_t*) GPIOF_BASEADDR)
#define GPIOG        ((GPIO_RegDef_t*) GPIOG_BASEADDR)
#define GPIOH        ((GPIO_RegDef_t*) GPIOH_BASEADDR)
#define GPIOI        ((GPIO_RegDef_t*) GPIOI_BASEADDR)
#define GPIOJ        ((GPIO_RegDef_t*) GPIOJ_BASEADDR)
#define GPIOK        ((GPIO_RegDef_t*) GPIOK_BASEADDR)

#define RCC          ((RCC_RegDef_t*) RCC_BASEADDR)

#define EXTI         ((EXTI_RegDef_t*) EXTI_BASEADDR)

#define SYSCFG       ((SYSCFG_RegDef_t*) SYSCFG_BASEADDR)

#define SPI1         ((SPI_RegDef_t*) SPI1_BASEADDR)
#define SPI2         ((SPI_RegDef_t*) SPI2_BASEADDR)
#define SPI3         ((SPI_RegDef_t*) SPI3_BASEADDR)
#define SPI4         ((SPI_RegDef_t*) SPI4_BASEADDR)
#define SPI5         ((SPI_RegDef_t*) SPI5_BASEADDR)
#define SPI6         ((SPI_RegDef_t*) SPI6_BASEADDR)

#define I2C1         ((I2C_RegDef_t*) I2C1_BASEADDR)
#define I2C2         ((I2C_RegDef_t*) I2C2_BASEADDR)
#define I2C3         ((I2C_RegDef_t*) I2C3_BASEADDR)

#define USART1       ((USART_RegDef_t*) USART1_BASEADDR)
#define USART2       ((USART_RegDef_t*) USART2_BASEADDR)
#define USART3       ((USART_RegDef_t*) USART3_BASEADDR)
#define UART4        ((USART_RegDef_t*) UART4_BASEADDR)
#define UART5        ((USART_RegDef_t*) UART5_BASEADDR)
#define USART6       ((USART_RegDef_t*) USART6_BASEADDR)

#define ADC1         ((ADC_RegDef_t*) ADC1_BASEADDR)
#define ADC2         ((ADC_RegDef_t*) ADC2_BASEADDR)
#define ADC3         ((ADC_RegDef_t*) ADC3_BASEADDR)
#define ADCCOMMON    ((ADC_RegDef_t*) ADCCOMMON_BASEADDR)

#define DMA1         ((DMA_RegDef_t*) DMA1_BASEADDR)
#define DMA2         ((DMA_RegDef_t*) DMA2_BASEADDR)


/*
 * Clock enable macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN() (RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN() (RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN() (RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN() (RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN() (RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN() (RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN() (RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN() (RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN() (RCC->AHB1ENR |= (1 << 8))
#define GPIOJ_PCLK_EN() (RCC->AHB1ENR |= (1 << 9))
#define GPIOK_PCLK_EN() (RCC->AHB1ENR |= (1 << 10))


/*
 * Clock enable macros for I2Cx peripherals
 */

#define I2C1_PCLK_EN() (RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN() (RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN() (RCC->APB1ENR |= (1 << 23))


/*
 * Clock enable macros for SPIx peripherals
 */

#define SPI1_PCLK_EN() (RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN() (RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN() (RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN() (RCC->APB2ENR |= (1 << 13))
#define SPI5_PCLK_EN() (RCC->APB2ENR |= (1 << 20))
#define SPI6_PCLK_EN() (RCC->APB2ENR |= (1 << 21))


/*
 * Clock enable macros for UARTx/USARTx peripherals
 */

#define USART1_PCLK_EN() (RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN() (RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN() (RCC->APB1ENR |= (1 << 18))
#define UART4_PCLK_EN()  (RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN()  (RCC->APB1ENR |= (1 << 20))
#define USART6_PCLK_EN() (RCC->APB2ENR |= (1 << 5))
#define UART7_PCLK_EN()  (RCC->APB1ENR |= (1 << 30))
#define UART8_PCLK_EN()  (RCC->APB1ENR |= (1 << 31))


/*
 * Clock enable macros for DMA peripheral
 */

#define DMA1_PCLK_EN() (RCC->AHB1ENR |= (1 << 21))
#define DMA2_PCLK_EN() (RCC->AHB1ENR |= (1 << 22))


/*
 * Clock enable macros for SYSCFG peripheral
 */

#define SYSCFG_PCLK_EN() (RCC->APB2ENR |= (1 << 14))

/*
 * Clock enable macros for ADC peripherals
 */

#define ADC1_PCLK_EN() (RCC->APB2ENR |= (1 << 8))
#define ADC2_PCLK_EN() (RCC->APB2ENR |= (1 << 9))
#define ADC3_PCLK_EN() (RCC->APB2ENR |= (1 << 10))

/*
 * Clock disable macros for GPIOx peripherals
 */

#define GPIOA_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 8))
#define GPIOJ_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 9))
#define GPIOK_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 10))


/*
 * Clock disable macros for I2Cx peripherals
 */

#define I2C1_PCLK_DI() (RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI() (RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI() (RCC->APB1ENR &= ~(1 << 23))


/*
 * Clock disable macros for SPIx peripherals
 */

#define SPI1_PCLK_DI() (RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI() (RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI() (RCC->APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_DI() (RCC->APB2ENR &= ~(1 << 13))
#define SPI5_PCLK_DI() (RCC->APB2ENR &= ~(1 << 20))
#define SPI6_PCLK_DI() (RCC->APB2ENR &= ~(1 << 21))


/*
 * Clock disable macros for UARTx/USARTx peripherals
 */

#define USART1_PCLK_DI() (RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI() (RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI() (RCC->APB1ENR &= ~(1 << 18))
#define UART4_PCLK_DI()  (RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DI()  (RCC->APB1ENR &= ~(1 << 20))
#define USART6_PCLK_DI() (RCC->APB2ENR &= ~(1 << 5))
#define UART7_PCLK_DI()  (RCC->APB1ENR &= ~(1 << 30))
#define UART8_PCLK_DI()  (RCC->APB1ENR &= ~(1 << 31))

/*
 * Clock disable macros for ADC peripherals
 */

#define ADC1_PCLK_DI() (RCC->APB2ENR &= ~(1 << 8))
#define ADC2_PCLK_DI() (RCC->APB2ENR &= ~(1 << 9))
#define ADC3_PCLK_DI() (RCC->APB2ENR &= ~(1 << 10))

/*
 * Clock disable macros for DMA peripheral
 */

#define DMA1_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 21))
#define DMA2_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 22))


/*
 * Peripheral disable macros for GPIOx
 */

#define GPIOA_REG_RESET() do{ (RCC->AHB1RSTR |= (1 << 0));  (RCC->AHB1RSTR &= ~(1 << 0)); } while(0)
#define GPIOB_REG_RESET() do{ (RCC->AHB1RSTR |= (1 << 1));  (RCC->AHB1RSTR &= ~(1 << 1)); } while(0)
#define GPIOC_REG_RESET() do{ (RCC->AHB1RSTR |= (1 << 2));  (RCC->AHB1RSTR &= ~(1 << 2)); } while(0)
#define GPIOD_REG_RESET() do{ (RCC->AHB1RSTR |= (1 << 3));  (RCC->AHB1RSTR &= ~(1 << 3)); } while(0)
#define GPIOE_REG_RESET() do{ (RCC->AHB1RSTR |= (1 << 4));  (RCC->AHB1RSTR &= ~(1 << 4)); } while(0)
#define GPIOF_REG_RESET() do{ (RCC->AHB1RSTR |= (1 << 5));  (RCC->AHB1RSTR &= ~(1 << 5)); } while(0)
#define GPIOG_REG_RESET() do{ (RCC->AHB1RSTR |= (1 << 6));  (RCC->AHB1RSTR &= ~(1 << 6)); } while(0)
#define GPIOH_REG_RESET() do{ (RCC->AHB1RSTR |= (1 << 7));  (RCC->AHB1RSTR &= ~(1 << 7)); } while(0)
#define GPIOI_REG_RESET() do{ (RCC->AHB1RSTR |= (1 << 8));  (RCC->AHB1RSTR &= ~(1 << 8)); } while(0)
#define GPIOJ_REG_RESET() do{ (RCC->AHB1RSTR |= (1 << 9));  (RCC->AHB1RSTR &= ~(1 << 9)); } while(0)
#define GPIOK_REG_RESET() do{ (RCC->AHB1RSTR |= (1 << 10));  (RCC->AHB1RSTR &= ~(1 << 10)); } while(0)


/*
 * IRQ numbers of STM32F42xxx MCU
 */

#define IRQ_NO_EXTI0         6
#define IRQ_NO_EXTI1         7
#define IRQ_NO_EXTI2         8
#define IRQ_NO_EXTI3         9
#define IRQ_NO_EXTI4         10
#define IRQ_NO_EXTI9_5       23
#define IRQ_NO_EXTI10_15     40
#define IRQ_NO_SPI1          35
#define IRQ_NO_SPI2          36
#define IRQ_NO_SPI3          51
#define IRQ_NO_SPI4          84
#define IRQ_NO_SPI5          85
#define IRQ_NO_SPI6          86
#define IRQ_NO_I2C1_EV       31
#define IRQ_NO_I2C1_ER       32
#define IRQ_NO_I2C2_EV       33
#define IRQ_NO_I2C2_ER       34
#define IRQ_NO_I2C3_EV       72
#define IRQ_NO_I2C3_ER       73

/*
 * IRQ priorities STM32F42xxx MCU
 */

#define NVIC_IRQ_PRIO0       0
#define NVIC_IRQ_PRIO1       1
#define NVIC_IRQ_PRIO2       2
#define NVIC_IRQ_PRIO3       3
#define NVIC_IRQ_PRIO4       4
#define NVIC_IRQ_PRIO5       5
#define NVIC_IRQ_PRIO6       6
#define NVIC_IRQ_PRIO7       7
#define NVIC_IRQ_PRIO8       8
#define NVIC_IRQ_PRIO9       9
#define NVIC_IRQ_PRIO10      10
#define NVIC_IRQ_PRIO11      11
#define NVIC_IRQ_PRIO12      12
#define NVIC_IRQ_PRIO13      13
#define NVIC_IRQ_PRIO14      14
#define NVIC_IRQ_PRIO15      15



// Generic Macros

#define ENABLE               1
#define DISABLE              0
#define SET                  ENABLE
#define RESET                DISABLE
#define GPIO_PIN_SET         SET
#define GPIO_PIN_RESET       RESET

#define GPIO_BASEADDR_TO_PORTCODE(x) ((x == GPIOA) ? 0 :\
                                       (x == GPIOB) ? 1 :\
                                       (x == GPIOC) ? 2 :\
                                       (x == GPIOD) ? 3 :\
                                       (x == GPIOE) ? 4 :\
                                       (x == GPIOF) ? 5 :\
                                       (x == GPIOG) ? 6 :\
                                       (x == GPIOH) ? 7 : 0)


// Bit position definitions for SPI
#define SPI_CR1_CPHA        0
#define SPI_CR1_CPOL        1
#define SPI_CR1_MSTR		2
#define SPI_CR1_BR          3
#define SPI_CR1_SPE		    6
#define SPI_CR1_LSBFIRST    7
#define SPI_CR1_SSI         8
#define SPI_CR1_SSM         9
#define SPI_CR1_RXONLY	    10
#define SPI_CR1_DFF         11
#define SPI_CR1_CRCNEXT     12
#define SPI_CR1_CRCEN       13
#define SPI_CR1_BIDIOE      14
#define SPI_CR1_BIDIMODE    15

#define SPI_CR2_SSOE        2
#define SPI_CR2_ERRIE       5
#define SPI_CR2_RXNEIE      6
#define SPI_CR2_TXEIE       7

#define SPI_SR_RXNE         0
#define SPI_SR_TXE          1
#define SPI_SR_OVR          6
#define SPI_SR_BSY          7

// Bit position definitions for I2C
#define I2C_CR1_PE          0
#define I2C_CR1_SMBUS       1
#define I2C_CR1_SMBTYPE     3
#define I2C_CR1_ENARP       4
#define I2C_CR1_ENPEC       5
#define I2C_CR1_ENGC        6
#define I2C_CR1_NOSTRECH    7
#define I2C_CR1_START       8
#define I2C_CR1_STOP        9
#define I2C_CR1_ACK         10
#define I2C_CR1_POS         11
#define I2C_CR1_PEC         12
#define I2C_CR1_ALERT       13
#define I2C_CR1_SWRST       14

#define I2C_CR2_FREQ        0
#define I2C_CR2_ITERREN     8
#define I2C_CR2_ITEVTEN     9
#define I2C_CR2_ITBUFEN     10
#define I2C_CR2_DMAEN       11
#define I2C_CR2_LAST        12

#define I2C_SR1_SB          0
#define I2C_SR1_ADDR        1
#define I2C_SR1_BTF         2
#define I2C_SR1_STOPF       4
#define I2C_SR1_RXNE        6
#define I2C_SR1_TXE         7
#define I2C_SR1_ARLO        9
#define I2C_SR1_AF          10
#define I2C_SR1_OVR         11
#define I2C_SR1_TIMEOUT     14

#define I2C_SR2_MSL         0
#define I2C_SR2_BUSY        1
#define I2C_SR2_TRA         2
#define I2C_SR2_RES         3
#define I2C_SR2_GENCALL     4
#define I2C_SR2_SMBDEFAULT  5
#define I2C_SR2_SMBHOST     6
#define I2C_SR2_DUALF       7
#define I2C_SR2_PEC         8

// Bit position definitions for USART
#define USART_SR_PE         0
#define USART_SR_FE         1
#define USART_SR_NF         2
#define USART_SR_ORE        3
#define USART_SR_IDLE       4
#define USART_SR_RXNE       5
#define USART_SR_TC         6
#define USART_SR_TXE        7
#define USART_SR_LBD        8
#define USART_SR_CTS        9

#define USART_BRR_DIV_Fraction  0
#define USART_BRR_DIV_Mantissa  4

#define USART_CR1_SBK           0
#define USART_CR1_RWU           1
#define USART_CR1_RE            2
#define USART_CR1_TE            3
#define USART_CR1_IDLEIE        4
#define USART_CR1_RXNEIE        5
#define USART_CR1_TCIE          6
#define USART_CR1_TXEIE         7
#define USART_CR1_PEIE          8
#define USART_CR1_PS            9
#define USART_CR1_PCE           10
#define USART_CR1_WAKE          11
#define USART_CR1_M             12
#define USART_CR1_UE            13
#define USART_CR1_OVER8         15

#define USART_CR2_ADD           0
#define USART_CR2_LBDL          5
#define USART_CR2_LBDIE         6
#define USART_CR2_LBCL          8
#define USART_CR2_CPHA          9
#define USART_CR2_CPOL          10
#define USART_CR2_CLKEN         11
#define USART_CR2_STOP          12
#define USART_CR2_LINEN         14

#define USART_CR3_EIE           0
#define USART_CR3_IREN          1
#define USART_CR3_IRLP          2
#define USART_CR3_HDSEL         3
#define USART_CR3_NACK          4
#define USART_CR3_SCEN          5
#define USART_CR3_DMAR          6
#define USART_CR3_DMAT          7
#define USART_CR3_RTSE          8
#define USART_CR3_CTSE          9
#define USART_CR3_CTSIE         10
#define USART_CR3_ONEBIT        11

#define USART_GTPR_PSC          0
#define USART_GTPR_GT           8

// Bit position definitions for ADC
#define ADC_SR_AWD              0
#define ADC_SR_EOC              1
#define ADC_SR_JEOC             2
#define ADC_SR_JSTRT            3
#define ADC_SR_STRT             4
#define ADC_SR_OVR              5

#define ADC_CR1_AWDCH           0
#define ADC_CR1_EOCIE           5
#define ADC_CR1_AWDIE           6
#define ADC_CR1_JEOCIE          7
#define ADC_CR1_SCAN            8
#define ADC_CR1_AWDSGL          9
#define ADC_CR1_JAUTO           10
#define ADC_CR1_DISCEN          11
#define ADC_CR1_JDISCEN         12
#define ADC_CR1_DISCNUM         13
#define ADC_CR1_JAWDEN          22
#define ADC_CR1_AWDEN           23
#define ADC_CR1_OVRIE           26

#define ADC_CR2_ADON            0
#define ADC_CR2_CONT            1
#define ADC_CR2_DMA             8
#define ADC_CR2_DDS             9
#define ADC_CR2_EOCS            10
#define ADC_CR2_ALIGN           11
#define ADC_CR2_JEXTSEL         16
#define ADC_CR2_JEXTEN          20
#define ADC_CR2_JSWSTART        22
#define ADC_CR2_EXTSEL          24
#define ADC_CR2_EXTEN           28
#define ADC_CR2_SWSTART         30

#define ADC_SMPR1_SMP10         0
#define ADC_SMPR1_SMP11         3
#define ADC_SMPR1_SMP12         6
#define ADC_SMPR1_SMP13         9
#define ADC_SMPR1_SMP14         12
#define ADC_SMPR1_SMP15         15
#define ADC_SMPR1_SMP16         18
#define ADC_SMPR1_SMP17         21
#define ADC_SMPR1_SMP18         24

#define ADC_SMPR2_SMP0          0
#define ADC_SMPR2_SMP1          3
#define ADC_SMPR2_SMP2          6
#define ADC_SMPR2_SMP3          9
#define ADC_SMPR2_SMP4          12
#define ADC_SMPR2_SMP5          15
#define ADC_SMPR2_SMP6          18
#define ADC_SMPR2_SMP7          21
#define ADC_SMPR2_SMP8          24
#define ADC_SMPR2_SMP9          27

#define ADC_SQR1_L              20


// DMA register bit positions

#define DMA_LISR_TCIF3                           27
#define DMA_LISR_HTIF3                           26
#define DMA_LISR_TEIF3                           25
#define DMA_LISR_DMEIF3                          24
#define DMA_LISR_FEIF3                           22
#define DMA_LISR_TCIF2                           21
#define DMA_LISR_HTIF2                           20
#define DMA_LISR_TEIF2                           19
#define DMA_LISR_DMEIF2                          18
#define DMA_LISR_FEIF2                           16
#define DMA_LISR_TCIF1                           11
#define DMA_LISR_HTIF1                           10
#define DMA_LISR_TEIF1                           9
#define DMA_LISR_DMEIF1                          8
#define DMA_LISR_FEIF1                           6
#define DMA_LISR_TCIF0                           5
#define DMA_LISR_HTIF0                           4
#define DMA_LISR_TEIF0                           3
#define DMA_LISR_DMEIF0                          2
#define DMA_LISR_FEIF0                           0


#define DMA_HISR_TCIF7                           27
#define DMA_HISR_HTIF7                           26
#define DMA_HISR_TEIF7                           25
#define DMA_HISR_DMEIF7                          24
#define DMA_HISR_FEIF7                           22
#define DMA_HISR_TCIF6                           21
#define DMA_HISR_HTIF6                           20
#define DMA_HISR_TEIF6                           19
#define DMA_HISR_DMEIF6                          18
#define DMA_HISR_FEIF6                           16
#define DMA_HISR_TCIF5                           11
#define DMA_HISR_HTIF5                           10
#define DMA_HISR_TEIF5                           9 
#define DMA_HISR_DMEIF5                          8 
#define DMA_HISR_FEIF5                           6 
#define DMA_HISR_TCIF4                           5 
#define DMA_HISR_HTIF4                           4 
#define DMA_HISR_TEIF4                           3 
#define DMA_HISR_DMEIF4                          2 
#define DMA_HISR_FEIF4                           0

 
#define DMA_LIFCR_CTCIF3                         27
#define DMA_LIFCR_CHTIF3                         26
#define DMA_LIFCR_TEIF3                          25
#define DMA_LIFCR_CDMEIF3                        24
#define DMA_LIFCR_CFEIF3                         22
#define DMA_LIFCR_CTCIF2                         21
#define DMA_LIFCR_CHTIF2                         20
#define DMA_LIFCR_CTEIF2                         19
#define DMA_LIFCR_CDMEIF2                        18
#define DMA_LIFCR_CFEIF2                         16
#define DMA_LIFCR_CTCIF1                         11
#define DMA_LIFCR_CHTIF1                         10
#define DMA_LIFCR_CTEIF1                         9 
#define DMA_LIFCR_CDMEIF1                        8 
#define DMA_LIFCR_CFEIF1                         6 
#define DMA_LIFCR_CTCIF0                         5 
#define DMA_LIFCR_CHTIF0                         4 
#define DMA_LIFCR_CTEIF0                         3 
#define DMA_LIFCR_CDMEIF0                        2 
#define DMA_LIFCR_CFEIF0                         0


#define DMA_HIFCR_CTCIF7                         27
#define DMA_HIFCR_CHTIF7                         26
#define DMA_HIFCR_CTEIF7                         25
#define DMA_HIFCR_CDMEIF7                        24
#define DMA_HIFCR_CFEIF7                         22
#define DMA_HIFCR_CTCIF6                         21
#define DMA_HIFCR_CHTIF6                         20
#define DMA_HIFCR_CTEIF6                         19
#define DMA_HIFCR_CDMEIF6                        18
#define DMA_HIFCR_CFEIF6                         16
#define DMA_HIFCR_CTCIF5                         11
#define DMA_HIFCR_CHTIF5                         10
#define DMA_HIFCR_CTEIF5                         9 
#define DMA_HIFCR_CDMEIF5                        8 
#define DMA_HIFCR_CFEIF5                         6 
#define DMA_HIFCR_CTCIF4                         5 
#define DMA_HIFCR_CHTIF4                         4 
#define DMA_HIFCR_CTEIF4                         3 
#define DMA_HIFCR_CDMEIF4                        2 
#define DMA_HIFCR_CFEIF4                         0

#define DMA_SxCR_CHSEL                           25
#define DMA_SxCR_MBURST                          23
#define DMA_SxCR_PBURST                          22
#define DMA_SxCR_CT                              19
#define DMA_SxCR_DBM                             18
#define DMA_SxCR_PL                              16
#define DMA_SxCR_PINCOS                          15
#define DMA_SxCR_MSIZE                           13
#define DMA_SxCR_PSIZE                           11
#define DMA_SxCR_MINC                            10
#define DMA_SxCR_PINC                            9
#define DMA_SxCR_CIRC                            8
#define DMA_SxCR_DIR                             6
#define DMA_SxCR_PFCTRL                          5
#define DMA_SxCR_TCIE                            4
#define DMA_SxCR_HTIE                            3
#define DMA_SxCR_TEIE                            2
#define DMA_SxCR_DMEIE                           1
#define DMA_SxCR_EN                              0

#define DMA_SxNDTR_NDT                           0

#define DMA_SxFCR_FEIE                           7
#define DMA_SxFCR_FS                             3
#define DMA_SxFCR_DMDIS                          2
#define DMA_SxFCR_FTH                            0


#include "stm32f429xx_gpio_driver.h"
#include "stm32f429xx_spi_driver.h"
#include "stm32f429xx_i2c_driver.h"
#include "stm32f429xx_usart_driver.h"
#include "stm32f429xx_adc_driver.h"
#include "stm32f429xx_dma_driver.h"
#include "stm32f429xx_timer_driver.h"

void sw_delay_ms(int delay);
void NVIC_IRQ_EnDi(uint8_t IRQNumber, uint8_t EnOrDi);

#endif /* INC_STM32F429XX_H_ */
