/*
 * stm32f429xx_timer_driver.h
 *
 *  Created on: 02-Aug-2023
 *      Author: rakshit
 */

#ifndef STM32F429XX_TIMER_DRIVER_H_
#define STM32F429XX_TIMER_DRIVER_H_

#include "stm32f429xx.h"

#define TIM2_BASEADDR    0x40000000
#define TIM2_CR1         (TIM2_BASEADDR)
#define TIM2_CR2         (TIM2_BASEADDR + 0x04)
#define TIM2_SMCR        (TIM2_BASEADDR + 0x08)
#define TIM2_DIER        (TIM2_BASEADDR + 0x0c)
#define TIM2_SR          (TIM2_BASEADDR + 0x10)
#define TIM2_EGR         (TIM2_BASEADDR + 0x14)
#define TIM2_CCMR1       (TIM2_BASEADDR + 0x18)
#define TIM2_CCMR2       (TIM2_BASEADDR + 0x1c)
#define TIM2_CCER        (TIM2_BASEADDR + 0x20)
#define TIM2_CNT         (TIM2_BASEADDR + 0x24)
#define TIM2_PSC         (TIM2_BASEADDR + 0x28)
#define TIM2_ARR         (TIM2_BASEADDR + 0x2c)
#define TIM2_CCR1        (TIM2_BASEADDR + 0x34)


uint32_t getTimerClkFreq();
void init_tim2_pclk();
void tim2_delay_ms_IT(uint32_t delay_ms);
void tim2_delay_ms(uint32_t delay_ms);
void tim2_init_input_capture();

#endif /* STM32F429XX_TIMER_DRIVER_H_ */
