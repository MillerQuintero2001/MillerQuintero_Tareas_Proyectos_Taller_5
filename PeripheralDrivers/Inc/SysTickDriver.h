/*
 * SysTickDriver.h
 *
 *  Created on: 26/04/2023
 *      Author: MillerQuintero2001
 */

#ifndef SYSTICKDRIVER_H_
#define SYSTICKDRIVER_H_

#include <stm32f4xx.h>

#define SYSTICK_LOAD_VALUE_16MHz_1ms	16000	// Número de ciclos en 1ms
#define SYSTICK_LOAD_VALUE_80MHz_1ms	80000	// Número de ciclos en 1ms
#define SYSTICK_LOAD_VALUE_100MHz_1ms	100000	// Número de ciclos en 1ms

#define HSI_CLOCK_CONFIGURED		0	// 16MHz
#define HSE_CLOCK_CONFIGURED		1	// Modificable en el .c la usar cristal externo (en realidad es el AHB/8)
#define PLL_CLOCK_80_CONFIGURED		2	// 80MHz
#define PLL_CLOCK_100_CONFIGURED	3	// 100MHz


void config_SysTick_ms(uint8_t systemClock);
uint64_t getTicks_ms(void);
void delay_ms(uint32_t wait_time_ms);

#endif /* SYSTICKDRIVER_H_ */
