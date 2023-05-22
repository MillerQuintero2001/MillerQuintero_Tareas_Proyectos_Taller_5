/*
 * PLLDriver.h
 *
 *  Created on: 26/04/2023
 *      Author: MillerQuintero2001
 */


#ifndef PLLDRIVER_H_
#define PLLDRIVER_H_

#include <stm32f4xx.h>

/* Frecuencias definidas para el driver en MHz */
//#define PLL_FREQUENCY_100MHz	100
#define PLL_FREQUENCY_80MHz		80
//#define PLL_FREQUENCY_64MHz		64
//#define PLL_FREQUENCY_60MHz		60
//#define PLL_FREQUENCY_40MHz		40
//#define PLL_FREQUENCY_32MHz		32
//#define PLL_FREQUENCY_20MHz		20
//#define PLL_FREQUENCY_10MHz		10

void configPLL(uint8_t freq);
uint64_t getConfigPLL(void);

#endif /* PLLDRIVER_H_ */
