/*
 * PLLDriver.h
 *
 *  Created on: 26/04/2023
 *      Author: MillerQuintero2001
 */


#ifndef PLLDRIVER_H_
#define PLLDRIVER_H_

#include <stm32f4xx.h>

#define HSI_FREQUENCY 	16000000
/* Frecuencias definidas para el driver en MHz */
#define PLL_FREQUENCY_80MHz		80


void configPLL(int PLL_Freq);
uint64_t getConfigPLL(void);

#endif /* PLLDRIVER_H_ */
