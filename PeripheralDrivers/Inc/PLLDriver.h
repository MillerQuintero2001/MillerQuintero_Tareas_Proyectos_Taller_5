/*
 * PLLDriver.h
 *
 *  Created on: 26/04/2023
 *      Author: MillerQuintero2001
 */


#ifndef PLLDRIVER_H_
#define PLLDRIVER_H_

#include <stm32f4xx.h>
#include <stdint.h>

#define HSI_FREQUENCY 	16000000

#define MCO_HSI_CLOCK 		0
#define MCO1_LSE_CLOCK 		1
#define MCO2_PLLI2S_CLOCK 	1
#define MCO_HSE_CLOCK		2
#define MCO_PLL_CLOCK 		3

#define MCO_DIVIDED_BY_1	0
#define MCO_DIVIDED_BY_2	0b100
#define MCO_DIVIDED_BY_3	0b101
#define MCO_DIVIDED_BY_4	0b110
#define MCO_DIVIDED_BY_5	0b111


/* NOTA: Para poder parametrizar el problema, y usar una función configPLL más o menos cómoda
 * las frecuencias deseadas deben ser ingresadas en MHz, y no pueden exceder los 100MHz, ya que eso
 * es lo máximo permitido por el MCU, si se ingresa un valor superior a 100, el sistema queda con el HSI */

void configPLL(uint8_t PLLFreqMHz);
uint32_t getConfigPLL(void);
void changeMCO1(uint8_t sourceClock, uint8_t preEscaler);
void changeMCO2(uint8_t sourceClock, uint8_t preEscaler);

#endif /* PLLDRIVER_H_ */
