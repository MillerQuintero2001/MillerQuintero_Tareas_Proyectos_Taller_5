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


/* NOTA: Para poder parametrizar el problema, y usar una función configPLL más o menos cómoda
 * las frecuencias deseadas deben ser ingresadas en MHz, y no pueden exceder los 100MHz, ya que eso
 * es lo máximo permitido por el MCU, si se ingresa un valor superior a 100, el sistema queda con el HSI */

void configPLL(uint16_t PLL_Freq);
uint64_t getConfigPLL(void);

#endif /* PLLDRIVER_H_ */
