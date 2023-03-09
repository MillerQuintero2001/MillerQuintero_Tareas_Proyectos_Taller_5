/*
 * GPIOxDriver.c
 *
 *  Created on: 9/03/2023
 *      Author: miller
 *
 *  Este archivo es la parte del programa donde escribimos adecuadamente el control,
 *  para que sea lo más genérico posible, de forma que independiente del puerto GPIO y
 *  el PIN seleccionado, el programa se ejecute y configure todo correctamente.
 *
 */

#include "GPIOxDriver.h"

/**
 * Para cualquier periférico, hay varios pasos que siempre se deben seguir en un
 * orden estricto para poder que el sistema permita configurar el periférico X.
 * Lo primero y mas importante es activar la señal del reloj principal hacia ese
 * elemento específico (relacionado con el periférico RCC), a esto lo llamaremos
 * simplemente "activar el periférico o activar la señal de reloj del periférico)
 */
void GPIO_Config (GPIO_Handler_t *pGPIOHandler){

	// Variable para hacer todo paso a paso
	uint32_t auxConfig = 0;
	uint32_t auxPosition = 0;

	// 1) Activar el periférico
	// Verificamos para GPIOA
	if(pGPIOHandler->pGPIOx == GPIOA){
		// Escribimos 1 (SET) en la posición correspondiente al GPIOA
		RCC->AHB1ENR |= (SET << RCC_AHB1ENR_GPIOA_EN);
	}
	// Verificamos para GPIOB
	else if(pGPIOHandler->pGPIOx == GPIOB){
		// Escribimos 1 (SET) en la posición correspondiente al GPIOB
		RCC->AHB1ENR |= (SET << RCC_AHB1ENR_GPIOB_EN);
	}
	// Verificamos para GPIOC
	else if(pGPIOHandler->pGPIOx == GPIOC){
		// Escribimos 1 (SET) en la posición correspondiente al GPIOC
		RCC->AHB1ENR |= (SET << RCC_AHB1ENR_GPIOC_EN);
		}
	// Verificamos para GPIOB
	else if(pGPIOHandler->pGPIOx == GPIOD){
		// Escribimos 1 (SET) en la posición correspondiente al GPIOD
		RCC->AHB1ENR |= (SET << RCC_AHB1ENR_GPIOD_EN);
		}
	// Verificamos para GPIOE
	else if(pGPIOHandler->pGPIOx == GPIOE){
		// Escribimos 1 (SET) en la posición correspondiente al GPIOE
		RCC->AHB1ENR |= (SET << RCC_AHB1ENR_GPIOE_EN);
		}
	// Verificamos para GPIOH
		else if(pGPIOHandler->pGPIOx == GPIOH){
			// Escribimos 1 (SET) en la posición correspondiente al GPIOB
			RCC->AHB1ENR |= (SET << RCC_AHB1ENR_GPIOH_EN);
		}
}
