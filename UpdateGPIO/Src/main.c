/**
 ******************************************************************************
 * @file           : main.c
 * @author         : MillerQUintero2001
 * @brief          : Configuracion Básica de un proyecto
 ******************************************************************************
 * Generacion del archivo de configuración por defecto
 * como plantilla para los proyectos funcionales
 ******************************************************************************
 */

#include <stm32f4xx.h>
#include <stdint.h>
#include "GPIOxDriver.h"

//Definición de un elemento
GPIO_Handler_t handlerLed2 = {0}; //PA5

/**
 * Funcion principal del programa
 * Esta función es el corazón del programa
 *
 */
int main(void)
{
	// Configurando el LED_2 -> PA5
	handlerLed2.pGPIOx								= GPIOA;
	handlerLed2.GPIO_PinConfig.GPIO_PinNumber		= PIN_5;
	handlerLed2.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_OUT;
	handlerLed2.GPIO_PinConfig.GPIO_PinOPType		= GPIO_OTYPE_PUSHPULL;
	handlerLed2.GPIO_PinConfig.GPIO_PinSpeed		= GPIO_OSPEED_FAST;
	handlerLed2.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;

	// Cargamos la configuración del pinA5
	GPIO_Config(&handlerLed2);

	GPIO_WritePin(&handlerLed2, SET);
//	while(1){
//		GPIO_TooglePin(&handlerLed2);
//
//		for(int i = 0; i < 2000000 ; i++){
//			__NOP();
//		}
//	}
}
