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

#include "stm32f411xx_hal.h"
#include "GPIOxDriver.h"
#include <stdint.h>

/**
 * Funcion principal del programa
 * Esta función es el corazón del programa
 *
 */
int main(void)
{

	/* 1er Punto, error en la funcion GPIO_ReadPin()
	 *	a) El error en este punto radica en que la función nos está retornando la información de todo el Input
	 *	Data Register (IDR), cuando nuestra intención es leer el estado de un pin en particular y nada más. Si
	 *	esta función no se corrige, podría estar retornando números como 11001, y un numero de esos no me indica
	 *	el estado de una entrada, ya que este estado solo puede tomar 2 valores, "0" ó "1".
	 *
	 * 	b) Para solucionar este error es necesario aplicar una máscara que limpie todos
	 * 	los bits, excepto el primer bit, esta máscara se aplica con un &(and bitwise)
	 *
	 * 	c) Para corroborar que esta solución es correcta vamos a configurar un PIN como entrada
	 * 	y poner su estado en 1.
	 */
	 //Definimos el objeto Handler para el pin a configurar como entrada
	GPIO_Handler_t handlerUsedAsInput = {0}; //Inicializo en 0
	uint32_t valorLectura = 0;

	handlerUsedAsInput.pGPIOx = GPIOA;
	handlerUsedAsInput.GPIO_PinConfig.GPIO_PinNumber		= PIN_0; 				//Quiero configurar el PA0
	handlerUsedAsInput.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_IN; 		//Modo entrada
	handlerUsedAsInput.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_PULLUP; 	//Pull-up para poner la entrada en alto

	// Cargamos la configuración del pin específico
	GPIO_Config(&handlerUsedAsInput);

	// Lectura del pin
	valorLectura = GPIO_ReadPin(&handlerUsedAsInput); // Este valor debería cargarse en 1.
	valorLectura += 1; //Se suma 1, para usar la variable y eliminar el warning, debería quedar en 2.

	/*----------------------------------------------------------------------------------------------------
	 * 2do. Punto, la función Toogle fue creada en el archivo GPIOxDriver.c y la Macro-Definición
	 * esta en el archivo GPIOxDriver.h, en un main la función solo es invocada, así como se hizo con la
	 * función GPIO_Config y GPIO_ReadPin
	 *---------------------------------------------------------------------------------------------------/


    /* Loop forever */
	while(1){
		NOP();
	}
}
