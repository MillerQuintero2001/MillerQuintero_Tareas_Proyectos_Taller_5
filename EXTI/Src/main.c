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
#include "BasicTimer.h"

uint32_t counter = 0;
uint32_t auxCounter = 0;
uint32_t *ptr_Counter;

uint8_t byteVariable;
uint8_t *ptr_ByteVariable;
/**
 * Funcion principal del programa
 * Esta función es el corazón del programa
 *
 */
int main(void)
{

	/* Trabajando con la variable de 32 bit */
	counter = 12345;

	auxCounter = counter; // C es pasado por valor, auxCOunter recibe el valor de counter

	ptr_Counter = (&counter); //Con esta instrucción el puntero es llevado a la posición de memoria de la variable

	*ptr_Counter = 1234567; // Con * puntero cambio el valor a donde está apuntando el puntero

	ptr_Counter++;

	*ptr_Counter = 1234567;

	byteVariable = 32;
	ptr_ByteVariable = &byteVariable;
	*ptr_ByteVariable = 123;

	/* Intentando mezclar punteros */
	auxCounter = (uint32_t)&counter; //Haciendo un casting para obtener el valor

	ptr_ByteVariable = (uint8_t *)auxCounter; //Pongo al puntero a apuntar al valor de memoria guardado en auxCounter
	*ptr_ByteVariable = 1; //Asigno el valor al que apunta ese puntero como 1
    /* Loop forever */
	while(1){

	}
	return 0;
}
