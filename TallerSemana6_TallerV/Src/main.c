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
//#include "GPIOxDriver.h"

#define SUMA 	'+'
#define RESTA 	'-'
#define MULTIPLICACION	'*'
#define DIVISION	'/'

#define FACTORIAL 'FA'
#define FIBONACCI 'FI'

// Función Switch Case
//uint16_t resultadoOperacion(uint8_t tipoDeOperacion, uint8_t numeroA, uint8_t numeroB);
uint32_t resultadoFibonacciFactorial(uint32_t tipoDeOperacion2, uint8_t numero);


int main(void)
{
	uint32_t operacionFinal = 0;
//    operacionFinal = resultadoOperacion(SUMA, 7, 10);
//    operacionFinal++;
	operacionFinal = resultadoFibonacciFactorial(FACTORIAL,4);
	operacionFinal+=1;

}

uint16_t resultadoOperacion(uint8_t tipoDeOperacion, uint8_t numeroA, uint8_t numeroB){
	uint16_t resultado = 0;

	switch(tipoDeOperacion){

	case SUMA:
	{
		resultado = numeroA + numeroB;
		break;
	}
	case RESTA:
	{
		resultado = numeroA - numeroB;
		break;
	}
	case MULTIPLICACION:
	{
			resultado = numeroA * numeroB;
			break;
	}
	case DIVISION:
	{
			resultado = numeroA * numeroB;
			break;
	}
	default:
	{
			resultado = 0;
			break;
	}
	}
	return resultado;
}

uint32_t resultadoFibonacciFactorial(uint32_t tipoDeOperacion2, uint8_t numero){

	uint32_t resultado = 0;
	uint32_t auxvar = 0;

	switch(tipoDeOperacion2){

	case FIBONACCI:
	{
		uint16_t a = 0;
		uint16_t b = 1;
		uint16_t c = 0;
		uint8_t i = 0;
		while(i<numero){
			c=a+b;
			a=b;
			b=c;
			i++;
		}

		resultado = c;
		break;
	}

	case FACTORIAL:
	{
		uint8_t j = 1;
		while(j<=numero){
			auxvar *= j;
			j++;
		}
		resultado = auxvar;
		break;
	}
	default:
	{
		resultado = 0;
		break;
	}
	}
	return resultado;
}



