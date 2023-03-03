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


#include <stdint.h>

int64_t var9 = 0;
uint16_t resultado = 0;
/**
 * Funcion principal del programa
 * Esta funcion es el corazon del programa
 *
 */

int main(void){

	uint16_t testShift = 0b0000011010110101;
	uint16_t testMask = 0b00000000000111000;

	while(1){
		resultado = testShift | testMask;
		resultado = testShift & (~testMask);
//		testMask = testMask <<3;
//		testMask = ~testMask;
//		testShift = testShift & testMask;

	}

}
