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

//Librerias adicionales

#include <stdbool.h> //Variables booleanas
#include <math.h> //Operaciones matemáticas

#define NOP() __asm("NOP")

#define CONSTANTE 100

bool miPrimerVariableGlobal = true;

//Definición de prototipos de funciones, se hace siempre que pongamos el main al final
void miPrimeraFuncion(void);
uint8_t getMaxChar(void);
uint16_t getMaxValue(uint16_t x,uint16_t y, uint16_t z);

#define UNSIGNED false
#define SIGNED true

uint64_t getMaxBits(uint8_t nBits, uint8_t signo);


int main(void)
{
	/*CONVERSOR DE BITS BINARIOS A DECIMALES */

	uint8_t bin, var, dec = 0;
	bin = 0b11101110;

	for(int i=0; i<8; i++){
		var = bin & (1<<i);
		var = var>>i;
		dec = dec + (var*pow(2,i));
	}

	/*TAREA Hacer un conversor de decimal a binario, usando operadores BITWISE y ciclo for*/

//	/*Ejemplo taxímetro*/
//	for(uint8_t decenas = 0; decenas<10; decenas++){
//		for(uint8_t unidades = 0; unidades<10;unidades++){
//			NOP();
//		}
//	}
//	uint8_t resultado = 0;
//
//	for (uint8_t contador = 0; contador <=10; contador ++){
//		resultado = 7 * contador;
//	}
//	uint64_t maxBits = getMaxBits(17,1);
//    miPrimeraFuncion();

//    uint8_t charMax = getMaxChar();
//    (void) charMax;
//
//    uint16_t maxValue = getMaxValue(120, 0b00101101, 0xFEA);
//


}

void miPrimeraFuncion(void){

	miPrimerVariableGlobal = false;
}

uint8_t getMaxChar(void){

	//uint8_t charBits = 8;
	uint8_t maxChar = (2*2*2*2*2*2*2*2)-1;

	return maxChar;
}


/**
 * Funcion principal del programa
 * Esta funcion es el corazon del programa
 */

uint16_t getMaxValue(uint16_t x,uint16_t y, uint16_t z){
	if((x>=y) && (x>=z)){
		return x;
	}
	else if((y>=z) && (y>=x)){
		return y;
	}
	else if((z>=x)&&(z>=y)){
		return z;
	}
	else{
		return 0;
	}
}

uint64_t getMaxBits(uint8_t nBits, uint8_t signo){
	if(nBits==(8||16||32||64)){

		if(signo==0){
			return(pow(2,nBits)-1);
		}
		else{
			return((pow(2,nBits)/2)-1);
		}
	}
	else{
		return 0;
	}
}
