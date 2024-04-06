/*
 * Taller7.c
 *
 *  Created on: 22/03/2023
 *      Author: miller
 */

#include <stdint.h>

int main(void){

	// 1.0 Ejemplo básico punteros
	//Definimos variables
	uint8_t dato = 124;

	// Creamos el puntero con la direccíón de memoria de la varible, el & (Ampersand) lo que hace es
	// poner al puntero en la direccción de memoria donde está la variable dato
	uint8_t *pDato = &dato;

	// Guardar el valor que esta en esa direccion de memoria
	uint8_t valorDato = *pDato;

	// Casteo básico, cambio el tipo de variable de dato, de 8 a 16 bits
	uint16_t casteoDato = (uint16_t) dato;

	// Crear un puntero en una dirección específica
	uint16_t *punteroDireccion = (uint16_t *) 0x20000001; //Poniendo el (uint16_t) sirve para decirle al puntero que apunte a una varibale de 16 bits que está en esa dirección

	// Cambiar direccion de memoria del puntero
	punteroDireccion = (uint16_t *) 0x20000002;

	// Guardar la dirección de memoria de un puntero
	uint32_t direccionDato = (uint32_t) pDato; //Cambio el puntero por un número guardado en una varibale de 32 bits

	//Cambiar el valor almacenada en el puntero
	*pDato = 200;

	// Aumentar 8 bits el puntero
	pDato++;

	// 2.0 Ejemplo arreglos
	#define sizeOfArray 4

	uint8_t miPrimerArreglo[sizeOfArray] = {5, 0xAE, 'a', 254};

	//Recorrer un arreglo con ciclos

	uint8_t contenido = 0;

	for(uint8_t i = 0; i<sizeOfArray; i++){
		contenido = miPrimerArreglo[i];
	}

	for(uint8_t i = 0; i<sizeOfArray; i++){
			contenido = *(miPrimerArreglo+i);
	}

	//Camibiar elmentos de un arreglo
	miPrimerArreglo[1] = 12;
	*(miPrimerArreglo+1) = 12; //Esas 2 líneas hacen lo mismo solo que en diferente notación

	// 3.0 Estructuras

	// Definición

	typedef struct{
		uint8_t elemento1;
		uint16_t elemetno2;
		uint8_t arreglo[sizeOfArray];
	}miPrimeraEstructura;

	miPrimeraEstructura estructuraEjemplo = {0};

	// Acceder a los elementos de la estructura
	estructuraEjemplo.elemento1 = 'F';
	estructuraEjemplo.arreglo[2] = 10;

	// Como nuestra estructura ya está en memoria, puedo crear un puntero a ella
	miPrimeraEstructura *ptrMiPrimeraEstructura = &estructuraEjemplo; //
	ptrMiPrimeraEstructura->elemento1 = 9;
	ptrMiPrimeraEstructura->arreglo[2] = 15;
	*((ptrMiPrimeraEstructura->arreglo+2)) = 15;

}
