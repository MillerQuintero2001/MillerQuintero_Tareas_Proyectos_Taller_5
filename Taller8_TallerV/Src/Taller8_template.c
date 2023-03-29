/*
 * Taller8_template.c
 *
 *  Created on: 28/03/2023
 *      Author: alvelezp - jutoroa
 */

// Taller 8 - Paso por valor y paso por referencia

#include <stdint.h>

// Crear string


// Creacion de una funcion que duplique el valor de un numero

// 1.0 Paso por valor Básico

void duplicarNumero(uint8_t numero){
	numero *= 2;
}

// 1.1 Paso por referencia

void duplicarNumeroRef(uint8_t *numero){
	*numero *= 2;
}

// 1.2 Paso por valor reasignando variables.

uint8_t duplicarNumeroReturn(uint8_t numero){
	return(numero*2);
}

// 1.3 Arreglos por Referencia

void abonarDeudas(uint16_t misDeudas[], uint8_t cantidadDeudas){

	// Vamos a recorrer el arreglo
	for (uint8_t i = 0; i < cantidadDeudas; i++){
		misDeudas[i] /= 2;
	}
}

// ***** // SOLUCION EJERCICIO // ***** //

void stringCaseConverter(char *string){

	//Crear contador
	uint8_t j=0;

	//Con notación de punteros
	while(string[j] != 00){
		// Es mayuscula
		if((*(string+j)>64) && (*(string+j)<91)){
			*(string+j) = *(string+j)+32;
		}
		// Es minuscula
		else if((*(string+j)>96) && (*(string+j)<123)){
			*(string+j) = *(string+j)-32;
		}
		else
			*(string+j) = *(string+j);
		j++;
	}
}

int main(void){

	// Crear variable
	uint8_t n = 10;

	char palabra[] = "XiMeNITtaa gonZALEsó";

	// Pasar por valor
	duplicarNumero(n);

	// Pasar por referencia
	duplicarNumeroRef(&n);

	// Reasignación paso por valor
	n = duplicarNumeroReturn(n);

	// Pasar referencia el arreglo
	uint16_t deudasMensuales[5] = {15000,25,1000,0,600};

	abonarDeudas(deudasMensuales, 5);

	stringCaseConverter(palabra);


	/* 1.5 EJERCICIO:

	Crear una función llamada stringCaseConverter que no retorne ningún
	valor, y reciba una string.

	Esta función deberá cambiar las minúsculas por mayúsculas y viceversa
	del string original. */

}
