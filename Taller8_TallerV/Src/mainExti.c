/*
 * mainExti.c
 *
 *  Created on: Mar 27, 2023
 *      Author: jutoroa
 */

#include <stdint.h>
#include "ExtiDriver.h"
#include "GPIOxDriver.h"

GPIO_Handler_t handlerStateLed = {0};
GPIO_Handler_t handlerFotoCompuerta = {0};

EXTI_Config_t handlerExti = {0};

void initSystem(void);

int main(void){

	initSystem();

	while(1){
		__NOP();
	}

}

void initSystem(void){
	handlerStateLed.pGPIOx = GPIOA;
	handlerStateLed.GPIO_PinConfig.GPIO_PinNumber = PIN_5;
	handlerStateLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	handlerStateLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OTYPE_PUSHPULL;
	handlerStateLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OSPEED_HIGH;

	GPIO_Config(&handlerStateLed);

	handlerFotoCompuerta.pGPIOx = GPIOA;
	handlerFotoCompuerta.GPIO_PinConfig.GPIO_PinNumber = PIN_0;
	handlerFotoCompuerta.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	handlerFotoCompuerta.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_PULLUP;
	handlerFotoCompuerta.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OSPEED_HIGH;


	handlerExti.edgeType = EXTERNAL_INTERRUPT_FALLING_EDGE;
	handlerExti.pGPIOHandler = &handlerFotoCompuerta;

	extInt_Config(&handlerExti);
}

void callback_extInt0(void){
	GPIOxTooglePin(&handlerStateLed);
}

/* Ejercicio */

/* Genere un programa, que contenga una interrupción del timer 2 para hacer el blinky de un led,
 * y una interrupción por un EXTI para una fotocompuerta. Cada vez que la fotocompuerta genere
 * una interrupción, se deberá duplicar el periodo del blinky del led, dado por la interrupción
 * del timer2
 * */

