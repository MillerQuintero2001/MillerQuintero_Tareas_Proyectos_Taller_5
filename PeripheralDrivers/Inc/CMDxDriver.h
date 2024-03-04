/*
 * CMDxDriver.h
 *
 *  Created on: 28/02/2024
 *      Author: MillerQuintero2001
 */

#ifndef CMDXDRIVER_H_
#define CMDXDRIVER_H_

#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

#include <stm32f4xx.h>

#include "USARTxDriver.h"
#include "GPIOxDriver.h"

/* Esto para configurar el USART */
extern GPIO_Handler_t handlerPinTX;			// Pin de transmisión de datos
extern GPIO_Handler_t handlerPinRX;			// Pin de recepción de datos
extern USART_Handler_t usartComm;			// Comunicación serial

//extern uint8_t usartData; 				// Variable en la que se guarda el dato transmitido
extern char bufferReception[64];			// Buffer para guardar caracteres ingresados
extern char cmd[16];					// Arreglo para guardar el comando ingresado y gestionarlo
extern uint8_t counterReception;		// Contador de carácteres para la recepción
extern bool stringComplete;
extern unsigned int firstParameter;
extern unsigned int secondParameter;


void commandConfig(void);
void commandBuild(uint8_t usartRxData);
void commandx1(void);
void commandx2(void);
void commandx3(void);
void commandx4(void);
void commandx5(void);
void commandx6(void);


#endif /* CMDXDRIVER_H_ */
