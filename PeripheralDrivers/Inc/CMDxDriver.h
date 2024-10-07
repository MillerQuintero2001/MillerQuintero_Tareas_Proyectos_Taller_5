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

#define CMD_USART1	1
#define CMD_USART2	2
#define CMD_USART6	6

#define USE_DEFAULT true
#define USE_OPPY	false

extern USART_Handler_t usartCmd;		// Variable tipo USART para trabajar en general
extern uint8_t usartData; 				// Variable en la que se guarda el dato transmitido
extern float firstParameter;			// Primer Parámetro global para trabajar comandos
extern float secondParameter;			// Segundo Parámetro global para trabajar comandos
extern float thirdParameter;			// Tercer Parámetro global para trabajar comandos

void commandConfig(uint8_t USARTport, uint8_t baudrate);
void commandBuild(bool use);
void commandx1(void);
void commandx2(void);
void commandx3(void);
void commandx4(void);
void commandx5(void);
void commandx6(void);
void commandx7(void);
void commandx8(void);
void commandx9(void);


#endif /* CMDXDRIVER_H_ */
