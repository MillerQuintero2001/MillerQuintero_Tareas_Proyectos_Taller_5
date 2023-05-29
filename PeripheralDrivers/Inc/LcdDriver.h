/*
 * 	LcdDriver.h
 *
 *  Created on: 28/05/2023
 *      Author: MillerQuintero2001
 */

#ifndef LCDDRIVER_H_
#define LCDDRIVER_H_

#include <stm32f4xx.h>
#include "I2CDriver.h"

/* Como la LCD va a ser controlada por medio del PCF8574
 * se van a emplear funciones del I2C, entonces se importa*/

// Macro-definiciones para facilitar
#define CLEAR_LCD_CMD				0x01
#define DISPLAY_ON_LCD_CMD			0x0C
#define DISPLAY_OFF_LCD_CMD			0x08

#define CLEAN_MASK		0xF0

#define LINE_1			0x00
#define LINE_2			0x40
#define LINE_3			0x14
#define LINE_4			0x54


/* Como la LCD va a ser controlada por medio de del PCF*/
//typedef struct
//{
//	I2C_TypeDef		*ptrI2Cx;
//	uint8_t			slaveAddress;
////	uint8_t			modeI2C;
////	uint8_t			dutyFastModeI2C;
////	uint8_t			dataI2C;
//}LCD_Handler_t

/* Después de 2 horas perdidas, simplemente la mejor forma era usar puntero
 * a la estructura handler del I2C, para emplear eficientemente todo */

// Función que me permitirá iniciar el hardware de la LCD con el PCF8574
void initLCD(I2C_Handler_t *ptrHandlerI2C);

// Función para enviar una orden al PCF8574
void sendCommandLCD(I2C_Handler_t *ptrHandlerI2C, char cmd);

// Función para escribir una orden al PCF8574
void writeCommandLCD(I2C_Handler_t *ptrHandlerI2C, uint8_t instruction);

// Función para enviar un carácter
void sendCharLCD(I2C_Handler_t *ptrHandlerI2C, char character);

// Funcíón para enviar todo un string
void sendStringLCD(I2C_Handler_t *ptrHandlerI2C, char *message);

// Función para mover el cursor
void moveCursorLCD(I2C_Handler_t *ptrHandlerI2C, uint8_t column, uint8_t line);

// Función para limpiar LCD
void clearLCD(I2C_Handler_t *ptrHandlerI2C);

// Función para limpiar toda la pantalla
void clearScreenLCD(I2C_Handler_t *ptrHandlerI2C, uint8_t cursor);

#endif /* LCDDRIVER_H_ */
