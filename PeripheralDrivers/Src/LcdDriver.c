/*
 * 	LcdDriver.c
 *
 *  Created on: 28/05/2023
 *      Author: MillerQuintero2001
 */

#include <stdint.h>
#include "LcdDriver.h"
#include "SysTickDriver.h"
#include "I2CDriver.h"


/** Función que me permitirá iniciar el hardware de la LCD con el PCF8574 */
void initLCD(I2C_Handler_t *ptrHandlerI2C){

	// Inicialización de 4 bits
	delay_ms(50);
	sendCommandLCD(ptrHandlerI2C, 0x30);
	delay_ms(5);
	sendCommandLCD(ptrHandlerI2C, 0x30);
	delay_ms(1);
	sendCommandLCD(ptrHandlerI2C, 0x30);
	delay_ms(50);
	// Modo de 4 bits
	sendCommandLCD(ptrHandlerI2C, 0x20);
	delay_ms(50);

	// Incialización del display
	sendCommandLCD(ptrHandlerI2C, 0x28);
	delay_ms(50);
	// Display desactivado
	sendCommandLCD(ptrHandlerI2C, DISPLAY_OFF_LCD_CMD);
	delay_ms(50);
	// Display limpio
	sendCommandLCD(ptrHandlerI2C, CLEAR_LCD_CMD);
	delay_ms(50);
	// Modo de entrada
	sendCommandLCD(ptrHandlerI2C, 0x06);
	delay_ms(50);
	// Display encendido
	sendCommandLCD(ptrHandlerI2C, DISPLAY_ON_LCD_CMD);
}

/** Función para enviar una orden al PCF8574 */
void sendCommandLCD(I2C_Handler_t *ptrHandlerI2C, char cmd){
	/* Vamos a emplear modo de 4 bits para la LCD
	 * lo cual requiere de trabajar con 4 bits
	 * para la parte alta del comando, y otros
	 * 4 bits para la parte baja*/
	char data_high = 0;
	char data_low = 0;
	uint8_t data_array[4] = {0};
	data_high = (cmd & CLEAN_MASK);			// Obtengo los bits de la parte alta
	data_low = ((cmd<<4) & CLEAN_MASK);		// Obtengo los bits de la parte baja

	/* Este arreglo de datos indica que lo que se envía son comandos
	 * por eso RS siempre es 0, y el enable solo se activa cuando debe*/

	data_array[0] = data_high|0x0C;			// Enable = 1, RS = 0
	data_array[1] = data_high|0x08;			// Enable = 0, Rs = 0
	data_array[2] = data_low|0x0C;			// Enable = 1, Rs = 0
	data_array[3] = data_low|0x08;			// Enable = 0, Rs = 0
	for(int i = 0; i < 4; i++){
		writeCommandLCD(ptrHandlerI2C, data_array[i]);
	}
}

/** Función para escribir una orden al PCF8574 */
void writeCommandLCD(I2C_Handler_t *ptrHandlerI2C, uint8_t instruction){

	/* 1. Generamos la condición de Start*/
	i2c_startTransaction(ptrHandlerI2C);

	/* 2. Enviamos la diracción del esclavo y la indicación de ESCRIBIR */
	i2c_sendSlaveAddressRW(ptrHandlerI2C, ptrHandlerI2C->slaveAddress, I2C_WRITE_DATA);

	/* 3. Enviamos la dirección de memoria que deseamos escribir */
	i2c_sendDataByte(ptrHandlerI2C, instruction);

	/* 4. Generamos la condición de stop */
	i2c_stopTransaction(ptrHandlerI2C);
}

/* Función para enviar un carácter */
void sendCharLCD(I2C_Handler_t *ptrHandlerI2C, char character){
	char data_high = 0;
	char data_low = 0;
	uint8_t data_array[4] = {0};
	data_high = (character & CLEAN_MASK);			// Obtengo los bits de la parte alta
	data_low = ((character<<4) & CLEAN_MASK);		// Obtengo los bits de la parte baja

	/* Este arreglo de datos indica que lo que se envía son datos
	 * por eso RS siempre es 1, y el enable solo se activa cuando debe*/

	data_array[0] = data_high|0x0D;			// Enable = 1, RS = 1
	data_array[1] = data_high|0x09;			// Enable = 0, RS = 1
	data_array[2] = data_low|0x0D;			// Enable = 1, RS = 1
	data_array[3] = data_low|0x09;			// Enable = 0, RS = 1
	for(int i = 0; i < 4; i++){
		writeCommandLCD(ptrHandlerI2C, data_array[i]);
	}
}

/** Funcíón para enviar todo un string */
void sendStringLCD(I2C_Handler_t *ptrHandlerI2C, char *message){
	while(*message != '\0'){
		sendCharLCD(ptrHandlerI2C, *message);
		// Incremento el puntero, para desplazar 1 en la posición de memoria
		message++;
	}
}

/* Función para mover el cursor según columna y línea deseada*/
void moveCursorLCD(I2C_Handler_t *ptrHandlerI2C, uint8_t column, uint8_t line){
	uint8_t cursor;
	switch (line) {
	case 0 :
		switch (column) {
		case 0 : cursor = LINE_1; break;
		case 1 : cursor = 0x01; break;
		case 2 : cursor = 0x02; break;
		case 3 : cursor = 0x03; break;
		case 4 : cursor = 0x04; break;
		case 5 : cursor = 0x05; break;
		case 6 : cursor = 0x06; break;
		case 7 : cursor = 0x07; break;
		case 8 : cursor = 0x08; break;
		case 9 : cursor = 0x09; break;
		case 10 : cursor = 0x0A; break;
		case 11 : cursor = 0x0B; break;
		case 12 : cursor = 0x0C; break;
		case 13 : cursor = 0x0D; break;
		case 14 : cursor = 0x0E; break;
		case 15 : cursor = 0x0F; break;
		case 16 : cursor = 0x10; break;
		case 17 : cursor = 0x11; break;
		case 18 : cursor = 0x12; break;
		case 19 : cursor = 0x13; break;
		} break;

	case 1 :
		switch (column) {
		case 0 : cursor = LINE_2; break;
		case 1 : cursor = 0x41; break;
		case 2 : cursor = 0x42; break;
		case 3 : cursor = 0x43; break;
		case 4 : cursor = 0x44; break;
		case 5 : cursor = 0x45; break;
		case 6 : cursor = 0x46; break;
		case 7 : cursor = 0x47; break;
		case 8 : cursor = 0x48; break;
		case 9 : cursor = 0x49; break;
		case 10 : cursor = 0x4A; break;
		case 11 : cursor = 0x4B; break;
		case 12 : cursor = 0x4C; break;
		case 13 : cursor = 0x4D; break;
		case 14 : cursor = 0x4E; break;
		case 15 : cursor = 0x4F; break;
		case 16 : cursor = 0x50; break;
		case 17 : cursor = 0x51; break;
		case 18 : cursor = 0x52; break;
		case 19 : cursor = 0x53; break;
		} break;

	case 2 :
		switch (column) {
		case 0 : cursor = LINE_3; break;
		case 1 : cursor = 0x15; break;
		case 2 : cursor = 0x16; break;
		case 3 : cursor = 0x17; break;
		case 4 : cursor = 0x18; break;
		case 5 : cursor = 0x19; break;
		case 6 : cursor = 0x1A; break;
		case 7 : cursor = 0x1B; break;
		case 8 : cursor = 0x1C; break;
		case 9 : cursor = 0x1D; break;
		case 10 : cursor = 0x1E; break;
		case 11 : cursor = 0x1F; break;
		case 12 : cursor = 0x20; break;
		case 13 : cursor = 0x21; break;
		case 14 : cursor = 0x22; break;
		case 15 : cursor = 0x23; break;
		case 16 : cursor = 0x24; break;
		case 17 : cursor = 0x25; break;
		case 18 : cursor = 0x26; break;
		case 19 : cursor = 0x27; break;
		} break;

	case 3 :
		switch (column) {
		case 0 : cursor = LINE_4; break;
		case 1 : cursor = 0x55; break;
		case 2 : cursor = 0x56; break;
		case 3 : cursor = 0x57; break;
		case 4 : cursor = 0x58; break;
		case 5 : cursor = 0x59; break;
		case 6 : cursor = 0x5A; break;
		case 7 : cursor = 0x5B; break;
		case 8 : cursor = 0x5C; break;
		case 9 : cursor = 0x5D; break;
		case 10 : cursor = 0x5E; break;
		case 11 : cursor = 0x5F; break;
		case 12 : cursor = 0x60; break;
		case 13 : cursor = 0x61; break;
		case 14 : cursor = 0x62; break;
		case 15 : cursor = 0x63; break;
		case 16 : cursor = 0x64; break;
		case 17 : cursor = 0x65; break;
		case 18 : cursor = 0x66; break;
		case 19 : cursor = 0x67; break;
		} break;
	}
	sendCommandLCD(ptrHandlerI2C, 0x80|cursor);
}

/** Función para limpiar toda la pantalla LCD, usando un comando */
void clearLCD(I2C_Handler_t *ptrHandlerI2C){
	sendCommandLCD(ptrHandlerI2C, CLEAR_LCD_CMD);
	delay_ms(50);
}

/** Función para limpiar solo una línea de la LCD*/
void clearLineLCD(I2C_Handler_t *ptrHandlerI2C, uint8_t cursor){
	char arrayClean[64] = "                    ";
	moveCursorLCD(ptrHandlerI2C, 0, cursor);
	sendStringLCD(ptrHandlerI2C, arrayClean);
}
