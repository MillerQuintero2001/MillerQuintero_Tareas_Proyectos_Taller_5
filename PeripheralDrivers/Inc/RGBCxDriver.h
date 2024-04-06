/*
 * RGBCxDriver.h (TCS34725)
 *
 *  Created on: Jun 13, 2023
 *      Author: MillerQuintero2001
 */

#ifndef RGBCXDRIVER_H_
#define RGBCXDRIVER_H_

#include "stm32f4xx.h"
#include <stdint.h>
#include "GPIOxDriver.h"
#include "SysTickDriver.h"
#include "I2CDriver.h"

// Registros del sensor TCS34725
#define RGB_SLAVE_ADDRESS		0b0101001	// 0x29
#define ENABLE_REGISTER			0			// 0x00
#define TIMING_REGISTER			1			// 0x01
#define WAIT_TIME_REGISTER		3			// 0x03
#define LOW_THRS_LOW_BYTE		4			// 0x04
#define LOW_THRS_HIGH_BYTE		5			// 0x05
#define HIGH_THRS_LOW_BYTE		6			// 0x06
#define HIGH_THRS_HIGH_BYTE		7			// 0x07
#define PERSISTENCE_REGISTER	12			// 0x0C
#define CONFIG_REGISTER			13			// 0x0D
#define CONTROL_REGISTER		15			// 0x0F
#define ID_REGISTER				18			// 0x12
#define STATUS_REGISTER			19			// 0x13
#define CLEAR_DATA_LOW			20			// 0x14
#define CLEAR_DATA_HIGH			21			// 0x15
#define RED_DATA_LOW			22			// 0x16
#define RED_DATA_HIGH			23			// 0x17
#define GREEN_DATA_LOW			24			// 0x18
#define GREEN_DATA_HIGH			25			// 0x19
#define BLUE_DATA_LOW			26			// 0x1A
#define BLUE_DATA_HIGH			27			// 0x1B

// Bit de comando, esencial para el sensor TC2S34725
#define COMMAND_BIT					0x80		// Para poder hacer lecturas/escrituras simples
#define COMMAND_BIT_AUTOINCREMENT	0xA0		// Para leer múltiples bytes usando propiedad de auto-incremento en los registros
#define COMMAND_BIT_SPECIAL_FUNC	0xE0		// Para usar las funciones especiales

// Macro-definiciones para realizar configuraciones
#define GAIN_X1		0
#define GAIN_X4		1
#define GAIN_X16	2
#define GAIN_X60	3

#define WAIT_TIME_DISABLE	0
#define WAIT_TIME_ENABLE	1

typedef struct{
	float		integrationTimeMs;
	uint8_t 	gain;
	uint8_t 	waitTimeEnable;
	float		waitTimeMs;
	uint8_t 	waitLongEnable;
}TCS34725_Handler_t;

/* Cabeceras de funciones */
void initSensorRGB(TCS34725_Handler_t *ptrHandlerTCS34725);
void setIntegrationTime(I2C_Handler_t *ptrHandlerSensor, float integrationTime);
void setGain(I2C_Handler_t *ptrHandlerSensor, uint8_t gain);
void waitConfig(I2C_Handler_t *ptrHandlerSensor, uint8_t waitTimeEnable, float waitTimeMs);
uint16_t calculateLux(uint16_t red, uint16_t green, uint16_t blue);
uint16_t calculateColorTemperature(uint16_t red, uint16_t green, uint16_t blue);


#endif /* RGBCXDRIVER_H_ */
