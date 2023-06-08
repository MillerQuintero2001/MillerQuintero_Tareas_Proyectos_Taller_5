/*
 * 	I2CDriver.h
 *
 *  Created on: 12/05/2023
 *      Author: MillerQuintero2001
 */

#ifndef I2CDRIVER_H_
#define I2CDRIVER_H_

#include <stm32f4xx.h>

#define I2C_WRITE_DATA 	0
#define I2C_READ_DATA	1

#define MAIN_CLOCK_4_Mhz_FOR_I2C	4
#define MAIN_CLOCK_16_Mhz_FOR_I2C	16
#define MAIN_CLOCK_20_Mhz_FOR_I2C	20
#define MAIN_CLOCK_80_Mhz_FOR_I2C	40 // Divide por 2 por que APB1 no puede más de 50MHz
#define MAIN_CLOCK_100_Mhz_FOR_I2C	50

#define I2C_MODE_SM		0
#define I2C_MODE_FM		1

#define I2C_DUTY_FM_2_1		0
#define I2C_DUTY_FM_16_9	1

#define I2C_MODE_SM_SPEED_100Khz 						80
#define I2C_MODE_SM_SPEED_100Khz_PLL_80Mhz 				202
#define I2C_MODE_SM_SPEED_100Khz_PLL_100Mhz				250
#define I2C_MODE_FM_DUTY_0_SPEED_400Khz 				14
#define I2C_MODE_FM_DUTY_0_SPEED_400Khz_PLL_80Mhz 		33
#define I2C_MODE_FM_DUTY_0_SPEED_400Khz_PLL_100Mhz		41
#define I2C_MODE_FM_DUTY_1_SPEED_400Khz 				14

#define I2C_MAX_RISE_TIME_SM				17
#define I2C_MAX_RISE_TIME_SM_PLL_80Mhz		41
#define I2C_MAX_RISE_TIME_SM_PLL_100Mhz		50
#define I2C_MAX_RISE_TIME_FM				5
#define I2C_MAX_RISE_TIME_FM_PLL_80Mhz		13
#define I2C_MAX_RISE_TIME_FM_PLL_100Mhz		15

typedef struct
{
	I2C_TypeDef		*ptrI2Cx;
	uint8_t			slaveAddress;
	uint8_t			modeI2C;
	uint8_t			dutyFastModeI2C;
	uint8_t			dataI2C;
}I2C_Handler_t;

/* Prototipos de las funciones públicas */
void i2c_config(I2C_Handler_t *ptrHandlerI2C);
void i2c_startTransaction(I2C_Handler_t *ptrHandlerI2C);
void i2c_reStartTransaction(I2C_Handler_t *ptrHandlerI2C);
void i2c_sendSlaveAddressRW(I2C_Handler_t *ptrHandlerI2C, uint8_t slaveAddress, uint8_t readOrWrite);
void i2c_sendMemoryAddress(I2C_Handler_t *ptrHandlerI2C, uint8_t memAddr);
void i2c_sendDataByte(I2C_Handler_t *ptrHandlerI2C, uint8_t dataToWrite);
uint8_t i2c_readDataByte(I2C_Handler_t *ptrHandlerI2C);
void i2c_stopTransaction(I2C_Handler_t *ptrHandlerI2C);
void i2c_sendAck(I2C_Handler_t *ptrHandlerI2C);
void i2c_sendNoAck(I2C_Handler_t *ptrHandlerI2C);

uint8_t i2c_readSingleRegister(I2C_Handler_t *ptrHandlerI2C, uint8_t regToRead);
void i2c_readMultipleRegisters(I2C_Handler_t *ptrHandlerI2C, uint8_t startReg, uint8_t numReg, uint8_t* arraySaveValues);
void i2c_writeSingleRegister(I2C_Handler_t *ptrHandlerI2C, uint8_t regToWrite,uint8_t newValue);

#endif /* I2CDRIVER_H_ */
