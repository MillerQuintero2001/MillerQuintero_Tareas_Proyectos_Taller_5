/*
 * MPU6050.c
 *
 *  Created on: 24/07/2024
 *      Author: MillerQuintero2001
 */

#include "MPU6050.h"

// Elementos para utilizar comunicación I2C
GPIO_Handler_t handlerI2C_SDA = {0};
GPIO_Handler_t handlerI2C_SCL = {0};
I2C_Handler_t handlerAccelerometer = {0};
uint8_t i2cBuffer = 0;
uint8_t arraySaveData[2] = {0};


/** Función que inicializa el hardware del MPU6050 */
void configMPU6050(void){

	/* Activamos el Coprocesador Matemático - FPU */
	SCB->CPACR |= (0XF << 20);

	/* Configuración del pin SCL del I2C1 */
	handlerI2C_SCL.pGPIOx								= GPIOB;
	handlerI2C_SCL.GPIO_PinConfig.GPIO_PinNumber		= PIN_6;
	handlerI2C_SCL.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_ALTFN;
	handlerI2C_SCL.GPIO_PinConfig.GPIO_PinOPType		= GPIO_OTYPE_OPENDRAIN;
	handlerI2C_SCL.GPIO_PinConfig.GPIO_PinSpeed			= GPIO_OSPEED_HIGH;
	handlerI2C_SCL.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
	handlerI2C_SCL.GPIO_PinConfig.GPIO_PinAltFunMode 	= AF4;
	GPIO_Config(&handlerI2C_SCL);

	/* Configuración del pin SDA del I2C1 */
	handlerI2C_SDA.pGPIOx								= GPIOB;
	handlerI2C_SDA.GPIO_PinConfig.GPIO_PinNumber		= PIN_7;
	handlerI2C_SDA.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_ALTFN;
	handlerI2C_SDA.GPIO_PinConfig.GPIO_PinOPType		= GPIO_OTYPE_OPENDRAIN;
	handlerI2C_SDA.GPIO_PinConfig.GPIO_PinSpeed			= GPIO_OSPEED_HIGH;
	handlerI2C_SDA.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
	handlerI2C_SDA.GPIO_PinConfig.GPIO_PinAltFunMode 	= AF4;
	GPIO_Config(&handlerI2C_SDA);

	/* Configuración para el I2C1 */
	handlerAccelerometer.ptrI2Cx		= I2C1;
	handlerAccelerometer.modeI2C		= I2C_MODE_FM;
	handlerAccelerometer.slaveAddress	= ACCEL_ADDRESS;
	i2c_config(&handlerAccelerometer);

	// Hacemos un primer reset del sensor para asegurar que funcione a la primera
	i2c_writeSingleRegister(&handlerAccelerometer, PWR_MGMT_1, 0x00);
}


/* Función para resetear el sensor */
void resetMPU6050(void){
	i2c_writeSingleRegister(&handlerAccelerometer, PWR_MGMT_1, 0x00);
}


/** Función para obtener los datos del eje Z del giroscopio */
float getGyroscopeData(void){
	i2c_readMultipleRegisters(&handlerAccelerometer,  GYRO_ZOUT_H, 2, arraySaveData);
	uint8_t GyroZ_high = arraySaveData[0];
	uint8_t GyroZ_low = arraySaveData[1];
	float GyroZ = (float)((int16_t)(GyroZ_high << 8 | GyroZ_low));
	return (GyroZ/131.00f);
}


/** Función que calcula el offset para calibrar los datos del giroscopio */
float getGyroscopeOffset(uint32_t samples){
	double sum = 0.0f;
	uint32_t counter = 0;
	while(counter < samples){
		sum += getGyroscopeData();
		counter++;
	}
	return sum/((float)counter);
}
