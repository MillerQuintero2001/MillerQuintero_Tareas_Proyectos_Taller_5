/*
 * RGBCxDriver.c
 *
 *  Created on: Jun 13, 2023
 *      Author: MillerQuintero2001
 */

#include "RGBCxDriver.h"

I2C_Handler_t handlerRGB_Sensor = {0};	// Handler para la configuración del I2C
GPIO_Handler_t handlerRGB_SCL = {0};	// Pin para el Serial Clock
GPIO_Handler_t handlerRGB_SDA = {0};	// Pin para el Serial Data

void initSensorRGB(TCS34725_Handler_t *ptrHandlerTCS34725){

	/*						I2C Sensor RGB						*/

	/* Configuración del pin SCL del I2C1 */
	handlerRGB_SCL.pGPIOx								= GPIOB;
	handlerRGB_SCL.GPIO_PinConfig.GPIO_PinNumber		= PIN_8;
	handlerRGB_SCL.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_ALTFN;
	handlerRGB_SCL.GPIO_PinConfig.GPIO_PinOPType		= GPIO_OTYPE_OPENDRAIN;
	handlerRGB_SCL.GPIO_PinConfig.GPIO_PinSpeed			= GPIO_OSPEED_HIGH;
	handlerRGB_SCL.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
	handlerRGB_SCL.GPIO_PinConfig.GPIO_PinAltFunMode 	= AF4;
	GPIO_Config(&handlerRGB_SCL);

	/* Configuración del pin SDA del I2C1 */
	handlerRGB_SDA.pGPIOx								= GPIOB;
	handlerRGB_SDA.GPIO_PinConfig.GPIO_PinNumber		= PIN_9;
	handlerRGB_SDA.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_ALTFN;
	handlerRGB_SDA.GPIO_PinConfig.GPIO_PinOPType		= GPIO_OTYPE_OPENDRAIN;
	handlerRGB_SDA.GPIO_PinConfig.GPIO_PinSpeed			= GPIO_OSPEED_HIGH;
	handlerRGB_SDA.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
	handlerRGB_SDA.GPIO_PinConfig.GPIO_PinAltFunMode 	= AF4;
	GPIO_Config(&handlerRGB_SDA);

	/* Configuración para el I2C1 */
	handlerRGB_Sensor.ptrI2Cx		= I2C1;
	handlerRGB_Sensor.modeI2C		= I2C_MODE_FM;
	handlerRGB_Sensor.slaveAddress	= RGB_SLAVE_ADDRESS;
	i2c_config(&handlerRGB_Sensor);

	/* 1. Configuramos el tiempo de integración ADC */
	setIntegrationTime(&handlerRGB_Sensor, ptrHandlerTCS34725->integrationTimeMs);

	/* 2. Configuramos la ganancia del sensor */
	setGain(&handlerRGB_Sensor,ptrHandlerTCS34725->gain);

	/* 3. Activamos el sensor escribiendo un 1 en el primer bit del Enable Register, esto corresponde al bit de ADC Enable */
	i2c_writeSingleRegister(&handlerRGB_Sensor, (COMMAND_BIT | ENABLE_REGISTER), 0b01);
	delay_ms(3);

	/* 4. Activamos el RGBC escribiendo un 1 en el segundo bit del Enable Register, esto corresponde al bit de Power On */
	i2c_writeSingleRegister(&handlerRGB_Sensor, (COMMAND_BIT | ENABLE_REGISTER), 0b11);
	delay_ms(ptrHandlerTCS34725->integrationTimeMs+1);

	/* 5. Definimos si se va a usar timepo de espera entre integraciones */
	waitConfig(&handlerRGB_Sensor,ptrHandlerTCS34725->waitTimeEnable, ptrHandlerTCS34725->waitTimeMs);

}

void setIntegrationTime(I2C_Handler_t *ptrHandlerSensor, float integrationTime){
	uint8_t auxIntegrationTimeHex = 0;
	// Verificamos que el valor esté entre 2.4 ms y 614 ms
	if((integrationTime >= 2.4) && (integrationTime <= 614)){
		auxIntegrationTimeHex = (uint8_t)(256 - integrationTime/2.4);
	}
	// De lo contrario configuramos en 614 ms por defecto
	else{
		auxIntegrationTimeHex = 0x00;
	}
	i2c_writeSingleRegister(ptrHandlerSensor, (COMMAND_BIT | TIMING_REGISTER), auxIntegrationTimeHex);
}

void setGain(I2C_Handler_t *ptrHandlerSensor, uint8_t gain){

	switch(gain){
	case GAIN_X1:{
		i2c_writeSingleRegister(ptrHandlerSensor, (COMMAND_BIT | CONTROL_REGISTER), 0x00);
		break;
	}
	case GAIN_X4:{
		i2c_writeSingleRegister(ptrHandlerSensor, (COMMAND_BIT | CONTROL_REGISTER), 0x01);
		break;
	}
	case GAIN_X16:{
		i2c_writeSingleRegister(ptrHandlerSensor, (COMMAND_BIT | CONTROL_REGISTER), 0x02);
		break;
	}
	case GAIN_X60:{
		i2c_writeSingleRegister(ptrHandlerSensor, (COMMAND_BIT | CONTROL_REGISTER), 0x03);
		break;
	}
	// Por defecto configuramos ganancia x1
	default:{
		i2c_writeSingleRegister(ptrHandlerSensor, (COMMAND_BIT | CONTROL_REGISTER), 0x00);
		break;
	}
	}
}

void waitConfig(I2C_Handler_t *ptrHandlerSensor, uint8_t waitTimeEnable, float waitTimeMs){
	if(waitTimeEnable){
		i2c_writeSingleRegister(ptrHandlerSensor, (COMMAND_BIT | ENABLE_REGISTER), 0b1011);
		uint8_t auxWaitTimeHex = 0;
		// Verificamos que el valor esté entre 2.4 ms y 614 ms
		if((waitTimeMs >= 2.4) && (waitTimeMs <= 614)){
			auxWaitTimeHex = (uint8_t)(256 - waitTimeMs/2.4);
		}
		// De lo contrario configuramos en 614 ms por defecto
		else{
			auxWaitTimeHex = 0x00;
		}
		i2c_writeSingleRegister(ptrHandlerSensor, (COMMAND_BIT | WAIT_TIME_REGISTER), auxWaitTimeHex);
	}
	else{
		__NOP();
	}

}
