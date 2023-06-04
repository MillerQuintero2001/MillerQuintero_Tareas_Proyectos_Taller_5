/*
 * 	I2CDriver.c
 *
 *  Created on: 12/05/2023
 *      Author: MillerQuintero2001
 */

#include <stdint.h>
#include "I2CDriver.h"

/*
 * Recordar que se debe configurar los pines para el I2C (SDA y SCL),
 * para lo cual se necesita el módulo GPIO, y los pines configurados
 * en modo Alternate Function.
 * Además, estos pines deben ser configurados como salidas open-drain
 * y con las resistencias en modo pull-up
 */
void i2c_config(I2C_Handler_t *ptrHandlerI2C){

	/* 1. Activamos la señal de reloj para el módulo I2C seleccionado */
	if(ptrHandlerI2C->ptrI2Cx == I2C1){
		RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
	}

	else if(ptrHandlerI2C->ptrI2Cx == I2C2){
		RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
	}

	else if(ptrHandlerI2C->ptrI2Cx == I2C3){
		RCC->APB1ENR |= RCC_APB1ENR_I2C3EN;
	}

	/* 2. Reiniciamos el periférico, de forma que inicia en un estado conocido */
	ptrHandlerI2C->ptrI2Cx->CR1 |= I2C_CR1_SWRST;
	__NOP();
	ptrHandlerI2C->ptrI2Cx->CR1 &= ~I2C_CR1_SWRST;

	/* 3. Indicamos cual es la velocidad del reloj principal, que es la señal utilizada
	 * por el periférico para generar la señal de reloj para el bus I2C*/
	ptrHandlerI2C->ptrI2Cx->CR2 &= ~I2C_CR2_FREQ_Msk; //  Borramos la configuración previa
	if((RCC->CFGR & RCC_CFGR_SW) == RCC_CFGR_SW_PLL){
		ptrHandlerI2C->ptrI2Cx->CR2 |= (MAIN_CLOCK_80_Mhz_FOR_I2C << I2C_CR2_FREQ_Pos);
	}
	else{
		ptrHandlerI2C->ptrI2Cx->CR2 |= I2C_CR2_FREQ_4; //Se escribe 16 MHz, o sino la macro DE 16 Mhz también
	}


	/* 4. Configuramos el modo I2C en el que el sistema funciona
	 * En esta configuración se incluye también la velocidad del reloj
	 * y el tiempo máximo para el cambio de la señal (T-Rise).
	 * Todo comienza con los dos registros en 0 */
	ptrHandlerI2C->ptrI2Cx->CCR = 0;
	ptrHandlerI2C->ptrI2Cx->TRISE = 0;

	if(ptrHandlerI2C->modeI2C == I2C_MODE_SM){
		//	Estamos en modo "Standard" (SM Mode)
		//	Seleccionamos el modo estándar
		ptrHandlerI2C->ptrI2Cx->CCR &= ~I2C_CCR_FS;
		if((RCC->CFGR & RCC_CFGR_SW) == RCC_CFGR_SW_PLL){
			//	Configuramos el registro que se encarga de generar la señal del reloj
			ptrHandlerI2C->ptrI2Cx->CCR |= (I2C_MODE_SM_SPEED_100Khz_PLL << I2C_CCR_CCR_Pos);
			//	Configuramos el registro que controla el tiempo T-Rise máximo
			ptrHandlerI2C->ptrI2Cx->TRISE |= (I2C_MAX_RISE_TIME_SM_PLL);
		}
		else{
			//	Configuramos el registro que se encarga de generar la señal del reloj
			ptrHandlerI2C->ptrI2Cx->CCR |= (I2C_MODE_SM_SPEED_100Khz << I2C_CCR_CCR_Pos);

			//	Configuramos el registro que controla el tiempo T-Rise máximo
			ptrHandlerI2C->ptrI2Cx->TRISE |= (I2C_MAX_RISE_TIME_SM);
		}

	}
	else if(ptrHandlerI2C->modeI2C == I2C_MODE_FM){
		//	Estamos en modo "Fast" (FM Mode)
		//	Seleccionamos el modo rápido
		ptrHandlerI2C->ptrI2Cx->CCR |= I2C_CCR_FS;

		//	Verificamos el tipo de Duty
		if(ptrHandlerI2C->dutyFastModeI2C == I2C_DUTY_FM_2_1){ //Duty 2:1
			ptrHandlerI2C->ptrI2Cx->CCR &= ~(0b1 << 14); //Bit DUTY = 0
			if((RCC->CFGR & RCC_CFGR_SW) == RCC_CFGR_SW_PLL){
				// //	Configuramos el registro que se encarga de generar la señal del reloj
				ptrHandlerI2C->ptrI2Cx->CCR |= (I2C_MODE_FM_DUTY_0_SPEED_400Khz_PLL << I2C_CCR_CCR_Pos);
				// Configuramos el registro que controla el tiempo T-Rise máximo
				ptrHandlerI2C->ptrI2Cx->TRISE |= I2C_MAX_RISE_TIME_FM_PLL;
			}
			else{
				// Configuramos el registro que controla el reloj
				ptrHandlerI2C->ptrI2Cx->CCR |= (I2C_MODE_FM_DUTY_0_SPEED_400Khz << I2C_CCR_CCR_Pos);
				// Configuramos el registro que controla el tiempo T-Rise máximo
				ptrHandlerI2C->ptrI2Cx->TRISE |= I2C_MAX_RISE_TIME_FM;
			}
		}
		else if(ptrHandlerI2C->dutyFastModeI2C == I2C_DUTY_FM_16_9){ //Duty 16:9
			ptrHandlerI2C->ptrI2Cx->CCR |= (0b1 << 14); //Bit DUTY = 1
			ptrHandlerI2C->ptrI2Cx->CCR |= (I2C_MODE_FM_DUTY_1_SPEED_400Khz << I2C_CCR_CCR_Pos);
			ptrHandlerI2C->ptrI2Cx->TRISE |= I2C_MAX_RISE_TIME_FM;
		}
		else{
			__NOP();
		}

	}
	else{
		__NOP();
	}

	/* 5. Activamos el módulo I2C */
	ptrHandlerI2C->ptrI2Cx->CR1 |= I2C_CR1_PE;
}

/** Función que genera la condición de stop */
void i2c_stopTransaction(I2C_Handler_t *ptrHandlerI2C){
	/* 1. Generamos la condición de stop */
	ptrHandlerI2C->ptrI2Cx->CR1 |= I2C_CR1_STOP;
}

/** Función para iniciar la transacción de datos */
void i2c_startTransaction(I2C_Handler_t *ptrHandlerI2C){
	/* 1. Verificamos que la línea no está ocupada - bit "busy" en I2C_CR2 */
	while(ptrHandlerI2C->ptrI2Cx->SR2 & I2C_SR2_BUSY){
		__NOP();
	}
	/* 2. Generamos la señal "start" */
	ptrHandlerI2C->ptrI2Cx->CR1 |= I2C_CR1_START;

	/* 2a. Esperamos a que la bandera del evento "start" se levante */
	/* Mientras esperamos, el valor de SB es 0, entonces la negación (!) es 1*/
	while( !(ptrHandlerI2C->ptrI2Cx->SR1 & I2C_SR1_SB)){ // Implicítamente estamos leyendo el registro, con eso se limpia
		__NOP();
	}
}

/** Función para reiniciar la transacción de datos */
void i2c_reStartTransaction(I2C_Handler_t *ptrHandlerI2C){
	/* 2. Generamos la señal "start" */
	ptrHandlerI2C->ptrI2Cx->CR1 |= I2C_CR1_START;

	/* 2a. Esperamos a que la bandera del evento "start" se levante */
	/* Mientras esperamos, el valor de SB es 0, entonces la negación (!) es 1*/
	while( !(ptrHandlerI2C->ptrI2Cx->SR1 & I2C_SR1_SB)){
		__NOP();
	}
}

/** Función para activar la indicación de No-Ack (Indicación para el Slave de terminar) */
void i2c_sendNoAck(I2C_Handler_t *ptrHandlerI2C){
	/* (Debemos escribir cero en la posición ACK del registro de control 1) */
	ptrHandlerI2C->ptrI2Cx->CR1 &= ~I2C_CR1_ACK;
}

/** 7b. Función para activar la indicación de Ack (Indicación para el Slave de continuar) */
void i2c_sendAck(I2C_Handler_t *ptrHandlerI2C){
	/* (Debemos escribir uno en la posición ACK del registro de control 1) */
	ptrHandlerI2C->ptrI2Cx->CR1 |= I2C_CR1_ACK;
}

/** Función para enviar la dirección de acceso del esclavo e indicar lectura o escritura */
void i2c_sendSlaveAddressRW(I2C_Handler_t *ptrHandlerI2C, uint8_t slaveAddress, uint8_t readOrWrite){
	/* 0. Definimos una variable auxiliar */
	uint8_t auxByte = 0;
	(void) auxByte;

	/* 3. Enviamos la dirección del Slave y el bit que indica que deseamos leer/escribir
	 * (en el siguiente paso se envía la dirección de memoria que se desea leer/escribir */
	ptrHandlerI2C->ptrI2Cx->DR = (slaveAddress << 1) | readOrWrite;

	/* 3.1. Esperamos hasta que la bandera del evento "ADDR" se levante
	 * (esto nos indica que la dirección fue enviada satifactoriamente) */
	while( !(ptrHandlerI2C->ptrI2Cx->SR1 & I2C_SR1_ADDR)){
		__NOP();
	}

	/* 3.2. Debemos limpiar la bandera de la recepción de ACK de la "ADDR", para lo cual
	 * debemos leer en secuencia primero el I2C_SR1 y luego I2C_SR2 */
	auxByte = ptrHandlerI2C->ptrI2Cx->SR1;
	auxByte = ptrHandlerI2C->ptrI2Cx->SR2;
}

/** Función para el envío de la dirección de memoria que se quiere leer del dispositivo */
void i2c_sendMemoryAddress(I2C_Handler_t *ptrHandlerI2C, uint8_t memAddr){
	/* 4. Enviamos la direccíón de memoria que deseamos leer */
	ptrHandlerI2C->ptrI2Cx->DR = memAddr;

	/* 4.1 Esperamos hasta que el byte sea transmitido */
	while( !(ptrHandlerI2C->ptrI2Cx->SR1 & I2C_SR1_TXE)){
		__NOP();
	}
}

/** Función para enviar un byte de datos */
void i2c_sendDataByte(I2C_Handler_t *ptrHandlerI2C, uint8_t dataToWrite){
	/* 5. Cargamos el valor que deseamos escribir */
	ptrHandlerI2C->ptrI2Cx->DR = dataToWrite;

	/* 6. Esperamos hasta que el byte sea transmitido */
	while( !(ptrHandlerI2C->ptrI2Cx->SR1 & I2C_SR1_BTF)){
		__NOP();
	}
}

/** Función para leer un byte de datos */
uint8_t i2c_readDataByte(I2C_Handler_t *ptrHandlerI2C){
	/* 9. Esperamos hasta que el byte entrante sea recibido */
	while( !(ptrHandlerI2C->ptrI2Cx->SR1 & I2C_SR1_RXNE)){
		__NOP();
	}

	ptrHandlerI2C->dataI2C = ptrHandlerI2C->ptrI2Cx->DR;
	return ptrHandlerI2C->dataI2C;

}

/** Función para leer un byte simple*/
uint8_t i2c_readSingleRegister(I2C_Handler_t *ptrHandlerI2C, uint8_t regToRead){

	/* 0. Creamos una variable auxiliar para recibir el dato que leemos */
	uint8_t auxRead = 0;

	/* 1. Generamos la condición de Start */
	i2c_startTransaction(ptrHandlerI2C);

	/* 2. Enviamos la dirección del esclavo y la indicación de ESCRIBIR */
	i2c_sendSlaveAddressRW(ptrHandlerI2C, ptrHandlerI2C->slaveAddress, I2C_WRITE_DATA);

	/* 3. Enviamos la dirección de memoria que deseamos leer */
	i2c_sendMemoryAddress(ptrHandlerI2C, regToRead);

	/* 4. Creamos una condición de Re-Start */
	i2c_reStartTransaction(ptrHandlerI2C);

	/* 5. Enviamos la dirección del esclavo y la indicación de LEER */
	i2c_sendSlaveAddressRW(ptrHandlerI2C, ptrHandlerI2C->slaveAddress, I2C_READ_DATA);

	/* 6. Leemos el dato que envía el esclavo */
	auxRead = i2c_readDataByte(ptrHandlerI2C);

	/* 7. Generamos la condición de NoAck, para el Master no responda y el Slave solo envíe 1 byte */
	i2c_sendNoAck(ptrHandlerI2C);

	/* 8. Generamos la condición Stop, para que el Slave se detenga después de 1 byte */
	i2c_stopTransaction(ptrHandlerI2C);

	return auxRead;
}

/** Función para leer bytes de múltiples registros*/
void i2c_readMultipleRegisters(I2C_Handler_t *ptrHandlerI2C, uint8_t* arrayRegToRead, uint8_t numberOfReg, uint8_t* arraySaveData){

	/* 1. Generamos la condición de Start */
	i2c_startTransaction(ptrHandlerI2C);

	/* 2. Enviamos la dirección del esclavo y la indicación de ESCRIBIR */
	i2c_sendSlaveAddressRW(ptrHandlerI2C, ptrHandlerI2C->slaveAddress, I2C_WRITE_DATA);

	for(int i = 0; i < numberOfReg; i++){
		/* 3. Enviamos la dirección de memoria que deseamos leer */
		i2c_sendMemoryAddress(ptrHandlerI2C, arrayRegToRead[i]);

		/* 4. Creamos una condición de Re-Start */
		i2c_reStartTransaction(ptrHandlerI2C);

		/* 5. Enviamos la dirección del esclavo y la indicación de LEER */
		i2c_sendSlaveAddressRW(ptrHandlerI2C, ptrHandlerI2C->slaveAddress, I2C_READ_DATA);

		/* 6. Leemos el dato que envía el esclavo y guardamos en arreglo */
		arraySaveData[i] = i2c_readDataByte(ptrHandlerI2C);
	}

	/* 7. Generamos la condición de NoAck, para el Master no responda y el Slave solo envíe 1 byte */
	i2c_sendNoAck(ptrHandlerI2C);

	/* 8. Generamos la condición Stop, para que el Slave se detenga después de 1 byte */
	i2c_stopTransaction(ptrHandlerI2C);
}

/** Función para escribir un byte simple */
void i2c_writeSingleRegister(I2C_Handler_t *ptrHandlerI2C, uint8_t regToWrite, uint8_t newValue){

	/* 1. Generamos la condición de Start */
	i2c_startTransaction(ptrHandlerI2C);

	/* 2. Enviamos la dirección del esclavo y la indicación de ESCRIBIR */
	i2c_sendSlaveAddressRW(ptrHandlerI2C, ptrHandlerI2C->slaveAddress, I2C_WRITE_DATA);

	/* 3. Enviamos la dirección de memoria que deseamos escribir */
	i2c_sendMemoryAddress(ptrHandlerI2C, regToWrite);

	/* 4. Enviamos el valor que deseamos escribir en el registro seleccionado */
	i2c_sendDataByte(ptrHandlerI2C, newValue);

	/* 5. Generamos la condición de Stop, para que el slave se detenga después de 1 byte */
	i2c_stopTransaction(ptrHandlerI2C);
}


