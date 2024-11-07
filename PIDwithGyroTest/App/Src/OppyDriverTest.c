/**
 **********************************************************************************
 * @file           : OppyDriverTest.c
 * @author         : Miller Quintero - miquinterog@unal.edu.co
 * @brief          : Simple test to check a well integration of different elements
 **********************************************************************************
 * Oppy driver test
 **********************************************************************************
 */

#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>

#include <stm32f4xx.h>

// Librerías de drivers a utilizar
#include "GPIOxDriver.h"
#include "BasicTimer.h"
#include "USARTxDriver.h"
#include "PwmDriver.h"
#include "PLLDriver.h"
#include "ExtiDriver.h"
#include "I2CDriver.h"

#include "CMDxDriver.h"
#include "OppyDriver.h"

/* Definición de los handlers necesarios */

// Elementos para el Blinky LED
GPIO_Handler_t handlerBlinkyPin = 			{0}; // LED de estado del Pin A5
BasicTimer_Handler_t handlerBlinkyTimer = 	{0}; // Timer del LED de estado

char bufferMandar[64] = {0};
float offsetGyro = 0.00f;


/* Definición de las cabeceras de funciones del main */
void initSystem(void); 										// Función que inicializa los periféricos básicos

/** Función principal del programa
 * ¡Esta función es el corazón del programa! */
int main(void){

	// Initialize al peripherals from the system
	initSystem();
	writeMsg(&usartCmd, "Hello world.\n");

	if(i2c_readSingleRegister(&handlerMPU6050, WHO_AM_I) == 0x68){
		writeMsg(&usartCmd, "MPU6050 working!.\n");
	}
	else{
		writeMsg(&usartCmd, "MPU6050 was not detected!.\n");
	}
	// Calculate the offset of the Gyroscope with 200 samples
	offsetGyro = getGyroscopeOffset(200);
	sprintf(bufferMandar, "Z'axis gyroscope offset is: %.3f °/s.\n", offsetGyro);
	writeMsg(&usartCmd, bufferMandar);

	sprintf(bufferMandar, "PWR_MGMT_1 register is: 0x%x.\n", i2c_readSingleRegister(&handlerMPU6050, PWR_MGMT_1));
	writeMsg(&usartCmd, bufferMandar);

    /* Loop forever */
	while(1){
		commandBuild(USE_DEFAULT);
	}
	return 0;
}

/** Función encargada de iniciar hardware para un pin*/
void initSystem(void){

	configPLL(100);

	/* Activamos el Coprocesador Matemático - FPU */
	SCB->CPACR |= (0XF << 20);

	/* GPIO y Timer del Blinky Led de Estado */
	handlerBlinkyPin.pGPIOx								= GPIOC;
	handlerBlinkyPin.GPIO_PinConfig.GPIO_PinNumber 		= PIN_5;
	handlerBlinkyPin.GPIO_PinConfig.GPIO_PinMode		= GPIO_MODE_OUT;
	handlerBlinkyPin.GPIO_PinConfig.GPIO_PinSpeed 		= GPIO_OSPEED_FAST;
	handlerBlinkyPin.GPIO_PinConfig.GPIO_PinOPType 		= GPIO_OTYPE_PUSHPULL;
	// Cargo la configuración
	GPIO_Config(&handlerBlinkyPin);
	// Pongo estado en alto
	GPIO_WritePin(&handlerBlinkyPin, SET);
	// Atributos para el Timer 5 del LED de estado
	handlerBlinkyTimer.ptrTIMx								= TIM5;
	handlerBlinkyTimer.TIMx_Config.TIMx_mode				= BTIMER_MODE_UP;
	handlerBlinkyTimer.TIMx_Config.TIMx_speed				= BTIMER_PLL_100MHz_SPEED_100us;
	handlerBlinkyTimer.TIMx_Config.TIMx_period				= 2500;
	handlerBlinkyTimer.TIMx_Config.TIMx_interruptEnable 	= BTIMER_INTERRUP_ENABLE;
	handlerBlinkyTimer.TIMx_Config.TIMx_priorityInterrupt	= 6;
	BasicTimer_Config(&handlerBlinkyTimer);
	startBasicTimer(&handlerBlinkyTimer);
	/* Fin del GPIO y Timer del LED de estado
	 * ----------------------------------------*/

	commandConfig(CMD_USART1, USART_BAUDRATE_19200);

	configOppy();
}


/** Interrupción del timer blinky LED*/
void BasicTimer5_Callback(void){
	GPIOxTooglePin(&handlerBlinkyPin); //Cambio el estado del LED PC5
}


void usart1Rx_Callback(void){
	usartData = getRxData();
	writeChar(&usartCmd, usartData);
	if((flagMove)&&(usartData == 's')){
		flagMove = false;
		stopMove();
		stopBasicTimer(&handlerSampleTimer);
	}
	else{
		__NOP();
	}
}

void commandx1(void){
	offsetGyro = getGyroscopeOffset((uint32_t)firstParameter);
	sprintf(bufferMandar, "Z'axis gyroscope offset is: %.3f °/s.\n", offsetGyro);
	writeMsg(&usartCmd, bufferMandar);
}

void commandx2(void){
	sprintf(bufferMandar, "Z'axis gyroscope data at scale 500 dps is: %.3f °/s.\n", getGyroscopeData(Z_AXIS));
	writeMsg(&usartCmd, bufferMandar);
}


void commandx3(void){
	resetMPU6050();
	sprintf(bufferMandar, "PWR_MGMT_1 register is: 0x%x.\n", i2c_readSingleRegister(&handlerMPU6050, PWR_MGMT_1));
	writeMsg(&usartCmd, bufferMandar);
	sprintf(bufferMandar, "Sample rate divider register is: 0x%x.\n", i2c_readSingleRegister(&handlerMPU6050, SMPRT_DIV));
	writeMsg(&usartCmd, bufferMandar);
	sprintf(bufferMandar, "Configuration register is: 0x%x.\n", i2c_readSingleRegister(&handlerMPU6050, CONFIG));
	writeMsg(&usartCmd, bufferMandar);
	sprintf(bufferMandar, "User control register is: 0x%x.\n", i2c_readSingleRegister(&handlerMPU6050, USER_CONTROL));
	writeMsg(&usartCmd, bufferMandar);
	sprintf(bufferMandar, "Gyro config register is: 0x%x.\n", i2c_readSingleRegister(&handlerMPU6050, GYRO_CONFIG));
	writeMsg(&usartCmd, bufferMandar);
}

void commandx4(void){
	startMove();
}

void commandx5(void){
	stopMove();
}

void commandx6(void){
	straightLinePID((uint16_t)firstParameter);
}

void commandx7(void){
	rotateOppy((int16_t)firstParameter);
	//rotationMPU6050((int16_t)firstParameter);
	writeMsg(&usartCmd, "Rotation already done.\n");
}

void commandx8(void){
	square((int16_t)firstParameter, (uint16_t)secondParameter);
}

void commandx9(void){
	updateDuttyCycle(&handlerPwmRight, (uint32_t)firstParameter);
	updateDuttyCycle(&handlerPwmLeft, (uint32_t)secondParameter);
}
