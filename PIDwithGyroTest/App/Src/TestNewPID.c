/**
 ******************************************************************************
 * @file           : GyroPID.c
 * @author         : Miller Quintero - miquinterog@unal.edu.co
 * @brief          : Oppy haciendo línea recta
 ******************************************************************************
 * Pruebas driver de comandos, motores y código de control
 ******************************************************************************
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
#include "MotorDriver.h"
#include "MPU6050.h"

/* Definición de los handlers necesarios */

// Elementos para el Blinky LED
GPIO_Handler_t handlerBlinkyPin = 			{0}; // LED de estado del Pin A5
BasicTimer_Handler_t handlerBlinkyTimer = 	{0}; // Timer del LED de estado

uint16_t counter = 0;
char bufferMandar[64] = {0};
float offsetGyro = 0.00f;


/* Definición de las cabeceras de funciones del main */
void initSystem(void); 										// Función que inicializa los periféricos básicos
void controlActionPID(void);								// Función que retorna el valor de la acción de control
void constraintControl(float* uControl, float maxChange);	// Función que limita el valor de la acción de control
void calculateOffsetGyro(uint16_t samples);					// Function that calculates the offset by averages in the MPU6050

/** Función principal del programa
 * ¡Esta función es el corazón del programa! */
int main(void){

	// Initialize al peripherals from the system
	initSystem();
	writeMsg(&usartCmd, "Hello world.\n");
	// Calculate the offset of the Gyroscope with 200 samples
	offsetGyro = getGyroscopeOffset(200);
	sprintf(bufferMandar, "Z'axis gyroscope offset is: %.3f °/s.\n", offsetGyro);
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
	handlerBlinkyPin.pGPIOx								= GPIOA;
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
	BasicTimer_Config(&handlerBlinkyTimer);
	startBasicTimer(&handlerBlinkyTimer);
	/* Fin del GPIO y Timer del LED de estado
	 * ----------------------------------------*/

	commandConfig(CMD_USART1, USART_BAUDRATE_19200);

	configMPU6050();
	configMotors();
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
	defaultMove();
	startMove();
}

void commandx2(void){
	stopMove();
	stopBasicTimer(&handlerSampleTimer);
	resetMPU6050();
}

void commandx3(void){
	square(firstParameter, secondParameter);
}

void commandx4(void){
	stopMove();
}

void commandx5(void){
	changeBaseDutty((uint16_t)firstParameter, (uint16_t)secondParameter);
}

void commandx6(void){
	offsetGyro = getGyroscopeOffset((uint32_t)firstParameter);
	sprintf(bufferMandar, "Z'axis gyroscope offset is: %.3f °/s.\n", offsetGyro);
	writeMsg(&usartCmd, bufferMandar);
}

void commandx7(void){
	configPID(firstParameter, secondParameter, thirdParameter, 0.020f);
	sprintf(bufferMandar,"Kp = %.6f,ti = %.6f,td = %.6f\n",firstParameter, secondParameter, thirdParameter);
	writeMsg(&usartCmd, bufferMandar);
}
