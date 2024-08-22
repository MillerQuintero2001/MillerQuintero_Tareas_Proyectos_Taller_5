/**
 ******************************************************************************
 * @file           : TransferFunctionAngle.c
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

bool flag = false;
bool flagInit = true;
BasicTimer_Handler_t handlerSampleTimer = {0};

uint16_t counter = 1;
char bufferMandar[64] = {0};

// Variables del giroscopio
bool flagTakeOffset = true;
bool flagData = false;
float dataGyroscope = 0.0f;
float sumAngularVelocity = 0.0f;
float totalCurrentAngle = 0.0f;
float offsetAngularVelocity = 0.0f;
uint16_t counterSamples = 0;

/* Variables para el control PID */
uint32_t counterPreviousRight = 0;
uint32_t counterPreviousLeft = 0;
float DuttyBaseRight = 12000.00f;
float DuttyBaseLeft = 12000.00f;
uint32_t counter1 = 0;
uint32_t counter2 = 0;
float timeSample = 0.020f;			// Tiempo de muestreo


/* Definición de las cabeceras de funciones del main */
void initSystem(void); 										// Función que inicializa los periféricos básicos
void calculateOffsetGyro(uint16_t samples);

/** Función principal del programa
 * ¡Esta función es el corazón del programa! */
int main(void){

	// Inicializamos todos los elementos del sistema
	initSystem();

	// Calculate the offset of the Gyroscope with 200 samples
	calculateOffsetGyro(200);

    /* Loop forever */
	while(1){
		commandBuild(USE_DEFAULT);
		if(counterSamples > 1000){
			stopMove();
			stopBasicTimer(&handlerSampleTimer);
			counterSamples = 0;
			flagData = false;
		}

		else if(flagData){
			counterSamples++;
			dataGyroscope = (getGyroscopeData() - offsetAngularVelocity)*0.020f;
			totalCurrentAngle += dataGyroscope;
			sprintf(bufferMandar, "%.6f\n", totalCurrentAngle);
			writeMsg(&usartCmd, bufferMandar);
			flagData = false;
		}
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
	BasicTimer_Config(&handlerBlinkyTimer);
	startBasicTimer(&handlerBlinkyTimer);
	/* Fin del GPIO y Timer del LED de estado
	 * ----------------------------------------*/

	commandConfig(CMD_USART1, USART_BAUDRATE_19200);

	configMotors();

	updateDuttyCycle(&handlerPwmRight, 12000);
	updateDuttyCycle(&handlerPwmLeft, 13120);

	configMPU6050();

	handlerSampleTimer.ptrTIMx								= TIM4;
	handlerSampleTimer.TIMx_Config.TIMx_mode				= BTIMER_MODE_UP;
	handlerSampleTimer.TIMx_Config.TIMx_speed				= BTIMER_PLL_100MHz_SPEED_100us;
	handlerSampleTimer.TIMx_Config.TIMx_period				= 200;
	handlerSampleTimer.TIMx_Config.TIMx_interruptEnable		= BTIMER_INTERRUP_ENABLE;
	BasicTimer_Config(&handlerSampleTimer);

}

/** Interrupción del timer blinky LED*/
void BasicTimer5_Callback(void){
	GPIOxTooglePin(&handlerBlinkyPin); //Cambio el estado del LED PC5
}

void BasicTimer4_Callback(void){
	if(flagTakeOffset){
		counterSamples++;
		// El dato del giroscopio está en °/s
		sumAngularVelocity += getGyroscopeData();
		if(counterSamples >= 200){
			offsetAngularVelocity = sumAngularVelocity/((float)counterSamples);
		}
		else{
			__NOP();
		}
	}
	else{
		flagData = true;
	}
}

void usart1Rx_Callback(void){
	usartData = getRxData();
	writeChar(&usartCmd, usartData);
	if((flagMove)&&(usartData == 's')){
		flagMove = false;
	}
	else{
		__NOP();
	}
}

void calculateOffsetGyro(uint16_t samples){
	startBasicTimer(&handlerSampleTimer);
	while(!(counterSamples >= samples)){
		__NOP();
	}
	// We verify if the offset is not appropiate
	if(fabs(offsetAngularVelocity) >= 0.85f){
		// Then, We repeat the process again
		counterSamples = 0;
		sumAngularVelocity = 0.0f;
		calculateOffsetGyro(samples);
	}
	else{
		// The offset angle is correct
		stopBasicTimer(&handlerSampleTimer);
		counterSamples = 0;
		sumAngularVelocity = 0.0f;
		flagTakeOffset = false;
		sprintf(bufferMandar,"El offset del giroscopio es %.6f °/s\n",offsetAngularVelocity);
		writeMsg(&usartCmd, bufferMandar);
	}
}

void commandx1(void){
	startMove();
}

void commandx2(void){
	stopMove();
	stopBasicTimer(&handlerSampleTimer);
	counterPreviousRight = 0;
	counterPreviousLeft = 0;
}

void commandx3(void){
	startBasicTimer(&handlerSampleTimer);
	startMove();
	totalCurrentAngle = 0.0f;
}

void commandx4(void){
	sprintf(bufferMandar,"Dato simple del giroscopio es %.6f °/s\n", getGyroscopeData());
	writeMsg(&usartCmd, bufferMandar);
	sprintf(bufferMandar,"Dato offset del giroscopio es %.6f °/s\n", offsetAngularVelocity);
	writeMsg(&usartCmd, bufferMandar);
}

void commandx5(void){
	updateDuttyCycle(&handlerPwmRight, (uint16_t)firstParameter);
	updateDuttyCycle(&handlerPwmLeft, (uint16_t)secondParameter);
}

void commandx6(void){
	calculateOffsetGyro((uint16_t)firstParameter);
}
