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

bool flag = false;
bool flagInit = true;
BasicTimer_Handler_t handlerSampleTimer = {0};

uint16_t counter = 0;
char bufferMandar[64] = {0};

// Variables del giroscopio
bool flagTakeOffset = true;
bool flagData = false;
float dataAngle = 0.0f;
float sumAngularVelocity = 0.0f;
float totalCurrentAngle = 0.0f;
float offsetAngularVelocity = 0.0f;
uint16_t counterSamples = 0;

/* Variables para el control PID */
float u_max = 400.00f;
float u_min = 0.0f;
uint32_t counterPreviousRight = 0;
uint32_t counterPreviousLeft = 0;
float duttyBaseRight = 12000.00f;
float duttyBaseLeft = 12840.00f;
uint32_t counter1 = 0;
uint32_t counter2 = 0;

// Calculo de la acción de control
float u_control = 0.0f; 			// Acción de control actual
float u_1_control = 0.0f;			// Accioń de control previa
float error = 0.0f;					// Error actual
float error_1 = 0.0f;				// Error una muestra antes
float error_2 = 0.0f;				// Error dos muestras antes
float kp = 0.900f;					// Constante proporcional
float ti = 0.120f;					// Tiempo integrativo [s]
float td = 0.030f;					// Tiempo derivativo [s]
float timeSample = 0.020f;			// Tiempo de muestreo [s]

float q0 = 0.0f;					// Constante de PID discreto
float q1 = 0.0f;					// Constante de PID discreto
float q2 = 0.0f;					// Constante de PID discreto

/* Definición de las cabeceras de funciones del main */
void initSystem(void); 										// Función que inicializa los periféricos básicos
void controlActionPID(void);								// Función que retorna el valor de la acción de control
void controlDrift(float* currentAngle);						// Function that exam if the drift angle has been reach and take action
void constraintControl(float* uControl, float maxChange);	// Función que limita el valor de la acción de control
void calculateOffsetGyro(uint16_t samples);					// Function that calculates the offset by averages in the MPU6050

/** Función principal del programa
 * ¡Esta función es el corazón del programa! */
int main(void){

	// Initialize al peripherals from the system
	initSystem();

	// Calculate the offset of the Gyroscope with 200 samples
	calculateOffsetGyro(200);

    /* Loop forever */
	while(1){
		commandBuild(USE_DEFAULT);

		if(flagData){
			dataAngle = (getGyroscopeData() - offsetAngularVelocity)*timeSample;
			totalCurrentAngle += dataAngle;
			// Calculate the error for input PID, is like this because the setPointAngle is equal to zero
			error = -totalCurrentAngle;
			//controlDrift(&totalCurrentAngle);
			controlActionPID();
			flagData = false;
		}

		else{
			__NOP();
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
	updateDuttyCycle(&handlerPwmLeft, 12840);

	configMPU6050();

	// Calculamos las constantes del PID
//	kp = (((1.2f*tau)/(k*theta))/2.00f);
//	ti = 2.0f*theta;
//	td = 0.5f*theta;
	q0 = kp*(1.0f+(timeSample/(2.0f*ti))+(td/timeSample));
	q1 = -kp*(1.0f-(timeSample/(2.0f*ti))+((2.0f*td)/timeSample));
	q2 = (kp*td)/timeSample;

	handlerSampleTimer.ptrTIMx								= TIM4;
	handlerSampleTimer.TIMx_Config.TIMx_mode				= BTIMER_MODE_UP;
	handlerSampleTimer.TIMx_Config.TIMx_speed				= BTIMER_PLL_100MHz_SPEED_100us;
	handlerSampleTimer.TIMx_Config.TIMx_period				= 200;
	handlerSampleTimer.TIMx_Config.TIMx_interruptEnable		= BTIMER_INTERRUP_ENABLE;
	BasicTimer_Config(&handlerSampleTimer);

}


/** Función para calcular la acción de control */
void controlActionPID(void){

	u_control = (u_1_control)+(q0*error)+(q1*error_1)+(q2*error_2);
	// Control action is limited to a change of 10% Dutty Cycle
	constraintControl(&u_control, 4000.00f);

	// Control of print data
	if(counter >= 10){
		sprintf(bufferMandar, "%.3f\t%.3f\n", u_control, totalCurrentAngle);
		writeMsg(&usartCmd, bufferMandar);
		counter = 0;
	}
	else{
		__NOP();
	}

	updateDuttyCycle(&handlerPwmLeft, (uint16_t)(duttyBaseLeft - u_control));
	updateDuttyCycle(&handlerPwmRight, (uint16_t)(duttyBaseRight + u_control));

//	// If the error is less than zero, then the Oppy is going to the left side
//	if(error < 0){
//		updateDuttyCycle(&handlerPwmLeft, (uint16_t)(duttyBaseLeft + u_control));
//		updateDuttyCycle(&handlerPwmRight, (uint16_t)(duttyBaseRight - u_control));
//	}
//
//	// Else if the error is greater than zero, then the Oppy is going to the right side
//	else if(error > 0){
//		updateDuttyCycle(&handlerPwmLeft, (uint16_t)(duttyBaseLeft - u_control));
//		updateDuttyCycle(&handlerPwmRight, (uint16_t)(duttyBaseRight + u_control));
//	}
//
//	// Else, is straight!
//	else{
//		__NOP();
//	}

	// Update the values
	u_1_control = u_control;
	error_2 = error_1;
	error_1 = error;
}

void controlDrift(float* currentAngle){
	if(fabs(*currentAngle) > 5.0f){
		stopBasicTimer(&handlerSampleTimer);
		stopMove();
		resetMPU6050();
		calculateOffsetGyro(200);
		*currentAngle = totalCurrentAngle/2.0f;
		startBasicTimer(&handlerSampleTimer);
		startMove();
	}
	else{
		__NOP();
	}
}


void constraintControl(float* uControl, float maxChange){
	if(*uControl >= maxChange){
		*uControl = maxChange;
	}
	else if(*uControl <= -maxChange){
		*uControl = -maxChange;
	}
	else{
		__NOP();
	}
}


void calculateOffsetGyro(uint16_t samples){
	flagTakeOffset = true;
	startBasicTimer(&handlerSampleTimer);
	while(!(counterSamples >= samples)){
		__NOP();
	}
	offsetAngularVelocity = sumAngularVelocity/((float)counterSamples);
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


/** Interrupción del timer blinky LED*/
void BasicTimer5_Callback(void){
	GPIOxTooglePin(&handlerBlinkyPin); //Cambio el estado del LED PC5
}

void BasicTimer4_Callback(void){
	if(flagTakeOffset){
		counterSamples++;
		// Gyroscope data is in °/s
		sumAngularVelocity += getGyroscopeData();
	}
	else{
		flagData = true;
		counter++;
	}
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
	startMove();
}

void commandx2(void){
	stopMove();
	stopBasicTimer(&handlerSampleTimer);
	totalCurrentAngle = 0.0f;
	u_control = 0.0f;
	u_1_control = 0.0f;
	error = 0.0f;
	error_1 = 0.0f;
	error_2 = 0.0f;
	counterPreviousRight = 0;
	counterPreviousLeft = 0;
}

void commandx3(void){
	startBasicTimer(&handlerSampleTimer);
	startMove();
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
