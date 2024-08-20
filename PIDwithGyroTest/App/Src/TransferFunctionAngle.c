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
bool flagPID = false;				// Bandera que indica cuando hacer acción de control
float u_max = 400.00f;
float u_min = 0.0f;
uint32_t counterPreviousRight = 0;
uint32_t counterPreviousLeft = 0;
float DuttyBaseRight = 12000.00f;
float DuttyBaseLeft = 12000.00f;
uint32_t counter1 = 0;
uint32_t counter2 = 0;
// Calculo de la acción de control
float u_control = 0.0f; 			// Acción de control actual
float u_1_control = 0.0f;			// Accioń de control previa
float error = 0.0f;					// Error actual
float error_1 = 0.0f;				// Error una muestra antes
float error_2 = 0.0f;				// Error dos muestras antes
float kp = 0.000f;					// Constante proporcional
float ti = 0.000f;					// Tiempo integrativo
float td = 0.000f;					// Tiempo derivativo
float timeSample = 0.030f;			// Tiempo de muestreo
uint8_t setPointVelocity = 117;		// SetPoint de velocidad
float vR = 0.0f;					// Velocidad rueda derecha
float vL = 0.0f;					// Velocidad rueda izquierda
float q0 = 0.0f;					// Constante de PID discreto
float q1 = 0.0f;					// Constante de PID discreto
float q2 = 0.0f;					// Constante de PID discreto
// Parámetros Ziegler-Nichols
float k = 7.55f;							// Ganacia de la función de transferencia
float tau = 0.3175f;						// Constante de tiempo de la función de transferencia
float theta = 0.06f;						// Retardo del sistema en PID discreto (L+ts/2)

/* Definición de las cabeceras de funciones del main */
void initSystem(void); 										// Función que inicializa los periféricos básicos
void controlActionPID(void);								// Función que retorna el valor de la acción de control
float mapControlDutty(float u_input);	// Función que mapea la acción de control a un % Dutty Cycle
void constraintError(void);									// Funcíon que limita el error para que no sea superior al 100%
void constraintControl(void);								// Función que limita el valor de la acción de control
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
	setSignals(25, 30);

	configMPU6050();

	// Calculamos las constantes del PID
	kp = (((1.2f*tau)/(k*theta))/2.00f);
	ti = 2.0f*theta;
	td = 0.5f*theta;
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
	float newDutty = 0;
	float duttyChange = 0.00f;
	if(vR > vL){
		error = ((float)setPointVelocity) - vL;
		constraintError();
		u_control = (u_1_control)+(q0*error)+(q1*error_1)+(q2*error_2);
		constraintControl();
		newDutty = mapControlDutty(u_control);
		duttyChange = newDutty - DuttyBaseLeft;
		sprintf(bufferMandar,"%.2f\n",(duttyChange/400.00f));
		writeMsg(&usartCmd, bufferMandar);
		updateDuttyCycle(&handlerPwmRight, (uint16_t)(DuttyBaseRight + duttyChange));
		updateDuttyCycle(&handlerPwmLeft, (uint16_t)(DuttyBaseLeft + duttyChange));
		// Actualizamos el dutty base
		DuttyBaseRight += duttyChange;
		DuttyBaseLeft += duttyChange;
	}


//	if(velocityRight > velocityLeft){
//		error = ((float)setPointVelocity) - velocityLeft;
//		constraintError();
//		u_control = (u_1_control)+(q0*error)+(q1*error_1)+(q2*error_2);
//		constraintControl();
//		newDutty = (uint16_t)mapControlDutty(u_control, false);
//		duttyChange = ((float)newDutty)-(30.00f*400.00f);			// Númerico
//		updateDuttyCycle(&handlerPwmLeft, newDutty);
//		updateDuttyCycle(&handlerPwmRight, ((uint16_t)(30.00f*400.00f) - duttyChange));
//		counter1++;
//	}
//	else if(velocityLeft > velocityRight){
//		error = (float)setPointVelocity - velocityRight;
//		constraintError();
//		u_control = (u_1_control)+(q0*error)+(q1*error_1)+(q2*error_2);
//		constraintControl();
//		newDutty = (uint16_t)mapControlDutty(u_control, true);
//		duttyChange = ((float)newDutty)-(30.00f*400.00f);	// Númerico
//		updateDuttyCycle(&handlerPwmRight, newDutty);
//		updateDuttyCycle(&handlerPwmLeft, ((uint16_t)(30.00f*400.00f) - duttyChange));
//		counter2++;
//	}
	else{
		__NOP();
	}
	u_1_control = u_control;
	error_2 = error_1;
	error_1 = error;
}

/** Función para realizar el mapeo entre acción de control y PWM */
float mapControlDutty(float u_input){
	float duttyPer = 0.0f;
	if(error < 0){
		duttyPer = 30.00f - ((35.00f-25.00f)/u_max-u_min)*u_input;
	}
	else if (error > 0){
		duttyPer = ((35.00f-25.00f)/u_max-u_min)*u_input + 30.00f;
	}
	//Constraint
	if(duttyPer > 35){
		duttyPer = 35.00f;
	}
	else if(duttyPer < 25){
		duttyPer = 25.00f;
	}
	return duttyPer*400.00f;
}

/** Función que limita el error para que no supere el 100%*/
void constraintError(void){
	if(error < -setPointVelocity){
		error = -setPointVelocity;
	}
	else if(error > setPointVelocity){
		error = setPointVelocity;
	}
	else{
		__NOP();
	}
}

/** Función que limita el valor de la acción de control */
void constraintControl(void){
	if(u_control < 0){
		u_control = 0.00f;
	}
	else if(u_control > 400){
		u_control = 400.00f;
	}
	else{
		__NOP();
	}
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
	flagPID = 0;
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
