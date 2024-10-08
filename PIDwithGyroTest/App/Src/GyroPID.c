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

BasicTimer_Handler_t handlerSampleTimers = {0};

uint16_t counter = 0;
char bufferMandar[64] = {0};

// Variables del giroscopio
bool flagTakeOffsets = true;
bool flagDatas = false;
float dataAngle = 0.0f;
float previousDataAngle = 0.0f;
float sumAngularVelocitys = 0.0f;

float differentialCurrentAngle = 0.0f;
float totalCurrentAngle = 0.0f;
float offsetAngularVelocity = 0.0f;
uint16_t counterSampless = 0;

/* Variables para el control PID */
uint32_t counterPreviousRight = 0;
uint32_t counterPreviousLeft = 0;

float differentialDistance = 0.0f;
float currentDistanceX = 0.0f;
float currentDistanceY = 0.0f;

float duttyBaseRights = 12000.00f;
float duttyBaseLefts = 12710.00f;
uint8_t sideIndicator = 0;
uint8_t sideGoal = 0;
uint8_t direction = 0; // Variable to specify if the the square direction will be clock wise = 0, or counter clock wise = 1;

// Calculo de la acción de control
float u_controls = 0.0f; 			// Acción de control actual
float u_1_controls = 0.0f;			// Accioń de control previa
float errors = 0.0f;					// Error actual
float error_1s = 0.0f;				// Error una muestra antes
float error_2s = 0.0f;				// Error dos muestras antes
float kp = 2.200f;					// Constante proporcional
float ti = 0.080f;					// Tiempo integrativo [s]
float td = 0.050f;					// Tiempo derivativo [s]
float timeSamples = 0.020f;			// Tiempo de muestreo [s]

float q0s = 0.0f;					// Constante de PID discreto
float q1s = 0.0f;					// Constante de PID discreto
float q2s = 0.0f;					// Constante de PID discreto

/* Definición de las cabeceras de funciones del main */
void initSystem(void); 										// Función que inicializa los periféricos básicos
void controlActionPIDs(void);								// Función que retorna el valor de la acción de control
void constraintControls(float* uControl, float maxChange);	// Función que limita el valor de la acción de control
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

		if(flagDatas){
			// Take angular velocity in the time sample and multiply it by time sample to get the angle in that time
			dataAngle = (getGyroscopeData() - offsetAngularVelocity)*timeSamples;
			// Sum with totalCurrentAngle to update the real current angle of the Oppy
			totalCurrentAngle += dataAngle;

			// Take the difference between previous and current data angle to get the differential angle and can calculate X and Y
			differentialCurrentAngle = previousDataAngle + dataAngle;

			differentialDistance = (((counterIntLeft - counterPreviousLeft)+(counterIntRight - counterPreviousRight))/2.0f)*(M_PI*51.725f/120.0f);
			currentDistanceX += differentialDistance*cosf(differentialCurrentAngle*M_PI/180.0f);
			currentDistanceY += differentialDistance*sinf(differentialCurrentAngle*M_PI/180.0f);

			previousDataAngle = dataAngle;

			counterPreviousRight = counterIntRight;
			counterPreviousLeft = counterIntLeft;

			// Calculate the error for input PID, is like this because the setPointAngle is equal to zero
			errors = -totalCurrentAngle;
			controlActionPIDs();
			flagDatas = false;
		}
		else if(currentDistanceX >= 3000.0f){
			currentDistanceX = 0.0f;
			currentDistanceY = 0.0f;
			sideIndicator++;
			// Stop
			stopMove();
			stopBasicTimer(&handlerSampleTimers);
			sprintf(bufferMandar, "%.3f\n",totalCurrentAngle);
			writeMsg(&usartCmd, bufferMandar);

			if(sideIndicator < sideGoal){
				calculateOffsetGyro(200);

				// Normalize movement conditions and do the correct rotation
				updateDuttyCycle(&handlerPwmRight, duttyBaseRights);
				updateDuttyCycle(&handlerPwmLeft, duttyBaseLefts);
				if(direction == MOVEMENT_CW){
					rotation(MOVEMENT_CW, 90+totalCurrentAngle);
				}
				else{
					rotation(MOVEMENT_CCW, 90-totalCurrentAngle);
				}

				defaultMove();

				// Reset variables
				totalCurrentAngle = 0.0f;
				dataAngle = 0.0f;
				u_controls = 0.0f;
				u_1_controls = 0.0f;
				errors = 0.0f;
				error_1s = 0.0f;
				error_2s = 0.0f;
				differentialDistance = 0.0f;
				counterPreviousRight = 0;
				counterPreviousLeft = 0;

				resetMPU6050();
				calculateOffsetGyro(200);
				startBasicTimer(&handlerSampleTimers);
				startMove();
			}

			else{
				commandx2();
				writeMsg(&usartCmd, "Square 3x3 complete!\n");
			}
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
	handlerBlinkyTimer.TIMx_Config.TIMx_priorityInterrupt	= 6;
	BasicTimer_Config(&handlerBlinkyTimer);
	startBasicTimer(&handlerBlinkyTimer);
	/* Fin del GPIO y Timer del LED de estado
	 * ----------------------------------------*/

	commandConfig(CMD_USART1, USART_BAUDRATE_19200);

	configMotors();
	updateDuttyCycle(&handlerPwmRight, (uint16_t)duttyBaseRights);
	updateDuttyCycle(&handlerPwmLeft, (uint16_t)duttyBaseLefts);

	configMPU6050();

	// Calculamos las constantes del PID
//	kp = (((1.2f*tau)/(k*theta))/2.00f);
//	ti = 2.0f*theta;
//	td = 0.5f*theta;
	q0s = kp*(1.0f+(timeSamples/(2.0f*ti))+(td/timeSamples));
	q1s = -kp*(1.0f-(timeSamples/(2.0f*ti))+((2.0f*td)/timeSamples));
	q2s = (kp*td)/timeSamples;

	handlerSampleTimers.ptrTIMx								= TIM4;
	handlerSampleTimers.TIMx_Config.TIMx_mode				= BTIMER_MODE_UP;
	handlerSampleTimers.TIMx_Config.TIMx_speed				= BTIMER_PLL_100MHz_SPEED_100us;
	handlerSampleTimers.TIMx_Config.TIMx_period				= 200;
	handlerSampleTimers.TIMx_Config.TIMx_interruptEnable	= BTIMER_INTERRUP_ENABLE;
	handlerSampleTimers.TIMx_Config.TIMx_priorityInterrupt	= 6;
	BasicTimer_Config(&handlerSampleTimers);

}


/** Función para calcular la acción de control */
void controlActionPIDs(void){

	u_controls = (u_1_controls)+(q0s*errors)+(q1s*error_1s)+(q2s*error_2s);
	// Control action is limited to a change of 10% Dutty Cycle
	constraintControls(&u_controls, 4000.00f);

	// Control of print data
	if(counter >= 10){
		sprintf(bufferMandar, "%.3f\t%.3f\n",totalCurrentAngle, currentDistanceY);
		writeMsg(&usartCmd, bufferMandar);
		counter = 0;
	}
	else{
		__NOP();
	}
	updateDuttyCycle(&handlerPwmLeft, (uint16_t)(duttyBaseLefts - u_controls));
	updateDuttyCycle(&handlerPwmRight, (uint16_t)(duttyBaseRights + u_controls));

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
	u_1_controls = u_controls;
	error_2s = error_1s;
	error_1s = errors;
}


void constraintControls(float* uControl, float maxChange){
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
	flagTakeOffsets = true;
	startBasicTimer(&handlerSampleTimer);
	while(!(counterSampless >= samples)){
		__NOP();
	}
	offsetAngularVelocity = sumAngularVelocitys/((float)counterSampless);
	// We verify if the offset is not appropiate
	if(fabs(offsetAngularVelocity) >= 0.85f){
		// Then, We repeat the process again
		counterSampless = 0;
		sumAngularVelocitys = 0.0f;
		calculateOffsetGyro(samples);
	}
	else{
		// The offset angle is correct
		stopBasicTimer(&handlerSampleTimer);
		counterSampless = 0;
		sumAngularVelocitys = 0.0f;
		flagTakeOffsets = false;
		sprintf(bufferMandar,"El offset del giroscopio es %.6f °/s\n",offsetAngularVelocity);
		writeMsg(&usartCmd, bufferMandar);
	}
}


/** Interrupción del timer blinky LED*/
void BasicTimer5_Callback(void){
	GPIOxTooglePin(&handlerBlinkyPin); //Cambio el estado del LED PC5
}

void BasicTimer4_Callback(void){
	if(flagTakeOffsets){
		counterSampless++;
		// Gyroscope data is in °/s
		sumAngularVelocitys += getGyroscopeData();
	}
	else{
		counter++;
		flagDatas = true;
	}
}

void usart1Rx_Callback(void){
	usartData = getRxData();
	writeChar(&usartCmd, usartData);
	if((flagMove)&&(usartData == 's')){
		flagMove = false;
		stopMove();
		stopBasicTimer(&handlerSampleTimers);
	}
	else{
		__NOP();
	}
}

void commandx1(void){
	defaultMove();
	updateDuttyCycle(&handlerPwmRight, duttyBaseRights);
	updateDuttyCycle(&handlerPwmLeft, duttyBaseLefts);
	startMove();
}

void commandx2(void){
	stopMove();
	stopBasicTimer(&handlerSampleTimers);
	updateDuttyCycle(&handlerPwmRight, duttyBaseRights);
	updateDuttyCycle(&handlerPwmLeft, duttyBaseLefts);
	flagTakeOffsets = false;
	flagDatas = false;
	sideIndicator = 0;
	sideGoal = 0;
	differentialDistance = 0.0f;
	currentDistanceX = 0.0f;
	currentDistanceY = 0.0f;
	totalCurrentAngle = 0.0f;
	dataAngle = 0.0f;
	u_controls = 0.0f;
	u_1_controls = 0.0f;
	errors = 0.0f;
	error_1s = 0.0f;
	error_2s = 0.0f;
	counterPreviousRight = 0;
	counterPreviousLeft = 0;
	resetMPU6050();
}

void commandx3(void){
	sideGoal = firstParameter;
	direction = secondParameter;
	defaultMove();
	startBasicTimer(&handlerSampleTimers);
	startMove();
}

void commandx4(void){
	stopMove();
}

void commandx5(void){
	duttyBaseRights = firstParameter;
	duttyBaseLefts = secondParameter;
	updateDuttyCycle(&handlerPwmRight, (uint16_t)duttyBaseRights);
	updateDuttyCycle(&handlerPwmLeft, (uint16_t)duttyBaseLefts);
}

void commandx6(void){
	calculateOffsetGyro((uint16_t)firstParameter);
}

void commandx7(void){
	kp = firstParameter;
	ti = secondParameter;
	td = thirdParameter;
	sprintf(bufferMandar,"Kp = %.6f,ti = %.6f,td = %.6f\n",kp, ti, td);
	writeMsg(&usartCmd, bufferMandar);
}
