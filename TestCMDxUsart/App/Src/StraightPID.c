/**
 ******************************************************************************
 * @file           : StraightLine.c
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
#include "SysTickDriver.h"
#include "CMDxDriver.h"
#include "MotorDriver.h"


/* Definición de los handlers necesarios */

// Elementos para el Blinky LED
GPIO_Handler_t handlerBlinkyPin = 			{0}; // LED de estado del Pin A5
BasicTimer_Handler_t handlerBlinkyTimer = 	{0}; // Timer del LED de estado

bool flag = false;
bool flagInit = true;
BasicTimer_Handler_t handlerSampleTimer = {0};

uint16_t counter = 1;
char bufferMandar[64] = {0};

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

float velocityRight = 0.0f;
float velocityLeft = 0.0f;

/* Definición de las cabeceras de funciones del main */
void initSystem(void); 										// Función que inicializa los periféricos básicos
void controlActionPID(void);								// Función que retorna el valor de la acción de control
float mapControlDutty(float u_input);	// Función que mapea la acción de control a un % Dutty Cycle
void constraintError(void);									// Funcíon que limita el error para que no sea superior al 100%
void constraintControl(void);								// Función que limita el valor de la acción de control

/** Función principal del programa
 * ¡Esta función es el corazón del programa! */
int main(void){

	// Inicializamos todos los elementos del sistema
	initSystem();

    /* Loop forever */
	while(1){
		commandBuild(USE_DEFAULT);
		if(flagPID){
			vR = (((float)(counterIntRight-counterPreviousRight))*((M_PI*51.70f)/120.00f))/(timeSample);
			vL = (((float)(counterIntLeft-counterPreviousLeft))*((M_PI*51.75f)/120.00f))/(timeSample);
			sprintf(bufferMandar, "%.2f\t %.2f\n", velocityRight, velocityLeft);
			writeMsg(&usartCmd, bufferMandar);
			controlActionPID();
			//sprintf(bufferMandar, "%.2f\n", u_control);
			//writeMsg(&usartCmd, bufferMandar);
			counterPreviousRight = counterIntRight;
			counterPreviousLeft = counterIntLeft;
			flagPID = false;
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

	/* Configuramos el SysTick */
	config_SysTick_ms(PLL_CLOCK_100_CONFIGURED);

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

	configMotors();

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
	handlerSampleTimer.TIMx_Config.TIMx_period				= 500;
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
	GPIOxTooglePin(&handlerBlinkyPin); //Cambio el estado del LED PA5
}

void BasicTimer4_Callback(void){
	if(flagInit){
		stopBasicTimer(&handlerSampleTimer);
		handlerSampleTimer.TIMx_Config.TIMx_period	= 300;
		BasicTimer_Config(&handlerSampleTimer);
		flagInit = false;
		counterIntRight = 0;
		counterIntLeft = 0;
		startBasicTimer(&handlerSampleTimer);
	}
	else{
		flagPID = true;
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

void commandx1(void){
	setPointVelocity = firstParameter;
	setSignals(25, 30);
}

void commandx2(void){
	startBasicTimer(&handlerSampleTimer);
	startMove();
}

void commandx4(void){
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










