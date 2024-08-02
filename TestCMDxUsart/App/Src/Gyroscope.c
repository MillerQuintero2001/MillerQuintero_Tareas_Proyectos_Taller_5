/**
 ******************************************************************************
 * @file           : Gyroscope.c
 * @author         : Miller Quintero - miquinterog@unal.edu.co
 * @brief          : Oppy haciendo lintea
 ******************************************************************************
 * Pruebas driver de MPU6050, calibración del giroscopio y línea recta
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
#include "MPU6050.h"

// Elementos para el Blinky LED
GPIO_Handler_t handlerBlinkyPin = 			{0}; // LED de estado del Pin A5
BasicTimer_Handler_t handlerBlinkyTimer = 	{0}; // Timer del LED de estado

char bufferPrint[64] = {0};
float offsetGyro = 0.0f;
uint32_t numberOfSamples = 0;
float time = 0.020f;
// Timer de muestreo
BasicTimer_Handler_t handlerSampleTimer = {0};

/* Definición de las cabeceras de funciones del main */
void initSystem(void); 										// Función que inicializa los periféricos básicos

int main(void){

	// Inicializamos todos los elementos del sistema
	initSystem();
	sprintf(bufferPrint,"La frecuencia de reloj es %lu Hz\n", getConfigPLL());
	writeMsg(&usartCmd, bufferPrint);

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

	commandConfig(CMD_USART2, USART_BAUDRATE_115200);

//	configMotors();
//	setSignals(25, 30);

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
	GPIOxTooglePin(&handlerBlinkyPin); //Cambio el estado del LED PA5
}

/** Interrupción del Timer de muestreo */
void BasicTimer4_Callback(void){
	uint32_t counter = 0;
	while(counter < numberOfSamples){
		float angularVelocity = getGyroscopeData();
		sprintf(bufferPrint,"El dato #%lu del giroscopio es %.6f°/s\n",counter+1,angularVelocity);
		writeMsg(&usartCmd, bufferPrint);
		float angle = (angularVelocity - offsetGyro)*(time);
		sprintf(bufferPrint,"El dato #%lu del ángulo es %.6f°\n",counter+1,angle);
		writeMsg(&usartCmd, bufferPrint);
		counter++;
	}
	stopBasicTimer(&handlerSampleTimer);
}

void usart2Rx_Callback(void){
	usartData = getRxData();
	writeChar(&usartCmd, usartData);
//	if((flagMove)&&(usartData == 's')){
//		flagMove = false;
//	}
//	else{
//		__NOP();
//	}
}

void commandx1(void){
	offsetGyro = getGyroscopeOffset(20000);
	sprintf(bufferPrint,"El offset del giroscopio es %.6f °/s\n",offsetGyro);
	writeMsg(&usartCmd, bufferPrint);
}

void commandx2(void){
	numberOfSamples = firstParameter;
	startBasicTimer(&handlerSampleTimer);
}

void commandx3(void){
	float dataGyro = getGyroscopeData();
	sprintf(bufferPrint,"El dato del giroscopio es %.6f °/s\n",dataGyro);
	writeMsg(&usartCmd, bufferPrint);
}

void commandx4(void){
	stopMove();
	stopBasicTimer(&handlerSampleTimer);
}



