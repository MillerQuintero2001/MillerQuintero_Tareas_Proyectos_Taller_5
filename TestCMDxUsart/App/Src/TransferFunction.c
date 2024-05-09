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
float velocityRight = 0.0f;
float velocityLeft = 0.0f;
uint16_t counter = 1;
//float duttyPer = 15.00;
char bufferMandar[64] = {0};

uint32_t counterPreviousRight = 0;
uint32_t counterPreviousLeft = 0;

/* Definición de las cabeceras de funciones del main */
void initSystem(void); 			// Función que inicializa los periféricos básicos

/** Función principal del programa
 * ¡Esta función es el corazón del programa! */
int main(void){

	// Inicializamos todos los elementos del sistema
	initSystem();

    /* Loop forever */
	while(1){
		commandBuild(USE_DEFAULT);
		if(counter > 601){
			stopMove();
			stopBasicTimer(&handlerSampleTimer);
			counter = 1;
			flag = false;
		}
		else if(flag){
			velocityRight = (((float)(counterIntRight - counterPreviousRight))*((M_PI*51.70f)/120.00f))/(0.025f);
			velocityLeft = (((float)(counterIntLeft - counterPreviousLeft))*((M_PI*51.75f)/120.00f))/(0.025f);
			sprintf(bufferMandar, "%.2f\t %.2f\n", velocityRight, velocityLeft);
			writeMsg(&usartCmd, bufferMandar);
			counterPreviousRight = counterIntRight;
			counterPreviousLeft = counterIntLeft;
			counter++;
			flag = false;
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

	handlerSampleTimer.ptrTIMx								= TIM4;
	handlerSampleTimer.TIMx_Config.TIMx_mode				= BTIMER_MODE_UP;
	handlerSampleTimer.TIMx_Config.TIMx_speed				= BTIMER_PLL_100MHz_SPEED_100us;
	handlerSampleTimer.TIMx_Config.TIMx_period				= 250;
	handlerSampleTimer.TIMx_Config.TIMx_interruptEnable		= BTIMER_INTERRUP_ENABLE;
	BasicTimer_Config(&handlerSampleTimer);
}

/** Interrupción del timer blinky LED*/
void BasicTimer5_Callback(void){
	GPIOxTooglePin(&handlerBlinkyPin); //Cambio el estado del LED PA5
}

void BasicTimer4_Callback(void){
	flag = true;
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
	startBasicTimer(&handlerSampleTimer);
	startMove();
}










