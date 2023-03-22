/**
 ******************************************************************************
 * @file           : main.c
 * @author         : MillerQUintero2001
 * @brief          : Configuracion Básica de un proyecto
 ******************************************************************************
 * Generacion del archivo de configuración por defecto
 * como plantilla para los proyectos funcionales
 ******************************************************************************
 */

#include <stm32f4xx.h>
#include <stdint.h>
#include "GPIOxDriver.h"
#include "BasicTimer.h"

//Inicializo handler
GPIO_Handler_t handlerOnBoardLed 				= {0};
BasicTimer_Handler_t handlerTimerOnBoardLed 	= {0};

//Inicializo variable bandera
uint8_t flag = 0;

/**
 * Funcion principal del programa
 * Esta función es el corazón del programa
 *
 */

int main(void)
{

	handlerOnBoardLed.pGPIOx = GPIOA;
    handlerOnBoardLed.GPIO_PinConfig.GPIO_PinNumber = PIN_5;
    handlerOnBoardLed.GPIO_PinConfig.GPIO_PinMode 	= GPIO_MODE_OUT;
    handlerOnBoardLed.GPIO_PinConfig.GPIO_PinSpeed 	= GPIO_OSPEED_FAST;
    handlerOnBoardLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OTYPE_PUSHPULL;

	GPIO_Config(&handlerOnBoardLed);

	GPIO_WritePin(&handlerOnBoardLed, SET);

	//Configuración del timer
	handlerTimerOnBoardLed.ptrTIMx							= TIM2;
	handlerTimerOnBoardLed.TIMx_Config.TIMx_mode			= BTIMER_MODE_UP;
	handlerTimerOnBoardLed.TIMx_Config.TIMx_speed			= BTIMER_SPEED_1ms;
	handlerTimerOnBoardLed.TIMx_Config.TIMx_period			= 250;
	handlerTimerOnBoardLed.TIMx_Config.TIMx_interruptEnable = 1;

	BasicTimer_Config(&handlerTimerOnBoardLed);


    /* Loop forever */

	while(1){
		if(flag == 1){
			GPIOxTooglePin(&handlerOnBoardLed);
			flag = 0;
		}
	}
	return 0;
}

void BasicTimer2_Callback(){
	flag=1;
}


