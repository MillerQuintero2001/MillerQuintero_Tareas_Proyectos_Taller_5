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
#include "ExtiDriver.h"

//Definición de los elementos del programa
GPIO_Handler_t handler_Led = {0}; //PA5
BasicTimer_Handler_t handlerBlinkyTimer = {0};

EXTI_Config_t handlerExtiPC13 = {0};

uint32_t counterExti13 = 0;
// Definición de prototipo de función
void init_Hardware(void);


int main(void)
{
	init_Hardware();

	//Defino plantilla de configuración para interrupciones con el NVIC usando el EXTI para el PC13 (User_Button)
	handlerExtiPC13.edgeType										= EXTERNAL_INTERRUPT_RISING_EDGE;
	handlerExtiPC13.pGPIOHandler->pGPIOx							= GPIOC;
	handlerExtiPC13.pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber		= PIN_13;
	handlerExtiPC13.pGPIOHandler->GPIO_PinConfig.GPIO_PinMode		= GPIO_MODE_IN;

	//Cargo la información en la función que configura la interrupción
	extInt_Config(&handlerExtiPC13);
	while(1){

	}
	return 0;
}


/* Funcion encargada de iniciar hardware para un pin*/
void init_Hardware(void){
	handler_Led.pGPIOx								= GPIOA;
	handler_Led.GPIO_PinConfig.GPIO_PinNumber 		= PIN_5;
	handler_Led.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_OUT;
	handler_Led.GPIO_PinConfig.GPIO_PinSpeed 		= GPIO_OSPEED_FAST;
	handler_Led.GPIO_PinConfig.GPIO_PinOPType 		= GPIO_OTYPE_PUSHPULL;

	/* Cargo la configuración */
	GPIO_Config(&handler_Led);
	/* Pongo estado en alto */
	GPIO_WritePin(&handler_Led, SET);

	handlerBlinkyTimer.ptrTIMx							= TIM2;
	handlerBlinkyTimer.TIMx_Config.TIMx_mode			= BTIMER_MODE_UP;
	handlerBlinkyTimer.TIMx_Config.TIMx_speed			= BTIMER_SPEED_1ms;
	handlerBlinkyTimer.TIMx_Config.TIMx_period			= 250;
	handlerBlinkyTimer.TIMx_Config.TIMx_interruptEnable = 1;

	BasicTimer_Config(&handlerBlinkyTimer);
}

void callback_extInt13(void){
	counterExti13++;
}


void BasicTimer2_Callback(void){
	GPIOxTooglePin(&handler_Led);
}
