/*
 * mainEXTI.c
 *
 *  Created on: 23/03/2023
 *      Author: miller
 */

#include "stm32f4xx.h"
#include <stdint.h>
#include "GPIOxDriver.h"
#include "BasicTimer.h"

/* Definición de los elementos del programa*/
GPIO_Handler_t handler_Led = {0}; //PA5
GPIO_Handler_t handler_UserButton = {0}; //PC13

BasicTimer_Handler_t handlerBlinkyTimer = {0};

uint32_t counterExti13 = 0;

/* Prototipo de función */
void init_Hardware(void);
void callback_exti13(void);
void EXTI15_10_IRQHandler(void);

int main (void)
{

	init_Hardware();

	while(1){

	}
	return 0;
}

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

	/* Configurando boton de usario */
	handler_UserButton.pGPIOx								= GPIOC;
	handler_UserButton.GPIO_PinConfig.GPIO_PinNumber 		= PIN_13;
	handler_UserButton.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_IN;
	//handler_UserButton.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;

	/* 1er punto terminado con esto */
	/* Cargando la configuración */
	GPIO_Config(&handler_UserButton);

	/* 2a. Hay que activar señal de reloj del SYSCFG */
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	/* 2b. Configurar el multiplexor 13, asignando el puerto GPIOC */
	SYSCFG->EXTICR[3] &= ~(0xF << SYSCFG_EXTICR4_EXTI13_Pos); // Limpiamos la posicion del mux13
	SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI13_PC; //Asignamos el puerto C

	/* 3. Configurar el EXTI */
	//Vamos a hacerlo con flanco de subida, que es cuando presionado el boton, lo suelto
	EXTI->FTSR = 0; //Desactivando TODAS las posibles detecciones de flancos de bajada
	EXTI->RTSR = 0; //Llevamos a un valor conocido el registro
	EXTI->RTSR |= EXTI_RTSR_TR13; // Activadno la detección de flanco de subida de la entrada 13
	EXTI->IMR = 0;
	EXTI->IMR |= EXTI_IMR_IM13;

	/* 4a. Desactivar las interrupciones globales */
	__disable_irq();

	/* 4b. Incluir la interrupcion en el NVIC */
	NVIC_EnableIRQ(EXTI15_10_IRQn);

	/* 4c. Crear ISR y callback */
	/* 4d. Activamos las interrupciones globales */
	__enable_irq();



}

void EXTI15_10_IRQHandler(void){
	/* Verificamos la interrupción */
	// Solamente se entra en el condicional cuando la interrupción del 13 es la que se activó
	if((EXTI->PR & EXTI_PR_PR13) != 0){
		EXTI->PR |= EXTI_PR_PR13; // Bajando la bandera de la interrupción
		callback_exti13();
	}
}

void callback_exti13(void){
	counterExti13++;
}



void BasicTimer2_Callback(void){
	GPIOxTooglePin(&handler_Led);
}
