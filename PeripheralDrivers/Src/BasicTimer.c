/*
 * BasicTimer.c
 *
 *  Created on: Apr 18, 2022
 *      Author: MillerQUintero2001
 */

#include <BasicTimer.h>

/* Variable que guarda la referencia del periférico que se esta utilizando*/
TIM_TypeDef	*ptrTimerUsed;

/* Función en la que cargamos la configuración del Timer
 * Recordar que siempre se debe comenzar con activar la señal de reloj
 * del periférico que se está utilizando.
 * Además, en este caso, debemos ser cuidadosos al momento de utilizar las interrupciones.
 * Los Timer están conectados directamente al elemento NVIC del Cortex-Mx
 * Debemos configurar y/o utilizar:
 *  - TIMx_CR1  (control Register 1)
 *  - TIMx_SMCR ( slave mode control register) -> mantener en 0 para modo Timer Básico
 *  - TIMx_DIER (DMA and Interrupt enable register)
 *  - TIMx_SR (Status register)
 *  - TIMx_CNT (Counter)
 *  - TIMx_PSC (Pre-scaler)
 *  - TIMx_ARR  (Auto-reload register)
 *
 *  Como vamos a trabajar con interrupciones, antes de configurar una nueva, debemos desactivar
 *  el sistema global de interrupciones, activar la IRQ específica y luego volver a encender
 *  el sistema.
 */
void BasicTimer_Config(BasicTimer_Handler_t *ptrBTimerHandler){
	// Guardamos una referencia al periferico que estamos utilizando...
	ptrTimerUsed = ptrBTimerHandler->ptrTIMx;

	/* 0. Desactivamos las interrupciones globales mientras configuramos el sistema.*/
	__disable_irq();
	/* 1. Activar la señal de reloj del periférico requerido */
	if(ptrTimerUsed == TIM2){
		// Registro del RCC que nos activa la señal de reloj para el TIM2
		RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	}
	else if(ptrTimerUsed == TIM3){
		// Registro del RCC que nos activa la señal de reloj para el TIM3
		RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	}
	else if(ptrTimerUsed == TIM4){
		// Registro del RCC que nos activa la señal de reloj para el TIM4
		RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
	}
	else if(ptrTimerUsed == TIM5){
		// Registro del RCC que nos activa la señal de reloj para el TIM5
		RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
	}
	else{
		__NOP();
	}

	/* 2. Configuramos el Pre-scaler
	 * Recordar que el prescaler nos indica la velocidad a la que se incrementa el counter, de forma que
	 * periodo_incremento * veces_incremento_counter = periodo_update
	 * Modificar el valor del registro PSC en el TIM utilizado
	 */
	ptrTimerUsed->PSC = ptrBTimerHandler->TIMx_Config.TIMx_speed;

	/* 3. Configuramos la dirección del counter (up/down)*/
	if(ptrBTimerHandler->TIMx_Config.TIMx_mode == BTIMER_MODE_UP){

		/* 3a. Estamos en UP_Mode, el limite se carga en ARR y se comienza en 0 */
		// Configurar el registro que nos controla el modo up or down
		ptrTimerUsed->CR1 &= ~TIM_CR1_DIR;

		/* 3b. Configuramos el Auto-reload. Este es el "limite" hasta donde el CNT va a contar */
		ptrTimerUsed->ARR = ptrBTimerHandler->TIMx_Config.TIMx_period - 1;

		/* 3c. Reiniciamos el registro counter*/
		ptrTimerUsed->CNT = 0;
	}

	else{
		/* 3a. Estamos en DOWN_Mode, el limite se carga en ARR (0) y se comienza en un valor alto
		 * Trabaja contando en direccion descendente*/
		ptrTimerUsed->CR1 |= TIM_CR1_DIR;

		/* 3b. Configuramos el Auto-reload. Este es el "limite" hasta donde el CNT va a contar
		 * En modo descendente, con numero positivos, cual es el minimo valor al que ARR puede llegar*/
		ptrTimerUsed->ARR = 0;

		/* 3c. Reiniciamos el registro counter
		 * Este es el valor con el que el counter comienza */
		ptrTimerUsed->CNT = ptrBTimerHandler->TIMx_Config.TIMx_period - 1;
	}

	/* 4. Activamos el Timer (el CNT debe comenzar a contar)*/
	ptrTimerUsed->CR1 |= TIM_CR1_CEN;

	/* 5. Activamos la interrupción debida al Timerx Utilizado
	 * Modificar el registro encargado de activar la interrupcion generada por el TIMx*/
	/* Escriba codigo aca */
	if(ptrBTimerHandler->TIMx_Config.TIMx_interruptEnable == BTIMER_INTERRUP_ENABLE){
		ptrTimerUsed->DIER |= TIM_DIER_UIE;
	}
	else{
		ptrTimerUsed->DIER &= ~(TIM_DIER_UIE);
	}

	/* 6. Activamos el canal del sistema NVIC para que lea la interrupción*/
	if(ptrTimerUsed == TIM2){
		// Activando en NVIC para la interrupción del TIM2
		NVIC_EnableIRQ(TIM2_IRQn);
		__NVIC_SetPriority(TIM2_IRQn, ptrBTimerHandler->TIMx_Config.TIMx_priorityInterrupt);
	}
	else if(ptrTimerUsed == TIM3){
		// Activando en NVIC para la interrupción del TIM3
		NVIC_EnableIRQ(TIM3_IRQn);
		__NVIC_SetPriority(TIM3_IRQn, ptrBTimerHandler->TIMx_Config.TIMx_priorityInterrupt);
	}
	else if(ptrTimerUsed == TIM4){
		// Activando en NVIC para la interrupción del TIM3
		NVIC_EnableIRQ(TIM4_IRQn);
		__NVIC_SetPriority(TIM4_IRQn, ptrBTimerHandler->TIMx_Config.TIMx_priorityInterrupt);
	}
	else if(ptrTimerUsed == TIM5){
		// Activando en NVIC para la interrupción del TIM3
		NVIC_EnableIRQ(TIM5_IRQn);
		__NVIC_SetPriority(TIM5_IRQn, ptrBTimerHandler->TIMx_Config.TIMx_priorityInterrupt);
	}
	else{
		__NOP();
	}

	/* 7. Volvemos a activar las interrupciones del sistema */
	__enable_irq();
}


__attribute__((weak)) void BasicTimer2_Callback(void){
	  /* NOTE : Esta función no debe ser modificada, cuando el callback la necesite,
	            el BasicTimer2_Callback puede ser implementado en el archivo main
	   */
	__NOP();
}

__attribute__((weak)) void BasicTimer3_Callback(void){
	  /* NOTE : Esta función no debe ser modificada, cuando el callback la necesite,
		            el BasicTimer3_Callback puede ser implementado en el archivo main
	   */
	__NOP();
}

__attribute__((weak)) void BasicTimer4_Callback(void){
	  /* NOTE : Esta función no debe ser modificada, cuando el callback la necesite,
		            el BasicTimer4_Callback puede ser implementado en el archivo main
	   */
	__NOP();
}

__attribute__((weak)) void BasicTimer5_Callback(void){
	  /* NOTE : Esta función no debe ser modificada, cuando el callback la necesite,
		            el BasicTimer5_Callback puede ser implementado en el archivo main
	   */
	__NOP();
}


/** Funciones de Interrupt Request Handler
 * en si mismas contienen la acción de bajar la bandera
 * y la función Callback del respectivo timer, con esto
 * es suficiente para construir el (Interrupt Service Routine) */


/* Esta es la función a la que apunta el sistema en el vector de interrupciones.
 * Se debe utilizar usando exactamente el mismo nombre definido en el vector de interrupciones,
 * Al hacerlo correctamente, el sistema apunta a esta función y cuando la interrupción se lanza
 * el sistema inmediatamente salta a este lugar en la memoria*/
void TIM2_IRQHandler(void){
	/* Limpiamos la bandera que indica que la interrupción se ha generado */
	TIM2->SR &= ~TIM_SR_UIF;

	/* LLamamos a la función que se debe encargar de hacer algo con esta interrupción*/
	BasicTimer2_Callback();

}

void TIM3_IRQHandler(void){
	/* Limpiamos la bandera que indica que la interrupción se ha generado */
	TIM3->SR &= ~TIM_SR_UIF;

	/* LLamamos a la función que se debe encargar de hacer algo con esta interrupción*/
	BasicTimer3_Callback();

}

void TIM4_IRQHandler(void){
	/* Limpiamos la bandera que indica que la interrupción se ha generado */
	TIM4->SR &= ~TIM_SR_UIF;

	/* LLamamos a la función que se debe encargar de hacer algo con esta interrupción*/
	BasicTimer4_Callback();

}

void TIM5_IRQHandler(void){
	/* Limpiamos la bandera que indica que la interrupción se ha generado */
	TIM5->SR &= ~TIM_SR_UIF;

	/* LLamamos a la función que se debe encargar de hacer algo con esta interrupción*/
	BasicTimer5_Callback();

}



