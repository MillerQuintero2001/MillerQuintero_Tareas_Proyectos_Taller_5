/*
 * PwmDriver.c
 *
 *  Created on: May 4, 2023
 *      Author: MillerQuintero2001
 */
#include "PwmDriver.h"

/* Variable que guarda la referencia del periférico que se esta utilizando*/
TIM_TypeDef	*ptrTimerPWM;

/* Función principal de configuración del PWM */
void pwm_Config(PWM_Handler_t *ptrPwmHandler){

	// Guardo la referencia al periférico que se está utilizando
	ptrTimerPWM = ptrPwmHandler->ptrTIMx;

	/* 1. Activar la señal de reloj del periférico requerido */
	if(ptrTimerPWM == TIM2){
		// Registro del RCC que nos activa la señal de reloj para el TIM2
		RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	}
	else if(ptrTimerPWM == TIM3){
		// Registro del RCC que nos activa la señal de reloj para el TIM3
		RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	}
	else if(ptrTimerPWM == TIM4){
		// Registro del RCC que nos activa la señal de reloj para el TIM4
		RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
	}
	else if(ptrTimerPWM == TIM5){
		// Registro del RCC que nos activa la señal de reloj para el TIM5
		RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
	}
	else{
		__NOP();
	}

	/* 1. Cargamos la frecuencia deseada */
	setPeriod(ptrPwmHandler);

	/* 2. Cargamos el valor del dutty-Cycle*/
	setDuttyCycle(ptrPwmHandler);

	/* 2a. Estamos en UP_Mode, el limite se carga en ARR y se comienza en 0 */
	// Habilitamos el conteo como upcounter
	ptrPwmHandler->ptrTIMx->CR1 &= ~TIM_CR1_DIR;
	// Como es upcounter mode, el counter empieza en 0
	ptrPwmHandler->ptrTIMx->CNT = 0;

	/* 3. Configuramos los bits CCxS del registro TIMy_CCMR1, de forma que sea modo salida
	 * (para cada canal hay un conjunto CCxS)
	 *
	 * 4. Además, en el mismo "case" podemos configurar el modo del PWM, su polaridad...
	 *
	 * 5. Y además activamos el preload bit, para que cada vez que exista un update-event
	 * el valor cargado en el CCRx será recargado en el registro "shadow" del PWM */
	switch(ptrPwmHandler->PWMx_Config.PWMx_Channel){
	case PWM_CHANNEL_1:{
		// Seleccionamos como salida el canal
		ptrTimerPWM->CCMR1 &= ~TIM_CCMR1_CC1S;
		// Personalmente deseo activar el modo de comparación rápida
		ptrTimerPWM->CCMR1 |= TIM_CCMR1_OC1FE;
		// Configuramos el canal como PWM modo 1
		ptrTimerPWM->CCMR1 |= (0b110)<<4;
		// Activamos la funcionalidad de pre-load
		ptrTimerPWM->CCMR1 |= TIM_CCMR1_OC1PE;
		// Seleccionamos polaridad convencional (no invertida)
		ptrTimerPWM->CCER &= ~TIM_CCER_CC1P;
		// Configuramos la polaridad
		setPolarity(ptrPwmHandler, ptrPwmHandler->PWMx_Config.PWMx_Polarity);
		break;
	}

	case PWM_CHANNEL_2:{
		// Seleccionamos como salida el canal
		ptrTimerPWM->CCMR1 &= ~TIM_CCMR1_CC2S;
		// Personalmente deseo activar el modo de comparación rápida
		ptrTimerPWM->CCMR1 |= TIM_CCMR1_OC2FE;
		// Configuramos el canal como PWM modo 1
		ptrTimerPWM->CCMR1 |= (0b110)<<12;
		// Activamos la funcionalidad de pre-load
		ptrTimerPWM->CCMR1 |= TIM_CCMR1_OC2PE;
		// Seleccionamos polaridad convencional (no invertida)
		ptrTimerPWM->CCER &= ~TIM_CCER_CC2P;
		// Configuramos la polaridad
		setPolarity(ptrPwmHandler, ptrPwmHandler->PWMx_Config.PWMx_Polarity);
		break;
	}

	case PWM_CHANNEL_3:{
		// Seleccionamos como salida el canal
		ptrTimerPWM->CCMR2 &= ~TIM_CCMR2_CC3S;
		// Personalmente deseo activar el modo de comparación rápida
		ptrTimerPWM->CCMR2 |= TIM_CCMR2_OC3FE;
		// Configuramos el canal como PWM
		ptrTimerPWM->CCMR2 |= (0b110)<<4;
		// Activamos la funcionalidad de pre-load
		ptrTimerPWM->CCMR2 |= TIM_CCMR2_OC3PE;
		// Seleccionamos polaridad convencional (no invertida)
		ptrTimerPWM->CCER &= ~TIM_CCER_CC3P;
		// Configuramos la polaridad
		setPolarity(ptrPwmHandler, ptrPwmHandler->PWMx_Config.PWMx_Polarity);
		break;
	}

	case PWM_CHANNEL_4:{
		// Seleccionamos como salida el canal
		ptrTimerPWM->CCMR2 &= ~TIM_CCMR2_CC4S;
		// Personalmente deseo activar el modo de comparación rápida
		ptrTimerPWM->CCMR2 |= TIM_CCMR2_OC4FE;
		// Configuramos el canal como PWM
		ptrTimerPWM->CCMR2 |= (0b110)<<12;
		// Activamos la funcionalidad de pre-load
		ptrTimerPWM->CCMR2 |= TIM_CCMR2_OC4PE;
		// Seleccionamos polaridad convencional (no invertida)
		ptrTimerPWM->CCER &= ~TIM_CCER_CC4P;
		// Configuramos la polaridad
		setPolarity(ptrPwmHandler, ptrPwmHandler->PWMx_Config.PWMx_Polarity);
		break;
	}

	default:{
		__NOP();
		break;
	}

	}// fin del switch-case



//	/* 6. Activamos la salida seleccionada */
//	enableOutput(ptrPwmHandler);
}

/** Función para activar el Timer y activar todo el módulo PWM */
void startPwmSignal(PWM_Handler_t *ptrPwmHandler) {
	ptrTimerPWM->CR1 |= TIM_CR1_CEN;
}

/** Función para desactivar el Timer y detener todo el módulo PWM*/
void stopPwmSignal(PWM_Handler_t *ptrPwmHandler) {
	ptrTimerPWM->CR1 &= ~TIM_CR1_CEN;
}

/** Función encargada de activar cada uno de los canales con los que cuenta el TimerX */
void enableOutput(PWM_Handler_t *ptrPwmHandler) {
	switch (ptrPwmHandler->PWMx_Config.PWMx_Channel) {
	case PWM_CHANNEL_1: {
		// Activamos la salida del canal 1
		ptrTimerPWM->CCER |= TIM_CCER_CC1E;
		break;
	}
	case PWM_CHANNEL_2: {
		// Activamos la salida del canal 1
		ptrTimerPWM->CCER |= TIM_CCER_CC2E;
		break;
	}
	case PWM_CHANNEL_3: {
		// Activamos la salida del canal 1
		ptrTimerPWM->CCER |= TIM_CCER_CC3E;
		break;
	}
	case PWM_CHANNEL_4: {
		// Activamos la salida del canal 1
		ptrTimerPWM->CCER |= TIM_CCER_CC4E;
		break;
	}
	default: {
		__NOP();
		break;
	}
	}
}

void disableOutput(PWM_Handler_t *ptrPwmHandler){
	switch (ptrPwmHandler->PWMx_Config.PWMx_Channel) {
		case PWM_CHANNEL_1: {
			// Activamos la salida del canal 1
			ptrTimerPWM->CCER &= ~TIM_CCER_CC1E;
			break;
		}
		case PWM_CHANNEL_2: {
			// Activamos la salida del canal 1
			ptrTimerPWM->CCER &= ~TIM_CCER_CC2E;
			break;
		}
		case PWM_CHANNEL_3: {
			// Activamos la salida del canal 1
			ptrTimerPWM->CCER &= ~TIM_CCER_CC3E;
			break;
		}
		case PWM_CHANNEL_4: {
			// Activamos la salida del canal 1
			ptrTimerPWM->CCER &= ~TIM_CCER_CC4E;
			break;
		}
		default: {
			__NOP();
			break;
		}
		}
}

/**
 * La frecuencia es definida por el conjunto formado por el preescaler (PSC)
 * y el valor límite al que llega el Timer (ARR), con estos dos se establece
 * la frecuencia.
 * */
void setPeriod(PWM_Handler_t *ptrPwmHandler){

	// Cargamos el valor del prescaler, nos define la velocidad (en ns) a la cual se incrementa el timer
	ptrTimerPWM->PSC = ptrPwmHandler->PWMx_Config.PWMx_Prescaler;

	// Cargamos el valor del ARR, el cual es el límite de incrementos del Timer antes de hacer un update y reload.
	ptrTimerPWM->ARR = (ptrPwmHandler->PWMx_Config.PWMx_Period) - 1;

	ptrTimerPWM->CR1 |= TIM_CR1_ARPE;
}


/** Función para actualizar la frecuencia, funciona de la mano con setFrequency */
void updatePeriod(PWM_Handler_t *ptrPwmHandler, uint16_t newPeriod){
	// Actualizamos el registro que manipula el periodo
	ptrPwmHandler->PWMx_Config.PWMx_Period = newPeriod;

	// Llamamos a la función que cambia la frecuencia
	setPeriod(ptrPwmHandler);
}

/** El valor del dutty debe estar dado en valores de %, entre 0% y 100%*/
void setDuttyCycle(PWM_Handler_t *ptrPwmHandler){

	// Seleccionamos el canal para configurar su dutty
	switch(ptrPwmHandler->PWMx_Config.PWMx_Channel){
	case PWM_CHANNEL_1:{
		ptrTimerPWM->CCR1 = (ptrPwmHandler->PWMx_Config.PWMx_DuttyCicle) -1;
		break;
	}
	case PWM_CHANNEL_2:{
		ptrTimerPWM->CCR2 = (ptrPwmHandler->PWMx_Config.PWMx_DuttyCicle) -1;
		break;
	}
	case PWM_CHANNEL_3:{
		ptrTimerPWM->CCR3 = (ptrPwmHandler->PWMx_Config.PWMx_DuttyCicle) -1;
		break;
	}
	case PWM_CHANNEL_4:{
		ptrTimerPWM->CCR4 = (ptrPwmHandler->PWMx_Config.PWMx_DuttyCicle) -1;
		break;
	}
	default:{
		break;
	}

	}// fin del switch-case

}


/** Función para actualizar el Dutty, funciona de la mano con setDuttyCycle */
void updateDuttyCycle(PWM_Handler_t *ptrPwmHandler, uint16_t newDutty){
	// Actualizamos el registro que manipula el dutty
	ptrPwmHandler->PWMx_Config.PWMx_DuttyCicle = newDutty;

	// Llamamos a la fucnión que cambia el dutty y cargamos el nuevo valor
	setDuttyCycle(ptrPwmHandler);
}

/** Función encargada de establecer la polaridad deseada */
void setPolarity(PWM_Handler_t *ptrPwmHandler, uint8_t polarity){

	// Seleccionamos el canal para cambiar la polaridad
	switch(ptrPwmHandler->PWMx_Config.PWMx_Channel){
	case PWM_CHANNEL_1:{
		ptrTimerPWM->CCER &= (~TIM_CCER_CC1P);	// Limpio
		ptrTimerPWM->CCER |= (TIM_CCER_CC1P);	// Escribo
		break;
	}
	case PWM_CHANNEL_2:{
		ptrTimerPWM->CCER &= (~TIM_CCER_CC2P); 	// Limpio
		ptrTimerPWM->CCER |= (TIM_CCER_CC2P);	// Escribo
		break;
	}
	case PWM_CHANNEL_3:{
		ptrTimerPWM->CCER &= (~TIM_CCER_CC3P);	// Limpio
		ptrTimerPWM->CCER |= (TIM_CCER_CC3P);	// Escribo
		break;
	}
	case PWM_CHANNEL_4:{
		ptrTimerPWM->CCER &= (~TIM_CCER_CC4P);	// Limpio
		ptrTimerPWM->CCER |= (TIM_CCER_CC4P);	// Escribo
		break;
	}
	default:{
		break;
	}
	}
}

/** Función para invertir la polaridad de la señal */
void tooglePolarity(PWM_Handler_t *ptrPwmHandler){

	// Seleccionamos el canal para cambiar la polaridad
	switch(ptrPwmHandler->PWMx_Config.PWMx_Channel){
	case PWM_CHANNEL_1:{
		ptrTimerPWM->CCER ^= (TIM_CCER_CC1P);
		break;
	}
	case PWM_CHANNEL_2:{
		ptrTimerPWM->CCER ^= (TIM_CCER_CC2P);
		break;
	}
	case PWM_CHANNEL_3:{
		ptrTimerPWM->CCER ^= (TIM_CCER_CC3P);
		break;
	}
	case PWM_CHANNEL_4:{
		ptrTimerPWM->CCER ^= (TIM_CCER_CC4P);
		break;
	}
	default:{
		break;
	}
	}
}






