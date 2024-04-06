/**
 ******************************************************************************
 * @file           : CounterInterrupts.c
 * @author         : Miller Quintero - miquinterog@unal.edu.co
 * @brief          : Código para toma de datos encoder
 ******************************************************************************
 * Esta aplicación sen encarga de por medio de timers, EXTI's, PWM contar
 * las interrupciones el fotointerruptor, para diversos dutycycle y frecuencias
 ******************************************************************************
 */

#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>

#include <stm32f4xx.h>

#include "GPIOxDriver.h"
#include "BasicTimer.h"
#include "USARTxDriver.h"
#include "ExtiDriver.h"
#include "PLLDriver.h"
#include "PwmDriver.h"
#include "CMDxDriver.h"

#include "arm_math.h"


/* Definición de los handlers necesarios */

// Pin del MCO para medir el micro
GPIO_Handler_t handlerMCO = {0};

// Elementos para el Blinky LED
GPIO_Handler_t handlerBlinkyPin = 			{0}; // LED de estado del Pin A5
BasicTimer_Handler_t handlerBlinkyTimer = 	{0}; // Timer del LED de estado

// Elementos para conteo encoder
EXTI_Config_t encoderLeft = {0};
uint16_t period = 4000;
uint8_t indexes = 0;
uint8_t indicator = 1;
uint16_t counterExti = 0;
uint16_t data[30] = {0};
uint8_t flagTimerSample = 0;
uint8_t flag25Hz = 1;
uint8_t flag50Hz = 0;
uint8_t flag100Hz = 0;
BasicTimer_Handler_t handlerSampleTimer = {0};


// Elementos para hacer la comunicación serial
uint8_t usartData = 0; 				// Variable en la que se guarda el dato transmitido
uint8_t sendMsg = 0; 			// Variable para controlar la comunicación
char bufferMsg[64] = {0}; 		// Buffer de datos como un arreglo de caracteres
char bufferData[64] = "Mensaje para enviar";

// Elementos para PWM
PWM_Handler_t handlerPwmSignal1 = {0};
GPIO_Handler_t handlerPinPwm1 = {0};

uint16_t clock = 100;

/* Definición de las cabeceras de funciones del main */
void initSystem(void); 			// Función que inicializa los periféricos básicos


/** Función principal del programa
 * ¡Esta función es el corazón del programa! */
int main(void){

	// Inicializamos todos los elementos del sistema
	initSystem();
	commandConfig();

    /* Loop forever */
	while(1){

		if(flagTimerSample){
			stopPwmSignal(&handlerPwmSignal1);
			disableOutput(&handlerPwmSignal1);
			data[indexes] = counterExti;
			indexes++;
			indicator++;
			counterExti = 0;
			if(indexes == 10){
				indicator = 1;
				period = 2000;
				updatePeriod(&handlerPwmSignal1, period);
			}
			else if(indexes == 20){
				indicator = 1;
				period = 1000;
				updatePeriod(&handlerPwmSignal1, period);
			}
			else{
				__NOP();
			}
			updateDuttyCycle(&handlerPwmSignal1, (period*indicator)/100);
			enableOutput(&handlerPwmSignal1);
			startPwmSignal(&handlerPwmSignal1);
			flagTimerSample = 0;
		}

	}
	return 0;
}

/** Función encargada de iniciar hardware para un pin*/
void initSystem(void){

	/* Activamos el Coprocesador Matemático - FPU */
	SCB->CPACR |= (0XF << 20);

	/* Configuramos el equipo a 100MHz, usando el driver del PLL */
	configPLL(clock);

	/* Calibración manual del HSI para tener mayor precisión:
	 * Midiendo con el LED de estado y haciendo cálculos, considerando el pre-escaler y auto-reload escogidos
	 * para el timer que controla el blinky, se llegó a una frecuencia real de 101044394,86532803052349080092 Hz
	 * según la bibliografía consultada, los bits HSITRIM[4:0] que son los del 3 al 7 del RCC_CR, están por
	 * defecto en un valor de 16, incrementar en 1 binario aumenta X% del HSI la frecuencia real, y decrementar
	 * en 1 binario, disminuye X% del HSI la frecuencia real.
	 * Haciendo pruebas se llego a que este es el valor con el que queda mejor calibrado */
	RCC->CR &= ~(0b11111 << RCC_CR_HSITRIM_Pos); 	// Limpio
	RCC->CR |= (13 << RCC_CR_HSITRIM_Pos);			// Escribo

	//Esperamos hasta que el HSI vuelva a ser estable
	while(!(RCC->CR & RCC_CR_HSIRDY)){
		__NOP();
	}
	// Fin de la calibración

	/* Configuración del Pin para el MC01 */
	handlerMCO.pGPIOx								= GPIOA;
	handlerMCO.GPIO_PinConfig.GPIO_PinNumber		= PIN_8;
	handlerMCO.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_ALTFN;
	handlerMCO.GPIO_PinConfig.GPIO_PinOPType		= GPIO_OTYPE_PUSHPULL;
	handlerMCO.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
	handlerMCO.GPIO_PinConfig.GPIO_PinSpeed			= GPIO_OSPEED_FAST;
	handlerMCO.GPIO_PinConfig.GPIO_PinAltFunMode	= AF0;
	GPIO_Config(&handlerMCO);
	changeMCO1(MCO1_PLL_CLOCK, MCO1_DIVIDED_BY_5);

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
	// Atributos para el Timer 2 del LED de estado
	handlerBlinkyTimer.ptrTIMx								= TIM4;
	handlerBlinkyTimer.TIMx_Config.TIMx_mode				= BTIMER_MODE_UP;
	handlerBlinkyTimer.TIMx_Config.TIMx_speed				= BTIMER_PLL_100MHz_SPEED_100us;
	handlerBlinkyTimer.TIMx_Config.TIMx_period				= 2500;
	handlerBlinkyTimer.TIMx_Config.TIMx_interruptEnable 	= BTIMER_INTERRUP_ENABLE;
	BasicTimer_Config(&handlerBlinkyTimer);
	/* Fin del GPIO y Timer del LED de estado
	 * ----------------------------------------*/

	/* Configuración del Timer de muestreo */
	handlerSampleTimer.ptrTIMx							= TIM5;
	handlerSampleTimer.TIMx_Config.TIMx_mode			= BTIMER_MODE_UP;
	handlerSampleTimer.TIMx_Config.TIMx_speed 			= BTIMER_SPEED_1ms;
	handlerSampleTimer.TIMx_Config.TIMx_period			= 10000;
	handlerSampleTimer.TIMx_Config.TIMx_interruptEnable	= BTIMER_INTERRUP_ENABLE;
	BasicTimer_Config(&handlerSampleTimer);

	/* Configuración del PWM motor 1*/

	// Pin de la señal
	handlerPinPwm1.pGPIOx								= GPIOA;
	handlerPinPwm1.GPIO_PinConfig.GPIO_PinNumber		= PIN_0;
	handlerPinPwm1.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_ALTFN;
	handlerPinPwm1.GPIO_PinConfig.GPIO_PinOPType		= GPIO_OTYPE_PUSHPULL;
	handlerPinPwm1.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
	handlerPinPwm1.GPIO_PinConfig.GPIO_PinSpeed			= GPIO_OSPEED_FAST;
	handlerPinPwm1.GPIO_PinConfig.GPIO_PinAltFunMode	= AF1;
	GPIO_Config(&handlerPinPwm1);

	// Configuración inicial PWM
	handlerPwmSignal1.ptrTIMx						= TIM2;
	handlerPwmSignal1.PWMx_Config.PWMx_Channel		= PWM_CHANNEL_1;
	handlerPwmSignal1.PWMx_Config.PWMx_Prescaler 	= BTIMER_SPEED_10us;
	handlerPwmSignal1.PWMx_Config.PWMx_Period		= period;
	handlerPwmSignal1.PWMx_Config.PWMx_DuttyCicle	= (period*indicator)/100;
	handlerPwmSignal1.PWMx_Config.PWMx_Polarity		= PWM_POLARITY_ACTIVE_HIGH;
	pwm_Config(&handlerPwmSignal1);

	enableOutput(&handlerPwmSignal1);
	startPwmSignal(&handlerPwmSignal1);


}

/** Interrupción del timer count-encoder */
void BasicTimer5_Callback(void){
	flagTimerSample = 1;
}

/** Interrupción del timer blinky LED */
void BasicTimer4_Callback(void){
	GPIOxTooglePin(&handlerBlinkyPin); //Cambio el estado del LED PA5
}


/** Interrupción del USART2 */
void usart2Rx_Callback(void){
	usartData = getRxData();
}




