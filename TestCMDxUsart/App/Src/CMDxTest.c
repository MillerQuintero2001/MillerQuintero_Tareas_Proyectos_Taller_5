/**
 ******************************************************************************
 * @file           : BasicProject_Main.c
 * @author         : Miller Quintero - miquinterog@unal.edu.co
 * @brief          : Solución básica de un proyecto con librerías externas
 ******************************************************************************
 * Generación del archivo de configuración por defecto
 * como plantilla para los proyectos funcionales
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
#include "CMDxDriver.h"
#include "PwmDriver.h"
#include "PLLDriver.h"


/* Definición de los handlers necesarios */

// Elementos para el Blinky LED
GPIO_Handler_t handlerBlinkyPin = 			{0}; // LED de estado del Pin A5
BasicTimer_Handler_t handlerBlinkyTimer = 	{0}; // Timer del LED de estado

// Elementos para hacer la comunicación serial
uint8_t usartData = 0; 				// Variable en la que se guarda el dato transmitido
uint8_t sendMsg = 0; 			// Variable para controlar la comunicación
char bufferMsg[64] = {0}; 		// Buffer de datos como un arreglo de caracteres
char bufferData[64] = "Mensaje para enviar";

// Elementos para PWM
PWM_Handler_t handlerPwmTest = {0};
GPIO_Handler_t handlerPinPwm = {0};
uint16_t freq = 100;

/* Definición de las cabeceras de funciones del main */
void initSystem(void); 			// Función que inicializa los periféricos básicos


/** Función principal del programa
 * ¡Esta función es el corazón del programa! */
int main(void){

	/* Configuramos el equipo a 100MHz, usando el driver del PLL */
	configPLL(100);

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

	// Inicializamos todos los elementos del sistema
	initSystem();
	commandConfig(CMD_USART1);

    /* Loop forever */
	while(1){

		if(usartData != '\0'){
			writeChar(&usartComm, usartData);
			commandBuild(usartData);
			usartData = '\0';
		}
		else{
			__NOP();
		}


	}
	return 0;
}

/** Función encargada de iniciar hardware para un pin*/
void initSystem(void){

	/* Activamos el Coprocesador Matemático - FPU */
	SCB->CPACR |= (0XF << 20);

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
	handlerBlinkyTimer.ptrTIMx								= TIM2;
	handlerBlinkyTimer.TIMx_Config.TIMx_mode				= BTIMER_MODE_UP;
	handlerBlinkyTimer.TIMx_Config.TIMx_speed				= BTIMER_PLL_100MHz_SPEED_100us;
	handlerBlinkyTimer.TIMx_Config.TIMx_period				= 2500;
	handlerBlinkyTimer.TIMx_Config.TIMx_interruptEnable 	= BTIMER_INTERRUP_ENABLE;
	BasicTimer_Config(&handlerBlinkyTimer);
	/* Fin del GPIO y Timer del LED de estado
	 * ----------------------------------------*/

	/* Configuración del PWM */

	// Parámetros de la señal
	handlerPwmTest.ptrTIMx 							= TIM3;
	handlerPwmTest.PWMx_Config.PWMx_Channel 		= PWM_CHANNEL_1;
	handlerPwmTest.PWMx_Config.PWMx_Prescaler 		= BTIMER_SPEED_1ms;
	handlerPwmTest.PWMx_Config.PWMx_Period 			= freq;
	handlerPwmTest.PWMx_Config.PWMx_DuttyCicle 		= 90;
	handlerPwmTest.PWMx_Config.PWMx_Polarity 		= PWM_POLARITY_ACTIVE_HIGH;
	pwm_Config(&handlerPwmTest);

	// Configuración del PIN
	handlerPinPwm.pGPIOx 								= GPIOA;
	handlerPinPwm.GPIO_PinConfig.GPIO_PinNumber 		= PIN_6;
	handlerPinPwm.GPIO_PinConfig.GPIO_PinMode 			= GPIO_MODE_ALTFN;
	handlerPinPwm.GPIO_PinConfig.GPIO_PinOPType 		= GPIO_OTYPE_PUSHPULL;
	handlerPinPwm.GPIO_PinConfig.GPIO_PinPuPdControl 	= GPIO_PUPDR_NOTHING;
	handlerPinPwm.GPIO_PinConfig.GPIO_PinSpeed 			= GPIO_OSPEED_FAST;
	handlerPinPwm.GPIO_PinConfig.GPIO_PinAltFunMode		= AF2;
	GPIO_Config(&handlerPinPwm);

	enableOutput(&handlerPwmTest);
	startPwmSignal(&handlerPwmTest);

}



/** Interrupción del timer blinky LED*/
void BasicTimer2_Callback(void){
	__NOP();
}


/** Interrupción del USART2 */
void usart1Rx_Callback(void){
	usartData = getRxData();
}

void commandx1(void){
	GPIOxTooglePin(&handlerBlinkyPin); //Cambio el estado del LED PA5
}

//void commandx2(void){
//	setPolarity(&handlerPwmTest, PWM_POLARITY_ACTIVE_HIGH);
//}
//
//void commandx3(void){
//	setPolarity(&handlerPwmTest, PWM_POLARITY_ACTIVE_LOW);
//}




