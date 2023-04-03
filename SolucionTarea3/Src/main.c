/**
 ******************************************************************************
 * @file           : main.c
 * @author         : MillerQUintero2001
 * @brief          : Tarea #3 Driver EXTI y Timers
 ******************************************************************************
 * Generacion del archivo de configuración por defecto
 * como plantilla para los proyectos funcionales
 ******************************************************************************
 */

#include <BasicTimer.h>
#include <ExtiDriver.h>
#include <stm32f4xx.h>
#include <stdint.h>
#include "GPIOxDriver.h"


/* Inicializo handler de los periféricos a utilizar */
GPIO_Handler_t handlerLedState = {0}; // LED de estado, que estará conectado al PIN A5 y será blinky
BasicTimer_Handler_t handlerLedStateTimer = {0}; // Timer del LED de estado

GPIO_Handler_t handlerTransistorUnd = {0}; // Transistor que controla alimentación para las unidades del display
GPIO_Handler_t handlerTransistorDec = {0}; // Transistor que controla alimentación para las decenas del display
BasicTimer_Handler_t handlerSwitching = {0}; //Timer para el el suicheo de transistores en el display

// Handler para el segmento 'a' del display con el Pin ...
// Handler para el segmento 'b' del display con el Pin ...
// Handler para el segmento 'c' del display con el Pin ...
// Handler para el segmento 'd' del display con el Pin ...
// Handler para el segmento 'e' del display con el Pin ...
// Handler para el segmento 'f' del display con el Pin ...
// Handler para el segmento 'g' del display con el Pin ...

/* Inicializo variables a emplear */
uint8_t flagLedState = 0;	// Variable bandera timer del LED de estado
uint8_t flagSwitch = 0;		// Variable bandera suicheo de transistores

/* Definición de prototipos de función */
void init_Hardware(void); 					// Función que inicializa los periféricos
void display_Segun_Digito(uint8_t digito);	// Función encargada de encender los leds del display según el número
/**
 * Funcion principal del programa
 * Esta función es el corazón del programa
 *
 */
int main(void){

	init_Hardware();

    /* Loop forever */
	while(1){

		// Blinky Led de Estado
		if(flagLedState == 1){
			GPIOxTooglePin(&handlerLedState);
			flagLedState = 0;
		}

//		// Suicheo transistores
//		if(flagSwitch == 1){
//			GPIOxTooglePin(&handlerTransistorUnd);
//			GPIOxTooglePin(&handlerTransistorDec);
//			flagSwitch = 0;
//		}




	}
}

/* Funcion encargada de iniciar hardware para un pin*/
void init_Hardware(void){

	/* GPIO y Timer del Blinky Led de Estado */
	handlerLedState.pGPIOx								= GPIOA;
	handlerLedState.GPIO_PinConfig.GPIO_PinNumber 		= PIN_5;
	handlerLedState.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_OUT;
	handlerLedState.GPIO_PinConfig.GPIO_PinSpeed 		= GPIO_OSPEED_FAST;
	handlerLedState.GPIO_PinConfig.GPIO_PinOPType 		= GPIO_OTYPE_PUSHPULL;
	// Cargo la configuración
	GPIO_Config(&handlerLedState);
	// Pongo estado en alto
	GPIO_WritePin(&handlerLedState, SET);
	// Atributos para el Timer 2 del LED de estado
	handlerLedStateTimer.ptrTIMx							= TIM2;
	handlerLedStateTimer.TIMx_Config.TIMx_mode				= BTIMER_MODE_UP;
	handlerLedStateTimer.TIMx_Config.TIMx_speed				= BTIMER_SPEED_1ms;
	handlerLedStateTimer.TIMx_Config.TIMx_period			= 250;
	handlerLedStateTimer.TIMx_Config.TIMx_interruptEnable 	= 1;

	BasicTimer_Config(&handlerLedStateTimer);
	/* Fin del GPIO y Timer del LED de estado
	 * ----------------------------------------*/

	/* GPIO's y Timer para el suicheo de Transistores*/
	// Transistor Unidades
	handlerTransistorUnd.pGPIOx								= GPIOA;
	handlerTransistorUnd.GPIO_PinConfig.GPIO_PinNumber 		= PIN_6;
	handlerTransistorUnd.GPIO_PinConfig.GPIO_PinMode		= GPIO_MODE_OUT;
	handlerTransistorUnd.GPIO_PinConfig.GPIO_PinSpeed 		= GPIO_OSPEED_FAST;
	handlerTransistorUnd.GPIO_PinConfig.GPIO_PinOPType 		= GPIO_OTYPE_PUSHPULL;
	// Cargo la configuración
	GPIO_Config(&handlerTransistorUnd);
	// Pongo estado en alto para comenzar con unidades
	GPIO_WritePin(&handlerTransistorUnd, SET);

	// Transistor Decenas
	handlerTransistorDec.pGPIOx								= GPIOA;
	handlerTransistorDec.GPIO_PinConfig.GPIO_PinNumber 		= PIN_7;
	handlerTransistorDec.GPIO_PinConfig.GPIO_PinMode		= GPIO_MODE_OUT;
	handlerTransistorDec.GPIO_PinConfig.GPIO_PinSpeed 		= GPIO_OSPEED_FAST;
	handlerTransistorDec.GPIO_PinConfig.GPIO_PinOPType 		= GPIO_OTYPE_PUSHPULL;
	// Cargo la configuración
	GPIO_Config(&handlerTransistorDec);
	// Pongo estado en bajo ya que comienzo con unidades, y para garantizar que cuando uno este en alto el otro en bajo
	GPIO_WritePin(&handlerTransistorDec, RESET);

	// Atributos para el Timer 3 suicheo de transistores
	handlerSwitching.ptrTIMx							= TIM3;
	handlerSwitching.TIMx_Config.TIMx_mode				= BTIMER_MODE_UP;
	handlerSwitching.TIMx_Config.TIMx_speed				= BTIMER_SPEED_1ms;
	handlerSwitching.TIMx_Config.TIMx_period			= 33;
	handlerSwitching.TIMx_Config.TIMx_interruptEnable 	= 1;
	BasicTimer_Config(&handlerSwitching);
	/* Fin del GPIO Transistores y Timer del suicheo
	 * ----------------------------------------------*/
}

void BasicTimer2_Callback(void){
	flagLedState = 1;
}

void BasicTimer3_Callback(void){
	flagSwitch = 1;
}

void display_Segun_Digito(uint8_t digito){
	switch(digito){
	case 0: {
		break;
	}
	case 1: {
		break;
	}
	case 2: {
		break;
	}
	case 3: {
		break;
	}
	case 4: {
		break;
	}
	case 5: {
		break;
	}
	case 6: {
		break;
	}
	case 7: {
		break;
	}
	case 8: {
		break;
	}
	case 9: {
		break;
	}
	default: {
		__NOP();
		break;
	}
	}
}

