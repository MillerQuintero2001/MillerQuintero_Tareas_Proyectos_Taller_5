/**
 ******************************************************************************
 * @file           : CMDxTest.c
 * @author         : Miller Quintero - miquinterog@unal.edu.co
 * @brief          : Solución básica de un proyecto con librerías externas
 ******************************************************************************
 * Pruebas driver de comandos y PWM con el polarity
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
#include "PwmDriver.h"
#include "PLLDriver.h"
#include "SysTickDriver.h"
#include "CMDxDriver.h"
#include "MotorDriver.h"


/* Definición de los handlers necesarios */

// Elementos para el Blinky LED
GPIO_Handler_t handlerBlinkyPin = 			{0}; // LED de estado del Pin A5
BasicTimer_Handler_t handlerBlinkyTimer = 	{0}; // Timer del LED de estado

// Elementos para PWM
PWM_Handler_t handlerPwmTest = {0};
GPIO_Handler_t handlerPinPwm = {0};

/* Definición de las cabeceras de funciones del main */
void initSystem(void); 			// Función que inicializa los periféricos básicos

/** Función principal del programa
 * ¡Esta función es el corazón del programa! */
int main(void){

	// Inicializamos todos los elementos del sistema
	initSystem();

    /* Loop forever */
	while(1){
		commandBuild(USE_OPPY);
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
	/* Fin del GPIO y Timer del LED de estado
	 * ----------------------------------------*/

	/* Configuración del PWM */

	// Configuración del PIN
	handlerPinPwm.pGPIOx 								= GPIOA;
	handlerPinPwm.GPIO_PinConfig.GPIO_PinNumber 		= PIN_6;
	handlerPinPwm.GPIO_PinConfig.GPIO_PinMode 			= GPIO_MODE_ALTFN;
	handlerPinPwm.GPIO_PinConfig.GPIO_PinOPType 		= GPIO_OTYPE_PUSHPULL;
	handlerPinPwm.GPIO_PinConfig.GPIO_PinPuPdControl 	= GPIO_PUPDR_NOTHING;
	handlerPinPwm.GPIO_PinConfig.GPIO_PinSpeed 			= GPIO_OSPEED_FAST;
	handlerPinPwm.GPIO_PinConfig.GPIO_PinAltFunMode		= AF2;
	GPIO_Config(&handlerPinPwm);

	// Parámetros de la señal
	handlerPwmTest.ptrTIMx 							= TIM3;
	handlerPwmTest.PWMx_Config.PWMx_Channel 		= PWM_CHANNEL_1;
	handlerPwmTest.PWMx_Config.PWMx_Prescaler 		= BTIMER_PLL_100MHz_SPEED_100us;
	handlerPwmTest.PWMx_Config.PWMx_Period 			= 10;
	handlerPwmTest.PWMx_Config.PWMx_DuttyCicle 		= 9;
	handlerPwmTest.PWMx_Config.PWMx_Polarity 		= PWM_POLARITY_ACTIVE_HIGH;
	pwm_Config(&handlerPwmTest);

	startPwmSignal(&handlerPwmTest);

	commandConfig(CMD_USART1, USART_BAUDRATE_19200);

	configMotors();
}

/** Interrupción del timer blinky LED*/
void BasicTimer5_Callback(void){
	GPIOxTooglePin(&handlerBlinkyPin); //Cambio el estado del LED PA5
}

void usart1Rx_Callback(void){
	usartData = getRxData();
	writeChar(&usartCmd, usartData);
}

void commandx1(void){
	stopPwmSignal(&handlerPwmTest);
}

void commandx2(void){
	setPolarity(&handlerPwmTest, PWM_POLARITY_ACTIVE_HIGH);
}

void commandx3(void){
	setPolarity(&handlerPwmTest, PWM_POLARITY_ACTIVE_LOW);
}

void commandx5(void){
	startPwmSignal(&handlerPwmTest);
}

void commandx6(void){
	tooglePolarity(&handlerPwmTest);
	writeMsg(&usartCmd, "Toogle Polarity realizado");
}








