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
#include "ExtiDriver.h"
#include "USARTxDriver.h"
#include "SysTickDriver.h"
#include "PwmDriver.h"
#include "I2CDriver.h"

#include "arm_math.h"



/* Definición de los handlers necesarios */

// Elementos para el Blinky LED
GPIO_Handler_t handlerBlinkyPin = 			{0}; // LED de estado del Pin A5
BasicTimer_Handler_t handlerBlinkyTimer = 	{0}; // Timer del LED de estado


// Elementos para hacer la comunicación serial
GPIO_Handler_t handlerPinTX = {0};	// Pin de transmisión de datos
GPIO_Handler_t handlerPinRX = {0};	// Pin de recepción de datos
USART_Handler_t usart2Comm =  {0};	// Comunicación serial
uint8_t sendMsg = 0; // Variable para controlar la comunicación
uint8_t usartData = 0; // Variable en la que se guarda el dato transmitido
char bufferMsg[64] = {0}; // Buffer de datos como un arreglo de caracteres

// Elementos para el PWM
GPIO_Handler_t handlerPinPwmXChannel = {0};
GPIO_Handler_t handlerPinDirX = {0};
PWM_Handler_t handlerSignalXPWM = {0};
GPIO_Handler_t handlerPinPwmYChannel = {0};
GPIO_Handler_t handlerPinDirY = {0};
PWM_Handler_t handlerSignalYPWM = {0};

uint16_t period = 10;
uint16_t dutty = 5;


/* Definición de las cabeceras de funciones del main */
void initSystem(void); 			// Función que inicializa los periféricos básicos
void createSignal(void);		// Función encargada de crear la señal con los parámetros

/** Función principal del programa
 * ¡Esta función es el corazón del programa! */
int main(void){

	// Inicializamos todos los elementos del sistema
	initSystem();

    /* Loop forever */
	while(1){

		if(usartData != '\0'){
			/* Down del Dutty Cycle */
			if(usartData == 'I'){
				enableOutput(&handlerSignalXPWM);
				startPwmSignal(&handlerSignalXPWM);
				enableOutput(&handlerSignalYPWM);
				startPwmSignal(&handlerSignalYPWM);
				usartData = '\0';
			}

			/* Up del Dutty Cycle */
			else if(usartData == 'P'){
				stopPwmSignal(&handlerSignalXPWM);
				stopPwmSignal(&handlerSignalYPWM);
				disableOutput(&handlerSignalXPWM);
				disableOutput(&handlerSignalYPWM);
				usartData = '\0';

			}

			// Cambiar dirección
			else if(usartData == 'D'){
				GPIOxTooglePin(&handlerPinDirX);
				GPIOxTooglePin(&handlerPinDirY);
				usartData = '\0';
			}

			else if(usartData == 'S'){
				period = period+10;
				dutty = period/2;
				updatePeriod(&handlerSignalXPWM, period);
				updateDuttyCycle(&handlerSignalXPWM, dutty);
				updatePeriod(&handlerSignalYPWM, period);
				updateDuttyCycle(&handlerSignalYPWM, dutty);
				usartData = '\0';

			}
			else if(usartData == 'M'){
				period = period-10;
				dutty = period/2;
				updatePeriod(&handlerSignalXPWM, period);
				updateDuttyCycle(&handlerSignalXPWM, dutty);
				updatePeriod(&handlerSignalYPWM, period);
				updateDuttyCycle(&handlerSignalYPWM, dutty);
				usartData = '\0';

			}
			else if(usartData == 'H'){
				updatePeriod(&handlerSignalXPWM, 1000);
				updateDuttyCycle(&handlerSignalXPWM, 500);
				updatePeriod(&handlerSignalYPWM, period);
				updateDuttyCycle(&handlerSignalYPWM, dutty);
				usartData = '\0';
			}

			else{
				usartData = '\0';
			}

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
	handlerBlinkyTimer.TIMx_Config.TIMx_speed				= BTIMER_SPEED_1ms;
	handlerBlinkyTimer.TIMx_Config.TIMx_period				= 250;
	handlerBlinkyTimer.TIMx_Config.TIMx_interruptEnable 	= BTIMER_INTERRUP_ENABLE;
	BasicTimer_Config(&handlerBlinkyTimer);
	/* Fin del GPIO y Timer del LED de estado */


	/* Configuración de pines para el USART2 */
	handlerPinTX.pGPIOx								= GPIOA;
	handlerPinTX.GPIO_PinConfig.GPIO_PinNumber 		= PIN_2;
	handlerPinTX.GPIO_PinConfig.GPIO_PinMode		= GPIO_MODE_ALTFN;
	handlerPinTX.GPIO_PinConfig.GPIO_PinAltFunMode	= AF7;
	GPIO_Config(&handlerPinTX);

	handlerPinRX.pGPIOx								= GPIOA;
	handlerPinRX.GPIO_PinConfig.GPIO_PinNumber 		= PIN_3;
	handlerPinRX.GPIO_PinConfig.GPIO_PinMode		= GPIO_MODE_ALTFN;
	handlerPinRX.GPIO_PinConfig.GPIO_PinAltFunMode	= AF7;
	GPIO_Config(&handlerPinRX);

	/* Configuración de la comunicación serial */
	usart2Comm.ptrUSARTx						= USART2;
	usart2Comm.USART_Config.USART_baudrate 		= USART_BAUDRATE_115200;
	usart2Comm.USART_Config.USART_datasize		= USART_DATASIZE_8BIT;
	usart2Comm.USART_Config.USART_parity		= USART_PARITY_NONE;
	usart2Comm.USART_Config.USART_stopbits		= USART_STOPBIT_1;
	usart2Comm.USART_Config.USART_mode			= USART_MODE_RXTX;
	usart2Comm.USART_Config.USART_enableIntRX	= USART_RX_INTERRUP_ENABLE;
	usart2Comm.USART_Config.USART_enableIntTX	= USART_TX_INTERRUP_DISABLE;
	USART_Config(&usart2Comm);

	/* Configuración del Pin PWM */
	handlerPinPwmXChannel.pGPIOx								= GPIOC;
	handlerPinPwmXChannel.GPIO_PinConfig.GPIO_PinNumber			= PIN_8;
	handlerPinPwmXChannel.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_ALTFN;
	handlerPinPwmXChannel.GPIO_PinConfig.GPIO_PinOPType			= GPIO_OTYPE_PUSHPULL;
	handlerPinPwmXChannel.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
	handlerPinPwmXChannel.GPIO_PinConfig.GPIO_PinSpeed			= GPIO_OSPEED_FAST;
	handlerPinPwmXChannel.GPIO_PinConfig.GPIO_PinAltFunMode		= AF2;
	GPIO_Config(&handlerPinPwmXChannel);

	// Pin de dirección
	handlerPinDirX.pGPIOx								= GPIOC;
	handlerPinDirX.GPIO_PinConfig.GPIO_PinNumber		= PIN_6;
	handlerPinDirX.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_OUT;
	handlerPinDirX.GPIO_PinConfig.GPIO_PinSpeed			= GPIO_OSPEED_FAST;
	handlerPinDirX.GPIO_PinConfig.GPIO_PinOPType		= GPIO_OTYPE_PUSHPULL;
	GPIO_Config(&handlerPinDirX);
	GPIO_WritePin(&handlerPinDirX, RESET);

	/* Configurando el Timer para que genera la señal PWM */
	handlerSignalXPWM.ptrTIMx						= TIM3;
	handlerSignalXPWM.PWMx_Config.PWMx_Channel		= PWM_CHANNEL_3;
	handlerSignalXPWM.PWMx_Config.PWMx_DuttyCicle	= dutty;
	handlerSignalXPWM.PWMx_Config.PWMx_Period		= period;
	handlerSignalXPWM.PWMx_Config.PWMx_Prescaler	= BTIMER_SPEED_100us;
	pwm_Config(&handlerSignalXPWM);


	/* Configuración del Pin PWM */
	handlerPinPwmYChannel.pGPIOx								= GPIOC;
	handlerPinPwmYChannel.GPIO_PinConfig.GPIO_PinNumber			= PIN_7;
	handlerPinPwmYChannel.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_ALTFN;
	handlerPinPwmYChannel.GPIO_PinConfig.GPIO_PinOPType			= GPIO_OTYPE_PUSHPULL;
	handlerPinPwmYChannel.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
	handlerPinPwmYChannel.GPIO_PinConfig.GPIO_PinSpeed			= GPIO_OSPEED_FAST;
	handlerPinPwmYChannel.GPIO_PinConfig.GPIO_PinAltFunMode		= AF2;
	GPIO_Config(&handlerPinPwmYChannel);

	// Pin de dirección
	handlerPinDirY.pGPIOx								= GPIOC;
	handlerPinDirY.GPIO_PinConfig.GPIO_PinNumber		= PIN_4;
	handlerPinDirY.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_OUT;
	handlerPinDirY.GPIO_PinConfig.GPIO_PinSpeed			= GPIO_OSPEED_FAST;
	handlerPinDirY.GPIO_PinConfig.GPIO_PinOPType		= GPIO_OTYPE_PUSHPULL;
	GPIO_Config(&handlerPinDirY);
	GPIO_WritePin(&handlerPinDirY, RESET);

	/* Configurando el Timer para que genera la señal PWM */
	handlerSignalYPWM.ptrTIMx						= TIM3;
	handlerSignalYPWM.PWMx_Config.PWMx_Channel		= PWM_CHANNEL_2;
	handlerSignalYPWM.PWMx_Config.PWMx_DuttyCicle	= dutty;
	handlerSignalYPWM.PWMx_Config.PWMx_Period		= period;
	handlerSignalYPWM.PWMx_Config.PWMx_Prescaler	= BTIMER_SPEED_100us;
	pwm_Config(&handlerSignalYPWM);

}


/** Interrupción del timer blinky LED*/
void BasicTimer2_Callback(void){
	GPIOxTooglePin(&handlerBlinkyPin); //Cambio el estado del LED PA5
}


/** Interrupción del USART2 */
void usart2Rx_Callback(void){
	usartData = getRxData();	// Pongo en alto la variable bandera del USART2 para el main
	writeChar(&usart2Comm, usartData);
}



