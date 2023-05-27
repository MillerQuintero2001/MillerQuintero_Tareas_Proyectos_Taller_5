/**
 ******************************************************************************
 * @file           : Testing_PWM_Main.c
 * @author         : Miller Quintero - miquinterog@unal.edu.co
 * @brief          : Testing a basic PWM
 ******************************************************************************
 * Probar driver de señales PWM de los Timers
 ******************************************************************************
 */

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

#define HSI_CLOCK_CONFIGURED	0	// 16MHz
#define HSE_CLOCK_CONFIGURED	1
#define PLL_CLOCK_CONFIGURED	2

/* Definición de los handlers necesarios */

// Elementos para el Blinky LED
GPIO_Handler_t handlerBlinkyPin = 			{0}; // LED de estado del Pin A5
BasicTimer_Handler_t handlerBlinkyTimer = 	{0}; // Timer del LED de estado

// Elementos para la interrupción externa del User Button
GPIO_Handler_t handlerUserButton = 			{0}; // Boton de usuario del Pin C13
EXTI_Config_t handlerUserButtonExti = 		{0}; // Interrupción externa del botón de usuario

// Elementos para hacer la comunicación serial
GPIO_Handler_t handlerPinTX = {0};	// Pin de transmisión de datos
GPIO_Handler_t handlerPinRX = {0};	// Pin de recepción de datos
USART_Handler_t usart2Comm =  {0};	// Comunicación serial
uint8_t sendMsg = 0; // Variable para controlar la comunicación
uint8_t usart2DataReceived = 0; // Variable en la que se guarda el dato transmitido
char bufferMsg[64] = {0}; // Buffer de datos como un arreglo de caracteres

// Elementos para el PWM
GPIO_Handler_t handlerPinPwmChannel = {0};
PWM_Handler_t handlerSignalPWM = {0};

/* Inicializo variables a emplear */
uint8_t flagUserButton = 0;	// Variable bandera de la interrupción del User Button
uint8_t flagUsart2 = 0;	// Variable bandera de la interrupción del USART2
uint16_t duttyValue = 1500; // Variable que guarda el ciclo de trabajo del PWM


/* Definición de las cabeceras de funciones del main */
void initSystem(void); 			// Función que inicializa los periféricos básicos

/** Función principal del programa
 * ¡Esta función es el corazón del programa! */
int main(void){

	/* Activamos el Coprocesador Matemático - FPU */
	SCB->CPACR |= (0XF << 20); // Esto es escribir 0b1111 en bits del 20 al 23, activando los 2 CP necesarios

	// Inicializamos todos los elementos del sistema
	initSystem();

    /* Loop forever */
	while(1){

//		if(sendMsg > 4){
//			writeMsg(&usart2Comm, "Probando...");
//			sendMsg = 0;
//		}

		if(usart2DataReceived != '\0'){
			/* Down del Dutty Cycle */
			if(usart2DataReceived == 'D'){
				duttyValue -= 50;
				updateDuttyCycle(&handlerSignalPWM, duttyValue);

			}

			/* Up del Dutty Cycle */
			else if(usart2DataReceived == 'U'){
				duttyValue += 50;
				updateDuttyCycle(&handlerSignalPWM, duttyValue);

			}

			// Imprimimos el mensaje
			sprintf(bufferMsg, "dutty = %u \n", (unsigned int)duttyValue);
			writeMsg(&usart2Comm, bufferMsg);
			// Cambio de nuevo a carácter nulo
			usart2DataReceived = '\0';

		}

	}
	return 0;
}

/** Función encargada de iniciar hardware para un pin*/
void initSystem(void){

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
	/* Fin del GPIO y Timer del LED de estado
	 * ----------------------------------------*/

	/* GPIO's y EXTI's*/
	// Botón de Usario, GPIO y EXTI
	handlerUserButton.pGPIOx							= GPIOC;
	handlerUserButton.GPIO_PinConfig.GPIO_PinNumber 	= PIN_13;
	handlerUserButton.GPIO_PinConfig.GPIO_PinMode		= GPIO_MODE_IN; // Entrada
	handlerUserButtonExti.pGPIOHandler					= &handlerUserButton;
	handlerUserButtonExti.edgeType						= EXTERNAL_INTERRUPT_RISING_EDGE; // Detecto flanco de subida en el clock
	extInt_Config(&handlerUserButtonExti);

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
	usart2Comm.USART_Config.USART_baudrate 		= USART_BAUDRATE_19200;
	usart2Comm.USART_Config.USART_datasize		= USART_DATASIZE_8BIT;
	usart2Comm.USART_Config.USART_parity		= USART_PARITY_NONE;
	usart2Comm.USART_Config.USART_stopbits		= USART_STOPBIT_1;
	usart2Comm.USART_Config.USART_mode			= USART_MODE_RXTX;
	usart2Comm.USART_Config.USART_enableIntRX	= USART_RX_INTERRUP_ENABLE;
	usart2Comm.USART_Config.USART_enableIntTX	= USART_TX_INTERRUP_DISABLE;
	USART_Config(&usart2Comm);

	/* Configuración del Pin PWM */
	handlerPinPwmChannel.pGPIOx								= GPIOC;
	handlerPinPwmChannel.GPIO_PinConfig.GPIO_PinNumber		= PIN_7;
	handlerPinPwmChannel.GPIO_PinConfig.GPIO_PinMode		= GPIO_MODE_ALTFN;
	handlerPinPwmChannel.GPIO_PinConfig.GPIO_PinOPType		= GPIO_OTYPE_PUSHPULL;
	handlerPinPwmChannel.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
	handlerPinPwmChannel.GPIO_PinConfig.GPIO_PinSpeed		= GPIO_OSPEED_FAST;
	handlerPinPwmChannel.GPIO_PinConfig.GPIO_PinAltFunMode	= AF2;
	GPIO_Config(&handlerPinPwmChannel);

	/* Configurando el Timer para que genera la señal PWM */
	handlerSignalPWM.ptrTIMx						= TIM3;
	handlerSignalPWM.PWMx_Config.PWMx_Channel		= PWM_CHANNEL_2;
	handlerSignalPWM.PWMx_Config.PWMx_DuttyCicle	= duttyValue;
	handlerSignalPWM.PWMx_Config.PWMx_Period		= 20000;
	handlerSignalPWM.PWMx_Config.PWMx_Prescaler		= 16;
	pwm_Config(&handlerSignalPWM);

	/* Activando la señal */
	enableOutput(&handlerSignalPWM);
	startPwmSignal(&handlerSignalPWM);
}

/** Interrupción del timer blinky LED*/
void BasicTimer2_Callback(void){
	GPIOxTooglePin(&handlerBlinkyPin); //Cambio el estado del LED PA5
	//sendMsg++;
}

/** Interrupción externa del User Button */
void callback_extInt13(void){
	__NOP();	// Pongo en alto la variable bandera del Exti 13 para el main
}

/** Interrupción del USART2 */
void usart2Rx_Callback(void){
	usart2DataReceived = getRxData();	// Pongo en alto la variable bandera del USART2 para el main
}



