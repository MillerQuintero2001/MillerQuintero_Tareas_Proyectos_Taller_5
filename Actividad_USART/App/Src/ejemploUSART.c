/**
 ******************************************************************************
 * @file           : USART_Test_Main.c
 * @author         : Miller Quintero - miquinterog@unal.edu.co
 * @brief          : Solucion basica de un proyecto con librerias externas
 ******************************************************************************
 * Generacion del archivo de configuracion por defecto
 * como plantilla para los proyectos funcionales
 ******************************************************************************
 */

#include <stm32f4xx.h>
#include <stdint.h>
#include <stdio.h>
#include "GPIOxDriver.h"
#include <BasicTimer.h>
#include <ExtiDriver.h>
#include <USARTxDriver.h>

/* Definición de los handlers necesarios */
GPIO_Handler_t handlerBlinkyPin = 			{0}; // LED de estado del Pin A5
BasicTimer_Handler_t handlerBlinkyTimer = 	{0}; // Timer del LED de estado

GPIO_Handler_t handlerUserButton = 			{0}; // Boton de usuario del Pin C13
EXTI_Config_t handlerUserButtonExti = 		{0}; // Interrupción externa del boton de usuario

USART_Handler_t usart2Comm = 	{0};	// Elemento para hacer comunicación serial

// Handler de los pines empleados para la comunicación serial del USART2
GPIO_Handler_t handlerPinTX	= 	{0}; // Transmisión
GPIO_Handler_t handlerPinRX	= 	{0}; // Recepción


/* Inicializo variables a emplear */
uint8_t flagUserButton = 	0;		// Variable bandera de la interrupción del User Button
uint8_t printMsg = 0;

/* Definición de prototipos de función */
void initSystem(void); 			// Función que inicializa los periféricos básicos

/** Funcion principal del programa */
int main(void){

	initSystem();

    /* Loop forever */
	while(1){

		if(printMsg > 4){
			writeChar(&usart2Comm, 'H');
			writeChar(&usart2Comm, 'O');
			writeChar(&usart2Comm, 'L');
			writeChar(&usart2Comm, 'A');
			writeChar(&usart2Comm, ' ');
			writeChar(&usart2Comm, 'M');
			writeChar(&usart2Comm, 'U');
			writeChar(&usart2Comm, 'N');
			writeChar(&usart2Comm, 'D');
			writeChar(&usart2Comm, 'O');
			printMsg = 0;
		}
	}
	return 0;
}

/** Funcion encargada de iniciar hardware para un pin*/
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
	// Boton de Usario, GPIO y EXTI
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
	usart2Comm.USART_Config.USART_baudrate 		= USART_BAUDRATE_115200;
	usart2Comm.USART_Config.USART_datasize		= USART_DATASIZE_8BIT;
	usart2Comm.USART_Config.USART_parity		= USART_PARITY_NONE;
	usart2Comm.USART_Config.USART_stopbits		= USART_STOPBIT_1;
	usart2Comm.USART_Config.USART_mode			= USART_MODE_RXTX;
	usart2Comm.USART_Config.USART_enableIntRX	= USART_RX_INTERRUP_DISABLE;
	usart2Comm.USART_Config.USART_enableIntTX	= USART_TX_INTERRUP_DISABLE;
	USART_Config(&usart2Comm);


}

/** Interrupción del timer blinky LED*/
void BasicTimer2_Callback(void){
	GPIOxTooglePin(&handlerBlinkyPin); //Cambio el estado del LED PA5
	printMsg++;
}

/** Interrupción externa del User Button */
void callback_extInt13(void){
	flagUserButton = 1;	// Pongo en alto la variable bandera del Exti 13 para el main
}


