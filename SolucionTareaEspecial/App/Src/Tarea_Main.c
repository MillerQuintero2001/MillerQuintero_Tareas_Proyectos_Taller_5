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
#include "PLLDriver.h"

#define HSI_CLOCK_CONFIGURED	0;	// 16MHz
#define HSE_CLOCK_CONFIGURED	1;
#define PLL_CLOCK_CONFIGURED	2;

/* Definición de los handlers necesarios */

// Elementos para el Blinky LED
GPIO_Handler_t handlerBlinkyPin = 			{0}; // LED de estado del Pin A5
BasicTimer_Handler_t handlerBlinkyTimer = 	{0}; // Timer del LED de estado
int freq = 800;
int clock = 80;
uint8_t bandera = 0;


//// Elementos para la interrupción externa del User Button
//GPIO_Handler_t handlerUserButton = 			{0}; // Boton de usuario del Pin C13
//EXTI_Config_t handlerUserButtonExti = 		{0}; // Interrupción externa del botón de usuario
//
// Elementos para hacer la comunicación serial
GPIO_Handler_t handlerPinTX = {0};	// Pin de transmisión de datos
GPIO_Handler_t handlerPinRX = {0};	// Pin de recepción de datos
USART_Handler_t usart1Comm =  {0};	// Comunicación serial
uint8_t sendMsg = 0; // Variable para controlar la comunicación
uint8_t usart1RxData = 0; // Variable en la que se guarda el dato transmitido
char bufferMsg[64] = {0}; // Buffer de datos como un arreglo de caracteres
char bufferData[64] = "Mensaje para enviar";
//
//// Elementos del SysTick
//uint32_t systemTicks = 0;
//uint32_t systemTicksStart = 0;
//uint32_t systemTicksEnd = 0;
//
//// Elementos para utilizar comunicación I2C
//GPIO_Handler_t handlerI2C_SDA = {0};
//GPIO_Handler_t handlerI2C_SCL = {0};
//I2C_Handler_t handlerSensor = {0};
//uint8_t i2cBuffer = 0;
//
///* Inicializo variables a emplear */
//uint8_t flagUserButton = 0;	// Variable bandera de la interrupción del User Button
//uint8_t flagUsart2 = 0;	// Variable bandera de la interrupción del USART2

/* Definición de las cabeceras de funciones del main */
void initSystem(void); 			// Función que inicializa los periféricos básicos

/** Función principal del programa
 * ¡Esta función es el corazón del programa! */
int main(void){

	// Inicializamos todos los elementos del sistema
	initSystem();

	config_SysTick_ms(2);

    /* Loop forever */
	while(1){
//
//		if(flagUserButton){
//			flagUserButton = 0;
//			freq++;
//		}
//
//		if(flagUsart2){
//			flagUsart2 = 0;
//		}
		bandera = (USART2->CR1 & USART_CR1_TXEIE);
		freq = (unsigned int)getConfigPLL();
		sprintf(bufferMsg, "La frecuencia configurada es de %u Hz \n", freq);
		if(sendMsg > 4){
			enableTXEIE(&usart1Comm);
			sendMsg = 0;
		}

	}
	return 0;
}

/** Función encargada de iniciar hardware para un pin*/
void initSystem(void){

	/* Activamos el Coprocesador Matemático - FPU */
	SCB->CPACR |= (0XF << 20);

	configPLL(clock);

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
	handlerBlinkyTimer.TIMx_Config.TIMx_speed				= BTIMER_PLL_80MHz_SPEED_100us;
	handlerBlinkyTimer.TIMx_Config.TIMx_period				= 2500;
	handlerBlinkyTimer.TIMx_Config.TIMx_interruptEnable 	= BTIMER_INTERRUP_ENABLE;
	BasicTimer_Config(&handlerBlinkyTimer);
	/* Fin del GPIO y Timer del LED de estado
	 * ----------------------------------------*/

//	/* GPIO's y EXTI's*/
//	// Botón de Usario, GPIO y EXTI
//	handlerUserButton.pGPIOx							= GPIOC;
//	handlerUserButton.GPIO_PinConfig.GPIO_PinNumber 	= PIN_13;
//	handlerUserButton.GPIO_PinConfig.GPIO_PinMode		= GPIO_MODE_IN; // Entrada
//	handlerUserButtonExti.pGPIOHandler					= &handlerUserButton;
//	handlerUserButtonExti.edgeType						= EXTERNAL_INTERRUPT_RISING_EDGE; // Detecto flanco de subida en el clock
//	extInt_Config(&handlerUserButtonExti);
//
	/* Configuración de pines para el USART1 */
	handlerPinTX.pGPIOx								= GPIOA;
	handlerPinTX.GPIO_PinConfig.GPIO_PinNumber 		= PIN_9;
	handlerPinTX.GPIO_PinConfig.GPIO_PinMode		= GPIO_MODE_ALTFN;
	handlerPinTX.GPIO_PinConfig.GPIO_PinOPType		= GPIO_OTYPE_PUSHPULL;
	handlerPinTX.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
	handlerPinTX.GPIO_PinConfig.GPIO_PinSpeed		= GPIO_OSPEED_FAST;
	handlerPinTX.GPIO_PinConfig.GPIO_PinAltFunMode	= AF7;
	GPIO_Config(&handlerPinTX);

	handlerPinRX.pGPIOx								= GPIOA;
	handlerPinRX.GPIO_PinConfig.GPIO_PinNumber 		= PIN_10;
	handlerPinRX.GPIO_PinConfig.GPIO_PinMode		= GPIO_MODE_ALTFN;
	handlerPinRX.GPIO_PinConfig.GPIO_PinOPType		= GPIO_OTYPE_PUSHPULL;
	handlerPinRX.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
	handlerPinRX.GPIO_PinConfig.GPIO_PinSpeed		= GPIO_OSPEED_FAST;
	handlerPinRX.GPIO_PinConfig.GPIO_PinAltFunMode	= AF7;
	GPIO_Config(&handlerPinRX);

	/* Configuración de la comunicación serial */
	usart1Comm.ptrUSARTx						= USART1;
	usart1Comm.USART_Config.USART_baudrate 		= USART_BAUDRATE_115200;
	usart1Comm.USART_Config.USART_datasize		= USART_DATASIZE_8BIT;
	usart1Comm.USART_Config.USART_parity		= USART_PARITY_NONE;
	usart1Comm.USART_Config.USART_stopbits		= USART_STOPBIT_1;
	usart1Comm.USART_Config.USART_mode			= USART_MODE_RXTX;
	usart1Comm.USART_Config.USART_enableIntRX	= USART_RX_INTERRUP_ENABLE;
	usart1Comm.USART_Config.USART_enableIntTX	= USART_TX_INTERRUP_DISABLE;
	USART_Config(&usart1Comm);
}

/** Interrupción del timer blinky LED*/
void BasicTimer2_Callback(void){
	GPIOxTooglePin(&handlerBlinkyPin); //Cambio el estado del LED PA5
	sendMsg++;
}

///** Interrupción externa del User Button */
//void callback_extInt13(void){
//	flagUserButton = 1;	// Pongo en alto la variable bandera del Exti 13 para el main
//}
//
/** Interrupción del USART2 */
//void usart2Rx_Callback(void){
//	usart1RxData = getRxData();	// Pongo en alto la variable bandera del USART2 para el main
//}
void usart1Tx_Callback(void){
	writeMsg(&usart1Comm, bufferMsg);
	disableTXEIE(&usart1Comm);
}



