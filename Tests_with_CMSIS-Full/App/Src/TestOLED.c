/**
 ******************************************************************************
 * @file           : TestOLED.c
 * @author         : Miller Quintero - miquinterog@unal.edu.co
 * @brief          : Archivo para probar driver OLED 0.96
 ******************************************************************************
 * Archivo con el código encargado de testear el driver creado para la OLED
 * adquirida de 0.96 pulgadas.
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
#include "SysTickDriver.h"
#include "I2CDriver.h"
#include "FontsSSD1306.h"
#include "SSD1306.h"

// Definiciones a emplear
#define SSD1306_I2C_ADDRESS_OLED 0x3C

/* Elementos para el Blinky LED */
GPIO_Handler_t handlerBlinkyPin = 			{0}; // LED de estado del Pin A5
BasicTimer_Handler_t handlerBlinkyTimer = 	{0}; // Timer del LED de estado

/* Elementos para hacer la comunicación serial */
GPIO_Handler_t handlerPinTX = {0};			// Pin de transmisión de datos
GPIO_Handler_t handlerPinRX = {0};			// Pin de recepción de datos
USART_Handler_t usartComm =  {0};			// Comunicación serial
uint8_t usartData = 0; 						// Variable en la que se guarda el dato transmitido
char bufferData[128] = {0}; 				// Buffer de datos como un arreglo de caracteres
char userMsg[] = "Funciona !";			// Mensaje de usuario

/* Elementos para el display OLED */
I2C_Handler_t handlerOLED = {0};		// Handler para la configuración del I2C
GPIO_Handler_t handlerOLED_SCL = {0};	// Pin para el Serial Clock
GPIO_Handler_t handlerOLED_SDA = {0};	// Pin para el Serial Data
uint8_t i2cBuffer = 0;					// Buffer del I2C

SSD1306_t handlerSSD1306 = {0};

/* Definición de prototipos de funciones para el main */
void initSystem(void); 						// Función que inicializa los periféricos básicos


/** Función principal del programa
 * ¡Esta función es el corazón del programa! */
int main(void){

	// Inicializamos todos los elementos del sistema
	initSystem();

    /* Loop forever */
	while(1){
		if(usartData != '\0'){
			writeChar(&usartComm, usartData);
			if(usartData == 'W'){
				ssd1306WriteString(userMsg, Font_7x10, White);
				writeMsg(&usartComm, userMsg);
			}
			else{
				__NOP();
			}
			usartData = '\0';
		}
	}
	return 0;
}


/** Función encargada de iniciar hardware para un pin*/
void initSystem(void){

	/* Activamos el Coprocesador Matemático - FPU */
	SCB->CPACR |= (0XF << 20);

	/* Configuramos el SysTick */
	config_SysTick_ms(HSI_CLOCK_CONFIGURED);

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
	usartComm.ptrUSARTx							= USART2;
	usartComm.USART_Config.USART_baudrate 		= USART_BAUDRATE_115200;
	usartComm.USART_Config.USART_datasize		= USART_DATASIZE_8BIT;
	usartComm.USART_Config.USART_parity			= USART_PARITY_NONE;
	usartComm.USART_Config.USART_stopbits		= USART_STOPBIT_1;
	usartComm.USART_Config.USART_mode			= USART_MODE_RXTX;
	usartComm.USART_Config.USART_enableIntRX	= USART_RX_INTERRUP_ENABLE;
	usartComm.USART_Config.USART_enableIntTX	= USART_TX_INTERRUP_DISABLE;
	USART_Config(&usartComm);

	/*						I2C OLED					*/

	/* Configuración del pin SCL */
	handlerOLED_SCL.pGPIOx								= GPIOA;
	handlerOLED_SCL.GPIO_PinConfig.GPIO_PinNumber		= PIN_8;
	handlerOLED_SCL.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_ALTFN;
	handlerOLED_SCL.GPIO_PinConfig.GPIO_PinOPType		= GPIO_OTYPE_OPENDRAIN;
	handlerOLED_SCL.GPIO_PinConfig.GPIO_PinSpeed		= GPIO_OSPEED_HIGH;
	handlerOLED_SCL.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
	handlerOLED_SCL.GPIO_PinConfig.GPIO_PinAltFunMode 	= AF4;
	GPIO_Config(&handlerOLED_SCL);

	/* Configuración del pin SDA */
	handlerOLED_SDA.pGPIOx								= GPIOB;
	handlerOLED_SDA.GPIO_PinConfig.GPIO_PinNumber		= PIN_4;
	handlerOLED_SDA.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_ALTFN;
	handlerOLED_SDA.GPIO_PinConfig.GPIO_PinOPType		= GPIO_OTYPE_OPENDRAIN;
	handlerOLED_SDA.GPIO_PinConfig.GPIO_PinSpeed		= GPIO_OSPEED_HIGH;
	handlerOLED_SDA.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
	handlerOLED_SDA.GPIO_PinConfig.GPIO_PinAltFunMode 	= AF9;
	GPIO_Config(&handlerOLED_SDA);

	/* Configuración para el I2C3 */
	handlerOLED.ptrI2Cx		 = I2C3;
	handlerOLED.modeI2C		 = I2C_MODE_FM;
	handlerOLED.slaveAddress = SSD1306_I2C_ADDRESS_OLED;
	i2c_config(&handlerOLED);

	handlerSSD1306.CurrentX = 0;
	handlerSSD1306.CurrentY = 0;
	handlerSSD1306.Inverted = 0;
	handlerSSD1306.Initialized = 0;

	ssd1306Init(&handlerOLED);

}


/** Interrupción del timer blinky LED*/
void BasicTimer2_Callback(void){
	GPIOxTooglePin(&handlerBlinkyPin); //Cambio el estado del LED PA5
}


/** Interrupción del USART2 */
void usart2Rx_Callback(void){
	usartData = getRxData();	// Pongo en alto la variable bandera del USART2 para el main
}

