/**
 ******************************************************************************
 * @file           : TestSpi.c
 * @author         : Miller Quintero - miquinterog@unal.edu.co
 * @brief          : Ejemplo de uso del SPI - Pressure sensor MPE280
 ******************************************************************************
 * Test del driver para el protocolo de comunicación SPI
 ******************************************************************************
 */

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include <stm32f4xx.h>

#include "GPIOxDriver.h"
#include "BasicTimer.h"
#include "USARTxDriver.h"
#include "SysTickDriver.h"
#include "SpiDriver.h"
#include "MPE280Driver.h"


/* Definición de los handlers necesarios */

// Elementos para el Blinky LED
GPIO_Handler_t handlerBlinkyPin = 			{0}; // LED de estado del Pin A5
BasicTimer_Handler_t handlerBlinkyTimer = 	{0}; // Timer del LED de estado

// Elementos para hacer la comunicación serial
GPIO_Handler_t handlerPinTX = {0};	// Pin de transmisión de datos
GPIO_Handler_t handlerPinRX = {0};	// Pin de recepción de datos
USART_Handler_t usartComm =  {0};	// Comunicación serial
uint8_t rxData = '\0';
char printMsg[64] = {0};
uint8_t periodicMsg = 0;

/* Para el SPI */
uint8_t spiRxBuffer[64] = {0};

/* Definición de prototipos de funciones */
void initSystem(void); 			// Función que inicializa los periféricos básicos

/** Función principal del programa */
int main(void){

	// Inicializamos todos los elementos del sistema
	initSystem();

	mpe280_init();

	writeChar(&usartComm, ' ');
	writeMsg(&usartComm, "--- Press MPE280 - V0.01 --- \n\r");

    /* Loop forever */
	while(1){

		if(periodicMsg == 4){
			writeMsg(&usartComm, "SPI-Testing \n");

			/* Chip ID */
			mpe280_readData(MPE280_CHIP_ID, spiRxBuffer, 1);
			sprintf(printMsg, "MPE280_CHIP_ID = 0x%X \n", spiRxBuffer[0]);
			writeMsg(&usartComm, printMsg);

			/* Configurando algo */
			mpe280_writeData(MPE280_CTRL_MEAS, 0x00);
			mpe280_readData(MPE280_CTRL_MEAS, spiRxBuffer, 1);

			/* Configurando un valor diferente */
			mpe280_writeData(MPE280_CTRL_MEAS, 0b01101111);
			mpe280_readData(MPE280_CTRL_MEAS, spiRxBuffer, 1);
			sprintf(printMsg, "MPE280_CTRL_MEAS = 0x%X \n", spiRxBuffer[0]);
			writeMsg(&usartComm, printMsg);

			/* Leyendo un valor xxx */
			mpe280_readData(MPE280_PRESS_MSB, spiRxBuffer, 3);

			periodicMsg = 0;

		}

	}
	return 0;
}

/** Función encargada de iniciar hardware para un pin*/
void initSystem(void){

	/* Activamos el Coprocesador Matemático - FPU */
	SCB->CPACR |= (0XF << 20);

	config_SysTick_ms(HSI_CLOCK_CONFIGURED);

	/* GPIO y Timer del Blinky Led de Estado */
	handlerBlinkyPin.pGPIOx								= GPIOH;
	handlerBlinkyPin.GPIO_PinConfig.GPIO_PinNumber 		= PIN_1;
	handlerBlinkyPin.GPIO_PinConfig.GPIO_PinMode		= GPIO_MODE_OUT;
	handlerBlinkyPin.GPIO_PinConfig.GPIO_PinOPType		= GPIO_OTYPE_PUSHPULL;
	handlerBlinkyPin.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
	handlerBlinkyPin.GPIO_PinConfig.GPIO_PinSpeed 		= GPIO_OSPEED_HIGH;

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

	/* Configuración de pines para el USART2 */
	handlerPinTX.pGPIOx								= GPIOA;
	handlerPinTX.GPIO_PinConfig.GPIO_PinNumber 		= PIN_2;
	handlerPinTX.GPIO_PinConfig.GPIO_PinMode		= GPIO_MODE_ALTFN;
	handlerPinTX.GPIO_PinConfig.GPIO_PinOPType		= GPIO_OTYPE_PUSHPULL;
	handlerPinTX.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
	handlerPinTX.GPIO_PinConfig.GPIO_PinSpeed		= GPIO_OSPEED_FAST;
	handlerPinTX.GPIO_PinConfig.GPIO_PinAltFunMode	= AF7;
	GPIO_Config(&handlerPinTX);

	handlerPinRX.pGPIOx								= GPIOA;
	handlerPinRX.GPIO_PinConfig.GPIO_PinNumber 		= PIN_3;
	handlerPinRX.GPIO_PinConfig.GPIO_PinMode		= GPIO_MODE_ALTFN;
	handlerPinRX.GPIO_PinConfig.GPIO_PinOPType		= GPIO_OTYPE_PUSHPULL;
	handlerPinRX.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
	handlerPinRX.GPIO_PinConfig.GPIO_PinSpeed		= GPIO_OSPEED_FAST;
	handlerPinRX.GPIO_PinConfig.GPIO_PinAltFunMode	= AF7;
	GPIO_Config(&handlerPinRX);

	/* Configuración de la comunicación serial */
	usartComm.ptrUSARTx						= USART2;
	usartComm.USART_Config.USART_baudrate 		= USART_BAUDRATE_115200;
	usartComm.USART_Config.USART_datasize		= USART_DATASIZE_8BIT;
	usartComm.USART_Config.USART_parity		= USART_PARITY_NONE;
	usartComm.USART_Config.USART_stopbits		= USART_STOPBIT_1;
	usartComm.USART_Config.USART_mode			= USART_MODE_RXTX;
	usartComm.USART_Config.USART_enableIntRX	= USART_RX_INTERRUP_ENABLE;
	usartComm.USART_Config.USART_enableIntTX	= USART_TX_INTERRUP_DISABLE;
	USART_Config(&usartComm);

	/* Pequeña espera para que el USART se configure correctamente */
	delay_ms(5);
}

/** Interrupción del timer blinky LED*/
void BasicTimer2_Callback(void){
	GPIOxTooglePin(&handlerBlinkyPin); //Cambio el estado del LED PA5
	periodicMsg++;
}


/** Interrupción del USART2 */
void usart2Rx_Callback(void){
	rxData = getRxData();
}



