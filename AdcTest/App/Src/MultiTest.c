/*
 * MultiTest.c
 *
 *  Created on: 7/06/2023
 *      Author: miller
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
#include "AdcDriver.h"


/* Definición de los handlers necesarios */

// Elementos para el Blinky LED
GPIO_Handler_t handlerBlinkyPin = 			{0}; // LED de estado del Pin A5
BasicTimer_Handler_t handlerBlinkyTimer = 	{0}; // Timer del LED de estado


// Elementos para hacer la comunicación serial
GPIO_Handler_t handlerPinTX = {0};	// Pin de transmisión de datos
GPIO_Handler_t handlerPinRX = {0};	// Pin de recepción de datos
USART_Handler_t usart2Comm =  {0};	// Comunicación serial
uint8_t sendMsg = 0; // Variable para controlar la comunicación
uint8_t usart2RxData = 0; // Variable en la que se guarda el dato transmitido
char bufferMsg[64] = {0}; // Buffer de datos como un arreglo de caracteres
char bufferData[64] = "Mensaje para enviar";

ADC_Multichannel_Config_t handlerDualADC = {0};
uint8_t adcIsComplete = 0;
uint8_t adcCounter = 0;
uint16_t dataADC[2] = {0};
uint8_t channelOrder[2] = {ADC_CHANNEL_0, ADC_CHANNEL_1};
uint16_t samplingOrder[2] = {ADC_SAMPLING_PERIOD_84_CYCLES, ADC_SAMPLING_PERIOD_84_CYCLES};
uint8_t numberOfChannels = 2;
PWM_Handler_t handlerPwmEventADC = {0};


/* Definición de las cabeceras de funciones del main */
void initSystem(void); 			// Función que inicializa los periféricos básicos

/** Función principal del programa
 * ¡Esta función es el corazón del programa! */
int main(void){

	// Inicializamos todos los elementos del sistema
	initSystem();
	writeMsg(&usart2Comm, "Iniciando sistema...");

    /* Loop forever */
	while(1){

		if(usart2RxData != '\0'){
			if(usart2RxData == 'a'){
				// Iniciamos una conversion ADC
				adcCounter = 0;
				startPwmSignal(&handlerPwmEventADC);
			}
			usart2RxData = '\0';
		}

		if(adcIsComplete == 1){
			stopPwmSignal(&handlerPwmEventADC);
			sprintf(bufferData, "%u\t%u\n", dataADC[0],dataADC[1]);
			writeMsg(&usart2Comm, bufferData);
			adcIsComplete = 0;
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
	usart2Comm.ptrUSARTx						= USART2;
	usart2Comm.USART_Config.USART_baudrate 		= USART_BAUDRATE_115200;
	usart2Comm.USART_Config.USART_datasize		= USART_DATASIZE_8BIT;
	usart2Comm.USART_Config.USART_parity		= USART_PARITY_NONE;
	usart2Comm.USART_Config.USART_stopbits		= USART_STOPBIT_1;
	usart2Comm.USART_Config.USART_mode			= USART_MODE_RXTX;
	usart2Comm.USART_Config.USART_enableIntRX	= USART_RX_INTERRUP_ENABLE;
	usart2Comm.USART_Config.USART_enableIntTX	= USART_TX_INTERRUP_DISABLE;
	USART_Config(&usart2Comm);

	handlerDualADC.orderADC 			= channelOrder;
	handlerDualADC.samplingPeriod		= samplingOrder;
	handlerDualADC.resolution			= ADC_RESOLUTION_12_BIT;
	handlerDualADC.dataAlignment		= ADC_ALIGNMENT_RIGHT;
	handlerDualADC.extTriggerEnable		= ADC_EXTEN_RISING_EDGE;
	handlerDualADC.extTriggerSelect		= ADC_EXTSEL_TIM5_CC3;
	adc_ConfigMultichannel(&handlerDualADC, 2);


	/* Configuración del PWM usado como disparador de la conversión ADC */
	// Utilizo el canal 3 para PWM del Timer 5 ya que el 1 y 2 están ocupados por ADC
	handlerPwmEventADC.ptrTIMx						= TIM5;
	handlerPwmEventADC.PWMx_Config.PWMx_Channel		= PWM_CHANNEL_3;
	handlerPwmEventADC.PWMx_Config.PWMx_Prescaler	= BTIMER_SPEED_10us;
	handlerPwmEventADC.PWMx_Config.PWMx_Period		= 10;
	handlerPwmEventADC.PWMx_Config.PWMx_DuttyCicle	= 5;
	pwm_Config(&handlerPwmEventADC);

}

/** Interrupción del timer blinky LED*/
void BasicTimer2_Callback(void){
	GPIOxTooglePin(&handlerBlinkyPin); //Cambio el estado del LED PA5
}

/** Interrupción del USART2 */
void usart2Rx_Callback(void){
	usart2RxData = getRxData();
}

void adcComplete_Callback(void){
	dataADC[adcCounter] = getADC();
	if(adcCounter < (numberOfChannels-1)){
		adcCounter++;
	}
	else{
		adcIsComplete = 1;
		adcCounter = 0;
	}
}
