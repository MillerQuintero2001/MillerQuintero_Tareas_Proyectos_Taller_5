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
uint8_t usart2RxData = 0; // Variable en la que se guarda el dato transmitido
char bufferMsg[64] = {0}; // Buffer de datos como un arreglo de caracteres
char bufferData[64] = "Mensaje para enviar";

// Arreglos para pruebas librerías CMSIS
float32_t srcNumber[4] = {-0.987, 32.26, -45.21, -987.321};
float32_t destNumber[4] = {0};
uint32_t dataSize = 0;

// Para utlizar la función seno
float32_t sineValue = 0.0;
float32_t sineArgValue = 0.0;

// Elementos para generar una señal
#define SINE_DATA_SIZE	4096					// Tamaño del arreglo de datos
float32_t fs = 8000.0;							// Frecuencia de muestreo
float32_t f0 = 250.0;							// Frecuencia fundamental de la señal
float32_t dt = 0.0;								// Período de muestreo (será 1/fs)
float32_t stopTime = 1.0;						// Quizas no sea necesario
float32_t amplitud = 5;							// Amplitud de la señal generada
float32_t sineSignal[SINE_DATA_SIZE];			// Arreglo donde se guardan los datos de la señal generada
float32_t transformedSignal[SINE_DATA_SIZE];	// Arreglo donde se guardan los datos de la señal transformada
float32_t* ptrSineSignal;

uint32_t ifftFlag = 0;
uint32_t doBitReverse = 1;
arm_rfft_fast_instance_f32 config_Rfft_fast_f32;
arm_cfft_radix4_instance_f32 configRadix4_f32;
arm_status status = ARM_MATH_ARGUMENT_ERROR;
arm_status statusInitFFT = ARM_MATH_ARGUMENT_ERROR ;
uint16_t fftSize = 1024;



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

		/* Crear señal */
		if(usart2RxData == 'C'){
			createSignal();
			sprintf(bufferMsg, "Creando señal ... \n");
			writeMsg(&usart2Comm, bufferMsg);
			usart2RxData = '\0';
		}

		/* Para probar el seno */
		if(usart2RxData == 'P'){
			stopTime = 0.0;
			int i = 0;
			writeMsg(&usart2Comm, "Signal values: Time - Sine\n");

			while(stopTime < 0.01){
				stopTime = dt*i;
				i++;
				sprintf(bufferMsg, "%#.5f ; %#.6f\n", stopTime, sineSignal[i]);
				writeMsg(&usart2Comm, bufferMsg);

			}

			usart2RxData = '\0';

		}

		/* Sacamos el valor absoluto de los valores con funcioens de arm */
		if(usart2RxData == 'A'){
			stopTime = 0.0;
			int i = 0;

			sprintf(bufferMsg, "Valor Absoluto \n");
			writeMsg(&usart2Comm, bufferMsg);

			arm_abs_f32(sineSignal, transformedSignal, SINE_DATA_SIZE);

			while(stopTime < 0.01){
				stopTime = dt*i;
				i++;
				sprintf(bufferMsg, "%#.5f ; %#.6f \n", stopTime, transformedSignal[i]);
				writeMsg(&usart2Comm, bufferMsg);

 			}
			usart2RxData = '\0';
 		}

		/* IMPORTANTE, con las librerías de ARM siempre hay 2 pasos importantes
		 * con algunas funciones, los cuáles son: Inicializar la función,
		 * y después utlizarla, para esto empleamos la variable de statusInitFFT  */
		if(usart2RxData == 'I'){
			statusInitFFT = arm_rfft_fast_init_f32(&config_Rfft_fast_f32, fftSize);

			if(statusInitFFT == ARM_MATH_SUCCESS){
				sprintf(bufferMsg, "Initialization ... SUCCESS! \n");
				writeMsg(&usart2Comm, bufferMsg);
			}
			usart2RxData = '\0';
		}

		if(usart2RxData == 'F'){
			stopTime = 0.0;
			int i = 0;
			int j = 0;

			sprintf(bufferMsg, "FFT \n");
			writeMsg(&usart2Comm, bufferMsg);

			if(statusInitFFT == ARM_MATH_SUCCESS){
				arm_rfft_fast_f32(&config_Rfft_fast_f32, sineSignal, transformedSignal, ifftFlag);

				arm_abs_f32(transformedSignal, sineSignal, fftSize);

				for(i = 1; i < fftSize; i++){
					if(i%2){
						sprintf(bufferMsg, "%u ; %#.6f\n", j , 2*sineSignal[i]);
						writeMsg(&usart2Comm, bufferMsg);
						j++;
					}
				}
			}
			else{
				writeMsg(&usart2Comm, "FFT not Initialized ...");
			}
			usart2RxData = '\0';
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
}

/** Función encargada de crear la señal */
void createSignal(void){
	/*
	 * Esta es la señal creada en Matlab
	 * sineSignal = amplitud * sin(2*pi*f0*t)
	 */
	// Creando la señal, necesitamos el período dt
	dt = 1/fs;

	/* Llenamos el arreglo con la señal seno */
	for(int i = 0; i < SINE_DATA_SIZE; i++){
		sineSignal[i] = amplitud * arm_sin_f32(2*M_PI*f0*(dt*i));
	}
}

/** Interrupción del timer blinky LED*/
void BasicTimer2_Callback(void){
	GPIOxTooglePin(&handlerBlinkyPin); //Cambio el estado del LED PA5
	sendMsg++;
}


/** Interrupción del USART2 */
void usart2Rx_Callback(void){
	usart2RxData = getRxData();	// Pongo en alto la variable bandera del USART2 para el main
}



