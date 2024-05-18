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
#include "USARTxDriver.h"
#include "PwmDriver.h"
#include "AdcDriver.h"
#include "SysTickDriver.h"

/* Definición de los handlers necesarios */

// Elementos para el Blinky LED
GPIO_Handler_t handlerBlinkyPin = 			{0}; // LED de estado del Pin A5
BasicTimer_Handler_t handlerBlinkyTimer = 	{0}; // Timer del LED de estado

// Elementos para hacer la comunicación serial
GPIO_Handler_t handlerPinTX = {0};						// Pin de transmisión de datos
GPIO_Handler_t handlerPinRX = {0};						// Pin de recepción de datos
USART_Handler_t usartComm =  {0};						// Comunicación serial
uint8_t usartRxData = 0; 								// Variable en la que se guarda el dato transmitido
char bufferReception[32] = {0};							// Buffer para guardar caracteres ingresados
char bufferMsg[64] = "Mensaje para enviar";				// Buffer de mensajes
char cmd[16] = {0};										// BUffer de comandos
uint16_t counterReception = 0;							// Contador de caracteres recibidos
bool stringComplete = false;							// Bandera de comando completado
unsigned int firstParameter = 0;						// Primer parámetro del comando
unsigned int secondParameter = 0;						// Segundo parámetro del comando

// Elementos para el ADC
ADC_Config_t channelADC = {0};							// Canal ADC
BasicTimer_Handler_t handlerTimerADC = {0};				// Timer de muestreo del ADC
bool adcIsComplete = false;								// Bandera de ADC Completado
uint32_t adcCounter = 0;								// Contador de conversiones
uint32_t dataQuantity = 30000;							// Variable que guarda la cantidad de datos solicitados
uint16_t* dataADC;										// Arreglo para la cantidad de datos
float dataChannelADC = 0.00f;							// Variable que guarda el valor calculado de voltaje
bool flagCapture = false;								// Bandera para indicar modo de captura
bool flagConstant = false;								// Bandera para indicar modo de muestreo constante
bool flagSingleData = false;							// Bandera para indicar una conversión en modo constante


/* Definición de las cabeceras de funciones del main */
void initSystem(void); 							// Función que inicializa los periféricos básicos
void commandBuild(void);						// Función encargada de recibir los comandos
void commandUSART(char* ptrBufferReception);	// FUnción encargada de gestionar acciones con los comandos

/** Función principal del programa */
int main(void){

	// Inicializamos todos los elementos del sistema
	initSystem();
	sprintf(bufferMsg, "La frecuencia del microcontrolador es %u.\n", (unsigned int)getConfigPLL());
	writeMsg(&usartComm, bufferMsg);

    /* Loop forever */
	while(1){

		// Hacemos un "eco" con el valor que nos llega por el serial
		if(usartRxData != '\0'){
			writeChar(&usartComm, usartRxData);
			commandBuild();
		}

		// Verificamos si se ha dado por completado del comando
		else if(stringComplete){
			commandUSART(bufferReception);
			stringComplete = false;
		}

		else if(flagSingleData){
			sprintf(bufferMsg, "=%.3f/\n",dataChannelADC);
			writeMsg(&usartComm, bufferMsg);
			adcCounter++;
			flagSingleData = false;
		}

		else{
			__NOP();
		}
	}
	return 0;
}

/** Función encargada de iniciar hardware para un pin*/
void initSystem(void){

	/* Configuramos el sistema a 100MHz*/
	configPLL(100);

	/* Activamos el Coprocesador Matemático - FPU */
	SCB->CPACR |= (0XF << 20);

	/* Activamos el SysTick */
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
	// Atributos para el Timer 2 del LED de estado
	handlerBlinkyTimer.ptrTIMx								= TIM4;
	handlerBlinkyTimer.TIMx_Config.TIMx_mode				= BTIMER_MODE_UP;
	handlerBlinkyTimer.TIMx_Config.TIMx_speed				= BTIMER_PLL_100MHz_SPEED_100us;
	handlerBlinkyTimer.TIMx_Config.TIMx_period				= 2500;
	handlerBlinkyTimer.TIMx_Config.TIMx_interruptEnable 	= BTIMER_INTERRUP_ENABLE;
	BasicTimer_Config(&handlerBlinkyTimer);
	startBasicTimer(&handlerBlinkyTimer);
	/* Fin del GPIO y Timer del LED de estado
	 * ----------------------------------------*/

	/* Configuración de pines para el USART2 */
	handlerPinTX.pGPIOx								= GPIOA;
	handlerPinTX.GPIO_PinConfig.GPIO_PinNumber 		= PIN_2;
	handlerPinTX.GPIO_PinConfig.GPIO_PinMode		= GPIO_MODE_ALTFN;
	handlerPinTX.GPIO_PinConfig.GPIO_PinOPType		= GPIO_OTYPE_PUSHPULL;
	handlerPinTX.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
	handlerPinTX.GPIO_PinConfig.GPIO_PinSpeed		= GPIO_OSPEED_HIGH;
	handlerPinTX.GPIO_PinConfig.GPIO_PinAltFunMode	= AF7;
	GPIO_Config(&handlerPinTX);

	handlerPinRX.pGPIOx								= GPIOA;
	handlerPinRX.GPIO_PinConfig.GPIO_PinNumber 		= PIN_3;
	handlerPinRX.GPIO_PinConfig.GPIO_PinMode		= GPIO_MODE_ALTFN;
	handlerPinRX.GPIO_PinConfig.GPIO_PinOPType		= GPIO_OTYPE_PUSHPULL;
	handlerPinRX.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
	handlerPinRX.GPIO_PinConfig.GPIO_PinSpeed		= GPIO_OSPEED_HIGH;
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

	/* Configuración del ADC single channel */
	channelADC.channel				= ADC_CHANNEL_4;
	channelADC.dataAlignment		= ADC_ALIGNMENT_RIGHT;
	channelADC.samplingPeriod		= ADC_SAMPLING_PERIOD_3_CYCLES;
	channelADC.resolution			= ADC_RESOLUTION_12_BIT;
	adc_Config(&channelADC);

}

/** Función encargada de construir el string con el comando */
void commandBuild(void){
	bufferReception[counterReception] = usartRxData;
	counterReception++;

	// Aqui hacemmos la instrucción que detiene la recepción del comando
	if(usartRxData == '\r'){
		stringComplete = true;

		//Sustituyo el último caracter de \n por un null
		bufferReception[counterReception] = '\0';
		counterReception = 0;
	}
	if(usartRxData == '\b'){
		counterReception--;
		counterReception--;
	}

	// Volvemos a null para terminar
	usartRxData = '\0';
}

/** Función encargada de gestionar e identificar el comando ingresado, recibe el puntero al string */
void commandUSART(char* ptrBufferReception){

	/* El funcionamiento es de la siguiente forma: Empleamos el puntero al buffer para
	 * acceder a los elementos del string, y por medio de la función sscanf se almacena
	 * en 3 elementos diferentes, el string del comando "cmd", y dos números enteros llamados
	 * "firstParameter" y "SecondParameter". De esta froma, podemos introducir información
	 * al micro desde el puerto serial */
	sscanf(ptrBufferReception, "%s %u %u", cmd, &firstParameter, &secondParameter);

	/* Usamos la funcion strcmp, string compare, que me retorna un 0 si los 2 strings son iguales */

	// "Menu" este primer comando imprime una lista con los otros comandos que tiene el equipo
	if(strcmp(cmd, "Menu") == 0){
		writeMsg(&usartComm, "\nMenu CMDs:\n");
		writeMsg(&usartComm, "1) Menu							-- Imprime este menu.\n");
		writeMsg(&usartComm, "2) Capturar #Freq[Hz] #Muestras	-- Inicia la toma de datos según la frecuencia de muestreo y el número de muestras. "
				"La frecuencia debe ser un número entre 1 y 1000000, las muestras entre 1 a 30000.\n");
		writeMsg(&usartComm, "3) Imprimir						-- Imprime los datos recolectados.\n");
		writeMsg(&usartComm, "4) Constante #Freq[Hz] #Muestras	-- Envia datos constantemente a la frecuencia indicada hasta alcanzar la cantidad solicitada. "
				"La frecuencia debe ser un número entre 1 y 1000000, las muestras desde 1 dato hasta 5 millones de datos.\n");
	}

	// Lanzar captura de datos
	else if(strcmp(cmd, "Capturar") == 0){
		if((!flagConstant)){
			if((firstParameter > 1000000)||(firstParameter < 1)||(secondParameter < 1)||(secondParameter > 30000)){
				if((firstParameter > 1000000)||(firstParameter < 1)){
					writeMsg(&usartComm, "Recuerde que la frecuencia debe ser entre 1 Hz a 1000000 Hz.\n");
				}
				else{
					__NOP();
				}
				if((secondParameter < 1)||(secondParameter > 30000)){
					writeMsg(&usartComm, "Recuerde que el número de muestras debe ser entre 1 a 30000.\n");
				}
				else{
					__NOP();
				}
			}
			else{
				float arrValue = 0.00f;
				arrValue = (10000000.00f/((float)firstParameter));

				/* Configuración del Timer para el ADC */
				handlerTimerADC.ptrTIMx								= TIM5;
				handlerTimerADC.TIMx_Config.TIMx_mode				= BTIMER_MODE_UP;
				handlerTimerADC.TIMx_Config.TIMx_speed				= BTIMER_PLL_100MHz_SPEED_100ns;
				handlerTimerADC.TIMx_Config.TIMx_period 			= (uint32_t)arrValue;
				handlerTimerADC.TIMx_Config.TIMx_interruptEnable 	= BTIMER_INTERRUP_ENABLE;
				BasicTimer_Config(&handlerTimerADC);

				dataQuantity = secondParameter;
				dataADC = (uint16_t*)malloc(dataQuantity*sizeof(uint16_t));
				for(uint32_t j = 0; j<dataQuantity;j++){
					dataADC[j] = 0;
				}
				writeMsg(&usartComm, "La captura de datos ha comenzado.\n");
				flagCapture = true;
				startBasicTimer(&handlerTimerADC);
			}
		}
		else{
			writeMsg(&usartComm, "Ya se está haciendo un muestreo constante.\n");
		}
	}

	// Imprimir datos
	else if(strcmp(cmd, "Imprimir") == 0){
		if((adcIsComplete)&&(flagCapture)){
			flagCapture = false;
			adcIsComplete = false;
			writeMsg(&usartComm, "Los datos se imprimirán en 5 segundos.\n");
			delay_ms(5000);
			for (uint32_t i = 0; i < dataQuantity; i++){
				dataChannelADC = ((((float)dataADC[i])*3.30f)/4095.00f);
				sprintf(bufferMsg, "#%lu\t%.3f\n",i+1,dataChannelADC);
				writeMsg(&usartComm, bufferMsg);
			}
			// Una vez imprimidos se libera la memoria dinámica
			free(dataADC);
		}
		else{
			writeMsg(&usartComm, "No se han capturado datos.\n");
		}
	}

	// Muestreo y envio constante de datos
	else if(strcmp(cmd, "Constante") == 0){
		if(!flagCapture){
			if((firstParameter > 1000000)||(firstParameter < 1)||(secondParameter < 1)||(secondParameter > 5000000)){
				if((firstParameter > 1000000)||(firstParameter < 1)){
					writeMsg(&usartComm, "Recuerde que la frecuencia debe ser entre 1 Hz a 1000000 Hz.\n");
				}
				else{
					__NOP();
				}
				if((secondParameter < 1)||(secondParameter > 5000000)){
					writeMsg(&usartComm, "Recuerde que el número de muestras debe ser entre 1 a 5000000.\n");
				}
				else{
					__NOP();
				}
			}
			else{
				float arrValue = 0.00f;
				arrValue = (10000000.00f/((float)firstParameter));

				/* Configuración del Timer para el ADC */
				handlerTimerADC.ptrTIMx								= TIM5;
				handlerTimerADC.TIMx_Config.TIMx_mode				= BTIMER_MODE_UP;
				handlerTimerADC.TIMx_Config.TIMx_speed				= BTIMER_PLL_100MHz_SPEED_100ns;
				handlerTimerADC.TIMx_Config.TIMx_period 			= (uint32_t)arrValue;
				handlerTimerADC.TIMx_Config.TIMx_interruptEnable 	= BTIMER_INTERRUP_ENABLE;
				BasicTimer_Config(&handlerTimerADC);

				dataQuantity = secondParameter;
				flagConstant = true;
				startBasicTimer(&handlerTimerADC);
			}
		}
		else{
			writeMsg(&usartComm, "Está en modo de captura, espere los datos o imprimalos.\n");
		}
	}

	// En cualquier otro caso, indicamos que el comando es incorrecto
	else{
		writeMsg(&usartComm, "Comando incorrecto.\n");
	}

	// Limpiamos los parámetros para evitar configuraciones accidentales
	firstParameter = 0;
	secondParameter = 0;
}

/** Interrupción del timer ADC */
void BasicTimer5_Callback(void){
	startSingleADC();
}

/** Interrupción del timer blinky LED */
void BasicTimer4_Callback(void){
	GPIOxTooglePin(&handlerBlinkyPin); //Cambio el estado del LED PA5
}

/** Interrupción del USART2 */
void usart2Rx_Callback(void){
	usartRxData = getRxData();
}

void adcComplete_Callback(void){
	if(flagCapture){
		if(adcCounter >= dataQuantity){
			stopBasicTimer(&handlerTimerADC);
			adcIsComplete = true;
			adcCounter = 0;
			writeMsg(&usartComm, "¡Muestra completada!\n");
		}
		else{
			dataADC[adcCounter] = getADC();
			adcCounter++;
		}
	}
	else{
		if(adcCounter >= dataQuantity){
			stopBasicTimer(&handlerTimerADC);
			adcCounter = 0;
			flagSingleData = false;
			flagConstant = false;
		}
		else{
			flagSingleData = true;
			dataChannelADC =((((float)getADC())*3.30f)/4095.00f);
		}
	}
}



