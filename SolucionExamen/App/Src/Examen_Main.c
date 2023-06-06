/**
 ********************************************************************************
 * @file         : Examen_Main.c
 * @author       : Miller Quintero - miquinterog@unal.edu.co - MillerQuintero2001
 * @brief        : Solución del Examen
 ********************************************************************************
 * El equipo cargado con este programa deberá ,
 *
 ********************************************************************************
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
#include "PLLDriver.h"

// Macro-definiciones específicas para el sensor
#define ACCEL_ADDRESS 	0b1101000; // 0xD2 -> Dirección del sensor Acelerómetro con ADO=0
#define ACCEL_XOUT_H	59 // 0x3B
#define ACCEL_XOUT_L	60 // OX3C
#define ACCEL_YOUT_H	61 // 0x3D
#define ACCEL_YOUT_L 	62 // 0x3E
#define ACCEL_ZOUT_H 	63 // 0x3F
#define ACCEL_ZOUT_L 	64 // 0x40
uint8_t arrayRegisters[6] = {ACCEL_XOUT_L,ACCEL_XOUT_H, ACCEL_YOUT_L, ACCEL_YOUT_H, ACCEL_ZOUT_L, ACCEL_ZOUT_H};

#define PWR_MGMT_1	107
#define WHO_AM_I	117

/* Definición de los handlers necesarios */

// Elementos para el Blinky LED
GPIO_Handler_t handlerBlinkyPin = 			{0}; // LED de estado del Pin A5
BasicTimer_Handler_t handlerBlinkyTimer = 	{0}; // Timer del LED de estado

// Elemento para el Pin MC01 para chequear la señal
GPIO_Handler_t handlerMCO = {0};

// Elementos para el muestreo a 200Hz
BasicTimer_Handler_t handlerSamplingTimer = {0}; 	// Timer del muestreo de datos

uint8_t arraySaveData[6] = {0};						// Array que guarda los datos leídos de los registros de forma multiple
float arrayXYZdata[3] = {0};						// Array que guarda la información de los 3 ejes, cada 5 ms
float arrayXdataprint[2000] = {0};					// Array que guarda la información de datos eje X, capacidad de 2 segundos de muestreo
float arrayYdataprint[2000] = {0};					// Array que guarda la información de datos eje Y, capacidad de 2 segundos de muestreo
float arrayZdataprint[2000] = {0};					// Array que guarda la información de datos eje Z, capacidad de 2 segundos de muestreo
uint8_t flag200HzSamplingData = 0;					// Bandera para indicar tomar dato cada 5ms
uint16_t counter_ms = 0;							// Contador de milisegundos con el Timer 4

// Elementos para hacer la comunicación serial
GPIO_Handler_t handlerPinTX = {0};	// Pin de transmisión de datos
GPIO_Handler_t handlerPinRX = {0};	// Pin de recepción de datos
USART_Handler_t usartComm =  {0};	// Comunicación serial
uint8_t usartRxData = 0; 			// Variable en la que se guarda el dato transmitido
char bufferData[64] = {0}; 			// Buffer de datos como un arreglo de caracteres
char userMsg[] = "Funciona por comandos c: \n";
char bufferReception[32] = {0};
char cmd[16] = {0};
uint8_t counterReception = 0;
bool stringComplete = false;
unsigned int firstParameter = 0;
unsigned int secondParameter = 0;

// Elementos para utilizar comunicación I2C con Acelerometro
GPIO_Handler_t handlerI2C_SDA = {0};
GPIO_Handler_t handlerI2C_SCL = {0};
I2C_Handler_t handlerAccelerometer = {0};
uint8_t i2cBuffer = 0;

/* Inicializo variables a emplear */
unsigned int freq = 0;				// Variable en la que guardo los Hz de reloj del micro, entregados por getConfigPLL()
uint16_t clock100 = 100;			// Variable que guarda 100, para argumento de la función configPLL(), y poner a 100MHz
float factConv = 9.78/16384.0; 		// Factor de conversión para pasar medidas de acelerómetro, gravedad de Medellín 9.78

/* Definición de las cabeceras de funciones del main */
void initSystem(void); 								// Función que inicializa los periféricos básicos
void commandBuild(void);							// Función que construye el string del comando USART
void commandUSART(char* arrayCmd);					// Función encargada de gestionar el comando ingresado por USART
void actionRxData(void);							// Funcíón con la que se gestionan los casos de teclas recibidas
void saveData(void);								// Función encargada de guardar los datos acelerómetro
void initLSE(void);									// Función encargada de iniciar el Low Speed External, cristal resonador

/** Función principal del programa
 * ¡Esta función es el corazón del programa! */
int main(void){

	// Inicializamos todos los elementos del sistema
	initSystem();
	// Cargo en variable el valor en Hz del reloj MCU
	freq = (unsigned int)getConfigPLL();


    /* Loop forever */
	while(1){

		// Hacemos un "eco" con el valor que nos llega por el serial
		if(usartRxData != '\0'){
			commandBuild();
		}

		if(stringComplete){
			commandUSART(bufferReception);
			stringComplete = false;
		}

		// Muestreo de datos constante cada 1ms
		if(flag200HzSamplingData){
			if(counter_ms > 50){			// Controlo el contador de milisegundos
				counter_ms = 0;
			}
			//saveData(); //Función que guarda los datos en arreglos, al ser sujeta a la flag1KHz, esta función se llama solo cada 1ms
			flag200HzSamplingData = 0; 	// Bajo bandera porque ya se tomaron datos en este milisegundo
		}

	}
	return 0;
}

/** Función encargada de iniciar hardware para un pin*/
void initSystem(void){

	/* Activamos el Coprocesador Matemático - FPU */
	SCB->CPACR |= (0XF << 20);

	/* Configuramos el equipo a 100MHz, usando el driver del PLL */
	configPLL(clock100);

	/* Calibración manual del HSI para tener mayor precisión:
	 * Midiendo con el LED de estado y haciendo cálculos, considerando el pre-escaler y auto-reload escogidos
	 * para el timer que controla el blinky, se llegó a una frecuencia real de 101044394,86532803052349080092 Hz
	 * según la bibliografía consultada, los bits HSITRIM[4:0] que son los del 3 al 7 del RCC_CR, están por
	 * defecto en un valor de 16, incrimentar en 1 binario aumenta X% del HSI la frecuencia real, y decrementar
	 * en 1 binario, disminuye X% del HSI la frecuencia real.
	 * Haciendo pruebas se llego a que este es el valor con el que queda mejor calibrado */
	RCC->CR &= ~(0b11111 << RCC_CR_HSITRIM_Pos); 	// Limpio
	RCC->CR |= (13 << RCC_CR_HSITRIM_Pos);			// Escribo

	//Esperamos hasta que el HSI vuelva a ser estable
	while(!(RCC->CR & RCC_CR_HSIRDY)){
		__NOP();
	}

	/* Configuración del Pin para el MC01 */
	handlerMCO.pGPIOx								= GPIOA;
	handlerMCO.GPIO_PinConfig.GPIO_PinNumber		= PIN_8;
	handlerMCO.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_ALTFN;
	handlerMCO.GPIO_PinConfig.GPIO_PinOPType		= GPIO_OTYPE_PUSHPULL;
	handlerMCO.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
	handlerMCO.GPIO_PinConfig.GPIO_PinSpeed			= GPIO_OSPEED_FAST;
	handlerMCO.GPIO_PinConfig.GPIO_PinAltFunMode	= AF0;

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
	handlerBlinkyTimer.TIMx_Config.TIMx_speed				= BTIMER_PLL_100MHz_SPEED_100us;
	handlerBlinkyTimer.TIMx_Config.TIMx_period				= 2500;
	handlerBlinkyTimer.TIMx_Config.TIMx_interruptEnable 	= BTIMER_INTERRUP_ENABLE;
	BasicTimer_Config(&handlerBlinkyTimer);
	// Fin del GPIO y Timer del LED de estado

	/*----------------------------------------------------------------------------------------*/

	/* Configuración del Timer 4 para controlar el muestreo*/
	handlerSamplingTimer.ptrTIMx							= TIM4;
	handlerSamplingTimer.TIMx_Config.TIMx_mode				= BTIMER_MODE_UP;
	handlerSamplingTimer.TIMx_Config.TIMx_speed				= BTIMER_PLL_100MHz_SPEED_100us;
	handlerSamplingTimer.TIMx_Config.TIMx_period			= 50;
	handlerSamplingTimer.TIMx_Config.TIMx_interruptEnable 	= BTIMER_INTERRUP_ENABLE;
	BasicTimer_Config(&handlerSamplingTimer);

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
	usartComm.ptrUSARTx							= USART1;
	usartComm.USART_Config.USART_baudrate 		= USART_BAUDRATE_115200;
	usartComm.USART_Config.USART_datasize		= USART_DATASIZE_8BIT;
	usartComm.USART_Config.USART_parity			= USART_PARITY_NONE;
	usartComm.USART_Config.USART_stopbits		= USART_STOPBIT_1;
	usartComm.USART_Config.USART_mode			= USART_MODE_RXTX;
	usartComm.USART_Config.USART_enableIntRX	= USART_RX_INTERRUP_ENABLE;
	usartComm.USART_Config.USART_enableIntTX	= USART_TX_INTERRUP_DISABLE;
	USART_Config(&usartComm);

	/* 			Configuraciones del I2C1 para el acelerómetro 			*/

	/* Configuración del pin SCL del I2C1 */
	handlerI2C_SCL.pGPIOx								= GPIOB;
	handlerI2C_SCL.GPIO_PinConfig.GPIO_PinNumber		= PIN_8;
	handlerI2C_SCL.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_ALTFN;
	handlerI2C_SCL.GPIO_PinConfig.GPIO_PinOPType		= GPIO_OTYPE_OPENDRAIN;
	handlerI2C_SCL.GPIO_PinConfig.GPIO_PinSpeed			= GPIO_OSPEED_HIGH;
	handlerI2C_SCL.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
	handlerI2C_SCL.GPIO_PinConfig.GPIO_PinAltFunMode 	= AF4;
	GPIO_Config(&handlerI2C_SCL);

	/* Configuración del pin SDA del I2C1 */
	handlerI2C_SDA.pGPIOx								= GPIOB;
	handlerI2C_SDA.GPIO_PinConfig.GPIO_PinNumber		= PIN_9;
	handlerI2C_SDA.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_ALTFN;
	handlerI2C_SDA.GPIO_PinConfig.GPIO_PinOPType		= GPIO_OTYPE_OPENDRAIN;
	handlerI2C_SDA.GPIO_PinConfig.GPIO_PinSpeed			= GPIO_OSPEED_HIGH;
	handlerI2C_SDA.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
	handlerI2C_SDA.GPIO_PinConfig.GPIO_PinAltFunMode 	= AF4;
	GPIO_Config(&handlerI2C_SDA);

	/* Configuración para el I2C1 */
	handlerAccelerometer.ptrI2Cx		= I2C1;
	handlerAccelerometer.modeI2C		= I2C_MODE_FM;
	handlerAccelerometer.slaveAddress	= ACCEL_ADDRESS;
	i2c_config(&handlerAccelerometer);

	/*--------------------------------------------------------------------------------*/

	/* Inicio el cristal LSE como RTC activo */
	initLSE();

	//Fin initSystem

}

/** Función encargada de construir el string con el comando */
void commandBuild(void){
	bufferReception[counterReception] = usartRxData;
	counterReception++;

	// Aqui hacemmos la instrucción que detine la recepción del comando
	if(usartRxData == '@'){
		stringComplete = true;

		//Sustituyo el último caracter de @ por un null
		bufferReception[counterReception] = '\0';
		counterReception = 0;
	}

	// Volvemos a null para terminar
	usartRxData = '\0';
}

/** Función encargada de gestionar e identificar el comando ingresado, recibe el puntero al string */
void commandUSART(char* ptrBufferReception){

	/* El funcionamiento es de la siguiente forma: Empleamos el puntero al buffer para
	 * acceder a los elementos del string, y por medio de la función sscanf se almacena
	 * en 3 elemetos diferentes, el string del comando "cmd", y dos números enteros llamados
	 * "firstParameter" y "SecondParameter". De esta froma, podemos introducir información
	 * al micro desde el puerto serial */
	sscanf(ptrBufferReception, "%s %u %u %s", cmd, &firstParameter, &secondParameter, userMsg);

	/* Usamos la funcion strcmp, string compare, que me retorna un 0 si los 2 strings son iguales */

	// Este primer comando imprime una lista con los otros comando que tiene el equipo
	if(strcmp(cmd, "help") == 0){
		writeMsg(&usartComm, "Help Menu CMDs:\n");
		writeMsg(&usartComm, "1) help				-- Print this menu \n");
		writeMsg(&usartComm, "2) freq				-- Print current MCU frequency \n");
		writeMsg(&usartComm, "3) MCO #Source #Prescaler \n"
				" -- Source: 0 = HSE; 1 = LSE, 3 = PLL \n"
				" -- Pre-escaler from 1 to 5 \n");
		writeMsg(&usartComm, "4) reset			-- Reset MCO config, as HSI with Prescaler = 1 \n");
		writeMsg(&usartComm, "9) sampling		-- ADC sampling cmd, #A and #B are uint32_t \n");
		writeMsg(&usartComm, "10) show			-- Show ADC data saved in arrays \n");
		writeMsg(&usartComm, "11) capture		-- Launch capture of Accelerometer data \n");
		writeMsg(&usartComm, "12) fourier		-- Show frequency data with CMSIS-FFT \n");

	}
	else if(strcmp(cmd, "freq") == 0){
		writeMsg(&usartComm, "CMD: freq \n");
		sprintf(bufferData, "The clock frequency is %u Hz \n", freq);
		writeMsg(&usartComm, bufferData);
	}
	else if(strcmp(cmd, "MCO") == 0){
		writeMsg(&usartComm, "CMD: MCO \n");
		if((0 <= firstParameter)&&(firstParameter < 4)&&(firstParameter != 2)&&(secondParameter<6)){
			changeMCO1(firstParameter, secondParameter+2);
			writeMsg(&usartComm, "MCO1 configuration succesfull \n");
		}
		else{
			writeMsg(&usartComm, "Wrong source or prescaler \n");
		}
	}
	else if(strcmp(cmd, "reset") == 0){
		writeMsg(&usartComm, "CMD: reset \n");
		changeMCO1(MCO1_HSI_CLOCK, MCO1_DIVIDED_BY_1);
		writeMsg(&usartComm, "The MC01 source is HSI with prescaler of 1 \n");
	}
	else{
		writeMsg(&usartComm, "Wrong command \n");
	}
}


/** Función encargada de guardar los datos tomados cada 1ms en los respectivos arreglos */
void saveData(void){

	i2c_readMultipleRegisters(&handlerAccelerometer, arrayRegisters, 6, arraySaveData);

	int16_t AccelX = arraySaveData[1] << 8 | arraySaveData[0]; // Aquí lo que se hace es básicamente concatenar los valores
	arrayXYZdata[0] = (float)((int)AccelX*factConv);

	int16_t AccelY = arraySaveData[3] << 8 | arraySaveData[2]; // Aquí lo que se hace es básicamente concatenar los valores
	arrayXYZdata[1] = (float)((int)AccelY*factConv);

	int16_t AccelZ = arraySaveData[5] << 8 | arraySaveData[4]; // Aquí lo que se hace es básicamente concatenar los valores
	arrayXYZdata[2] = (float)((int)AccelZ*factConv);
}


/* Interrupción del timer blinky LED */
void BasicTimer2_Callback(void){
	GPIOxTooglePin(&handlerBlinkyPin); //Cambio el estado del LED PA5
}

/* Interrupción del timer de muestreo */
void BasicTimer4_Callback(void){
	flag200HzSamplingData = 1; 	// Levanto bandera para tomar datos cada 1ms
	counter_ms++;
}

/* Interrupción del USART1 */
void usart1Rx_Callback(void){
	usartRxData = getRxData();
}

/** Función encargada de gestionar las acciones según la tecla de recepción*/
void actionRxData(void){

	if(usartRxData == 'i'){
		sprintf(bufferData,"Welcome, the current frequency of the MCU is %u Hz \n", freq);
		writeMsg(&usartComm, bufferData);
		usartRxData = '\0';
	}

	else if(usartRxData == 'w'){
		sprintf(bufferData, "WHO_AM_I? (r)\n"); // La "(r)" en el texto es para indicar que estamos leyendo
		writeMsg(&usartComm, bufferData);

		i2cBuffer = i2c_readSingleRegister(&handlerAccelerometer, WHO_AM_I);
		sprintf(bufferData, "dataRead = 0x%x \n", (unsigned int) i2cBuffer);
		writeMsg(&usartComm, bufferData);
		usartRxData = '\0';
	}
	else if(usartRxData == 'r'){
		sprintf(bufferData, "PWR_MGMT_1 reset (w) \n"); // La "(w)" en el texto es para indicar que estamos escribiendo
		writeMsg(&usartComm, bufferData);

		i2c_writeSingleRegister(&handlerAccelerometer, PWR_MGMT_1, 0x00);
		usartRxData = '\0';
	}
	else if(usartRxData == 'x'){
		sprintf(bufferData, "Axis X data (r) \n");
		writeMsg(&usartComm, bufferData);

		i2c_readMultipleRegisters(&handlerAccelerometer, arrayRegisters, 6, arraySaveData);

		int16_t AccelX = arraySaveData[1] << 8 | arraySaveData[0]; // Aquí lo que se hace es básicamente concatenar los valores
		sprintf(bufferData, "AccelX = %.3f m/s² \n", AccelX*factConv);
		writeMsg(&usartComm, bufferData);
		usartRxData = '\0';
	}
	else if(usartRxData == 'y'){
		sprintf(bufferData, "Axis Y data (r) \n");
		writeMsg(&usartComm, bufferData);

		i2c_readMultipleRegisters(&handlerAccelerometer, arrayRegisters, 6, arraySaveData);

		int16_t AccelY = arraySaveData[3] << 8 | arraySaveData[2]; // Aquí lo que se hace es básicamente concatenar los valores
		sprintf(bufferData, "AccelY = %.3f m/s² \n", AccelY*factConv);
		writeMsg(&usartComm, bufferData);
		usartRxData = '\0';
	}
	else if(usartRxData == 'z'){
		sprintf(bufferData, "Axis Z data (r) \n");
		writeMsg(&usartComm, bufferData);

		i2c_readMultipleRegisters(&handlerAccelerometer, arrayRegisters, 6, arraySaveData);

		int16_t AccelZ = arraySaveData[5] << 8 | arraySaveData[4]; // Aquí lo que se hace es básicamente concatenar los valores
		sprintf(bufferData, "AccelZ = %.3f m/s² \n", AccelZ*factConv);
		writeMsg(&usartComm, bufferData);
		usartRxData = '\0';
	}
	else{
		usartRxData = '\0';
	}
}

void initLSE(void){
	// Activamos el LSE
	RCC->BDCR |= RCC_BDCR_LSEON;
	// Esperamos hasta que sea estable
	while(!(RCC->BDCR & RCC_BDCR_LSERDY)){
		__NOP();
	}
	// Seleccionamos el LSE como RTC
	RCC->BDCR |= RCC_BDCR_RTCSEL_0;
	// Activamos el RTC
	RCC->BDCR |= RCC_BDCR_RTCEN;
}
