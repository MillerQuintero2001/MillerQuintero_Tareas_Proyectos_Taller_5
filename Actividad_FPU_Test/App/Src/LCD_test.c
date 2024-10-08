/**
 ********************************************************************************
 * @file         : LCD_Test.c
 * @author       : Miller Quintero - miquinterog@unal.edu.co - MillerQuintero2001
 * @brief        : Solución básica de la Tarea Especial
 ********************************************************************************
 * El equipo cargado con este programa deberá enviar datos por USART1 a CoolTerm
 * dichos datos son de un acelerometro MPU6050, y tambien son mostrados en LCD
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
#include "ExtiDriver.h"
#include "USARTxDriver.h"
#include "SysTickDriver.h"
#include "PwmDriver.h"
#include "I2CDriver.h"
#include "PLLDriver.h"
#include "LcdDriver.h"

// Macro-definiciones específicas para el sensor
#define ACCEL_ADDRESS 	0b1101000; // 0xD2 -> Dirección del sensor Acelerómetro con ADO=0
#define ACCEL_XOUT_H	59 // 0x3B
#define ACCEL_XOUT_L	60 // OX3C
#define ACCEL_YOUT_H	61 // 0x3D
#define ACCEL_YOUT_L 	62 // 0x3E
#define ACCEL_ZOUT_H 	63 // 0x3F
#define ACCEL_ZOUT_L 	64 // 0x40

#define PWR_MGMT_1	107
#define WHO_AM_I	117

// Macro-definiciones específicas para la LCD
#define LCD_ADDRESS 	34; // 0x22 -> Dirección de la LCD, AD2=0, AD1=1, AD0=0


/* Definición de los handlers necesarios */

// Elementos para el Blinky LED
GPIO_Handler_t handlerBlinkyPin = 			{0}; // LED de estado del Pin A5
BasicTimer_Handler_t handlerBlinkyTimer = 	{0}; // Timer del LED de estado

// Elementos para el muestreo a 1KHz
BasicTimer_Handler_t handlerSamplingTimer = {0}; 	// Timer del muestreo de datos
float arrayXdata[1000] = {0};						// Array que guarda la información de datos eje X
float arrayYdata[1000] = {0};						// Array que guarda la información de datos eje Y
float arrayZdata[1000] = {0};						// Array que guarda la información de datos eje Z
float arrayXdataprint[2000] = {0};					// Array que guarda la información de datos eje X
float arrayYdataprint[2000] = {0};					// Array que guarda la información de datos eje Y
float arrayZdataprint[2000] = {0};					// Array que guarda la información de datos eje Z
int arrayLCDdata[3] = {0};							// Array que guarda la tripleta de datos para la LCD
uint16_t countPrint = 0;
uint16_t counter_ms = 0;							// Contador de milisegundos con el Timer 3
uint8_t flagPrintSamples = 0;						// Bandera para indicar que se solictaron 6000 datos desde la terminal
uint8_t flag6000Data = 0;
uint8_t flag2seg = 0;								// Bandera para indicar que han pasado 2seg
uint8_t flag1KHzSamplingData = 0;					// Bandera para indicar tomar dato cada 1ms


// Elementos para hacer la comunicación serial
GPIO_Handler_t handlerPinTX = {0};	// Pin de transmisión de datos
GPIO_Handler_t handlerPinRX = {0};	// Pin de recepción de datos
USART_Handler_t usart1Comm =  {0};	// Comunicación serial
uint8_t sendMsg = 0; // Variable para controlar la comunicación
uint8_t usart1RxData = 0; 	// Variable en la que se guarda el dato transmitido
char bufferData[64] = {0}; // Buffer de datos como un arreglo de caracteres

// Elementos para utilizar comunicación I2C con Acelerometro
GPIO_Handler_t handlerI2C_SDA = {0};
GPIO_Handler_t handlerI2C_SCL = {0};
I2C_Handler_t handlerAccelerometer = {0};
uint8_t i2cBuffer = 0;

// Elementos para utilizar comunicación I2C con LCD
GPIO_Handler_t handlerI2C_SDA_LCD = {0};
GPIO_Handler_t handlerI2C_SCL_LCD = {0};
I2C_Handler_t handlerLCD = {0};
uint8_t i2cLCDBuffer = 0;

// Elementos para los pines en modo PWM, representando duttyCycle en función del eje
GPIO_Handler_t handlerPinPwmAxisX = {0};
GPIO_Handler_t handlerPinPwmAxisY = {0};
GPIO_Handler_t handlerPinPwmAxisZ = {0};

PWM_Handler_t handlerXSignalPWM = {0};
PWM_Handler_t handlerYSignalPWM = {0};
PWM_Handler_t handlerZSignalPWM = {0};

uint16_t duttyValueX = 10000;
uint16_t duttyValueY = 10000;
uint16_t duttyValueZ = 10000;


///* Inicializo variables a emplear */
unsigned int freq = 0;
uint16_t clock80 = 80;
float factConv = 9.78/16384.0; //Factor de conversión para pasar medidas de acelerómetro, gravedad de Medellín 9.78

/* Definición de las cabeceras de funciones del main */
void initSystem(void); 								// Función que inicializa los periféricos básicos
void actionRxData(void);							// Funcíón con la que se gestionan los casos de teclas recibidas
void saveData(void);								// Función encargada de guardar los datos
int16_t duttyAccordData(int data);					// Función encargada de entegar el valor de ajusta para el duty cycle PWM según datos Accel X,Y,Z

/** Función principal del programa
 * ¡Esta función es el corazón del programa! */
int main(void){

	// Inicializamos todos los elementos del sistema
	initSystem();
	// Mensajes de inicio
	freq = (unsigned int)getConfigPLL();

    /* Loop forever */
	while(1){

		if(counter_ms > 12){
			moveCursorLCD(&handlerLCD, 0, 0);
			sendStringLCD(&handlerLCD, "Mis");
			//delay_ms(1);
			moveCursorLCD(&handlerLCD, 0, 1);
			sprintf(bufferData, "X = %d m/s^2", (int)countPrint);
			sendStringLCD(&handlerLCD, bufferData);
			//delay_ms(1);
			moveCursorLCD(&handlerLCD, 0, 2);
			sendStringLCD(&handlerLCD, "Accel");
			//delay_ms(1);
			moveCursorLCD(&handlerLCD, 0, 3);
			sendStringLCD(&handlerLCD, "Test");
			delay_ms(1000);
			clearLineLCD(&handlerLCD, 1);
			counter_ms = 0;
		}

	}
	return 0;
}

/** Función encargada de iniciar hardware para un pin*/
void initSystem(void){

	// Activamos el Coprocesador Matemático - FPU
	SCB->CPACR |= (0XF << 20);

	// Configuramos el equipo a 8MHz, usando el driver del PLL
	configPLL(clock80);

	// Configuramos el Systick con el reloj PLL de 80MHz
	config_SysTick_ms(PLL_CLOCK_80_CONFIGURED);

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
	// Fin del GPIO y Timer del LED de estado
	/*----------------------------------------------------------------------------------------*/


	/* Configuración de la LCD por I2C, usaremos el puerto 2 del I2C*/

	/* Configuración del pin SCL del I2C2 */
	handlerI2C_SCL_LCD.pGPIOx								= GPIOA;
	handlerI2C_SCL_LCD.GPIO_PinConfig.GPIO_PinNumber		= PIN_8;
	handlerI2C_SCL_LCD.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_ALTFN;
	handlerI2C_SCL_LCD.GPIO_PinConfig.GPIO_PinOPType		= GPIO_OTYPE_OPENDRAIN;
	handlerI2C_SCL_LCD.GPIO_PinConfig.GPIO_PinSpeed			= GPIO_OSPEED_HIGH;
	handlerI2C_SCL_LCD.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
	handlerI2C_SCL_LCD.GPIO_PinConfig.GPIO_PinAltFunMode 	= AF4;
	GPIO_Config(&handlerI2C_SCL_LCD);


	/* Configuración del pin SDA del I2C2 */
	handlerI2C_SDA_LCD.pGPIOx								= GPIOC;
	handlerI2C_SDA_LCD.GPIO_PinConfig.GPIO_PinNumber		= PIN_9;
	handlerI2C_SDA_LCD.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_ALTFN;
	handlerI2C_SDA_LCD.GPIO_PinConfig.GPIO_PinOPType		= GPIO_OTYPE_OPENDRAIN;
	handlerI2C_SDA_LCD.GPIO_PinConfig.GPIO_PinSpeed			= GPIO_OSPEED_HIGH;
	handlerI2C_SDA_LCD.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
	handlerI2C_SDA_LCD.GPIO_PinConfig.GPIO_PinAltFunMode 	= AF4;
	GPIO_Config(&handlerI2C_SDA_LCD);

	/* Configuración para el I2C1 */
	handlerLCD.ptrI2Cx		= I2C3;
	handlerLCD.modeI2C		= I2C_MODE_FM;
	handlerLCD.slaveAddress	= LCD_ADDRESS;
	i2c_config(&handlerLCD);

	initLCD(&handlerLCD);

}

/** Función encargada de gestionar las acciones según la tecla de recepción*/
void actionRxData(void){

	if(usart1RxData == 'i'){
		sprintf(bufferData,"Welcome, the current frequency of the MCU is %u Hz \n", freq);
		writeMsg(&usart1Comm, bufferData);
		usart1RxData = '\0';
	}

	else if(usart1RxData == 'w'){
		sprintf(bufferData, "WHO_AM_I? (r)\n"); // La "(r)" en el texto es para indicar que estamos leyendo
		writeMsg(&usart1Comm, bufferData);

		i2cBuffer = i2c_readSingleRegister(&handlerAccelerometer, WHO_AM_I);
		sprintf(bufferData, "dataRead = 0x%x \n", (unsigned int) i2cBuffer);
		writeMsg(&usart1Comm, bufferData);
		usart1RxData = '\0';
	}

	else if(usart1RxData == 'm'){
		flag6000Data = 1;		// Levanto bandera de solicitar 6 mil datos (2 mil por eje), para iniciar el conteo de 2 segundos
		usart1RxData = '\0';
	}
	else if(usart1RxData == 'r'){
		sprintf(bufferData, "PWR_MGMT_1 reset (w) \n"); // La "(w)" en el texto es para indicar que estamos escribiendo
		writeMsg(&usart1Comm, bufferData);

		i2c_writeSingleRegister(&handlerAccelerometer, PWR_MGMT_1, 0x00);
		usart1RxData = '\0';
	}
	else if(usart1RxData == 'x'){
		sprintf(bufferData, "Axis X data (r) \n");
		writeMsg(&usart1Comm, bufferData);

		uint8_t AccelX_low = i2c_readSingleRegister(&handlerAccelerometer, ACCEL_XOUT_L);
		uint8_t AccelX_high = i2c_readSingleRegister(&handlerAccelerometer, ACCEL_XOUT_H);
		int16_t AccelX = AccelX_high << 8 | AccelX_low; // Aquí lo que se hace es básicamente concatenar los valores
		sprintf(bufferData, "AccelX = %.3f m/s² \n", AccelX*factConv);
		writeMsg(&usart1Comm, bufferData);
		usart1RxData = '\0';
	}
	else if(usart1RxData == 'y'){
		sprintf(bufferData, "Axis Y data (r) \n");
		writeMsg(&usart1Comm, bufferData);

		uint8_t AccelY_low = i2c_readSingleRegister(&handlerAccelerometer, ACCEL_YOUT_L);
		uint8_t AccelY_high = i2c_readSingleRegister(&handlerAccelerometer, ACCEL_YOUT_H);
		int16_t AccelY = AccelY_high << 8 | AccelY_low; // Aquí lo que se hace es básicamente concatenar los valores
		sprintf(bufferData, "AccelY = %.3f m/s² \n", AccelY*factConv);
		writeMsg(&usart1Comm, bufferData);
		usart1RxData = '\0';
	}
	else if(usart1RxData == 'z'){
		sprintf(bufferData, "Axis Z data (r) \n");
		writeMsg(&usart1Comm, bufferData);

		uint8_t AccelZ_low = i2c_readSingleRegister(&handlerAccelerometer, ACCEL_ZOUT_L);
		uint8_t AccelZ_high = i2c_readSingleRegister(&handlerAccelerometer, ACCEL_ZOUT_H);
		int16_t AccelZ = AccelZ_high << 8 | AccelZ_low; // Aquí lo que se hace es básicamente concatenar los valores
		sprintf(bufferData, "AccelZ = %.3f m/s² \n", AccelZ*factConv);
		writeMsg(&usart1Comm, bufferData);
		usart1RxData = '\0';
	}

	else if(usart1RxData == 'h'){
		sprintf(bufferData, "Axis X data;   Axis Y data;   Axis Z data \n");
		writeMsg(&usart1Comm, bufferData);

		sprintf(bufferData, "%.3f m/s²;    %.3f m/s²;    %.3f m/s²;\n", arrayXdata[999], arrayYdata[999], arrayZdata[999]);
		writeMsg(&usart1Comm, bufferData);
		usart1RxData = '\0';
	}
	else{
		usart1RxData = '\0';
	}
}

/** Función encargada de guardar los datos tomados cada 1ms en los respectivos arreglos */
void saveData(void){

	uint8_t AccelX_low = i2c_readSingleRegister(&handlerAccelerometer, ACCEL_XOUT_L);
	uint8_t AccelX_high = i2c_readSingleRegister(&handlerAccelerometer, ACCEL_XOUT_H);
	int16_t AccelX = AccelX_high << 8 | AccelX_low; // Aquí lo que se hace es básicamente concatenar los valores
	arrayLCDdata[0] = (int)AccelX;

	uint8_t AccelY_low = i2c_readSingleRegister(&handlerAccelerometer, ACCEL_YOUT_L);
	uint8_t AccelY_high = i2c_readSingleRegister(&handlerAccelerometer, ACCEL_YOUT_H);
	int16_t AccelY = AccelY_high << 8 | AccelY_low; // Aquí lo que se hace es básicamente concatenar los valores
	arrayLCDdata[1] = (int)AccelY;

	uint8_t AccelZ_low = i2c_readSingleRegister(&handlerAccelerometer, ACCEL_ZOUT_L);
	uint8_t AccelZ_high = i2c_readSingleRegister(&handlerAccelerometer, ACCEL_ZOUT_H);
	int16_t AccelZ = AccelZ_high << 8 | AccelZ_low; // Aquí lo que se hace es básicamente concatenar los valores
	arrayLCDdata[2] = (int)AccelZ;

	if(counter_ms > 0){
		arrayXdata[counter_ms-1] = AccelX*factConv; 	// Se guardan datos para el muestreo constante de 1KHz
		arrayYdata[counter_ms-1] = AccelY*factConv;
		arrayZdata[counter_ms-1] = AccelZ*factConv;
	}
	if((flag6000Data)&&(countPrint > 0)){
		arrayXdataprint[countPrint-1] = AccelX*factConv;	// Se guardan los datos 2000 datos por eje, solo cuando se solicitan
		arrayYdataprint[countPrint-1] = AccelY*factConv;	// con la tecla, y se usa el muestreo 1KHz
		arrayZdataprint[countPrint-1] = AccelZ*factConv;
	}

}

/** Función que retorna el valor necesario para ajustar el Dutty-Cycle de los PWM */
int16_t duttyAccordData(int data){
	int16_t ajuste = 0;
	if(data == 0){
		// Si el dato es nulo, el ajuste es 0, quedando en 10000 el dutty, para tener 50% de Dutty-Cycle
		ajuste = 0;
	}
	else{
		/* Hago una división del dato, por 1.5 veces el máximo valor que en teoría entrega el sensor,
		 * para controlar un poco más el duty; luego multiplico por 10000 ya que es lo que falta
		 * para llegar a los 20000 del ARR, finalmente casteo el resultado a un signed int de
		 * 16 bits, ya que con este tipo de dato es que trabaja la función updateDuttyCycle*/
		ajuste = (int16_t)(10000*((float)data/(1.5*16384.0)));
	}
	return ajuste; // Este valor se lo vamos a sumar al DuttyValue del PWM según eje, nótese que está garantizada para negativos y positivos
}

/** Interrupción del timer blinky LED*/
void BasicTimer2_Callback(void){
	GPIOxTooglePin(&handlerBlinkyPin); //Cambio el estado del LED PA5
	counter_ms++;
	if(counter_ms > 4){
		countPrint++;
	}
}




