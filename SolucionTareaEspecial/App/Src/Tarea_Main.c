/**
 ********************************************************************************
 * @file         : Tarea_Main.c
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
int arrayXdata[2000] = {0};							// Array que guarda la información de datos eje X
int arrayYdata[2000] = {0};							// Array que guarda la información de datos eje X
int arrayZdata[2000] = {0};							// Array que guarda la información de datos eje X
int arrayLCDdata[3] = {0};							// Array que guarda la tripleta de datos para la LCD
uint16_t counter_ms = 0;							// Contador de milisegundos con el Timer 3
uint8_t flagPrintSamples = 0;						// Bandera para indicar que se solictaron 6000 datos desde la terminal
uint8_t flag2seg = 0;								// Bandera para indicar que han pasado 2seg
uint8_t flag1KHzSamplingData = 0;					// Bandera para indicar tomar dato cada 1ms


// Elementos para hacer la comunicación serial
GPIO_Handler_t handlerPinTX = {0};	// Pin de transmisión de datos
GPIO_Handler_t handlerPinRX = {0};	// Pin de recepción de datos
USART_Handler_t usart1Comm =  {0};	// Comunicación serial
uint8_t sendMsg = 0; // Variable para controlar la comunicación
uint8_t usart1RxData = 0; 	// Variable en la que se guarda el dato transmitido
char bufferData[64] = {0}; // Buffer de datos como un arreglo de caracteres
char bufferPrint[ ] = {0}; // Buffer de datos como un arreglo de caracteres

// Elementos para utilizar comunicación I2C con Acelerometro
GPIO_Handler_t handlerI2C_SDA = {0};
GPIO_Handler_t handlerI2C_SCL = {0};
I2C_Handler_t handlerAccelerometer = {0};
uint8_t i2cBuffer = 0;

//// Elementos para utilizar comunicación I2C con LCD
//GPIO_Handler_t handlerI2C_SDA_LCD = {0};
//GPIO_Handler_t handlerI2C_SCL_LCD = {0};
//I2C_Handler_t handlerLCD = {0};
//uint8_t i2cLCDBuffer = 0;
//
//// Elementos para los pines en modo PWM, representando duttyCycle en función del eje
//GPIO_Handler_t handlerPinPwmAxisX = {0};
//GPIO_Handler_t handlerPinPwmAxisY = {0};
//GPIO_Handler_t handlerPinPwmAxisZ = {0};
//
//PWM_Handler_t handlerXSignalPWM = {0};
//PWM_Handler_t handlerYSignalPWM = {0};
//PWM_Handler_t handlerZSignalPWM = {0};
//
//uint16_t duttyValueX = 10000;
//uint16_t duttyValueY = 10000;
//uint16_t duttyValueZ = 10000;

// Elementos del SysTick
uint32_t systemTicks = 0;
uint32_t systemTicksStart = 0;
uint32_t systemTicksEnd = 0;

//// Elementos para la interrupción externa del User Button
//GPIO_Handler_t handlerUserButton = 			{0}; // Boton de usuario del Pin C13
//EXTI_Config_t handlerUserButtonExti = 		{0}; // Interrupción externa del botón de usuario
//

///* Inicializo variables a emplear */
unsigned int freq = 0;
uint16_t clock80 = 80;
float factConv = 9.78/16384.0; //Factor de conversión para pasar medidas de acelerómetro a m/s²

/* Definición de las cabeceras de funciones del main */
void initSystem(void); 					// Función que inicializa los periféricos básicos
void actionRxData(void);				// Funcíón con la que se gestionan los casos de teclas recibidas
void saveData(void);					// Función encargada de guardar los datos

/** Función principal del programa
 * ¡Esta función es el corazón del programa! */
int main(void){

	// Inicializamos todos los elementos del sistema
	initSystem();
	// Mensajes de inicio
	freq = (unsigned int)getConfigPLL();
	sprintf(bufferData,"Welcome, the current frequency of the MCU is %u Hz \n", freq);
	writeMsg(&usart1Comm, bufferData);

    /* Loop forever */
	while(1){

		// Hacemos un "eco" con el valor que nos llega por el serial
		if(usart1RxData != '\0'){
			actionRxData();
		}

		// Muestreo de datos constante cada 1ms
		if(flag1KHzSamplingData){
			saveData();
			flag1KHzSamplingData = 0;
		}

		// Si han pasado 2seg y se dio la orden con la 'm' en la terminal, muestre los datos tomados en ese tiempo
		if(flagPrintSamples && flag2seg){
			sprintf(bufferData, "      X;            Y;            Z;\n");
			writeMsg(&usart1Comm, bufferData);
			for(int i; i<2000; i++){
				sprintf(bufferData,"%.3f m/s²;   %.3f m/s²;   %.3f m/s²;   Dato #%d\n", arrayXdata[i]*factConv, arrayYdata[i]*factConv, arrayZdata[i]*factConv,i+1);
				writeMsg(&usart1Comm, bufferData);
			}
			flagPrintSamples = 0;	//Solo una vez que se hayan imprimido los datos, se baja la bandera
			flag2seg = 0;
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
	config_SysTick_ms(PLL_CLOCK_CONFIGURED);

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

	/* Configuración del Timer 3 para controlar el muestreo*/
	handlerSamplingTimer.ptrTIMx							= TIM3;
	handlerSamplingTimer.TIMx_Config.TIMx_mode				= BTIMER_MODE_UP;
	handlerSamplingTimer.TIMx_Config.TIMx_speed				= BTIMER_PLL_80MHz_SPEED_100us;
	handlerSamplingTimer.TIMx_Config.TIMx_period			= 10;
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
	usart1Comm.ptrUSARTx						= USART1;
	usart1Comm.USART_Config.USART_baudrate 		= USART_BAUDRATE_115200;
	usart1Comm.USART_Config.USART_datasize		= USART_DATASIZE_8BIT;
	usart1Comm.USART_Config.USART_parity		= USART_PARITY_NONE;
	usart1Comm.USART_Config.USART_stopbits		= USART_STOPBIT_1;
	usart1Comm.USART_Config.USART_mode			= USART_MODE_RXTX;
	usart1Comm.USART_Config.USART_enableIntRX	= USART_RX_INTERRUP_ENABLE;
	usart1Comm.USART_Config.USART_enableIntTX	= USART_TX_INTERRUP_DISABLE;
	USART_Config(&usart1Comm);

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

//	/* Configuración de los pines PWM para mostrar señal según valor de los datos*/
//
//	handlerPinPwmAxisX.pGPIOx								= GPIOA;
//	handlerPinPwmAxisX.GPIO_PinConfig.GPIO_PinNumber		= PIN_0;
//	handlerPinPwmAxisX.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_ALTFN;
//	handlerPinPwmAxisX.GPIO_PinConfig.GPIO_PinOPType		= GPIO_OTYPE_PUSHPULL;
//	handlerPinPwmAxisX.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
//	handlerPinPwmAxisX.GPIO_PinConfig.GPIO_PinSpeed			= GPIO_OSPEED_FAST;
//	handlerPinPwmAxisX.GPIO_PinConfig.GPIO_PinAltFunMode	= AF2;
//	GPIO_Config(&handlerPinPwmAxisX);
//
//	handlerXSignalPWM.ptrTIMx						= TIM5;
//	handlerXSignalPWM.PWMx_Config.PWMx_Channel		= PWM_CHANNEL_1;
//	handlerXSignalPWM.PWMx_Config.PWMx_DuttyCicle	= duttyValueX;
//	handlerXSignalPWM.PWMx_Config.PWMx_Period		= 20000;
//	handlerXSignalPWM.PWMx_Config.PWMx_Prescaler	= 80;
//	pwm_Config(&handlerXSignalPWM);
//
//	handlerPinPwmAxisY.pGPIOx								= GPIOA;
//	handlerPinPwmAxisY.GPIO_PinConfig.GPIO_PinNumber		= PIN_1;
//	handlerPinPwmAxisY.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_ALTFN;
//	handlerPinPwmAxisY.GPIO_PinConfig.GPIO_PinOPType		= GPIO_OTYPE_PUSHPULL;
//	handlerPinPwmAxisY.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
//	handlerPinPwmAxisY.GPIO_PinConfig.GPIO_PinSpeed			= GPIO_OSPEED_FAST;
//	handlerPinPwmAxisY.GPIO_PinConfig.GPIO_PinAltFunMode	= AF2;
//	GPIO_Config(&handlerPinPwmAxisY);
//
//	handlerYSignalPWM.ptrTIMx						= TIM5;
//	handlerYSignalPWM.PWMx_Config.PWMx_Channel		= PWM_CHANNEL_2;
//	handlerYSignalPWM.PWMx_Config.PWMx_DuttyCicle	= duttyValueY;
//	handlerYSignalPWM.PWMx_Config.PWMx_Period		= 20000;
//	handlerYSignalPWM.PWMx_Config.PWMx_Prescaler	= 80;
//	pwm_Config(&handlerYSignalPWM);
//
//	handlerPinPwmAxisZ.pGPIOx								= GPIOA;
//	handlerPinPwmAxisZ.GPIO_PinConfig.GPIO_PinNumber		= PIN_2;
//	handlerPinPwmAxisZ.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_ALTFN;
//	handlerPinPwmAxisZ.GPIO_PinConfig.GPIO_PinOPType		= GPIO_OTYPE_PUSHPULL;
//	handlerPinPwmAxisZ.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
//	handlerPinPwmAxisZ.GPIO_PinConfig.GPIO_PinSpeed			= GPIO_OSPEED_FAST;
//	handlerPinPwmAxisZ.GPIO_PinConfig.GPIO_PinAltFunMode	= AF2;
//	GPIO_Config(&handlerPinPwmAxisZ);
//
//	handlerZSignalPWM.ptrTIMx						= TIM5;
//	handlerZSignalPWM.PWMx_Config.PWMx_Channel		= PWM_CHANNEL_3;
//	handlerZSignalPWM.PWMx_Config.PWMx_DuttyCicle	= duttyValueZ;
//	handlerZSignalPWM.PWMx_Config.PWMx_Period		= 20000;
//	handlerZSignalPWM.PWMx_Config.PWMx_Prescaler	= 80;
//	pwm_Config(&handlerZSignalPWM);
//
//	/*-------------- Fin de la configuración de los 3 pines como PWM --------------*/
//
//	/* Configuración de la LCD por I2C, usaremos el puerto 2 del I2C*/
//
//	/* Configuración del pin SCL del I2C2 */
//	handlerI2C_SCL_LCD.pGPIOx								= GPIOB;
//	handlerI2C_SCL_LCD.GPIO_PinConfig.GPIO_PinNumber		= PIN_8;
//	handlerI2C_SCL_LCD.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_ALTFN;
//	handlerI2C_SCL_LCD.GPIO_PinConfig.GPIO_PinOPType		= GPIO_OTYPE_OPENDRAIN;
//	handlerI2C_SCL_LCD.GPIO_PinConfig.GPIO_PinSpeed			= GPIO_OSPEED_HIGH;
//	handlerI2C_SCL_LCD.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
//	handlerI2C_SCL_LCD.GPIO_PinConfig.GPIO_PinAltFunMode 	= AF4;
//	GPIO_Config(&handlerI2C_SCL_LCD);
//
//
//	/* Configuración del pin SDA del I2C2 */
//	handlerI2C_SDA_LCD.pGPIOx								= GPIOB;
//	handlerI2C_SDA_LCD.GPIO_PinConfig.GPIO_PinNumber		= PIN_9;
//	handlerI2C_SDA_LCD.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_ALTFN;
//	handlerI2C_SDA_LCD.GPIO_PinConfig.GPIO_PinOPType		= GPIO_OTYPE_OPENDRAIN;
//	handlerI2C_SDA_LCD.GPIO_PinConfig.GPIO_PinSpeed			= GPIO_OSPEED_HIGH;
//	handlerI2C_SDA_LCD.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
//	handlerI2C_SDA_LCD.GPIO_PinConfig.GPIO_PinAltFunMode 	= AF4;
//	GPIO_Config(&handlerI2C_SDA_LCD);
//
//	/* Configuración para el I2C1 */
//	handlerAccelerometer.ptrI2Cx		= I2C2;
//	handlerAccelerometer.modeI2C		= I2C_MODE_FM;
//	handlerAccelerometer.slaveAddress	= LCD_ADDRESS;
//	i2c_config(&handlerLCD);
}

/** Función encargada de gestionar las acciones según la tecla de recepción*/
void actionRxData(void){
	if(usart1RxData == 'w'){
		sprintf(bufferData, "WHO_AM_I? (r)\n"); // La "(r)" en el texto es para indicar que estamos leyendo
		writeMsg(&usart1Comm, bufferData);

		i2cBuffer = i2c_readSingleRegister(&handlerAccelerometer, WHO_AM_I);
		sprintf(bufferData, "dataRead = 0x%x \n", (unsigned int) i2cBuffer);
		writeMsg(&usart1Comm, bufferData);
		usart1RxData = '\0';
	}

	else if(usart1RxData == 'm'){
		flagPrintSamples = 1;
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

	if(counter_ms>0){
		arrayXdata[counter_ms-1] = (int)AccelX;
		arrayYdata[counter_ms-1] = (int)AccelY;
		arrayZdata[counter_ms-1] = (int)AccelZ;
	}

}

/** Interrupción del timer blinky LED*/
void BasicTimer2_Callback(void){
	GPIOxTooglePin(&handlerBlinkyPin); //Cambio el estado del LED PA5
}

/** Interrupción del timer de muestreo*/
void BasicTimer3_Callback(void){
	if(counter_ms>2000){
		counter_ms = 0;
		flag2seg = 1;
	}
	else{
		counter_ms++;
	}
	flag1KHzSamplingData = 1;
}


/* Interrupción del USART1 */
void usart1Rx_Callback(void){
	usart1RxData = getRxData();
}

