/**
 ******************************************************************************
 * @file           : Accel_I2C_Main.c
 * @author         : Miller Quintero - miquinterog@unal.edu.co
 * @brief          : Pruebas driver I2C
 ******************************************************************************
 * Proyecto de prueba del driver I2C, empleando un sensor de acelerómetro
 * con dicho protocolo, pero esta vez usando una funcion de múltiple lectura
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
char bufferData[64] = "Accelerometer MPU-6050 testing...\n"; //Mensaje de muestra

// Elementos del SysTick
uint32_t systemTicks = 0;
uint32_t systemTicksStart = 0;
uint32_t systemTicksEnd = 0;

// Elementos para utilizar comunicación I2C
GPIO_Handler_t handlerI2C_SDA = {0};
GPIO_Handler_t handlerI2C_SCL = {0};
I2C_Handler_t handlerAccelerometer = {0};
uint8_t i2cBuffer = 0;

// Macro-definiciones específicas para el sensor
#define ACCEL_ADDRESS 	0b1101000; // 0xD2 -> Dirección del sensor Acelerómetro con ADO=0
#define ACCEL_XOUT_H	59 // 0x3B
#define ACCEL_XOUT_L	60 // OX3C
#define ACCEL_YOUT_H	61 // 0x3D
#define ACCEL_YOUT_L 	62 // 0x3E
#define ACCEL_ZOUT_H 	63 // 0x3F
#define ACCEL_ZOUT_L 	64 // 0x40
#define GYRO_XOUT_H		67 // 0X43
#define GYRO_XOUT_L		68 // 0X44
#define GYRO_YOUT_H		69 // 0X45
#define GYRO_YOUT_L		70 // 0X46
#define GYRO_ZOUT_H		71 // 0X47
#define GYRO_ZOUT_L		72 // 0X48

uint8_t arraySaveData[6] = {0};

#define PWR_MGMT_1	107
#define WHO_AM_I	117

/* Definición de las cabeceras de funciones del main */
void initSystem(void); 			// Función que inicializa los periféricos básicos

/** Función principal del programa
 * ¡Esta función es el corazón del programa! */
int main(void){

	// Inicializamos todos los elementos del sistema
	initSystem();

	// Imprimimos un mensaje de inicio
	writeMsg(&usart2Comm, bufferData);

    /* Loop forever */
	while(1){

		// Hacemos un "eco" con el valor que nos llega por el serial
		if(usart2RxData != '\0'){
			writeChar(&usart2Comm, usart2RxData);

			if(usart2RxData == 'w'){
				sprintf(bufferData, "WHO_AM_I? (r)\n"); // La "(r)" en el texto es para indicar que estamos leyendo
				writeMsg(&usart2Comm, bufferData);

				i2cBuffer = i2c_readSingleRegister(&handlerAccelerometer, WHO_AM_I);
				sprintf(bufferData, "dataRead = 0x%x \n", (unsigned int) i2cBuffer);
				writeMsg(&usart2Comm, bufferData);
				usart2RxData = '\0';
			}
			else if(usart2RxData == 'a'){
				writeMsg(&usart2Comm, "Hola, funciona por favor!\n");
				usart2RxData = '\0';
			}

			else if(usart2RxData == 'p'){
				sprintf(bufferData, "PWR_MGMT_1 state (r) \n");
				writeMsg(&usart2Comm, bufferData);

				i2cBuffer = i2c_readSingleRegister(&handlerAccelerometer, PWR_MGMT_1);
				sprintf(bufferData, "dataRead = 0x%x \n", (unsigned int) i2cBuffer);
				writeMsg(&usart2Comm, bufferData);
				usart2RxData = '\0';
			}
			else if(usart2RxData == 'r'){
				sprintf(bufferData, "PWR_MGMT_1 reset (w) \n"); // La "(w)" en el texto es para indicar que estamos escribiendo
				writeMsg(&usart2Comm, bufferData);

				i2c_writeSingleRegister(&handlerAccelerometer, PWR_MGMT_1, 0x00);
				usart2RxData = '\0';
			}
			else if(usart2RxData == 'x'){
				sprintf(bufferData, "Axis X data (r) \n");
				writeMsg(&usart2Comm, bufferData);

				i2c_readMultipleRegisters(&handlerAccelerometer, ACCEL_XOUT_H, 6, arraySaveData);

				uint8_t AccelX_high = arraySaveData[0];
				uint8_t AccelX_low = arraySaveData[1];
				int16_t AccelX = AccelX_high << 8 | AccelX_low; // Aquí lo que se hace es básicamente concatenar los valores
				sprintf(bufferData, "AccelX = %.3f \n", (9.7761f*(float)AccelX)/16384.00f);
				writeMsg(&usart2Comm, bufferData);
				usart2RxData = '\0';
			}
			else if(usart2RxData == 'y'){
				sprintf(bufferData, "Axis Y data (r) \n");
				writeMsg(&usart2Comm, bufferData);

				i2c_readMultipleRegisters(&handlerAccelerometer,  ACCEL_XOUT_H, 6, arraySaveData);

				uint8_t AccelY_high = arraySaveData[2];
				uint8_t AccelY_low = arraySaveData[3];
				int16_t AccelY = AccelY_high << 8 | AccelY_low; // Aquí lo que se hace es básicamente concatenar los valores
				sprintf(bufferData, "AccelY = %.3f \n", (9.7761f*(float)AccelY)/16384.00f);
				writeMsg(&usart2Comm, bufferData);
				usart2RxData = '\0';
			}
			else if(usart2RxData == 'z'){
				sprintf(bufferData, "Axis Z data (r) \n");
				writeMsg(&usart2Comm, bufferData);

				i2c_readMultipleRegisters(&handlerAccelerometer,  ACCEL_XOUT_H, 6, arraySaveData);

				uint8_t AccelZ_high = arraySaveData[4];
				uint8_t AccelZ_low = arraySaveData[5];
				int16_t AccelZ = AccelZ_high << 8 | AccelZ_low; // Aquí lo que se hace es básicamente concatenar los valores
				sprintf(bufferData, "AccelZ = %.3f \n", (9.7761f*(float)AccelZ)/16384.00f);
				writeMsg(&usart2Comm, bufferData);
				usart2RxData = '\0';
			}
			else if(usart2RxData == 'g'){
				sprintf(bufferData, "Gyroscope data (r) \n");
				writeMsg(&usart2Comm, bufferData);

				i2c_readMultipleRegisters(&handlerAccelerometer,  GYRO_XOUT_H, 6, arraySaveData);

				uint8_t GyroX_high = arraySaveData[0];
				uint8_t GyroX_low = arraySaveData[1];
				uint8_t GyroY_high = arraySaveData[2];
				uint8_t GyroY_low = arraySaveData[3];
				uint8_t GyroZ_high = arraySaveData[4];
				uint8_t GyroZ_low = arraySaveData[5];
				int16_t GyroX = GyroX_high << 8 | GyroX_low;
				int16_t GyroY = GyroY_high << 8 | GyroY_low;
				int16_t GyroZ = GyroZ_high << 8 | GyroZ_low;
				sprintf(bufferData, "GyroX = %.3f\n", ((float)(GyroX)/131.00f));
				writeMsg(&usart2Comm, bufferData);
				sprintf(bufferData, "GyroY = %.3f\n", ((float)(GyroY)/131.00f));
				writeMsg(&usart2Comm, bufferData);
				sprintf(bufferData, "GyroZ = %.3f\n", ((float)(GyroZ)/131.00f));
				writeMsg(&usart2Comm, bufferData);
				usart2RxData = '\0';
			}
			else{
				usart2RxData = '\0';
			}
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

	/* GPIO y Timer del Blinky Led de Estado */
	handlerBlinkyPin.pGPIOx								= GPIOA;
	handlerBlinkyPin.GPIO_PinConfig.GPIO_PinNumber 		= PIN_5;
	handlerBlinkyPin.GPIO_PinConfig.GPIO_PinMode		= GPIO_MODE_OUT;
	handlerBlinkyPin.GPIO_PinConfig.GPIO_PinSpeed 		= GPIO_OSPEED_FAST;
	handlerBlinkyPin.GPIO_PinConfig.GPIO_PinOPType 		= GPIO_OTYPE_PUSHPULL;
	handlerBlinkyPin.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
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
	startBasicTimer(&handlerBlinkyTimer);
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

	// Hacemos un primer reset del sensor para asegurar que funcione a la primera
	i2c_writeSingleRegister(&handlerAccelerometer, PWR_MGMT_1, 0x00);
}

/** Interrupción del timer blinky LED*/
void BasicTimer2_Callback(void){
	GPIOxTooglePin(&handlerBlinkyPin); //Cambio el estado del LED PA5
}


/** Interrupción del USART2 */
void usart2Rx_Callback(void){
	usart2RxData = getRxData();	// Pongo en alto la variable bandera del USART2 para el main
}



