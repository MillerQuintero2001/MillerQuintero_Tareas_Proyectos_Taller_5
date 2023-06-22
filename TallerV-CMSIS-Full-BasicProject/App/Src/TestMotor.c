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
USART_Handler_t usartComm =  {0};	// Comunicación serial
uint8_t sendMsg = 0; // Variable para controlar la comunicación
uint8_t usartData = 0; // Variable en la que se guarda el dato transmitido
char bufferData[128] = {0}; // Buffer de datos como un arreglo de caracteres

// Elementos para el PWM
GPIO_Handler_t handlerPinPwmStepMotor1 = {0};
GPIO_Handler_t handlerPinDirStepMotor1 = {0};
PWM_Handler_t handlerSignalStepMotor1 = {0};
GPIO_Handler_t handlerPinPwmStepMotor2 = {0};
GPIO_Handler_t handlerPinDirStepMotor2 = {0};
PWM_Handler_t handlerSignalStepMotor2 = {0};

uint16_t period = 10;
uint16_t dutty = 5;

// Elementos para comunicación I2C con el Sensor RGB

// Elementos para comunicación I2C del sensor
uint8_t arraySaveData[8] = {0};
float dataRGB[3] = {0};
I2C_Handler_t handlerRGB_Sensor = {0};
GPIO_Handler_t handlerRGB_SCL = {0};
GPIO_Handler_t handlerRGB_SDA = {0};
uint8_t i2cBuffer = 0;

#define RGB_SLAVE_ADDRESS	0b0101001	// 0x29
#define ENABLE_REGISTER		0			// 0x00
#define TIMING_REGISTER		1			// 0x01
#define WAIT_TIME_REGISTER	3			// 0x03
#define CONTROL_REGISTER	15			// 0x0F
#define ID_REGISTER			18			// 0x12
#define CLEAR_DATA_LOW		20			// 0x14
#define CLEAR_DATA_HIGH		21			// 0x15
#define RED_DATA_LOW		22			// 0x16
#define RED_DATA_HIGH		23			// 0x17
#define GREEN_DATA_LOW		24			// 0x18
#define GREEN_DATA_HIGH		25			// 0x19
#define BLUE_DATA_LOW		26			// 0x1A
#define BLUE_DATA_HIGH		27			// 0x1B

/* Bit de comandos, es esencial para este sensor, ya que con este
 * se indica que se va accceder a un registro del sensor, y el tipo
 * de transacción que se va a reealizar */
#define COMMAND_BIT		0x80
#define COMMAND_BIT_AUTOINCREMENT	0b10100000


/* Definición de prototipos de funciones para el main */
void initSystem(void); 			// Función que inicializa los periféricos básicos
void saveData(void);			// Función encargada de guardar los datos RGB
void initRgbSensor(void);		// Función encargada de inicializar el sensor RGB con la configuración deseada
void upDirection(void);			// Función encargada de configurar dirección de los motores para mover CNC arriba
void downDirection(void);		// Función encargada de configurar dirección de los motores para mover CNC abajo
void rightDirection(void);		// Función encargada de configurar dirección de los motores para mover CNC derecha
void leftDirection(void);		// Función encargada de configurar dirección de los motores para mover CNC izquierda


/** Función principal del programa
 * ¡Esta función es el corazón del programa! */
int main(void){

	// Inicializamos todos los elementos del sistema
	initSystem();
	sprintf(bufferData, "System initialized \n");
	writeMsg(&usartComm, bufferData);

    /* Loop forever */
	while(1){


		if(usartData != '\0'){
			writeChar(&usartComm, usartData);
			if(usartData == 'w'){
				sprintf(bufferData, "WHO_AM_I? (r)\n"); // La "(r)" en el texto es para indicar que estamos leyendo
				writeMsg(&usartComm, bufferData);

				i2cBuffer = i2c_readSingleRegister(&handlerRGB_Sensor, (COMMAND_BIT | ID_REGISTER));
				sprintf(bufferData, "dataRead = 0x%x \n", (unsigned int) i2cBuffer);
				writeMsg(&usartComm, bufferData);
				usartData = '\0';
			}
			else if(usartData == 's'){

				// Espero toma de datos
				saveData();
				usartData = '\0';
			}

			else if(usartData == 'r'){
				sprintf(bufferData, "Red data (r) \n");
				writeMsg(&usartComm, bufferData);

				uint8_t Red_low = i2c_readSingleRegister(&handlerRGB_Sensor, (COMMAND_BIT | RED_DATA_LOW));
				uint8_t Red_high = i2c_readSingleRegister(&handlerRGB_Sensor, (COMMAND_BIT | RED_DATA_HIGH));
				uint16_t Red = ((uint16_t)Red_high) << 8 | (Red_low & 0xFF);
				sprintf(bufferData, "Red data = %u \n", Red);
				writeMsg(&usartComm, bufferData);
				usartData = '\0';
			}

			else if(usartData == 'g'){
				sprintf(bufferData, "Green data (r) \n");
				writeMsg(&usartComm, bufferData);

				uint8_t Green_low = i2c_readSingleRegister(&handlerRGB_Sensor, (COMMAND_BIT | GREEN_DATA_LOW));
				uint8_t Green_high = i2c_readSingleRegister(&handlerRGB_Sensor, (COMMAND_BIT | GREEN_DATA_HIGH));
				uint16_t Green = Green_high << 8 | (Green_low & 0xFF);
				sprintf(bufferData, "Green data = %u \n", Green);
				writeMsg(&usartComm, bufferData);
				usartData = '\0';
			}

			else if(usartData == 'b'){
				sprintf(bufferData, "Blue data (r) \n");
				writeMsg(&usartComm, bufferData);

				uint8_t Blue_low = i2c_readSingleRegister(&handlerRGB_Sensor, (COMMAND_BIT | BLUE_DATA_LOW));
				uint8_t Blue_high = i2c_readSingleRegister(&handlerRGB_Sensor, (COMMAND_BIT | BLUE_DATA_HIGH));
				uint16_t Blue = Blue_high << 8 | (Blue_low & 0xFF);
				sprintf(bufferData, "Blue data = %u \n", Blue);
				writeMsg(&usartComm, bufferData);
				usartData = '\0';
			}

			else if(usartData == 'c'){
				sprintf(bufferData, "Clear data (r) \n");
				writeMsg(&usartComm, bufferData);

				uint8_t Clear_low = i2c_readSingleRegister(&handlerRGB_Sensor, (COMMAND_BIT | CLEAR_DATA_LOW));
				uint8_t Clear_high = i2c_readSingleRegister(&handlerRGB_Sensor, (COMMAND_BIT | CLEAR_DATA_HIGH));
				uint16_t Clear = Clear_high << 8 | (Clear_low & 0xFF);
				sprintf(bufferData, "Clear data = %u \n", Clear);
				writeMsg(&usartComm, bufferData);
				usartData = '\0';
			}


			/* Down del Dutty Cycle */
			else if(usartData == 'I'){
				enableOutput(&handlerSignalStepMotor1);
				startPwmSignal(&handlerSignalStepMotor1);
				enableOutput(&handlerSignalStepMotor2);
				startPwmSignal(&handlerSignalStepMotor2);
				usartData = '\0';
			}

			/* Up del Dutty Cycle */
			else if(usartData == 'P'){
				stopPwmSignal(&handlerSignalStepMotor1);
				stopPwmSignal(&handlerSignalStepMotor1);
				disableOutput(&handlerSignalStepMotor2);
				disableOutput(&handlerSignalStepMotor2);
				usartData = '\0';

			}

			// Dirección arriba
			else if(usartData == 'U'){
				upDirection();
				usartData = '\0';
			}
			// Dirección abajo
			else if(usartData == 'D'){
				downDirection();
				usartData = '\0';
			}
			// Dirección derecha
			else if(usartData == 'R'){
				rightDirection();
				usartData = '\0';
			}
			// Dirección izquierda
			else if(usartData == 'L'){
				leftDirection();
				usartData = '\0';
			}

			else if(usartData == 'S'){
				period = period+4;
				dutty = period/2;
				updatePeriod(&handlerSignalStepMotor1, period);
				updateDuttyCycle(&handlerSignalStepMotor1, dutty);
				updatePeriod(&handlerSignalStepMotor2, period);
				updateDuttyCycle(&handlerSignalStepMotor2, dutty);
				usartData = '\0';

			}
			else if(usartData == 'M'){
				period = period-4;
				dutty = period/2;
				updatePeriod(&handlerSignalStepMotor1, period);
				updateDuttyCycle(&handlerSignalStepMotor1, dutty);
				updatePeriod(&handlerSignalStepMotor2, period);
				updateDuttyCycle(&handlerSignalStepMotor2, dutty);
				usartData = '\0';

			}
			else if(usartData == 'H'){
				updatePeriod(&handlerSignalStepMotor1, 1000);
				updateDuttyCycle(&handlerSignalStepMotor1, 500);
				updatePeriod(&handlerSignalStepMotor2, 1000);
				updateDuttyCycle(&handlerSignalStepMotor2, 500);
				usartData = '\0';
			}

			else{
				usartData = '\0';
			}

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


	/*					Configuración para el Step Motor 1					*/

	// Pin de PWM
	handlerPinPwmStepMotor1.pGPIOx								= GPIOC;
	handlerPinPwmStepMotor1.GPIO_PinConfig.GPIO_PinNumber		= PIN_8;
	handlerPinPwmStepMotor1.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_ALTFN;
	handlerPinPwmStepMotor1.GPIO_PinConfig.GPIO_PinOPType		= GPIO_OTYPE_PUSHPULL;
	handlerPinPwmStepMotor1.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
	handlerPinPwmStepMotor1.GPIO_PinConfig.GPIO_PinSpeed		= GPIO_OSPEED_FAST;
	handlerPinPwmStepMotor1.GPIO_PinConfig.GPIO_PinAltFunMode	= AF2;
	GPIO_Config(&handlerPinPwmStepMotor1);

	// Pin de dirección
	handlerPinDirStepMotor1.pGPIOx								= GPIOC;
	handlerPinDirStepMotor1.GPIO_PinConfig.GPIO_PinNumber		= PIN_6;
	handlerPinDirStepMotor1.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_OUT;
	handlerPinDirStepMotor1.GPIO_PinConfig.GPIO_PinSpeed		= GPIO_OSPEED_FAST;
	handlerPinDirStepMotor1.GPIO_PinConfig.GPIO_PinOPType		= GPIO_OTYPE_PUSHPULL;
	GPIO_Config(&handlerPinDirStepMotor1);
	GPIO_WritePin(&handlerPinDirStepMotor1, RESET);

	/* Configuración del Timer para que genera la señal PWM */
	handlerSignalStepMotor1.ptrTIMx								= TIM3;
	handlerSignalStepMotor1.PWMx_Config.PWMx_Channel			= PWM_CHANNEL_3;
	handlerSignalStepMotor1.PWMx_Config.PWMx_DuttyCicle			= dutty;
	handlerSignalStepMotor1.PWMx_Config.PWMx_Period				= period;
	handlerSignalStepMotor1.PWMx_Config.PWMx_Prescaler			= BTIMER_SPEED_100us;
	pwm_Config(&handlerSignalStepMotor1);


	/*					Configuración para el Step Motor 2					*/

	/* Configuración del Pin PWM */
	handlerPinPwmStepMotor2.pGPIOx								= GPIOC;
	handlerPinPwmStepMotor2.GPIO_PinConfig.GPIO_PinNumber			= PIN_7;
	handlerPinPwmStepMotor2.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_ALTFN;
	handlerPinPwmStepMotor2.GPIO_PinConfig.GPIO_PinOPType			= GPIO_OTYPE_PUSHPULL;
	handlerPinPwmStepMotor2.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
	handlerPinPwmStepMotor2.GPIO_PinConfig.GPIO_PinSpeed			= GPIO_OSPEED_FAST;
	handlerPinPwmStepMotor2.GPIO_PinConfig.GPIO_PinAltFunMode		= AF2;
	GPIO_Config(&handlerPinPwmStepMotor2);

	// Pin de dirección
	handlerPinDirStepMotor2.pGPIOx								= GPIOC;
	handlerPinDirStepMotor2.GPIO_PinConfig.GPIO_PinNumber		= PIN_4;
	handlerPinDirStepMotor2.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_OUT;
	handlerPinDirStepMotor2.GPIO_PinConfig.GPIO_PinSpeed			= GPIO_OSPEED_FAST;
	handlerPinDirStepMotor2.GPIO_PinConfig.GPIO_PinOPType		= GPIO_OTYPE_PUSHPULL;
	GPIO_Config(&handlerPinDirStepMotor2);
	GPIO_WritePin(&handlerPinDirStepMotor2, RESET);

	/* Configurando el Timer para que genera la señal PWM */
	handlerSignalStepMotor2.ptrTIMx						= TIM3;
	handlerSignalStepMotor2.PWMx_Config.PWMx_Channel		= PWM_CHANNEL_2;
	handlerSignalStepMotor2.PWMx_Config.PWMx_DuttyCicle	= dutty;
	handlerSignalStepMotor2.PWMx_Config.PWMx_Period		= period;
	handlerSignalStepMotor2.PWMx_Config.PWMx_Prescaler	= BTIMER_SPEED_100us;
	pwm_Config(&handlerSignalStepMotor2);


	/*						I2C Sensor RGB						*/

	/* Configuración del pin SCL del I2C1 */
	handlerRGB_SCL.pGPIOx								= GPIOB;
	handlerRGB_SCL.GPIO_PinConfig.GPIO_PinNumber		= PIN_8;
	handlerRGB_SCL.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_ALTFN;
	handlerRGB_SCL.GPIO_PinConfig.GPIO_PinOPType		= GPIO_OTYPE_OPENDRAIN;
	handlerRGB_SCL.GPIO_PinConfig.GPIO_PinSpeed			= GPIO_OSPEED_HIGH;
	handlerRGB_SCL.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
	handlerRGB_SCL.GPIO_PinConfig.GPIO_PinAltFunMode 	= AF4;
	GPIO_Config(&handlerRGB_SCL);

	/* Configuración del pin SDA del I2C1 */
	handlerRGB_SDA.pGPIOx								= GPIOB;
	handlerRGB_SDA.GPIO_PinConfig.GPIO_PinNumber		= PIN_9;
	handlerRGB_SDA.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_ALTFN;
	handlerRGB_SDA.GPIO_PinConfig.GPIO_PinOPType		= GPIO_OTYPE_OPENDRAIN;
	handlerRGB_SDA.GPIO_PinConfig.GPIO_PinSpeed			= GPIO_OSPEED_HIGH;
	handlerRGB_SDA.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
	handlerRGB_SDA.GPIO_PinConfig.GPIO_PinAltFunMode 	= AF4;
	GPIO_Config(&handlerRGB_SDA);

	/* Configuración para el I2C1 */
	handlerRGB_Sensor.ptrI2Cx		= I2C1;
	handlerRGB_Sensor.modeI2C		= I2C_MODE_FM;
	handlerRGB_Sensor.slaveAddress	= RGB_SLAVE_ADDRESS;
	i2c_config(&handlerRGB_Sensor);

	/* Iniciamos el sensor */
	initRgbSensor();
}

/** Inicialización y configuración del sensor */
void initRgbSensor(void){

	/* 1. Configuramos el tiempo de integración ADC a X ms */
	i2c_writeSingleRegister(&handlerRGB_Sensor, (COMMAND_BIT | TIMING_REGISTER), 0xC4);

	/* 2. Configuramos la ganancia del sensor x60 */
	i2c_writeSingleRegister(&handlerRGB_Sensor, (COMMAND_BIT | CONTROL_REGISTER), 0x03);

	/* 3. Bit 0 PON del Enable Regisiter en 1 para activar el sensor */
	i2c_writeSingleRegister(&handlerRGB_Sensor, (COMMAND_BIT | ENABLE_REGISTER), 0b01);
	delay_ms(3);

	/* 4. Bit 1 AEN del Enable Register en 1 para activar los ADC del RGBC */
	i2c_writeSingleRegister(&handlerRGB_Sensor, (COMMAND_BIT | ENABLE_REGISTER), 0b11);
	delay_ms(145);

}

/** Guardar datos sensor RGB */
void saveData(void){
	sprintf(bufferData, "Multiple read data (r) \n");
	writeMsg(&usartComm, bufferData);

	i2c_readMultipleRegisters(&handlerRGB_Sensor, COMMAND_BIT_AUTOINCREMENT | CLEAR_DATA_LOW, 8, arraySaveData);

	uint16_t Clear = ((uint16_t)arraySaveData[1] << 8) | ((uint16_t)arraySaveData[0] & 0xFF);
	uint16_t Red = ((uint16_t)arraySaveData[3] << 8) | ((uint16_t)arraySaveData[2] & 0xFF);
	uint16_t Green = ((uint16_t)arraySaveData[5] << 8) | ((uint16_t)arraySaveData[4]& 0xFF);
	uint16_t Blue = ((uint16_t)arraySaveData[7] << 8) | ((uint16_t)arraySaveData[6] & 0xFF);


	if(Clear == 0){
		Red = 0;
		Green = 0;
		Blue = 0;
		sprintf(bufferData, "RGB data are: 0, 0, 0\n");
		writeMsg(&usartComm, bufferData);
	}
	else{
		dataRGB[0] = (((float)Red/Clear)*255.0);
		dataRGB[1] = (((float)Green/Clear)*255.0)*1.1;
		dataRGB[2] = (((float)Blue/Clear)*255.0)*1.5;
		sprintf(bufferData, "RGB data are: %.6f, %.6f, %.6f \n", dataRGB[0], dataRGB[1], dataRGB[2]);
		writeMsg(&usartComm, bufferData);
	}

	delay_ms(145);

}

/** Configuración arriba para la CNC */
void upDirection(void){
	GPIO_WritePin(&handlerPinDirStepMotor1, RESET);
	GPIO_WritePin(&handlerPinDirStepMotor2, SET);
}

/** Configuración abajo para la CNC */
void downDirection(void){
	GPIO_WritePin(&handlerPinDirStepMotor1, SET);
	GPIO_WritePin(&handlerPinDirStepMotor2, RESET);
}

/** Configuración derecha para la CNC */
void rightDirection(void){
	GPIO_WritePin(&handlerPinDirStepMotor1, SET);
	GPIO_WritePin(&handlerPinDirStepMotor2, SET);
}

/** Configuración izquierda para la CNC */
void leftDirection(void){
	GPIO_WritePin(&handlerPinDirStepMotor1, RESET);
	GPIO_WritePin(&handlerPinDirStepMotor2, RESET);
}

/** Interrupción del timer blinky LED*/
void BasicTimer2_Callback(void){
	GPIOxTooglePin(&handlerBlinkyPin); //Cambio el estado del LED PA5
}


/** Interrupción del USART2 */
void usart2Rx_Callback(void){
	usartData = getRxData();	// Pongo en alto la variable bandera del USART2 para el main
}


