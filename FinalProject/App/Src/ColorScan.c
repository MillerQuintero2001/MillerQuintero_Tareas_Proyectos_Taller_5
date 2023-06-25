/**
 ******************************************************************************
 * @file           : ColorScan.c
 * @author         : Miller Quintero - miquinterog@unal.edu.co
 * @brief          : Solución del proyecto final Taller V
 ******************************************************************************
 * Archivo con el código encargado de ejecutar las acciones del proyecto como
 * elección de punto de origen y posterior escaneo con sensor RGB en el área
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

// Definiciones a emplear

/* Elementos para el Blinky LED */
GPIO_Handler_t handlerBlinkyPin = 			{0}; // LED de estado del Pin A5
BasicTimer_Handler_t handlerBlinkyTimer = 	{0}; // Timer del LED de estado

/* Elementos para hacer la comunicación serial */
GPIO_Handler_t handlerPinTX = {0};			// Pin de transmisión de datos
GPIO_Handler_t handlerPinRX = {0};			// Pin de recepción de datos
USART_Handler_t usartComm =  {0};			// Comunicación serial
uint8_t usartData = 0; 						// Variable en la que se guarda el dato transmitido
char bufferData[128] = {0}; 				// Buffer de datos como un arreglo de caracteres
char bufferReception[32] = {0};				// Buffer para el comando recibido
char cmd[16] = {0};
uint16_t counterReception = 0;				// Contador de carácteres para la recepción
bool stringComplete = false;				// Booleano que indica que se ha completado el comando
unsigned int firstParameter = 0;			// Parámetro en los comandos

/* Elementos para el PWM */
GPIO_Handler_t handlerPinPwmStepMotor1 = {0};
GPIO_Handler_t handlerPinDirStepMotor1 = {0};
PWM_Handler_t handlerSignalStepMotor1 = {0};
GPIO_Handler_t handlerPinPwmStepMotor2 = {0};
GPIO_Handler_t handlerPinDirStepMotor2 = {0};
PWM_Handler_t handlerSignalStepMotor2 = {0};

uint16_t period = 100;
uint16_t dutty = 50;

/* Elementos para comunicación I2C con el Sensor RGB */

// Creamos la estructura que nos ayudará a tener la matriz con la tripleta de datos RGB
typedef struct {
	uint8_t red;
	uint8_t green;
	uint8_t blue;
}RGB_Triple_t;

uint16_t datosFila = 0;
uint16_t datosColumna = 0;

RGB_Triple_t matrixRGBdata[datosFila][datosColumna] = {0};

uint8_t arraySaveData[8] = {0};

I2C_Handler_t handlerRGB_Sensor = {0};
GPIO_Handler_t handlerRGB_SCL = {0};
GPIO_Handler_t handlerRGB_SDA = {0};
uint8_t i2cBuffer = 0;

// Registros del sensor RGB
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


// Variables bandera de los modos de operación
uint8_t flagManualMove = 0;
uint8_t flagResolution = 0;
uint8_t flagScan = 0;
uint8_t flagReadyPrint = 0;
uint8_t flagPrint = 0;


/* Definición de prototipos de funciones para el main */
void initSystem(void); 			// Función que inicializa los periféricos básicos
void saveData(void);			// Función encargada de guardar los datos RGB
void initRgbSensor(void);		// Función encargada de inicializar el sensor RGB con la configuración deseada
void commandBuild(void);							// Función que construye el string del comando USART
void commandUSART(char* arrayCmd);					// Función encargada de gestionar el comando ingresado por USART
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

		// Hacemos un "eco" con el valor que nos llega por el serial, y que no estemos en modo de movimiento manual de la CNC
		if((usartData != '\0')&&(flagManualMove == 0)){
			writeChar(&usartComm, usartData);
			commandBuild();
		}


		// Verificamos si se ha dado por completado del comando
		if(stringComplete){
			commandUSART(bufferReception);
			stringComplete = false;
		}


		if(flagManualMove){

			if(usartData == '\r'){
				flagManualMove = 0;
				writeMsg(&usartComm, " Current point has been select has origin, ready to scan");
			}

			else if(usartData == 'W'){
				writeChar(&usartComm, usartData);
				upDirection();

				enableOutput(&handlerSignalStepMotor1);
				enableOutput(&handlerSignalStepMotor2);
				startPwmSignal(&handlerSignalStepMotor1);
				startPwmSignal(&handlerSignalStepMotor2);

				delay_ms(2);

				disableOutput(&handlerSignalStepMotor1);
				disableOutput(&handlerSignalStepMotor2);
				stopPwmSignal(&handlerSignalStepMotor1);
				stopPwmSignal(&handlerSignalStepMotor2);
				usartData = '\0';
			}

			else if(usartData == 'S'){
				writeChar(&usartComm, usartData);
				downDirection();

				enableOutput(&handlerSignalStepMotor1);
				enableOutput(&handlerSignalStepMotor2);
				startPwmSignal(&handlerSignalStepMotor1);
				startPwmSignal(&handlerSignalStepMotor2);

				delay_ms(2);

				disableOutput(&handlerSignalStepMotor1);
				disableOutput(&handlerSignalStepMotor2);
				stopPwmSignal(&handlerSignalStepMotor1);
				stopPwmSignal(&handlerSignalStepMotor2);
				usartData = '\0';

			}

			else if(usartData == 'A'){
				writeChar(&usartComm, usartData);
				leftDirection();

				enableOutput(&handlerSignalStepMotor1);
				enableOutput(&handlerSignalStepMotor2);
				startPwmSignal(&handlerSignalStepMotor1);
				startPwmSignal(&handlerSignalStepMotor2);

				delay_ms(2);

				disableOutput(&handlerSignalStepMotor1);
				disableOutput(&handlerSignalStepMotor2);
				stopPwmSignal(&handlerSignalStepMotor1);
				stopPwmSignal(&handlerSignalStepMotor2);
				usartData = '\0';

			}

			else if(usartData == 'D'){
				writeChar(&usartComm, usartData);
				rightDirection();

				enableOutput(&handlerSignalStepMotor1);
				enableOutput(&handlerSignalStepMotor2);
				startPwmSignal(&handlerSignalStepMotor1);
				startPwmSignal(&handlerSignalStepMotor2);

				delay_ms(2);

				disableOutput(&handlerSignalStepMotor1);
				disableOutput(&handlerSignalStepMotor2);
				stopPwmSignal(&handlerSignalStepMotor1);
				stopPwmSignal(&handlerSignalStepMotor2);
				usartData = '\0';

			}

			else{
				usartData = '\0';
			}
		}

		if(flagScan){
		    for (int i = 0; i < datosFila; i++) {
		        for (int j = 0; j < datosColumna; j++) {
		            sprintf(bufferData,"[%u, %u, %u]", matrixRGBdata[i][j].red, matrixRGBdata[i][j].green, matrixRGBdata[i][j].blue);
		            writeMsg(&usartComm, bufferData);
		        }
		        writeChar(&usartComm, '\n');
		    }
		    flagPrint = 0;
		}


		if(flagPrint){
		    for (int i = 0; i < datosFila; i++) {
		        for (int j = 0; j < datosColumna; j++) {
		            sprintf(bufferData,"[%u, %u, %u]", matrixRGBdata[i][j].red, matrixRGBdata[i][j].green, matrixRGBdata[i][j].blue);
		            writeMsg(&usartComm, bufferData);
		        }
		        writeChar(&usartComm, '\n');
		    }
		    flagPrint = 0;
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
	handlerPinPwmStepMotor1.pGPIOx									= GPIOC;
	handlerPinPwmStepMotor1.GPIO_PinConfig.GPIO_PinNumber			= PIN_8;
	handlerPinPwmStepMotor1.GPIO_PinConfig.GPIO_PinMode				= GPIO_MODE_ALTFN;
	handlerPinPwmStepMotor1.GPIO_PinConfig.GPIO_PinOPType			= GPIO_OTYPE_PUSHPULL;
	handlerPinPwmStepMotor1.GPIO_PinConfig.GPIO_PinPuPdControl		= GPIO_PUPDR_NOTHING;
	handlerPinPwmStepMotor1.GPIO_PinConfig.GPIO_PinSpeed			= GPIO_OSPEED_FAST;
	handlerPinPwmStepMotor1.GPIO_PinConfig.GPIO_PinAltFunMode		= AF2;
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
	handlerSignalStepMotor1.PWMx_Config.PWMx_Prescaler			= BTIMER_SPEED_10us;
	pwm_Config(&handlerSignalStepMotor1);


	/*					Configuración para el Step Motor 2					*/

	/* Configuración del Pin PWM */
	handlerPinPwmStepMotor2.pGPIOx									= GPIOC;
	handlerPinPwmStepMotor2.GPIO_PinConfig.GPIO_PinNumber			= PIN_7;
	handlerPinPwmStepMotor2.GPIO_PinConfig.GPIO_PinMode				= GPIO_MODE_ALTFN;
	handlerPinPwmStepMotor2.GPIO_PinConfig.GPIO_PinOPType			= GPIO_OTYPE_PUSHPULL;
	handlerPinPwmStepMotor2.GPIO_PinConfig.GPIO_PinPuPdControl		= GPIO_PUPDR_NOTHING;
	handlerPinPwmStepMotor2.GPIO_PinConfig.GPIO_PinSpeed			= GPIO_OSPEED_FAST;
	handlerPinPwmStepMotor2.GPIO_PinConfig.GPIO_PinAltFunMode		= AF2;
	GPIO_Config(&handlerPinPwmStepMotor2);

	// Pin de dirección
	handlerPinDirStepMotor2.pGPIOx								= GPIOC;
	handlerPinDirStepMotor2.GPIO_PinConfig.GPIO_PinNumber		= PIN_4;
	handlerPinDirStepMotor2.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_OUT;
	handlerPinDirStepMotor2.GPIO_PinConfig.GPIO_PinSpeed		= GPIO_OSPEED_FAST;
	handlerPinDirStepMotor2.GPIO_PinConfig.GPIO_PinOPType		= GPIO_OTYPE_PUSHPULL;
	GPIO_Config(&handlerPinDirStepMotor2);
	GPIO_WritePin(&handlerPinDirStepMotor2, RESET);

	/* Configurando el Timer para que genera la señal PWM */
	handlerSignalStepMotor2.ptrTIMx								= TIM3;
	handlerSignalStepMotor2.PWMx_Config.PWMx_Channel			= PWM_CHANNEL_2;
	handlerSignalStepMotor2.PWMx_Config.PWMx_DuttyCicle			= dutty;
	handlerSignalStepMotor2.PWMx_Config.PWMx_Period				= period;
	handlerSignalStepMotor2.PWMx_Config.PWMx_Prescaler			= BTIMER_SPEED_10us;
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

	i2c_readMultipleRegisters(&handlerRGB_Sensor, COMMAND_BIT_AUTOINCREMENT | CLEAR_DATA_LOW, 8, arraySaveData);

	uint16_t Clear = ((uint16_t)arraySaveData[1] << 8) | ((uint16_t)arraySaveData[0] & 0xFF);
	uint16_t Red = ((uint16_t)arraySaveData[3] << 8) | ((uint16_t)arraySaveData[2] & 0xFF);
	uint16_t Green = ((uint16_t)arraySaveData[5] << 8) | ((uint16_t)arraySaveData[4]& 0xFF);
	uint16_t Blue = ((uint16_t)arraySaveData[7] << 8) | ((uint16_t)arraySaveData[6] & 0xFF);

	if(Clear == 0){
		dataRGB[0] = 0;
		dataRGB[1] = 0;
		dataRGB[2] = 0;

	}
	else{
		float CalibrateRed		= 	(((float)Red/Clear)*255.0);
		float CalibrateGreen	= 	(((float)Green/Clear)*255.0)*1.2;
		float CalibrateBlue		= 	(((float)Blue/Clear)*255.0)*1.25;
		if(CalibrateRed > 255.0){
			CalibrateRed = 255.0;
		}
		if(CalibrateGreen > 255.0){
			CalibrateGreen = 255.0;
		}
		if(CalibrateBlue > 255.0){
			CalibrateBlue = 255.0;
		}

		dataRGB[0] = (uint8_t)CalibrateRed;
		dataRGB[1] = (uint8_t)CalibrateGreen;
		dataRGB[2] = (uint8_t)CalibrateBlue;
	}
	delay_ms(145);

}

/** Función encargada de construir el string con el comando */
void commandBuild(void){
	bufferReception[counterReception] = usartData;
	counterReception++;

	// Aqui hacemmos la instrucción que detine la recepción del comando
	if(usartData == '\r'){
		stringComplete = true;

		//Sustituyo el último caracter de @ por un null
		bufferReception[counterReception] = '\0';
		counterReception = 0;
	}
	if(usartData == '\b'){
		counterReception--;
		counterReception--;
	}

	// Volvemos a null para terminar
	usartData = '\0';
}


/** Función encargada de gestionar e identificar el comando ingresado, recibe el puntero al string */
void commandUSART(char* ptrBufferReception){

	sscanf(ptrBufferReception, "%s %u %s", cmd, &firstParameter, userMsg);

	/* Usamos la funcion strcmp, string compare, que me retorna un 0 si los 2 strings son iguales */

	// "help" este primer comando imprime una lista con los otros comandoS que tiene el equipo
	if(strcmp(cmd, "help") == 0){
		writeMsg(&usartComm, "\nHelp Menu CMDs:\n");
		writeMsg(&usartComm, "1) help				-- Print this menu \n");
		writeMsg(&usartComm, "2) changeOrigin		-- Enable control keyboard to select origin of scan \n");
		writeMsg(&usartComm, "4) setResolution #R	-- Select one of the following possible resolutions: \n"
				" 1 = XX.X mm, 2 = XX.X mm \n");
		writeMsg(&usartComm, "5) startScan			-- Start to scan area and collect RGB data \n");
		writeMsg(&usartComm, "5) printData			-- Print all data on serial terminal \n");
	}

	// Activamos el modo de control manual con una sola tecla
	else if(strcmp(cmd, "changeOrigin") == 0){
		writeMsg(&usartComm, "\nCMD: changeOrigin \n");
		writeMsg(&usartComm, "Use W is Up, S is Down, A is Left, D is Right and press Enter to set origin \n");
		flagManualMove = 1;
	}

	else if(strcmp(cmd, "setResolution") == 0){
		writeMsg(&usartComm, "\nCMD: setResolution \n");
		if((firstParameter == 1)||(firstParameter == 2)){
			writeMsg(&usartComm, "Resolution set successfully \n");
			flagResolution = 1;
		}
		else{
			writeMsg(&usartComm, "Wrong resolution, try again \n");
		}
	}

	else if(strcmp(cmd, "startScan") == 0){
		writeMsg(&usartComm, "\nCMD: startScan \n");
		if((flagManualMove == 0)&&(flagResolution == 1)){
			writeMsg(&usartComm, "Error, set an origin and resolution first \n");
		}
		else{
			writeMsg(&usartComm, "Scan started \n");
			flagScan = 1;
		}
	}

	else if(strcmp(cmd, "printData") == 0){
		writeMsg(&usartComm, "\nCMD: printData \n");
		if((flagReadyPrint == 1)&&(flagScan == 0)){
			flagReadyPrint = 0;
			flagPrint = 1;
		}
		else{
			writeChar(&usartComm, "Error, there is no data to print, please, first set the source and resolution, then launch the scan \n");
		}
	}


	// En cualquier otro caso, indicamos que el comando es incorrecto
	else{
		writeMsg(&usartComm, "\nWrong command \n");
	}

	// Limpiamos los parámetros para evitar configuraciones accidentales
	firstParameter = 0;

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


