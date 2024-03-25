/*
 * CMDxDriver.c
 *
 *  Created on: 28/02/2024
 *      Author: MillerQuintero2001
 */

#include <stdint.h>
#include "CMDxDriver.h"
#include "GPIOxDriver.h"
#include "USARTxDriver.h"

// Inicializo variables y elementos propios del driver

/* Esto para configurar el USART */
GPIO_Handler_t handlerPinTX = {0};			// Pin de transmisión de datos
GPIO_Handler_t handlerPinRX = {0};			// Pin de recepción de datos
USART_Handler_t usartComm =  {0};			// Comunicación serial

//uint8_t usartData = 0; 				// Variable en la que se guarda el dato transmitido
char bufferReception[64] = {0};		// Buffer para guardar caracteres ingresados
char cmd[16] = {0};					// Arreglo para guardar el comando ingresado y gestionarlo
uint8_t counterReception = 0;		// Contador de carácteres para la recepción
bool stringComplete = false;
unsigned int firstParameter = 0;
unsigned int secondParameter = 0;

/** Función necesaria para prepara el USART1 para comandos del robot */
void commandConfig(uint8_t USARTport){

	if(USARTport == CMD_USART1){

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
	}

	else if(USARTport == CMD_USART2){

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
		usartComm.ptrUSARTx							= USART2;
		usartComm.USART_Config.USART_baudrate 		= USART_BAUDRATE_115200;
		usartComm.USART_Config.USART_datasize		= USART_DATASIZE_8BIT;
		usartComm.USART_Config.USART_parity			= USART_PARITY_NONE;
		usartComm.USART_Config.USART_stopbits		= USART_STOPBIT_1;
		usartComm.USART_Config.USART_mode			= USART_MODE_RXTX;
		usartComm.USART_Config.USART_enableIntRX	= USART_RX_INTERRUP_ENABLE;
		usartComm.USART_Config.USART_enableIntTX	= USART_TX_INTERRUP_DISABLE;
		USART_Config(&usartComm);
	}

	else if(USARTport == CMD_USART6){
		/* Configuración de pines para el USART2 */
		handlerPinTX.pGPIOx								= GPIOA;
		handlerPinTX.GPIO_PinConfig.GPIO_PinNumber 		= PIN_11;
		handlerPinTX.GPIO_PinConfig.GPIO_PinMode		= GPIO_MODE_ALTFN;
		handlerPinTX.GPIO_PinConfig.GPIO_PinOPType		= GPIO_OTYPE_PUSHPULL;
		handlerPinTX.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
		handlerPinTX.GPIO_PinConfig.GPIO_PinSpeed		= GPIO_OSPEED_FAST;
		handlerPinTX.GPIO_PinConfig.GPIO_PinAltFunMode	= AF8;
		GPIO_Config(&handlerPinTX);

		handlerPinRX.pGPIOx								= GPIOA;
		handlerPinRX.GPIO_PinConfig.GPIO_PinNumber 		= PIN_12;
		handlerPinRX.GPIO_PinConfig.GPIO_PinMode		= GPIO_MODE_ALTFN;
		handlerPinRX.GPIO_PinConfig.GPIO_PinOPType		= GPIO_OTYPE_PUSHPULL;
		handlerPinRX.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
		handlerPinRX.GPIO_PinConfig.GPIO_PinSpeed		= GPIO_OSPEED_FAST;
		handlerPinRX.GPIO_PinConfig.GPIO_PinAltFunMode	= AF8;
		GPIO_Config(&handlerPinRX);

		/* Configuración de la comunicación serial */
		usartComm.ptrUSARTx							= USART6;
		usartComm.USART_Config.USART_baudrate 		= USART_BAUDRATE_115200;
		usartComm.USART_Config.USART_datasize		= USART_DATASIZE_8BIT;
		usartComm.USART_Config.USART_parity			= USART_PARITY_NONE;
		usartComm.USART_Config.USART_stopbits		= USART_STOPBIT_1;
		usartComm.USART_Config.USART_mode			= USART_MODE_RXTX;
		usartComm.USART_Config.USART_enableIntRX	= USART_RX_INTERRUP_ENABLE;
		usartComm.USART_Config.USART_enableIntTX	= USART_TX_INTERRUP_DISABLE;
		USART_Config(&usartComm);
	}

	else{
		__NOP();
	}

}


/** Función encargada de construir el string con el comando y ejecutarlo */
void commandBuild(uint8_t usartRxData){
	bufferReception[counterReception] = usartRxData;
	counterReception++;

	// Aqui hacemmos la instrucción que detine la recepción del comando
	if(usartRxData == '\r'){
		stringComplete = true;

		//Sustituyo el último caracter de @ por un null
		bufferReception[counterReception] = '\0';
		counterReception = 0;
	}
	else{
		__NOP();
	}

	// Para borrar lo que se haya digitado en la terminal
	if(usartRxData == '\b'){
		counterReception--;
		counterReception--;
	}
	else{
		__NOP();
	}

	// Volvemos a null para terminar
	usartRxData = '\0';


	// Sección de ejecución del comando
	if(stringComplete){

		/* El funcionamiento es de la siguiente forma: Empleamos el puntero al buffer para
		 * acceder a los elementos del string, y por medio de la función sscanf se almacena
		 * en 3 elemetos diferentes, el string del comando "cmd", y dos números enteros llamados
		 * "firstParameter" y "SecondParameter". De esta froma, podemos introducir información
		 * al micro desde el puerto serial */
		sscanf(bufferReception, "%s %u %u", cmd, &firstParameter, &secondParameter);

		/* Usamos la funcion strcmp, string compare, que me retorna un 0 si los 2 strings son iguales */

		// "help" este primer comando imprime una lista con los otros comandos que tiene el equipo
		if(strcmp(cmd, "Help") == 0){
			writeMsg(&usartComm, "\nHelp Menu CMDs:\n");
			writeMsg(&usartComm, "0) Help				-- Print this menu \n");
			writeMsg(&usartComm, "1) Commandx1			-- \n");
			writeMsg(&usartComm, "2) Commandx2			-- \n");
			writeMsg(&usartComm, "3) Commandx3			-- \n");
			writeMsg(&usartComm, "4) Commandx4			-- \n");
			writeMsg(&usartComm, "5) Commandx5			-- \n");
			writeMsg(&usartComm, "6) Commandx6			-- \n");
		}

		// "Commandx1"
		else if(strcmp(cmd, "Commandx1") == 0){
			writeMsg(&usartComm, "\nCMD: Commandx1 \n");
			commandx1();
		}

		// "Commandx2"
		else if(strcmp(cmd, "Commandx2") == 0){
			writeMsg(&usartComm, "\nCMD: Commandx2 \n");
			commandx2();
		}

		// "Commandx3"
		else if(strcmp(cmd, "Commandx3") == 0){
			writeMsg(&usartComm, "\nCMD: Commandx3 \n");
			commandx3();
		}

		// "Commandx4"
		else if(strcmp(cmd, "Commandx4") == 0){
			writeMsg(&usartComm, "\nCMD: Commandx4 \n");
			commandx4();
		}

		// "Commandx5"
		else if(strcmp(cmd, "Commandx5") == 0){
			writeMsg(&usartComm, "\nCMD: Commandx5 \n");
			commandx5();
		}

		// "Commandx6"
		else if(strcmp(cmd, "Commandx6") == 0){
			writeMsg(&usartComm, "\nCMD: Commandx6 \n");
			commandx6();
		}

		// En cualquier otro caso, indicamos que el comando es incorrecto
		else{
			writeMsg(&usartComm, "\nWrong command \n");
		}
		stringComplete = 0;
	}

	else{
		__NOP();
	}
}



/** Funciones command weak, que pueden ser sobre-escritas y modificadas en el main*/
__attribute__ ((weak)) void commandx1(void){
	__NOP();
}
__attribute__ ((weak)) void commandx2(void){
	__NOP();
}
__attribute__ ((weak)) void commandx3(void){
	__NOP();
}
__attribute__ ((weak)) void commandx4(void){
	__NOP();
}
__attribute__ ((weak)) void commandx5(void){
	__NOP();
}
__attribute__ ((weak)) void commandx6(void){
	__NOP();
}
