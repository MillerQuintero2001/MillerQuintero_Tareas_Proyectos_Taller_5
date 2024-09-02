/*
 * CMDxDriver.c
 *
 *  Created on: 28/02/2024
 *      Author: MillerQuintero2001
 */

#include "CMDxDriver.h"

/* Inicializo variables y elmentos del driver */
GPIO_Handler_t handlerPinTX = {0};	// Pin de transmisión de datos
GPIO_Handler_t handlerPinRX = {0};	// Pin de recepción de datos
USART_Handler_t usartCmd =  {0};	// Comunicación serial
uint8_t usartData = 0; 				// Variable en la que se guarda el dato transmitido
char userMsg[] = "Menu de comandos:\nIngresando por consola el comando 'Help' se puede ver la lista de todos los comandos.\n";
char bufferReception[64] = {0};		// Buffer para guardar caracteres ingresados
char cmd[32] = {0};					// Arreglo para guardar el comando ingresado y gestionarlo
uint8_t counterReception = 0;		// Contador de carácteres para la recepción
bool stringComplete = false;
float firstParameter = 0;
float secondParameter = 0;
float thirdParameter = 0;

/** Función necesaria para prepara el USART1 para comandos del robot */
void commandConfig(uint8_t USARTport, uint8_t baudrate){

	/* Activamos el Coprocesador Matemático - FPU */
	SCB->CPACR |= (0XF << 20);

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
		usartCmd.ptrUSARTx							= USART1;
		usartCmd.USART_Config.USART_baudrate 		= baudrate;
		usartCmd.USART_Config.USART_datasize		= USART_DATASIZE_8BIT;
		usartCmd.USART_Config.USART_parity			= USART_PARITY_NONE;
		usartCmd.USART_Config.USART_stopbits		= USART_STOPBIT_1;
		usartCmd.USART_Config.USART_mode			= USART_MODE_RXTX;
		usartCmd.USART_Config.USART_enableIntRX		= USART_RX_INTERRUP_ENABLE;
		usartCmd.USART_Config.USART_enableIntTX		= USART_TX_INTERRUP_DISABLE;
		USART_Config(&usartCmd);
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
		usartCmd.ptrUSARTx							= USART2;
		usartCmd.USART_Config.USART_baudrate 		= baudrate;
		usartCmd.USART_Config.USART_datasize		= USART_DATASIZE_8BIT;
		usartCmd.USART_Config.USART_parity			= USART_PARITY_NONE;
		usartCmd.USART_Config.USART_stopbits		= USART_STOPBIT_1;
		usartCmd.USART_Config.USART_mode			= USART_MODE_RXTX;
		usartCmd.USART_Config.USART_enableIntRX		= USART_RX_INTERRUP_ENABLE;
		usartCmd.USART_Config.USART_enableIntTX		= USART_TX_INTERRUP_DISABLE;
		USART_Config(&usartCmd);
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
		usartCmd.ptrUSARTx							= USART6;
		usartCmd.USART_Config.USART_baudrate 		= baudrate;
		usartCmd.USART_Config.USART_datasize		= USART_DATASIZE_8BIT;
		usartCmd.USART_Config.USART_parity			= USART_PARITY_NONE;
		usartCmd.USART_Config.USART_stopbits		= USART_STOPBIT_1;
		usartCmd.USART_Config.USART_mode			= USART_MODE_RXTX;
		usartCmd.USART_Config.USART_enableIntRX		= USART_RX_INTERRUP_ENABLE;
		usartCmd.USART_Config.USART_enableIntTX		= USART_TX_INTERRUP_DISABLE;
		USART_Config(&usartCmd);
	}

	else{
		__NOP();
	}

	writeMsg(&usartCmd, userMsg);

}


/** Función encargada de construir el string con el comando y ejecutarlo */
void commandBuild(bool use){

	if(usartData != '\0'){
		bufferReception[counterReception] = usartData;
		counterReception++;

		// Aqui hacemmos la instrucción que detine la recepción del comando
		if(usartData == '\r'){
			stringComplete = true;

			//Sustituyo el último caracter de \r por un null
			bufferReception[counterReception] = '\0';
			counterReception = 0;
		}
		else{
			__NOP();
		}

		// Para borrar lo que se haya digitado en la terminal
		if(usartData == '\b'){
			counterReception--;
			counterReception--;
		}
		else{
			__NOP();
		}

		// Volvemos a null para terminar
		usartData = '\0';
	}
	else{
		__NOP();
	}

	// Sección de ejecución del comando para uso por defecto (no motores)
	if((stringComplete)&&(use == USE_DEFAULT)){

		/* El funcionamiento es de la siguiente forma: Empleamos el puntero al buffer para
		 * acceder a los elementos del string, y por medio de la función sscanf se almacena
		 * en 3 elemetos diferentes, el string del comando "cmd", y dos números enteros llamados
		 * "firstParameter" y "secondParameter". De esta forma, podemos introducir información
		 * al micro desde el puerto serial */
		sscanf(bufferReception, "%s %f %f %f", cmd, &firstParameter, &secondParameter, &thirdParameter);

		/* Usamos la funcion strcmp, string compare, que me retorna un 0 si los 2 strings son iguales */

		// "help" este primer comando imprime una lista con los otros comandos que tiene el equipo
		if(strcmp(cmd, "Help") == 0){
			writeMsg(&usartCmd, "\nHelp Menu CMDs:\n");
			writeMsg(&usartCmd, "0) Help				-- Print this menu \n");
			writeMsg(&usartCmd, "1) Commandx1			-- \n");
			writeMsg(&usartCmd, "2) Commandx2			-- \n");
			writeMsg(&usartCmd, "3) Commandx3			-- \n");
			writeMsg(&usartCmd, "4) Commandx4			-- \n");
			writeMsg(&usartCmd, "5) Commandx5			-- \n");
			writeMsg(&usartCmd, "6) Commandx6			-- \n");
		}

		// "Commandx1"
		else if(strcmp(cmd, "Commandx1") == 0){
			writeMsg(&usartCmd, "\nCMD: Commandx1 \n");
			commandx1();
		}

		// "Commandx2"
		else if(strcmp(cmd, "Commandx2") == 0){
			writeMsg(&usartCmd, "\nCMD: Commandx2 \n");
			commandx2();
		}

		// "Commandx3"
		else if(strcmp(cmd, "Commandx3") == 0){
			writeMsg(&usartCmd, "\nCMD: Commandx3 \n");
			commandx3();
		}

		// "Commandx4"
		else if(strcmp(cmd, "Commandx4") == 0){
			writeMsg(&usartCmd, "\nCMD: Commandx4 \n");
			commandx4();
		}

		// "Commandx5"
		else if(strcmp(cmd, "Commandx5") == 0){
			writeMsg(&usartCmd, "\nCMD: Commandx5 \n");
			commandx5();
		}

		// "Commandx6"
		else if(strcmp(cmd, "Commandx6") == 0){
			writeMsg(&usartCmd, "\nCMD: Commandx6 \n");
			commandx6();
		}

		// "Commandx7"
		else if(strcmp(cmd, "Commandx7") == 0){
			writeMsg(&usartCmd, "\nCMD: Commandx7 \n");
			commandx7();
		}

		// En cualquier otro caso, indicamos que el comando es incorrecto
		else{
			writeMsg(&usartCmd, "\nWrong command \n");
		}
		// Limpiamos los párametros
		stringComplete = 0;
		firstParameter = 0;
		secondParameter = 0;
	}


	// Sección de ejecución del comando para uso por defecto (no motores)
	else if((stringComplete)&&(use == USE_OPPY)){

		/* El funcionamiento es de la siguiente forma: Empleamos el puntero al buffer para
		 * acceder a los elementos del string, y por medio de la función sscanf se almacena
		 * en 3 elemetos diferentes, el string del comando "cmd", y dos números enteros llamados
		 * "firstParameter" y "secondParameter". De esta forma, podemos introducir información
		 * al micro desde el puerto serial */
		sscanf(bufferReception, "%s %f %f %f", cmd, &firstParameter, &secondParameter, &thirdParameter);

		/* Usamos la funcion strcmp, string compare, que me retorna un 0 si los 2 strings son iguales */

		// "Help" este primer comando imprime una lista con los otros comandos que tiene el equipo
		if(strcmp(cmd, "Help") == 0){
			writeMsg(&usartCmd, "\nHelp Menu with Oppy Commands CMDs:\n");
			writeMsg(&usartCmd, "1) Help				-- Print this menu\n");
			writeMsg(&usartCmd, "2) SetSignals #Frequency[Hz] #%DuttyCycle \n"
					"-- Frequency[Hz] should be a positive integer between 1 and 100 \n"
					"-- %DuttyCycle should be a positive integer between 1 and 100 \n");
			writeMsg(&usartCmd, "3) SetVelocity #Velocity[mm/s] \n"
					"-- Velocity should be a positive integer between 88 and 140 \n");
			writeMsg(&usartCmd, "4) Default			-- Set polarity of signals in active high and direction in forward \n");
			writeMsg(&usartCmd, "5) Start			-- Start the movement of the Oppy \n");
			writeMsg(&usartCmd, "6) Stop			-- Stop the movement of the Oppy \n");
			writeMsg(&usartCmd, "7) Line #Distance[mm] \n"
					"-- Distance should be a positive integer between 1 and 65535 \n");
			writeMsg(&usartCmd, "8) Rotation #Direction #°Degrees \n"
					"-- Direction: ClockWise(CW) = 0; CounterClockWise(CCW) = 1 \n"
					"-- °Degrees should be a positive integer between 1 and 360 \n");
			writeMsg(&usartCmd, "9) Square #Direction #Side(mm) \n"
					"-- Direction: CW = 0; CCW = 1 \n"
					"-- Side should be in mm, a positive integer between 1 and 65535 \n");
		}

		// "SetMotorSignals" configura la frecuencia y el dutty de la PWM de los motores
		else if(strcmp(cmd, "SetSignals") == 0){
			writeMsg(&usartCmd, "\nCMD: SetSignals \n");
			if(((1 <= (uint8_t)firstParameter)&&((uint8_t)firstParameter <= 100))&&((1 <= (uint8_t)secondParameter)&&((uint8_t)secondParameter <= 100))){
				setSignals((uint8_t)firstParameter, (uint8_t)secondParameter);
				writeMsg(&usartCmd, "Configuration succesfull \n");
			}
			else{
				writeMsg(&usartCmd, "Wrong frequency or dutty, remember, only positive integers between 1 and 100 \n");
			}
		}

		// "SetVelocity" configura la velocidad de los motores
		else if(strcmp(cmd, "SetVelocity") == 0){
			writeMsg(&usartCmd, "\nCMD: SetVelocity \n");
			if((88 <= (uint8_t)firstParameter)&&((uint8_t)firstParameter <= 140)){
				setVelocity((uint8_t)firstParameter);
				getDutty((uint8_t)firstParameter);
				writeMsg(&usartCmd, "Configuration succesfull \n");
			}
			else{
				writeMsg(&usartCmd, "Wrong velocity, remember, only positive integers between 88 and 140 \n");
			}
		}

		// "DefaultMove" establece la configuración por defecto de las polaridades y las direcciones
		else if(strcmp(cmd, "Default") == 0){
			writeMsg(&usartCmd, "\nCMD: Default \n");
			defaultMove();
			writeMsg(&usartCmd, "Oppy has returned to his default movement \n");
		}

		// "StartMove" activa las señales PWM y los enable del Puente H, iniciando así el movimiento
		else if(strcmp(cmd, "Start") == 0){
			writeMsg(&usartCmd, "\nCMD: Start \n");
			startMove();
			writeMsg(&usartCmd, "Oppy is moving \n");
		}

		// "StopMove" desactiva los enable del Puente H y las señales PWM, deteniendo así el movimiento
		else if(strcmp(cmd, "Stop") == 0){
			writeMsg(&usartCmd, "\nCMD: Stop \n");
			stopMove();
			writeMsg(&usartCmd, "Oppy has been stoped \n");
		}

		// "StraightLine" inicia un recorrido en línea recta con control, según la distancia indicada
		else if(strcmp(cmd, "Line") == 0){
			writeMsg(&usartCmd, "\nCMD: Line \n");
			if((1 <= (uint16_t)firstParameter)&&((uint16_t)firstParameter <= 65535)){
				writeMsg(&usartCmd, "Oppy is doing a straight line \n");
				straightLine((uint16_t)firstParameter);
			}
			else{
				writeMsg(&usartCmd, "Wrong distance, remember, only positive integers between 1 and 65535 \n");
			}
		}

		// "Rotation" realiza una rotación en el sentido y grados indicados
		else if(strcmp(cmd, "Rotation") == 0){
			writeMsg(&usartCmd, "\nCMD: Rotation \n");
			if(((0 <= (uint8_t)firstParameter)&&((uint8_t)firstParameter <= 1))&&((1 <= (uint16_t)secondParameter)&&((uint16_t)secondParameter <= 360))){
				writeMsg(&usartCmd, "Oppy is doing a rotation \n");
				rotation((uint8_t)firstParameter, (uint16_t)secondParameter);
			}
			else{
				writeMsg(&usartCmd, "Wrong values, remember, rotation between 0 and 1 and degrees between 1 and 360 \n");
			}
		}

		// "Square" realiza un cuadrado con el Oppy
		else if(strcmp(cmd, "Square") == 0){
			writeMsg(&usartCmd, "\nCMD: Square \n");
			if(((0 <= (uint8_t)firstParameter)&&((uint8_t)firstParameter <= 1))&&((1 <= (uint16_t)secondParameter)&&((uint16_t)secondParameter <= 65535))){
				writeMsg(&usartCmd, "Oppy is doing a square \n");
				square((uint8_t)firstParameter, (uint16_t)secondParameter);
			}
			else{
				writeMsg(&usartCmd, "Wrong values, remember, rotation between 0 and 1 and side between 1 and 65535 \n");
			}
		}

		// En cualquier otro caso, indicamos que el comando es incorrecto
		else{
			writeMsg(&usartCmd, "\nWrong command \n");
		}

		// Limpiamos los párametros
		stringComplete = false;
		firstParameter = 0.0f;
		secondParameter = 0.0f;
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
__attribute__ ((weak)) void commandx7(void){
	__NOP();
}

