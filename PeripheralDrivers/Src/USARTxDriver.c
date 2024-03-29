/*
 * USARTxDriver.c
 *
 *  Created on: Apr 6, 2022
 *      Author: MillerQuintero2001
 */

#include <stm32f4xx.h>
#include <stdint.h>
#include <math.h>
#include "USARTxDriver.h"
#include "PLLDriver.h"


/**
 * Configurando el puerto Serial...
 * Recordar que siempre se debe comenzar con activar la señal de reloj
 * del periferico que se está utilizando.
 */

uint8_t auxRxData = 0;				// Variable que guarda el dato recibido
uint8_t charOrString = 0; 			// Variable que guarda el modo, 0 es carácter, 1 es string
char dataUSARTchar = '\0'; 			// Variable que guarda el carácter a enviar por interrupción
char *ptrdataUSARTstring = NULL; 	// Puntero que guarda la primera posición del string a enviar por interrupción
int pos = 0;						// Contador de posición para el arreglo de carácteres
float usartDiv = 0.0;				// Variable que guarda el valor calculo como decimal
uint16_t mantiza = 0;				// Variable que guarda el valor de la mantiza para los baudrate
uint8_t fraction = 0;				// Variable que guarda el valor de la fraction para los baudrate
uint16_t baudrate = 0;
uint64_t freqClock = 0;

void USART_Config(USART_Handler_t *ptrUsartHandler){
	/* 1. Activamos la señal de reloj que viene desde el BUS al que pertenece el periferico */
	/* Lo debemos hacer para cada uno de las pisbles opciones que tengamos (USART1, USART2, USART6) */
    /* 1.1 Configuramos el USART1 */
	if(ptrUsartHandler->ptrUSARTx == USART1){
		RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	}
    /* 1.2 Configuramos el USART2 */
	else if(ptrUsartHandler->ptrUSARTx == USART2){
		RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
	}
    /* 1.3 Configuramos el USART6 */
	else if(ptrUsartHandler->ptrUSARTx == USART6){
		RCC->APB2ENR |= RCC_APB2ENR_USART6EN;
	}else{
		__NOP();
	}

	/* 2. Configuramos el tamaño del dato, la paridad y los bit de parada */
	/* En el CR1 estan parity (PCE y PS) y tamaño del dato (M) */
	/* Mientras que en CR2 estan los stopbit (STOP)*/
	/* Configuracion del Baudrate (registro BRR) */
	/* Configuramos el modo: only TX, only RX, o RXTX */
	/* Por ultimo activamos el modulo USART cuando todo esta correctamente configurado */

	// 2.1 Comienzo por limpiar los registros, para cargar la configuración desde cero
	ptrUsartHandler->ptrUSARTx->CR1 = 0;
	ptrUsartHandler->ptrUSARTx->CR2 = 0;

	// 2.2 Configuracion del Parity:
	// Verificamos si el parity esta activado o no
    // Tenga cuidado, el parity hace parte del tamaño de los datos...
	if(ptrUsartHandler->USART_Config.USART_parity != USART_PARITY_NONE){

		//Activo el parity control
		ptrUsartHandler->ptrUSARTx->CR1 |= USART_CR1_PCE;

		// Verificamos si se ha seleccionado ODD or EVEN
		if(ptrUsartHandler->USART_Config.USART_parity == USART_PARITY_EVEN){
			// Es even, entonces cargamos la configuracion adecuada
			ptrUsartHandler->ptrUSARTx->CR1 &= ~USART_CR1_PS;
		}else{
			// Si es "else" significa que la paridad seleccionada es ODD, y cargamos esta configuracion
			ptrUsartHandler->ptrUSARTx->CR1 |= USART_CR1_PS;
		}
	}else{
		// Si llegamos aca, es porque no deseamos tener el parity-check
		ptrUsartHandler->ptrUSARTx->CR1 &= ~USART_CR1_PCE;
	}

	// 2.3 Configuramos el tamaño del dato
	// Si tenemos el parity activado serán 9 bits de tamaño
    if((ptrUsartHandler->USART_Config.USART_datasize == USART_DATASIZE_9BIT) //Si tengo 9 bits Ó si tengo 8 bits y la paridad activa
    		|| ((ptrUsartHandler->USART_Config.USART_parity != USART_PARITY_NONE)
    				&&(ptrUsartHandler->USART_Config.USART_datasize == USART_DATASIZE_8BIT))){
    	ptrUsartHandler->ptrUSARTx->CR1 |= USART_CR1_M;
    }else{ // De lo contrario serán 8 bits de tamaño
    	ptrUsartHandler->ptrUSARTx->CR1 &= ~USART_CR1_M;
    }

	// 2.4 Configuramos los stop bits (SFR USART_CR2)
	switch(ptrUsartHandler->USART_Config.USART_stopbits){
	case USART_STOPBIT_1: {
		// Debemos cargar el valor 0b00 en los dos bits de STOP
		ptrUsartHandler->ptrUSARTx->CR2 &= (0b00 << (12));
		break;
	}
	case USART_STOPBIT_0_5: {
		// Debemos cargar el valor 0b01 en los dos bits de STOP
		// Limpio
		ptrUsartHandler->ptrUSARTx->CR2 &= (0b00 << (12));
		// Cargo
		ptrUsartHandler->ptrUSARTx->CR2 |= (0b01 << (12));
		break;
	}
	case USART_STOPBIT_2: {
		// Debemos cargar el valor 0b10 en los dos bits de STOP
		ptrUsartHandler->ptrUSARTx->CR2 &= (0b00 << (12));
		// Cargo
		ptrUsartHandler->ptrUSARTx->CR2 |= (0b10 << (12));
		break;
	}
	case USART_STOPBIT_1_5: {
		// Debemos cargar el valor 0b11 en los dos bits de STOP
		ptrUsartHandler->ptrUSARTx->CR2 &= (0b00 << (12));
		// Cargo
		ptrUsartHandler->ptrUSARTx->CR2 |= (0b11 << (12));
		break;
	}
	default: {
		// En el caso por defecto seleccionamos 1 bit de parada
		ptrUsartHandler->ptrUSARTx->CR2 &= (0b00 << (12));
		break;
	}
	}

	// 2.5 Configuracion del Baudrate (SFR USART_BRR)
	// Si el PLL ha sido seleccionado como el System Clock a MHz
	if((RCC->CFGR & RCC_CFGR_SW) == RCC_CFGR_SW_PLL){
		if((ptrUsartHandler->ptrUSARTx == USART1)||(ptrUsartHandler->ptrUSARTx == USART6)){
			freqClock = getConfigPLL();
			if(ptrUsartHandler->USART_Config.USART_baudrate == USART_BAUDRATE_9600){
				usartDiv = freqClock/(16.0*9600.0);
				mantiza = (uint16_t)usartDiv;
				fraction = (uint8_t)roundf((usartDiv - (uint16_t)usartDiv)*16);
				baudrate = ((mantiza << 4)|fraction);
				ptrUsartHandler->ptrUSARTx->BRR = baudrate;
			}
			else if (ptrUsartHandler->USART_Config.USART_baudrate == USART_BAUDRATE_19200) {
				// El valor a cargar es 260.4166 -> Mantiza = 260,fraction = 0.4166
				// Mantiza = 260 = 0x104, fraction = 16 * 0.4166 = 6.66 = 7
				// Valor a cargar 0x01047
				// Configurando el Baudrate generator para una velocidad de 19200bps
				ptrUsartHandler->ptrUSARTx->BRR = 0x1484;
			}
			else if (ptrUsartHandler->USART_Config.USART_baudrate == USART_BAUDRATE_38400) {
				// El valor a cargar es 130.2083 -> Mantiza = 130,fraction = 0.2083
				// Mantiza = 130 = 0x082, fraction = 16 * 0.2083 = 3.33
				// Valor a cargar 0x0823
				// Configurando el Baudrate generator para una velocidad de 38400bps
				ptrUsartHandler->ptrUSARTx->BRR = 0x0823;
			}
			else if (ptrUsartHandler->USART_Config.USART_baudrate == USART_BAUDRATE_57600) {
				// El valor a cargar es 86.8055 -> Mantiza = 86,fraction = 0.8055
				// Mantiza = 86 = 0x56, fraction = 16 * 0.8055 = 12.88 = 13
				// Valor a cargar 0x056D
				// Configurando el Baudrate generator para una velocidad de 57600bps
				ptrUsartHandler->ptrUSARTx->BRR = 0x056D;
			}
			else if(ptrUsartHandler->USART_Config.USART_baudrate == USART_BAUDRATE_115200){
				// El valor a cargar es 54,25347 -> Mantiza = 54,fraction = 0.25347
				// Mantiza = 54 = 0x36, fraction = 16 * 0.25347 = 4.05 = 4
				// Valor a cargar 0x0364
				// Configurando el Baudrate generator para una velocidad de 57600bps
				ptrUsartHandler->ptrUSARTx->BRR = 0x0364;
			}
			else if(ptrUsartHandler->USART_Config.USART_baudrate == USART_BAUDRATE_230400){
				// El valor a cargar es 21.7013 -> Mantiza = 21,fraction = 0.7013
				// Mantiza = 21 = 0x015, fraction = 16 * 0.7013 = 11.22
				// Valor a cargar 0x015B
				// Configurando el Baudrate generator para una velocidad de 57600bps
				ptrUsartHandler->ptrUSARTx->BRR = 0x015B;
			}
			else if(ptrUsartHandler->USART_Config.USART_baudrate == USART_BAUDRATE_921600){
				// El valor a cargar es 5.4253 -> Mantiza = 5,fraction = 0.4253
				// Mantiza = 5 = 0x005, fraction = 16 * 0.4253 = 6.80 = 7
				// Valor a cargar 0x0057
				// Configurando el Baudrate generator para una velocidad de 57600bps
				ptrUsartHandler->ptrUSARTx->BRR = 0x0057;
			}
		}
		else if(ptrUsartHandler->ptrUSARTx == USART2){
			freqClock = getConfigPLL()/2;
			// Estamos con el USART2 usando el PLL a X MHz, que para el APB1 queda a X/2 MHz
			if(ptrUsartHandler->USART_Config.USART_baudrate == USART_BAUDRATE_9600){
				// El valor a cargar es 260.4166 -> Mantiza = 260,fraction = 0.4166
				// Mantiza = 260 = 0x104, fraction = 16 * 0.4166 = 6.66 = 7
				// Valor a cargar 0x1047
				// Configurando el Baudrate generator para una velocidad de 9600bps
				ptrUsartHandler->ptrUSARTx->BRR = 0x1047;
			}
			else if (ptrUsartHandler->USART_Config.USART_baudrate == USART_BAUDRATE_19200) {
				// El valor a cargar es 130.2083 -> Mantiza = 130,fraction = 0.2083
				// Mantiza = 130 = 0x82, fraction = 16 * 0.0625 = 3.33 = 3
				// Valor a cargar 0x0823
				// Configurando el Baudrate generator para una velocidad de 19200bps
				ptrUsartHandler->ptrUSARTx->BRR = 0x0823;
			}
			else if (ptrUsartHandler->USART_Config.USART_baudrate == USART_BAUDRATE_38400) {
				// El valor a cargar es 65.1041 -> Mantiza = 65,fraction = 0.1041
				// Mantiza = 65 = 0x41, fraction = 16 * 0.1041 = 1.66 = 2
				// Valor a cargar 0x0412
				// Configurando el Baudrate generator para una velocidad de 38400bps
				ptrUsartHandler->ptrUSARTx->BRR = 0x0412;
			}
			else if (ptrUsartHandler->USART_Config.USART_baudrate == USART_BAUDRATE_57600) {
				// El valor a cargar es 43.4027 -> Mantiza = 43,fraction = 0.4027
				// Mantiza = 43 = 0x2B, fraction = 16 * 0.4027 = 6.44
				// Valor a cargar 0x02B6
				// Configurando el Baudrate generator para una velocidad de 57600bps
				ptrUsartHandler->ptrUSARTx->BRR = 0x02B6;
			}
			else if(ptrUsartHandler->USART_Config.USART_baudrate == USART_BAUDRATE_115200){
				usartDiv = freqClock/(16.0*115200.0);
				mantiza = (uint16_t)usartDiv;
				fraction = (uint8_t)roundf((usartDiv - (uint16_t)usartDiv)*16);
				baudrate = ((mantiza << 4)|fraction);
				ptrUsartHandler->ptrUSARTx->BRR = baudrate;
				// El valor a cargar es  27.1267 -> Mantiza = 27,fraction = 0.1267
				// Mantiza = 27 = 0x1B, fraction = 16 * 0.1267 = 2.03 = 2
				// Valor a cargar 0x01B2
				// Configurando el Baudrate generator para una velocidad de 57600bps
				//ptrUsartHandler->ptrUSARTx->BRR = 0x01B2;
			}
			else if(ptrUsartHandler->USART_Config.USART_baudrate == USART_BAUDRATE_230400){
				// El valor a cargar es 10.8507 -> Mantiza = 10,fraction = 0.8507
				// Mantiza = 10 = 0xA, fraction = 16 * 0.8507 = 13.61 = 14
				// Valor a cargar 0x00AE
				// Configurando el Baudrate generator para una velocidad de 57600bps
				ptrUsartHandler->ptrUSARTx->BRR = 0x00AE;
			}
			else if(ptrUsartHandler->USART_Config.USART_baudrate == USART_BAUDRATE_921600){
				// El valor a cargar es 2.7126 -> Mantiza = 2,fraction = 0.7126
				// Mantiza = 1 = 0x1, fraction = 16 * 0.7126 = 11.40
				// Valor a cargar 0x001B
				// Configurando el Baudrate generator para una velocidad de 57600bps
				ptrUsartHandler->ptrUSARTx->BRR = 0x001B;
			}
		}
	}
	// Sino, estamos con el HSI
	else{
	// Ver tabla de valores (Tabla 75), Frec = 16MHz, over8 = 0;
		if(ptrUsartHandler->USART_Config.USART_baudrate == USART_BAUDRATE_9600){
			// El valor a cargar es 104.1875 -> Mantiza = 104,fraction = 0.1875
			// Mantiza = 104 = 0x68, fraction = 16 * 0.1875 = 3
			// Valor a cargar 0x0683
			// Configurando el Baudrate generator para una velocidad de 9600bps
			ptrUsartHandler->ptrUSARTx->BRR = 0x0683;
		}
		else if (ptrUsartHandler->USART_Config.USART_baudrate == USART_BAUDRATE_19200) {
			// El valor a cargar es 52.0625 -> Mantiza = 52,fraction = 0.0625
			// Mantiza = 52 = 0x34, fraction = 16 * 0.0625 = 1
			// Valor a cargar 0x0341
			// Configurando el Baudrate generator para una velocidad de 19200bps
			ptrUsartHandler->ptrUSARTx->BRR = 0x0341;
		}
		else if (ptrUsartHandler->USART_Config.USART_baudrate == USART_BAUDRATE_38400) {
			// El valor a cargar es 26.0625 -> Mantiza = 26,fraction = 0.0625
			// Mantiza = 26 = 0x1A, fraction = 16 * 0.0625 = 1
			// Valor a cargar 0x01A1
			// Configurando el Baudrate generator para una velocidad de 38400bps
			ptrUsartHandler->ptrUSARTx->BRR = 0x01A1;
		}
		else if (ptrUsartHandler->USART_Config.USART_baudrate == USART_BAUDRATE_57600) {
			// El valor a cargar es 17.3750 -> Mantiza = 17,fraction = 0.3750
			// Mantiza = 17 = 0x11, fraction = 16 * 0.3750 = 6
			// Valor a cargar 0x0116
			// Configurando el Baudrate generator para una velocidad de 57600bps
			ptrUsartHandler->ptrUSARTx->BRR = 0x0116;
		}
		else if(ptrUsartHandler->USART_Config.USART_baudrate == USART_BAUDRATE_115200){
			// El valor a cargar es 8.6805 -> Mantiza = 8,fraction = 0.6805
			// Mantiza = 8 = 0x8, fraction = 16 * 0.6805 = 11
			// Valor a cargar 0x008B
			// Configurando el Baudrate generator para una velocidad de 57600bps
			ptrUsartHandler->ptrUSARTx->BRR = 0x008B;
		}
		else if(ptrUsartHandler->USART_Config.USART_baudrate == USART_BAUDRATE_230400){
			// El valor a cargar es 4.3125 -> Mantiza = 4,fraction = 0.3125
			// Mantiza = 4 = 0x4, fraction = 16 * 0.3125 = 5
			// Valor a cargar 0x0045
			// Configurando el Baudrate generator para una velocidad de 57600bps
			ptrUsartHandler->ptrUSARTx->BRR = 0x0045;
		}
		else if(ptrUsartHandler->USART_Config.USART_baudrate == USART_BAUDRATE_921600){
			// El valor a cargar es 1.0625 -> Mantiza = 1,fraction = 0.0625
			// Mantiza = 1 = 0x1, fraction = 16 * 0.0625 = 1
			// Valor a cargar 0x0011
			// Configurando el Baudrate generator para una velocidad de 57600bps
			ptrUsartHandler->ptrUSARTx->BRR = 0x0011;
		}
	}


	// 2.6 Configuramos el modo: TX only, RX only, RXTX, disable
	switch(ptrUsartHandler->USART_Config.USART_mode){
	case USART_MODE_TX:
	{
		// Activamos la parte del sistema encargada de enviar
		ptrUsartHandler->ptrUSARTx->CR1 &= ~USART_CR1_TE; 	// Limpio Tx
		ptrUsartHandler->ptrUSARTx->CR1 |= USART_CR1_TE; 	// Escribo Tx
		break;
	}
	case USART_MODE_RX:
	{
		// Activamos la parte del sistema encargada de recibir
		ptrUsartHandler->ptrUSARTx->CR1 &= ~USART_CR1_RE; 	// Limpio Rx
		ptrUsartHandler->ptrUSARTx->CR1 |= USART_CR1_RE; 	// Escribo Rx
		break;
	}
	case USART_MODE_RXTX:
	{
		// Activamos ambas partes, tanto transmision como recepcion
		ptrUsartHandler->ptrUSARTx->CR1 &= ~USART_CR1_RE; 	// Limpio Rx
		ptrUsartHandler->ptrUSARTx->CR1 |= USART_CR1_RE; 	// Escribo Rx
		ptrUsartHandler->ptrUSARTx->CR1 &= ~USART_CR1_TE; 	// Limpio Tx
		ptrUsartHandler->ptrUSARTx->CR1 |= USART_CR1_TE; 	// Escribo Tx
		break;
	}
	case USART_MODE_DISABLE:
	{
		// Desactivamos ambos canales
		ptrUsartHandler->ptrUSARTx->CR1 &= ~USART_CR1_RE; 	// Rx
		ptrUsartHandler->ptrUSARTx->CR1 &= ~USART_CR1_TE; 	// Tx
		ptrUsartHandler->ptrUSARTx->CR1 &= ~USART_CR1_UE;
		break;
	}

	default:
	{
		// Actuando por defecto, desactivamos ambos canales
		ptrUsartHandler->ptrUSARTx->CR1 &= ~USART_CR1_RE; 	// Rx
		ptrUsartHandler->ptrUSARTx->CR1 &= ~USART_CR1_TE; 	// Tx
		break;
	}
	}

	// 2.7 Veamos si las interrupciones van a ser activadas

	__disable_irq(); // Desactivo interrupciones globales

	// Interrupción por recepción
	if(ptrUsartHandler->USART_Config.USART_enableIntRX == USART_RX_INTERRUP_ENABLE){
		ptrUsartHandler->ptrUSARTx->CR1 &= ~USART_CR1_RXNEIE;
		ptrUsartHandler->ptrUSARTx->CR1 |= USART_CR1_RXNEIE;
	}else{
		ptrUsartHandler->ptrUSARTx->CR1 &= ~USART_CR1_RXNEIE;
	}

	// Interrupción por transmisión
	if(ptrUsartHandler->USART_Config.USART_enableIntTX == USART_TX_INTERRUP_ENABLE){
		ptrUsartHandler->ptrUSARTx->CR1 &= ~USART_CR1_TXEIE;
		ptrUsartHandler->ptrUSARTx->CR1 |= USART_CR1_TXEIE;
	}else{
		ptrUsartHandler->ptrUSARTx->CR1 &= ~USART_CR1_TXEIE;
	}

	// Matriculamos en el NVIC para el USART correspondiente
	if(ptrUsartHandler->ptrUSARTx == USART1){
		__NVIC_EnableIRQ(USART1_IRQn);
		__NVIC_SetPriority(USART1_IRQn, ptrUsartHandler->USART_Config.USART_priorityInterrupt);
	}else if(ptrUsartHandler->ptrUSARTx == USART2){
		__NVIC_EnableIRQ(USART2_IRQn);
		__NVIC_SetPriority(USART2_IRQn, ptrUsartHandler->USART_Config.USART_priorityInterrupt);
	}else if(ptrUsartHandler->ptrUSARTx == USART6){
		__NVIC_EnableIRQ(USART6_IRQn);
		__NVIC_SetPriority(USART6_IRQn, ptrUsartHandler->USART_Config.USART_priorityInterrupt);
	}

	__enable_irq();	// Activo nuevamente las interrupciones globales

	// 2.8 Activamos el modulo serial.
	if(ptrUsartHandler->USART_Config.USART_mode != USART_MODE_DISABLE){
		ptrUsartHandler->ptrUSARTx->CR1 &= ~USART_CR1_UE;
		ptrUsartHandler->ptrUSARTx->CR1 |= USART_CR1_UE; //Activo USART
	}
}


/** Función para escribir un solo char */
int writeChar(USART_Handler_t *ptrUsartHandler, char dataToSend ){
	while( !(ptrUsartHandler->ptrUSARTx->SR & USART_SR_TXE)){
		__NOP();
	}
	ptrUsartHandler->ptrUSARTx->DR = dataToSend;
	return dataToSend;
}

/** Función para escribir un solo char por interrupción */
int writeCharTXE(USART_Handler_t *ptrUsartHandler, char dataToSend ){
	dataUSARTchar = dataToSend;
	charOrString = 0;
	ptrUsartHandler->ptrUSARTx->CR1 |= USART_CR1_TXEIE;
	return dataToSend;
}

/** Función para escribir todo un string */
void writeMsg(USART_Handler_t *ptrUsartHandler, char *messageToSend){
	int i = 0;
	while(messageToSend[i] != '\0'){
		writeChar(ptrUsartHandler, messageToSend[i]);
		i++;
	}
}

/** Función para escribir todo un string por interrupcíón */
void writeMsgTXE(USART_Handler_t *ptrUsartHandler, char *messageToSend){
	ptrdataUSARTstring = messageToSend;
	charOrString = 1;
	ptrUsartHandler->ptrUSARTx->CR1 |= USART_CR1_TXEIE;
}

/** Función para cuando hay interrupción Tx en Usart1, cargue carácter en el DR y desactive */
void usart1Tx_Char(void){
	USART1->DR = dataUSARTchar;
	USART1->CR1 &= ~USART_CR1_TXEIE;
}

/** Función para cuando hay interrupción Tx en Usart2, cargue carácter en el DR y desactive */
void usart2Tx_Char(void){
	USART2->DR = dataUSARTchar;
	USART2->CR1 &= ~USART_CR1_TXEIE;
}

/** Función para cuando hay interrupción Tx en Usart6, cargue carácter en el DR y desactive */
void usart6Tx_Char(void){
	USART6->DR = dataUSARTchar;
	USART6->CR1 &= ~USART_CR1_TXEIE;
}

/** Función para cuando hay interrupción Tx en Usart1, cargue string en el DR y desactive */
void usart1Tx_String(void){
	if(ptrdataUSARTstring[pos] != '\0'){
		USART1->DR = ptrdataUSARTstring[pos];
		pos++;
	}
	else{
		USART1->CR1 &= ~USART_CR1_TXEIE;
		pos = 0;
		ptrdataUSARTstring = NULL;
	}

}

/** Función para cuando hay interrupción Tx en Usart2, cargue string en el DR y desactive */
void usart2Tx_String(void){
	if(ptrdataUSARTstring[pos] != '\0'){
		USART2->DR = ptrdataUSARTstring[pos];
		pos++;
	}
	else{
		USART2->CR1 &= ~USART_CR1_TXEIE;
		pos = 0;
		ptrdataUSARTstring = NULL;
	}
}

/** Función para cuando hay interrupción Tx en Usart6, cargue string en el DR y desactive */
void usart6Tx_String(void){
	if(ptrdataUSARTstring[pos] != '\0'){
		USART6->DR = ptrdataUSARTstring[pos];
		pos++;
	}
	else{
		USART6->CR1 &= ~USART_CR1_TXEIE;
		pos = 0;
		ptrdataUSARTstring = NULL;
	}
}

/** Función para recibir datos */
uint8_t getRxData(void){

	return auxRxData;
}


/** ISR de la interrupción del USART1 */
void USART1_IRQHandler(void){
	// Evaluamos si la interrupción que se dio es por RX
	if(USART1->SR & USART_SR_RXNE){
		auxRxData = (uint8_t) USART1->DR;
		usart1Rx_Callback();
	}
	// Evaluamos si la interrupción que se dio es por TX
	else if(USART1->SR & USART_SR_TXE){
		if(charOrString){
			usart1Tx_String();
		}
		else{
			usart1Tx_Char();
		}
	}
	else{
		__NOP();
	}
}

/** ISR de la interrupción del USART2 */
void USART2_IRQHandler(void){
	// Evaluamos si la interrupción que se dio es por RX
	if(USART2->SR & USART_SR_RXNE){
		auxRxData = (uint8_t) USART2->DR;
		usart2Rx_Callback();
	}
	// Evaluamos si la interrupción que se dio es por TX
	else if(USART2->SR & USART_SR_TXE){
		if(charOrString){
			usart2Tx_String();
		}
		else{
			usart2Tx_Char();
		}
	}
	else{
		__NOP();
	}
}

/** ISR de la interrupción del USART6 */
void USART6_IRQHandler(void){
	// Evaluamos si la interrupción que se dio es por RX
	if(USART6->SR & USART_SR_RXNE){
		auxRxData = (uint8_t) USART6->DR;
		usart6Rx_Callback();
	}
	// Evaluamos si la interrupción que se dio es por TX
	else if(USART6->SR & USART_SR_TXE){
		if(charOrString){
			usart6Tx_String();
		}
		else{
			usart6Tx_Char();
		}
	}
	else{
		__NOP();
	}
}


/** Funciones callback weak, que pueden ser sobre-escritas*/
__attribute__((weak)) void usart1Rx_Callback(void){
		/* 	NOTE: This function should not be modified, when the callback is needed,
		  		  the usart1Rx_Callback could be implemented in the main file
		 */
	__NOP();
}
__attribute__((weak)) void usart2Rx_Callback(void){
		/* 	NOTE: This function should not be modified, when the callback is needed,
		  		  the usart2Rx_Callback could be implemented in the main file
		 */
	__NOP();
}
__attribute__((weak)) void usart6Rx_Callback(void){
		/* 	NOTE: This function should not be modified, when the callback is needed,
		  		  the usart6Rx_Callback could be implemented in the main file
		 */
	__NOP();
}

