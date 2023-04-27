/**
 ******************************************************************************
 * @file           : main.c
 * @author         : MillerQUintero2001
 * @brief          : Tarea #3 Driver EXTI y Timers
 ******************************************************************************
 * Generacion del archivo de configuración por defecto
 * como plantilla para los proyectos funcionales
 ******************************************************************************
 */

#include <BasicTimer.h>
#include <ExtiDriver.h>
#include <stm32f4xx.h>
#include <stdint.h>
#include "GPIOxDriver.h"

/* Inicializo handler de los periféricos a utilizar */

// GPIO y Timer del blinky
GPIO_Handler_t handlerLedState = {0}; // LED de estado, que estará conectado al PIN A5 y será blinky
BasicTimer_Handler_t handlerLedStateTimer = {0}; // Timer del LED de estado

// GPIO's y Timer de los transistores
GPIO_Handler_t handlerTransistorUnd = {0}; // Transistor que controla alimentación para las unidades del display
GPIO_Handler_t handlerTransistorDec = {0}; // Transistor que controla alimentación para las decenas del display
BasicTimer_Handler_t handlerSwitching = {0}; //Timer para el suicheo de transistores en el display
BasicTimer_Handler_t handlerWait = {0}; //Timer para el reposo de transistores en el display

// GPIO's LED's 7 segmentos
GPIO_Handler_t handlerSegmentA = {0}; // Handler para el segmento 'a' del display con el Pin B12
GPIO_Handler_t handlerSegmentB = {0}; // Handler para el segmento 'b' del display con el Pin A11
GPIO_Handler_t handlerSegmentC = {0}; // Handler para el segmento 'c' del display con el Pin C5
GPIO_Handler_t handlerSegmentD = {0}; // Handler para el segmento 'd' del display con el Pin C6
GPIO_Handler_t handlerSegmentE = {0}; // Handler para el segmento 'e' del display con el Pin B9
GPIO_Handler_t handlerSegmentF = {0}; // Handler para el segmento 'f' del display con el Pin A12
GPIO_Handler_t handlerSegmentG = {0}; // Handler para el segmento 'g' del display con el Pin B8

// GPIO's del Encoder
GPIO_Handler_t handlerClock = {0};	// Handler para el Clock del encoder con el Pin B2
GPIO_Handler_t handlerData = {0};	// Handler para el Data del encoder con el Pin D2
GPIO_Handler_t handlerSwitch = {0};	// Handler para el Switch del encoder con el Pin A3
// EXTI's del Clock y Switch del encoder
EXTI_Config_t handlerExtiClock = {0}; 	// EXTI 2 handler para la interrupción del Clock
EXTI_Config_t handlerExtiSwitch = {0};	// EXTI 4 handler para la interrupción del Switch

/* Inicializo variables a emplear */
uint8_t flagLedState = 0;		// Variable bandera timer del LED de estado
uint8_t flagSwitch = 0;			// Variable bandera suicheo de transistores
uint8_t flagWait = 0;			// Variable bandera reposo de transistores
uint8_t contador = 0;			// Variable contador para el display
uint8_t signo_culebrita = 1;	// Variable que indica que segmento encender en modo culebrita
uint8_t modo = 0b1;				// Binario que indica modo, en 0 es culebrita, en 1 es contador
uint8_t pantalla_1 = 0b0;		// Variable que guarda el estado del display de unidades
uint8_t pantalla_2 = 0b1;		// Variable que guarda el estado del display de decenas
uint8_t dataEncoder = 0; 		// Variable que guarda el estado de pin Data del encoder

/* Definición de prototipos de función */
void init_Hardware(void); 							// Función que inicializa los periféricos
void display_Segun_Digito(uint8_t digito);			// Función encargada de encender los leds del display según el número
void display_Culebrita(uint8_t contador_culebrita);	// Función encargada de encender los leds del display en modo culebrita
void display_Control(uint8_t modo, uint8_t contador, uint8_t signo_culebrita, uint8_t estadoUnidades, uint8_t estadoDecenas); // Función controlar display según el modo

/** Funcion principal del programa */
int main(void){

	init_Hardware();

    /* Loop forever */
	while(1){

		// Blinky Led de Estado
		if(flagLedState){
			GPIOxTooglePin(&handlerLedState); //Cambio el estado del LED PA5 cada que se lanza interrup. del Timer 2
			flagLedState = 0;
		}

		// Suicheo transistores
		if(flagSwitch&&modo){
			GPIO_WritePin(&handlerTransistorUnd, SET);
			GPIO_WritePin(&handlerTransistorDec, SET);
			pantalla_1 ^= 0b1;
			pantalla_2 ^= 0b1;
			//Alterno el estado de los transistores
			if((~pantalla_1)&&(pantalla_2)){
				GPIO_WritePin(&handlerTransistorUnd, RESET);
			}
			else if((pantalla_1)&&(~pantalla_2)){
				GPIO_WritePin(&handlerTransistorDec, RESET);
			}
			else{
				__NOP();
			}
			flagSwitch = 0;
		}

		/* Control Segmentos del Display */

		// Regular limites del modo culebrita
		if(signo_culebrita == 13){
			signo_culebrita = 1; // Para limitar hasta los 12 estados del culebrita y que sea ciclica
		}
		else if(signo_culebrita == 0){
			signo_culebrita = 12; // 12 estados y garantizar culebrita ciclica
		}
		else{
			__NOP();
		}
		// Regular limites del modo contador
		if (contador == 100){ //Limite para que solo llegue hasta 99
			contador --;
		}
		else if (contador > 100){
			contador = 0; // Limite para que solo llegue hasta 00
		}
		else{
			__NOP();
		}
		//estado_Transistor1 = GPIO_ReadPin(&handlerTransistorUnd);
		//estado_Transistor2 = GPIO_ReadPin(&handlerTransistorDec);
		display_Control(modo, contador, signo_culebrita, pantalla_1, pantalla_2); // Función que enciende leds

	}
}

/** Funcion encargada de iniciar hardware para un pin*/
void init_Hardware(void){

	/* GPIO y Timer del Blinky Led de Estado */
	handlerLedState.pGPIOx								= GPIOA;
	handlerLedState.GPIO_PinConfig.GPIO_PinNumber 		= PIN_5;
	handlerLedState.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_OUT;
	handlerLedState.GPIO_PinConfig.GPIO_PinSpeed 		= GPIO_OSPEED_FAST;
	handlerLedState.GPIO_PinConfig.GPIO_PinOPType 		= GPIO_OTYPE_PUSHPULL;
	// Cargo la configuración
	GPIO_Config(&handlerLedState);
	// Pongo estado en alto
	GPIO_WritePin(&handlerLedState, SET);
	// Atributos para el Timer 2 del LED de estado
	handlerLedStateTimer.ptrTIMx							= TIM2;
	handlerLedStateTimer.TIMx_Config.TIMx_mode				= BTIMER_MODE_UP;
	handlerLedStateTimer.TIMx_Config.TIMx_speed				= BTIMER_SPEED_1ms;
	handlerLedStateTimer.TIMx_Config.TIMx_period			= 250;
	handlerLedStateTimer.TIMx_Config.TIMx_interruptEnable 	= 1;

	BasicTimer_Config(&handlerLedStateTimer);
	/* Fin del GPIO y Timer del LED de estado
	 * ----------------------------------------*/

	/* GPIO's y Timer para el suicheo de Transistores*/
	// Transistor Unidades
	handlerTransistorUnd.pGPIOx								= GPIOC;
	handlerTransistorUnd.GPIO_PinConfig.GPIO_PinNumber 		= PIN_8;
	handlerTransistorUnd.GPIO_PinConfig.GPIO_PinMode		= GPIO_MODE_OUT;
	handlerTransistorUnd.GPIO_PinConfig.GPIO_PinSpeed 		= GPIO_OSPEED_FAST;
	handlerTransistorUnd.GPIO_PinConfig.GPIO_PinOPType 		= GPIO_OTYPE_PUSHPULL;
	// Cargo la configuración
	GPIO_Config(&handlerTransistorUnd);
	// Pongo estado en alto para comenzar con unidades, con transistores PNP esto es poner en alto
	GPIO_WritePin(&handlerTransistorUnd, SET);

	// Transistor Decenas
	handlerTransistorDec.pGPIOx								= GPIOC;
	handlerTransistorDec.GPIO_PinConfig.GPIO_PinNumber 		= PIN_9;
	handlerTransistorDec.GPIO_PinConfig.GPIO_PinMode		= GPIO_MODE_OUT;
	handlerTransistorDec.GPIO_PinConfig.GPIO_PinSpeed 		= GPIO_OSPEED_FAST;
	handlerTransistorDec.GPIO_PinConfig.GPIO_PinOPType 		= GPIO_OTYPE_PUSHPULL;
	// Cargo la configuración
	GPIO_Config(&handlerTransistorDec);
	// Pongo estado en bajo ya que comienzo con unidades, y para garantizar que cuando uno este en alto el otro en bajo
	GPIO_WritePin(&handlerTransistorDec, SET);

	// Atributos para el Timer 3 suicheo de transistores
	handlerSwitching.ptrTIMx							= TIM3;
	handlerSwitching.TIMx_Config.TIMx_mode				= BTIMER_MODE_UP;
	handlerSwitching.TIMx_Config.TIMx_speed				= BTIMER_SPEED_1ms;
	handlerSwitching.TIMx_Config.TIMx_period			= 10; // Con este periodo tenemos una frecuencia de 50 Hz
	handlerSwitching.TIMx_Config.TIMx_interruptEnable 	= 1;
	BasicTimer_Config(&handlerSwitching);

	// Atributos para el Timer 4 reposo de transistores
	handlerWait.ptrTIMx								= TIM4;
	handlerWait.TIMx_Config.TIMx_mode				= BTIMER_MODE_UP;
	handlerWait.TIMx_Config.TIMx_speed				= BTIMER_SPEED_1ms;
	handlerWait.TIMx_Config.TIMx_period				= 5;
	handlerWait.TIMx_Config.TIMx_interruptEnable 	= 1;
	BasicTimer_Config(&handlerWait);

	/* Fin del GPIO's Transistores y Timer's del suicheo
	 * ----------------------------------------------*/

	/* Configuración de los pines para cada segmento */
	// Segmento A
	handlerSegmentA.pGPIOx								= GPIOB;
	handlerSegmentA.GPIO_PinConfig.GPIO_PinNumber 		= PIN_12;
	handlerSegmentA.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_OUT;
	handlerSegmentA.GPIO_PinConfig.GPIO_PinSpeed 		= GPIO_OSPEED_FAST;
	handlerSegmentA.GPIO_PinConfig.GPIO_PinOPType 		= GPIO_OTYPE_PUSHPULL;
	GPIO_Config(&handlerSegmentA);

	// Segmento B
	handlerSegmentB.pGPIOx								= GPIOA;
	handlerSegmentB.GPIO_PinConfig.GPIO_PinNumber 		= PIN_11;
	handlerSegmentB.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_OUT;
	handlerSegmentB.GPIO_PinConfig.GPIO_PinSpeed 		= GPIO_OSPEED_FAST;
	handlerSegmentB.GPIO_PinConfig.GPIO_PinOPType 		= GPIO_OTYPE_PUSHPULL;
	GPIO_Config(&handlerSegmentB);

	// Segmento C
	handlerSegmentC.pGPIOx								= GPIOC;
	handlerSegmentC.GPIO_PinConfig.GPIO_PinNumber 		= PIN_5;
	handlerSegmentC.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_OUT;
	handlerSegmentC.GPIO_PinConfig.GPIO_PinSpeed 		= GPIO_OSPEED_FAST;
	handlerSegmentC.GPIO_PinConfig.GPIO_PinOPType 		= GPIO_OTYPE_PUSHPULL;
	GPIO_Config(&handlerSegmentC);

	// Segmento D
	handlerSegmentD.pGPIOx								= GPIOC;
	handlerSegmentD.GPIO_PinConfig.GPIO_PinNumber 		= PIN_6;
	handlerSegmentD.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_OUT;
	handlerSegmentD.GPIO_PinConfig.GPIO_PinSpeed 		= GPIO_OSPEED_FAST;
	handlerSegmentD.GPIO_PinConfig.GPIO_PinOPType 		= GPIO_OTYPE_PUSHPULL;
	GPIO_Config(&handlerSegmentD);

	// Segmento E
	handlerSegmentE.pGPIOx								= GPIOB;
	handlerSegmentE.GPIO_PinConfig.GPIO_PinNumber 		= PIN_9;
	handlerSegmentE.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_OUT;
	handlerSegmentE.GPIO_PinConfig.GPIO_PinSpeed 		= GPIO_OSPEED_FAST;
	handlerSegmentE.GPIO_PinConfig.GPIO_PinOPType 		= GPIO_OTYPE_PUSHPULL;
	GPIO_Config(&handlerSegmentE);

	// Segmento F
	handlerSegmentF.pGPIOx								= GPIOA;
	handlerSegmentF.GPIO_PinConfig.GPIO_PinNumber 		= PIN_12;
	handlerSegmentF.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_OUT;
	handlerSegmentF.GPIO_PinConfig.GPIO_PinSpeed 		= GPIO_OSPEED_FAST;
	handlerSegmentF.GPIO_PinConfig.GPIO_PinOPType 		= GPIO_OTYPE_PUSHPULL;
	GPIO_Config(&handlerSegmentF);

	// Segmento G
	handlerSegmentG.pGPIOx								= GPIOB;
	handlerSegmentG.GPIO_PinConfig.GPIO_PinNumber 		= PIN_8;
	handlerSegmentG.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_OUT;
	handlerSegmentG.GPIO_PinConfig.GPIO_PinSpeed 		= GPIO_OSPEED_FAST;
	handlerSegmentG.GPIO_PinConfig.GPIO_PinOPType 		= GPIO_OTYPE_PUSHPULL;
	GPIO_Config(&handlerSegmentG);
	/* Fin de la configuración de los pines para el 7 segmentos
	 * ----------------------------------------------------------*/

	/* GPIO's y EXTI's*/

	// Clock del encoder, con su GPIO y EXTI
	handlerClock.pGPIOx							= GPIOB;
	handlerClock.GPIO_PinConfig.GPIO_PinNumber 	= PIN_2;
	handlerClock.GPIO_PinConfig.GPIO_PinMode	= GPIO_MODE_IN; // Entrada
	handlerExtiClock.pGPIOHandler				= &handlerClock;
	handlerExtiClock.edgeType					= EXTERNAL_INTERRUPT_RISING_EDGE; // Detecto flanco de subida en el clock
	extInt_Config(&handlerExtiClock);

	// Data del encoder solo GPIO, no usa interrupciones externas
	handlerData.pGPIOx							= GPIOB;
	handlerData.GPIO_PinConfig.GPIO_PinNumber 	= PIN_5;
	handlerData.GPIO_PinConfig.GPIO_PinMode		= GPIO_MODE_IN; // Entrada
	GPIO_Config(&handlerData);

	// Switch del encoder, con su GPIO y EXTI
	handlerSwitch.pGPIOx						= GPIOC;
	handlerSwitch.GPIO_PinConfig.GPIO_PinNumber	= PIN_4;
	handlerSwitch.GPIO_PinConfig.GPIO_PinMode	= GPIO_MODE_IN; // Entrada
	handlerExtiSwitch.pGPIOHandler				= &handlerSwitch;
	handlerExtiSwitch.edgeType					= EXTERNAL_INTERRUPT_FALLING_EDGE; // Detecto flanco de bajada en el switch
	extInt_Config(&handlerExtiSwitch);
}

void BasicTimer2_Callback(void){
	flagLedState = 1; // Pongo en alto la variable bandera del timer 2 para el main
}

void BasicTimer3_Callback(void){
	flagSwitch = 1; // Pongo en alto la variable bandera del timer 3 para el main
}

void BasicTimer4_Callback(void){
	flagWait = 1; // Pongo en alto la variable bandera del timer 4 para el main
}

/** Interrupción clock del encoder */
void callback_extInt2(void){
	dataEncoder = GPIO_ReadPin(&handlerData);
	// Con flanco de subida en Clock, si Data es diferente de 0, va en sentido anti-horario
	if(dataEncoder != 0){
		if(modo){ //Verifico si estoy en modo contador
			contador--;
		}else{ // Sino en culebrita
			signo_culebrita--;
		}
	}
	// Sino, va en sentido horario
	else if (dataEncoder == 0){
		if(modo){ //Verifico si estoy en modo contador
			contador++;
		}else{ // Sino en culebrita
			signo_culebrita++;
		}
	}
	else{
		__NOP();
	}
}

/** Interrupción switch del encoder */
void callback_extInt4(void){
	modo ^= 0b1; //Con esto obtenemos el complemento, si estaba en culebrita (0) pasa a contador (1), y viceversa
}

/** Funcion encargda de activar segmentos según el digito ingresado */
void display_Segun_Digito(uint8_t digito){
	switch(digito){
	case 0: {
		GPIO_WritePin(&handlerSegmentA, RESET);
		GPIO_WritePin(&handlerSegmentB, RESET);
		GPIO_WritePin(&handlerSegmentC, RESET);
		GPIO_WritePin(&handlerSegmentD, RESET);
		GPIO_WritePin(&handlerSegmentE, RESET);
		GPIO_WritePin(&handlerSegmentF, RESET);
		GPIO_WritePin(&handlerSegmentG, SET);
		break;
	}
	case 1: {
		GPIO_WritePin(&handlerSegmentA, SET);
		GPIO_WritePin(&handlerSegmentB, RESET);
		GPIO_WritePin(&handlerSegmentC, RESET);
		GPIO_WritePin(&handlerSegmentD, SET);
		GPIO_WritePin(&handlerSegmentE, SET);
		GPIO_WritePin(&handlerSegmentF, SET);
		GPIO_WritePin(&handlerSegmentG, SET);
		break;
	}
	case 2: {
		GPIO_WritePin(&handlerSegmentA, RESET);
		GPIO_WritePin(&handlerSegmentB, RESET);
		GPIO_WritePin(&handlerSegmentC, SET);
		GPIO_WritePin(&handlerSegmentD, RESET);
		GPIO_WritePin(&handlerSegmentE, RESET);
		GPIO_WritePin(&handlerSegmentF, SET);
		GPIO_WritePin(&handlerSegmentG, RESET);
		break;
	}
	case 3: {
		GPIO_WritePin(&handlerSegmentA, RESET);
		GPIO_WritePin(&handlerSegmentB, RESET);
		GPIO_WritePin(&handlerSegmentC, RESET);
		GPIO_WritePin(&handlerSegmentD, RESET);
		GPIO_WritePin(&handlerSegmentE, SET);
		GPIO_WritePin(&handlerSegmentF, SET);
		GPIO_WritePin(&handlerSegmentG, RESET);
		break;
	}
	case 4: {
		GPIO_WritePin(&handlerSegmentA, SET);
		GPIO_WritePin(&handlerSegmentB, RESET);
		GPIO_WritePin(&handlerSegmentC, RESET);
		GPIO_WritePin(&handlerSegmentD, SET);
		GPIO_WritePin(&handlerSegmentE, SET);
		GPIO_WritePin(&handlerSegmentF, RESET);
		GPIO_WritePin(&handlerSegmentG, RESET);
		break;
	}
	case 5: {
		GPIO_WritePin(&handlerSegmentA, RESET);
		GPIO_WritePin(&handlerSegmentB, SET);
		GPIO_WritePin(&handlerSegmentC, RESET);
		GPIO_WritePin(&handlerSegmentD, RESET);
		GPIO_WritePin(&handlerSegmentE, SET);
		GPIO_WritePin(&handlerSegmentF, RESET);
		GPIO_WritePin(&handlerSegmentG, RESET);
		break;
	}
	case 6: {
		GPIO_WritePin(&handlerSegmentA, RESET);
		GPIO_WritePin(&handlerSegmentB, SET);
		GPIO_WritePin(&handlerSegmentC, RESET);
		GPIO_WritePin(&handlerSegmentD, RESET);
		GPIO_WritePin(&handlerSegmentE, RESET);
		GPIO_WritePin(&handlerSegmentF, RESET);
		GPIO_WritePin(&handlerSegmentG, RESET);
		break;
	}
	case 7: {
		GPIO_WritePin(&handlerSegmentA, RESET);
		GPIO_WritePin(&handlerSegmentB, RESET);
		GPIO_WritePin(&handlerSegmentC, RESET);
		GPIO_WritePin(&handlerSegmentD, SET);
		GPIO_WritePin(&handlerSegmentE, SET);
		GPIO_WritePin(&handlerSegmentF, SET);
		GPIO_WritePin(&handlerSegmentG, SET);
		break;
	}
	case 8: {
		GPIO_WritePin(&handlerSegmentA, RESET);
		GPIO_WritePin(&handlerSegmentB, RESET);
		GPIO_WritePin(&handlerSegmentC, RESET);
		GPIO_WritePin(&handlerSegmentD, RESET);
		GPIO_WritePin(&handlerSegmentE, RESET);
		GPIO_WritePin(&handlerSegmentF, RESET);
		GPIO_WritePin(&handlerSegmentG, RESET);
		break;
	}
	case 9: {
		GPIO_WritePin(&handlerSegmentA, RESET);
		GPIO_WritePin(&handlerSegmentB, RESET);
		GPIO_WritePin(&handlerSegmentC, RESET);
		GPIO_WritePin(&handlerSegmentD, RESET);
		GPIO_WritePin(&handlerSegmentE, SET);
		GPIO_WritePin(&handlerSegmentF, RESET);
		GPIO_WritePin(&handlerSegmentG, RESET);
		break;
	}
	default: {
		__NOP();
		break;
	}
	}
}

/** Función encargada de activar y desactivar transistores según el display de culebrita a mostrar*/
void display_Culebrita(uint8_t contador_culebrita){
	switch(contador_culebrita){
	case 1: {
		GPIO_WritePin(&handlerTransistorUnd, RESET);
		GPIO_WritePin(&handlerTransistorDec, SET);
		// Encendemos display A de unidades
		GPIO_WritePin(&handlerSegmentA, RESET);
		GPIO_WritePin(&handlerSegmentB, SET);
		GPIO_WritePin(&handlerSegmentC, SET);
		GPIO_WritePin(&handlerSegmentD, SET);
		GPIO_WritePin(&handlerSegmentE, SET);
		GPIO_WritePin(&handlerSegmentF, SET);
		GPIO_WritePin(&handlerSegmentG, SET);
		break;
	}
	case 2: {
		GPIO_WritePin(&handlerTransistorUnd, SET);
		GPIO_WritePin(&handlerTransistorDec, RESET);
		// Encendemos display A de decenas
		GPIO_WritePin(&handlerSegmentA, RESET);
		GPIO_WritePin(&handlerSegmentB, SET);
		GPIO_WritePin(&handlerSegmentC, SET);
		GPIO_WritePin(&handlerSegmentD, SET);
		GPIO_WritePin(&handlerSegmentE, SET);
		GPIO_WritePin(&handlerSegmentF, SET);
		GPIO_WritePin(&handlerSegmentG, SET);
		break;
	}
	case 3: {
		GPIO_WritePin(&handlerTransistorUnd, SET);
		GPIO_WritePin(&handlerTransistorDec, RESET);
		// Encendemos display F de decenas
		GPIO_WritePin(&handlerSegmentA, SET);
		GPIO_WritePin(&handlerSegmentB, SET);
		GPIO_WritePin(&handlerSegmentC, SET);
		GPIO_WritePin(&handlerSegmentD, SET);
		GPIO_WritePin(&handlerSegmentE, SET);
		GPIO_WritePin(&handlerSegmentF, RESET);
		GPIO_WritePin(&handlerSegmentG, SET);
		break;
	}
	case 4: {
		GPIO_WritePin(&handlerTransistorUnd, SET);
		GPIO_WritePin(&handlerTransistorDec, RESET);
		// Encendemos display E de decenas
		GPIO_WritePin(&handlerSegmentA, SET);
		GPIO_WritePin(&handlerSegmentB, SET);
		GPIO_WritePin(&handlerSegmentC, SET);
		GPIO_WritePin(&handlerSegmentD, SET);
		GPIO_WritePin(&handlerSegmentE, RESET);
		GPIO_WritePin(&handlerSegmentF, SET);
		GPIO_WritePin(&handlerSegmentG, SET);
		break;
	}
	case 5: {
		GPIO_WritePin(&handlerTransistorUnd, SET);
		GPIO_WritePin(&handlerTransistorDec, RESET);
		// Encendemos display D de decenas
		GPIO_WritePin(&handlerSegmentA, SET);
		GPIO_WritePin(&handlerSegmentB, SET);
		GPIO_WritePin(&handlerSegmentC, SET);
		GPIO_WritePin(&handlerSegmentD, RESET);
		GPIO_WritePin(&handlerSegmentE, SET);
		GPIO_WritePin(&handlerSegmentF, SET);
		GPIO_WritePin(&handlerSegmentG, SET);
		break;
	}
	case 6: {
		GPIO_WritePin(&handlerTransistorUnd, RESET);
		GPIO_WritePin(&handlerTransistorDec, SET);
		// Encendemos display E de unidades
		GPIO_WritePin(&handlerSegmentA, SET);
		GPIO_WritePin(&handlerSegmentB, SET);
		GPIO_WritePin(&handlerSegmentC, SET);
		GPIO_WritePin(&handlerSegmentD, SET);
		GPIO_WritePin(&handlerSegmentE, RESET);
		GPIO_WritePin(&handlerSegmentF, SET);
		GPIO_WritePin(&handlerSegmentG, SET);
		break;
	}
	case 7: {
		GPIO_WritePin(&handlerTransistorUnd, RESET);
		GPIO_WritePin(&handlerTransistorDec, SET);
		// Encendemos display F de unidades
		GPIO_WritePin(&handlerSegmentA, SET);
		GPIO_WritePin(&handlerSegmentB, SET);
		GPIO_WritePin(&handlerSegmentC, SET);
		GPIO_WritePin(&handlerSegmentD, SET);
		GPIO_WritePin(&handlerSegmentE, SET);
		GPIO_WritePin(&handlerSegmentF, RESET);
		GPIO_WritePin(&handlerSegmentG, SET);
		break;
	}
	case 8: {
		GPIO_WritePin(&handlerTransistorUnd, SET);
		GPIO_WritePin(&handlerTransistorDec, RESET);
		// Encendemos display B de decenas
		GPIO_WritePin(&handlerSegmentA, SET);
		GPIO_WritePin(&handlerSegmentB, RESET);
		GPIO_WritePin(&handlerSegmentC, SET);
		GPIO_WritePin(&handlerSegmentD, SET);
		GPIO_WritePin(&handlerSegmentE, SET);
		GPIO_WritePin(&handlerSegmentF, SET);
		GPIO_WritePin(&handlerSegmentG, SET);
		break;
	}
	case 9: {
		GPIO_WritePin(&handlerTransistorUnd, SET);
		GPIO_WritePin(&handlerTransistorDec, RESET);
		// Encendemos display C de decenas
		GPIO_WritePin(&handlerSegmentA, SET);
		GPIO_WritePin(&handlerSegmentB, SET);
		GPIO_WritePin(&handlerSegmentC, RESET);
		GPIO_WritePin(&handlerSegmentD, SET);
		GPIO_WritePin(&handlerSegmentE, SET);
		GPIO_WritePin(&handlerSegmentF, SET);
		GPIO_WritePin(&handlerSegmentG, SET);
		break;
	}
	case 10: {
		GPIO_WritePin(&handlerTransistorUnd, RESET);
		GPIO_WritePin(&handlerTransistorDec, SET);
		// Encendemos display D de unidades
		GPIO_WritePin(&handlerSegmentA, SET);
		GPIO_WritePin(&handlerSegmentB, SET);
		GPIO_WritePin(&handlerSegmentC, SET);
		GPIO_WritePin(&handlerSegmentD, RESET);
		GPIO_WritePin(&handlerSegmentE, SET);
		GPIO_WritePin(&handlerSegmentF, SET);
		GPIO_WritePin(&handlerSegmentG, SET);
		break;
	}
	case 11: {
		GPIO_WritePin(&handlerTransistorUnd, RESET);
		GPIO_WritePin(&handlerTransistorDec, SET);
		// Encendemos display C de unidades
		GPIO_WritePin(&handlerSegmentA, SET);
		GPIO_WritePin(&handlerSegmentB, SET);
		GPIO_WritePin(&handlerSegmentC, RESET);
		GPIO_WritePin(&handlerSegmentD, SET);
		GPIO_WritePin(&handlerSegmentE, SET);
		GPIO_WritePin(&handlerSegmentF, SET);
		GPIO_WritePin(&handlerSegmentG, SET);
		break;
	}
	case 12: {
		GPIO_WritePin(&handlerTransistorUnd, RESET);
		GPIO_WritePin(&handlerTransistorDec, SET);
		// Encendemos display B de unidades
		GPIO_WritePin(&handlerSegmentA, SET);
		GPIO_WritePin(&handlerSegmentB, RESET);
		GPIO_WritePin(&handlerSegmentC, SET);
		GPIO_WritePin(&handlerSegmentD, SET);
		GPIO_WritePin(&handlerSegmentE, SET);
		GPIO_WritePin(&handlerSegmentF, SET);
		GPIO_WritePin(&handlerSegmentG, SET);
		break;
	}
	default: {
		__NOP();
		break;
	}
	}
}

/** Función que controla el display según el modo en que se encuentre*/
void display_Control(uint8_t modo, uint8_t contador, uint8_t signo_culebrita, uint8_t estadoUnidades, uint8_t estadoDecenas){
	switch(modo){
	case 0:{ //Culebrita
		display_Culebrita(signo_culebrita);
		break;
	}
	case 1:{ //Contador
		if((~estadoUnidades)&&(estadoDecenas)){
			display_Segun_Digito(contador%10); // Mostrar digito unidades
		}
		else if((~estadoDecenas)&&(estadoUnidades)){
			display_Segun_Digito(contador/10); // Mostrar digito decenas
		}
		else{
			__NOP();
		}
		break;
	}
	default:{
		__NOP();
		break;
	}
	}
}

