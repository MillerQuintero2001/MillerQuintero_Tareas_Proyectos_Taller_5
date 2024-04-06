/**
 ******************************************************************************
 * @file           : BBasicMovement.c
 * @author         : Miller Quintero - miquinterog@unal.edu.co
 * @brief          : Pruebas para el Oppy
 ******************************************************************************
 * Lineas recta y rotaciones
 ******************************************************************************
 */

#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>

#include "stm32f4xx.h"

// Librerias de Drivers a utilizar
#include "GPIOxDriver.h"
#include "PLLDriver.h"
#include "PwmDriver.h"
#include "CMDxDriver.h"
#include "USARTxDriver.h"
#include "BasicTimer.h"
#include "ExtiDriver.h"

//Definimos los handlers para el Blinky
GPIO_Handler_t led_state = {0};
BasicTimer_Handler_t timerLed = {0};

// Definimos los handler y los elementos para el conteo de los encoder
uint16_t period = 200;
uint8_t indexes = 0;
uint16_t counterExti = 0;
uint16_t dataAzul[30] = {0};
uint16_t dataAmarillo[30] = {0};
uint8_t flagTimerSample = 0;
uint8_t flag25Hz = 0;
uint8_t flag50Hz = 0;
uint8_t flag100Hz = 0;
BasicTimer_Handler_t handlerSampleTimer = {0};

// Definimos los handler para el motor amarillo
PWM_Handler_t handlerPwmAmarillo = {0};
GPIO_Handler_t handlerPinPwmAmarillo = {0};
GPIO_Handler_t handlerEnableAmarillo = {0};
GPIO_Handler_t handlerDirAmarillo = {0};

// Definimos los handler para el motor Azul
PWM_Handler_t handlerPwmAzul = {0};
GPIO_Handler_t handlerPinPwmAzul = {0};
GPIO_Handler_t handlerEnableAzul = {0};
GPIO_Handler_t handlerDirAzul = {0};


//handler de los EXTI
EXTI_Config_t pruebaBotonAzul  = {0};
GPIO_Handler_t BotonAzul      = {0};
uint16_t counterB= 0;

EXTI_Config_t pruebaBotonAmarillo  = {0};
GPIO_Handler_t BotonAmarillo      = {0};
uint16_t counterA = 0;

float_t dutty = 10;
float_t newdutty = 0;

// PRUEBA MUESTREO
bool flag = false;
uint8_t counter = 1;


char bufferMandar[64] = "";

//Cabeceras de las funciones
void initSystem(void);
void defaultMov(void);
void lineaRecta(void);
void iniciarMov(void);
void stopMov(void);
void aroundCW(void);
void aroundCCW(void);

int main(void)
{
	initSystem();


    //*Loop forever*/
	while(1){
//		cmdIngresoConsola();

		//CODIGO PRUEBA
		if(flag){
			GPIO_WritePin(&handlerEnableAmarillo, SET);
			GPIO_WritePin(&handlerEnableAzul, SET);
			stopPwmSignal(&handlerPwmAmarillo);
			stopPwmSignal(&handlerPwmAzul);
			flag = false;
			counter++;
			if(dutty < 30){
				sprintf(bufferMandar,"%.2f%% \t  %u \t %u \t \n",dutty, counterA,counterB);
				startPwmSignal(&handlerPwmAmarillo);
				startPwmSignal(&handlerPwmAzul);
				GPIO_WritePin(&handlerEnableAmarillo, RESET);
				GPIO_WritePin(&handlerEnableAzul, RESET);
				counterA = 0;
				counterB = 0;
				//writeMSGCMD(bufferMandar);
			}
			if(counter == 11){
				dutty++;
				updateDuttyCycle(&handlerPwmAmarillo,period*(dutty/100));
				updateDuttyCycle(&handlerPwmAzul, period*(dutty/100));
				counter = 1;
			}

		}

	}
}

//Funcion de configuracion del Hardware necesario
void initSystem(void){

	configPLL(100);

	/* Activamos el Coprocesador Matemático - FPU */
	SCB->CPACR |= (0XF << 20);
	// Definimos los handler del Led de estado
	led_state.pGPIOx                             = GPIOC;
	led_state.GPIO_PinConfig.GPIO_PinNumber      = 5;
	led_state.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_OUT;
	led_state.GPIO_PinConfig.GPIO_PinOPType      = GPIO_OTYPE_PUSHPULL;
	led_state.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_NOTHING;
	led_state.GPIO_PinConfig.GPIO_PinSpeed       = GPIO_OSPEED_LOW;
	GPIO_Config(&led_state);
	GPIO_WritePin(&led_state, SET);

	timerLed.ptrTIMx                             = TIM4;
	timerLed.TIMx_Config.TIMx_mode               = BTIMER_MODE_UP;
	timerLed.TIMx_Config.TIMx_speed              = BTIMER_PLL_100MHz_SPEED_100us;
	timerLed.TIMx_Config.TIMx_period             = 2500;
	timerLed.TIMx_Config.TIMx_interruptEnable    = BTIMER_INTERRUP_ENABLE;
	BasicTimer_Config(&timerLed);

	// Configuracion del handler para el muestreo
	handlerSampleTimer.ptrTIMx							= TIM5;
	handlerSampleTimer.TIMx_Config.TIMx_mode			= BTIMER_MODE_UP;
	handlerSampleTimer.TIMx_Config.TIMx_speed 			= BTIMER_PLL_100MHz_SPEED_100us;
	handlerSampleTimer.TIMx_Config.TIMx_period			= 100000;
	handlerSampleTimer.TIMx_Config.TIMx_interruptEnable	= BTIMER_INTERRUP_ENABLE;
	BasicTimer_Config(&handlerSampleTimer);

	/* Configuración del PWM motor Amarillo*/

	// Pin de la señal
	handlerPinPwmAmarillo.pGPIOx								= GPIOA;
	handlerPinPwmAmarillo.GPIO_PinConfig.GPIO_PinNumber		    = PIN_0;
	handlerPinPwmAmarillo.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_ALTFN;
	handlerPinPwmAmarillo.GPIO_PinConfig.GPIO_PinOPType		    = GPIO_OTYPE_PUSHPULL;
	handlerPinPwmAmarillo.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
	handlerPinPwmAmarillo.GPIO_PinConfig.GPIO_PinSpeed			= GPIO_OSPEED_FAST;
	handlerPinPwmAmarillo.GPIO_PinConfig.GPIO_PinAltFunMode	    = AF1;
	GPIO_Config(&handlerPinPwmAmarillo);

	// Configuración inicial PWM
	handlerPwmAmarillo.ptrTIMx			    				= TIM2;
	handlerPwmAmarillo.PWMx_Config.PWMx_Channel	    		= PWM_CHANNEL_1;
	handlerPwmAmarillo.PWMx_Config.PWMx_Prescaler 			= BTIMER_PLL_100MHz_SPEED_100us;
	handlerPwmAmarillo.PWMx_Config.PWMx_Period		   		= period;
	handlerPwmAmarillo.PWMx_Config.PWMx_DuttyCicle			= period*(dutty/100);
	handlerPwmAmarillo.PWMx_Config.PWMx_Polarity			= PWM_POLARITY_ACTIVE_HIGH;
	pwm_Config(&handlerPwmAmarillo);

	enableOutput(&handlerPwmAmarillo);
	startPwmSignal(&handlerPwmAmarillo);

	// Configuracion del pin del enable y del pin de la direccion
	handlerEnableAmarillo.pGPIOx                             = GPIOC;
	handlerEnableAmarillo.GPIO_PinConfig.GPIO_PinNumber      = PIN_10;
	handlerEnableAmarillo.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_OUT;
	handlerEnableAmarillo.GPIO_PinConfig.GPIO_PinOPType      = GPIO_OTYPE_PUSHPULL;
	handlerEnableAmarillo.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_NOTHING;
	handlerEnableAmarillo.GPIO_PinConfig.GPIO_PinSpeed       = GPIO_OSPEED_FAST;
	GPIO_Config(&handlerEnableAmarillo);
	GPIO_WritePin(&handlerEnableAmarillo,SET);

	handlerDirAmarillo.pGPIOx                                = GPIOC;
	handlerDirAmarillo.GPIO_PinConfig.GPIO_PinNumber         = PIN_12;
	handlerDirAmarillo.GPIO_PinConfig.GPIO_PinMode           = GPIO_MODE_OUT;
	handlerDirAmarillo.GPIO_PinConfig.GPIO_PinOPType         = GPIO_OTYPE_PUSHPULL;
	handlerDirAmarillo.GPIO_PinConfig.GPIO_PinPuPdControl    = GPIO_PUPDR_NOTHING;
	handlerDirAmarillo.GPIO_PinConfig.GPIO_PinSpeed          = GPIO_OSPEED_FAST;
	GPIO_Config(&handlerDirAmarillo);
	GPIO_WritePin(&handlerDirAmarillo, RESET);



	/* Configuración del PWM motor Azul*/

	// Pin de la señal
	handlerPinPwmAzul.pGPIOx								= GPIOA;
	handlerPinPwmAzul.GPIO_PinConfig.GPIO_PinNumber		    = PIN_1;
	handlerPinPwmAzul.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_ALTFN;
	handlerPinPwmAzul.GPIO_PinConfig.GPIO_PinOPType		    = GPIO_OTYPE_PUSHPULL;
	handlerPinPwmAzul.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
	handlerPinPwmAzul.GPIO_PinConfig.GPIO_PinSpeed			= GPIO_OSPEED_FAST;
	handlerPinPwmAzul.GPIO_PinConfig.GPIO_PinAltFunMode	    = AF1;
	GPIO_Config(&handlerPinPwmAzul);

	// Configuración inicial PWM
	handlerPwmAzul.ptrTIMx			    			= TIM2;
	handlerPwmAzul.PWMx_Config.PWMx_Channel	  	 	= PWM_CHANNEL_2;
	handlerPwmAzul.PWMx_Config.PWMx_Prescaler 		= BTIMER_PLL_100MHz_SPEED_100us;
	handlerPwmAzul.PWMx_Config.PWMx_Period	   		= period;
	handlerPwmAzul.PWMx_Config.PWMx_DuttyCicle		= period*(dutty/100);
	handlerPwmAzul.PWMx_Config.PWMx_Polarity	    = PWM_POLARITY_ACTIVE_HIGH;
	pwm_Config(&handlerPwmAzul);

	enableOutput(&handlerPwmAzul);
	startPwmSignal(&handlerPwmAzul);

	// Configuracion del pin del enable y del pin de la direccion
	handlerEnableAzul.pGPIOx                             = GPIOC;
	handlerEnableAzul.GPIO_PinConfig.GPIO_PinNumber      = PIN_11;
	handlerEnableAzul.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_OUT;
	handlerEnableAzul.GPIO_PinConfig.GPIO_PinOPType      = GPIO_OTYPE_PUSHPULL;
	handlerEnableAzul.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_NOTHING;
	handlerEnableAzul.GPIO_PinConfig.GPIO_PinSpeed       = GPIO_OSPEED_FAST;
	GPIO_Config(&handlerEnableAzul);
	GPIO_WritePin(&handlerEnableAzul, SET);

	handlerDirAzul.pGPIOx                                = GPIOD;
	handlerDirAzul.GPIO_PinConfig.GPIO_PinNumber         = PIN_2;
	handlerDirAzul.GPIO_PinConfig.GPIO_PinMode           = GPIO_MODE_OUT;
	handlerDirAzul.GPIO_PinConfig.GPIO_PinOPType         = GPIO_OTYPE_PUSHPULL;
	handlerDirAzul.GPIO_PinConfig.GPIO_PinPuPdControl    = GPIO_PUPDR_NOTHING;
	handlerDirAzul.GPIO_PinConfig.GPIO_PinSpeed          = GPIO_OSPEED_FAST;
	GPIO_Config(&handlerDirAzul);
	GPIO_WritePin(&handlerDirAzul, RESET);


	// PRUEBA EXTI
	BotonAzul.pGPIOx                             = GPIOC;
	BotonAzul.GPIO_PinConfig.GPIO_PinNumber      = PIN_3; // C1, C3, C7
	BotonAzul.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_IN;
	BotonAzul.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_NOTHING;
	GPIO_Config(&BotonAzul);

	pruebaBotonAzul.edgeType     = EXTERNAL_INTERRUPT_RISING_EDGE;
	pruebaBotonAzul.pGPIOHandler = &BotonAzul;
	extInt_Config(&pruebaBotonAzul);

	BotonAmarillo.pGPIOx                             = GPIOC;
	BotonAmarillo.GPIO_PinConfig.GPIO_PinNumber      = PIN_1; // C1, C3, C7
	BotonAmarillo.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_IN;
	BotonAmarillo.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_NOTHING;
	GPIO_Config(&BotonAmarillo);

	pruebaBotonAmarillo.edgeType     = EXTERNAL_INTERRUPT_RISING_EDGE;
	pruebaBotonAmarillo.pGPIOHandler = &BotonAmarillo;
	extInt_Config(&pruebaBotonAmarillo);

	commandConfig(CMD_USART1);

}

/** Interrupción del timer LED */
void BasicTimer4_Callback(void){
	GPIOxTooglePin(&led_state);
}

/** Interrupción del timer del conteo */
void BasicTimer5_Callback(void){
//	if(flag){
//		sprintf(bufferMandar,"%.2f%% \t  %u \t %u \t \n",dutty, counterA,counterB);
//		writeMSGCMD(bufferMandar);
//		counterA = 0;
//		counterB = 0;
//	}
	flag = true;
}

void callback_extInt3 (void){
	counterB++;
}

void callback_extInt1 (void){
	counterA++;
}


// REGRESAMOS A LA CONFIGURACION INICAL DE LOS MOTORES
void defaultMov(void){
	setPolarity(&handlerPwmAmarillo, PWM_POLARITY_ACTIVE_HIGH);
	setPolarity(&handlerPwmAzul, PWM_POLARITY_ACTIVE_HIGH);
	GPIO_WritePin(&handlerDirAmarillo, RESET);
	GPIO_WritePin(&handlerDirAzul, RESET);
}

// INICIAMOS EL MOVIMIENTO DEL ROBOT
void iniciarMov(void){
	startPwmSignal(&handlerPwmAmarillo);
	startPwmSignal(&handlerPwmAzul);
	GPIO_WritePin(&handlerEnableAmarillo, RESET);
	GPIO_WritePin(&handlerEnableAzul, RESET);
	flag = true;
}

// DETENEMOS EL MOVIMIENTO DEL ROBOT
void stopMov(void){
	GPIO_WritePin(&handlerEnableAmarillo, SET);
	GPIO_WritePin(&handlerEnableAzul, SET);
	stopPwmSignal(&handlerPwmAmarillo);
	stopPwmSignal(&handlerPwmAzul);
	flag = false;
}

// EL ROBOT SE MUEVE EN LINEA RECTA
void lineaRecta(void){
	// PRIMERO DETENEMOS EL ROBOT
	stopMov();
	// LO DEVOLOVEMOS A SU ESTADO INICIAL
	defaultMov();
	// ACTUALIZAMOS EL DUTTY CICLE DE LOS MOTORES
	updateDuttyCycle(&handlerPwmAmarillo, period*(dutty/100));
	updateDuttyCycle(&handlerPwmAzul, period*(dutty/100));
	// IMPRIMIMOS EL SIGUIENTE MENSAJE
	//writeMSGCMD("El robot esta listo para moverse en linea recta, ingrese el comando start\n");
}

// EL ROBOT GIRA SOBRE SU PROPIO EJE DIRECCION CW
void aroundCW(void){
	// PRIMERO DETENEMOS EL ROBOT
	stopMov();
	// LO DEVOLVEMOS A SU ESTADO INICIAL
	defaultMov();
	// CAMBIAMOS LA POLARIDAD DEL MOTOR AMARILLO
	setPolarity(&handlerPwmAmarillo,PWM_POLARITY_ACTIVE_LOW);
	GPIO_WritePin(&handlerDirAmarillo, SET);
	// IMPRIMIMOS EL SIGUIENTE MENSAJE
	//writeMSGCMD("El robot esta listo para girar cw sobre su eje, ingrese el comando start\n");
}

// EL ROBOR GIRA SOBRE SU PROPIO EJE DIRECCION CCW
void aroundCCW(void){
	// PRIMERO DETENEMOS EL ROBOT
	defaultMov();
	// LO DEVOLVEMOS A SU ESTADO INICIAL
	stopMov();
	// CAMBIAMOS LA POLARIDAD DEL MOTOR AZUL
	setPolarity(&handlerPwmAzul,PWM_POLARITY_ACTIVE_LOW);
	GPIO_WritePin(&handlerDirAzul, SET);
	// IMPRIMIMOS EL SIGUIENTE MENSAJE
	//writeMSGCMD("El robot esta listo para girar ccw sobre su eje, ingrese el comando start\n");
}
// COMANDO START
void comando1(void){
	iniciarMov();
	//writeMSGCMD("El robot comenzo a moverse\n");
}

//COMANDO STOP
void comando2(void){
	stopMov();
	//writeMSGCMD("El robot se detuvo\n");
}

void comando3(void){
	aroundCW();
}

void comando4(void){
	aroundCCW();
}

void comando5(void){
	lineaRecta();
}

void comando6(void){
	//Detenemos el robot
	stopMov();
	// Actualizamos el nuevo dutty cicle
	newdutty = (float_t)calPWM/100;
	updateDuttyCycle(&handlerPwmAmarillo, period*((dutty+newdutty)/100));
	updateDuttyCycle(&handlerPwmAzul, period*(dutty/100));
	// Imprimimos el emnsaje
	//writeMSGCMD("EL coeficiente de calibracion ha cambiado\n");
}
