/*
 * MotorDriver.c
 *
 *  Created on: 08/04/2024
 *      Author: MillerQuintero2001
 */

#include "MotorDriver.h"

/* Inicializo variables y elementos propios del driver */

// Definimos los handler para el motor de la rueda derecha (Amarillo)
PWM_Handler_t handlerPwmRight 		= {0};
GPIO_Handler_t handlerPinPwmRight 	= {0};
GPIO_Handler_t handlerEnableRight	= {0};
GPIO_Handler_t handlerDirRight 		= {0};
EXTI_Config_t handlerIntRight  		= {0};
GPIO_Handler_t handlerPinIntRight   = {0};

// Definimos los handler para el motor de la rueda izquierda (Azul)
PWM_Handler_t handlerPwmLeft 		= {0};
GPIO_Handler_t handlerPinPwmLeft 	= {0};
GPIO_Handler_t handlerEnableLeft 	= {0};
GPIO_Handler_t handlerDirLeft 		= {0};
EXTI_Config_t handlerIntLeft  		= {0};
GPIO_Handler_t handlerPinIntLeft   	= {0};

// Variables generales
uint32_t counterIntRight = 0;
uint32_t counterIntLeft = 0;
uint16_t period = 400;
uint16_t dutty = 80;
uint8_t interruptsRev = 72;	// Interrupciones por revolución del encoder
float wheelDiameter = 51.55;	// Diámetro promedio de las ruedas en milímetros
float duttyChange = 8;			// Cambio porcentual de dutty mínimo, (en este caso sería 2%)


/** Función de hacer una configuración por defecto de todo, esta es con motores en off, frecuencia 25Hz y Dutty de 20% */
void configMotors(void){

	/* Configuración del motor derecho (amarillo) */

	// Señal inicial PWM
	handlerPwmRight.ptrTIMx			    			= TIM2;
	handlerPwmRight.PWMx_Config.PWMx_Channel	    = PWM_CHANNEL_1;
	handlerPwmRight.PWMx_Config.PWMx_Prescaler 		= BTIMER_PLL_100MHz_SPEED_100us;
	handlerPwmRight.PWMx_Config.PWMx_Period	   		= period;
	handlerPwmRight.PWMx_Config.PWMx_DuttyCicle		= dutty;
	handlerPwmRight.PWMx_Config.PWMx_Polarity		= PWM_POLARITY_ACTIVE_HIGH;
	pwm_Config(&handlerPwmRight);

	// Pin de la señal
	handlerPinPwmRight.pGPIOx								= GPIOA;
	handlerPinPwmRight.GPIO_PinConfig.GPIO_PinNumber	    = PIN_0;
	handlerPinPwmRight.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_ALTFN;
	handlerPinPwmRight.GPIO_PinConfig.GPIO_PinOPType	    = GPIO_OTYPE_PUSHPULL;
	handlerPinPwmRight.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
	handlerPinPwmRight.GPIO_PinConfig.GPIO_PinSpeed			= GPIO_OSPEED_FAST;
	handlerPinPwmRight.GPIO_PinConfig.GPIO_PinAltFunMode	= AF1;
	GPIO_Config(&handlerPinPwmRight);

	// Pin del enable
	handlerEnableRight.pGPIOx                             	= GPIOC;
	handlerEnableRight.GPIO_PinConfig.GPIO_PinNumber      	= PIN_10;
	handlerEnableRight.GPIO_PinConfig.GPIO_PinMode        	= GPIO_MODE_OUT;
	handlerEnableRight.GPIO_PinConfig.GPIO_PinOPType      	= GPIO_OTYPE_PUSHPULL;
	handlerEnableRight.GPIO_PinConfig.GPIO_PinPuPdControl 	= GPIO_PUPDR_NOTHING;
	handlerEnableRight.GPIO_PinConfig.GPIO_PinSpeed       	= GPIO_OSPEED_FAST;
	GPIO_Config(&handlerEnableRight);
	GPIO_WritePin(&handlerEnableRight, MOTOR_OFF);

	// Pin de dirección
	handlerDirRight.pGPIOx                                	= GPIOC;
	handlerDirRight.GPIO_PinConfig.GPIO_PinNumber         	= PIN_12;
	handlerDirRight.GPIO_PinConfig.GPIO_PinMode           	= GPIO_MODE_OUT;
	handlerDirRight.GPIO_PinConfig.GPIO_PinOPType         	= GPIO_OTYPE_PUSHPULL;
	handlerDirRight.GPIO_PinConfig.GPIO_PinPuPdControl    	= GPIO_PUPDR_NOTHING;
	handlerDirRight.GPIO_PinConfig.GPIO_PinSpeed          	= GPIO_OSPEED_FAST;
	GPIO_Config(&handlerDirRight);
	GPIO_WritePin(&handlerDirRight, MOTOR_FORWARD);


	/* Configuración del motor izquierda (azul) */

	// Señal inicial PWM
	handlerPwmLeft.ptrTIMx			    			= TIM2;
	handlerPwmLeft.PWMx_Config.PWMx_Channel	   		= PWM_CHANNEL_2;
	handlerPwmLeft.PWMx_Config.PWMx_Prescaler		= BTIMER_PLL_100MHz_SPEED_100us;
	handlerPwmLeft.PWMx_Config.PWMx_Period	    	= period;
	handlerPwmLeft.PWMx_Config.PWMx_DuttyCicle		= dutty;
	handlerPwmLeft.PWMx_Config.PWMx_Polarity		= PWM_POLARITY_ACTIVE_HIGH;
	pwm_Config(&handlerPwmLeft);

	// Pin de la señal
	handlerPinPwmLeft.pGPIOx								= GPIOA;
	handlerPinPwmLeft.GPIO_PinConfig.GPIO_PinNumber		    = PIN_1;
	handlerPinPwmLeft.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_ALTFN;
	handlerPinPwmLeft.GPIO_PinConfig.GPIO_PinOPType		    = GPIO_OTYPE_PUSHPULL;
	handlerPinPwmLeft.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
	handlerPinPwmLeft.GPIO_PinConfig.GPIO_PinSpeed			= GPIO_OSPEED_FAST;
	handlerPinPwmLeft.GPIO_PinConfig.GPIO_PinAltFunMode	    = AF1;
	GPIO_Config(&handlerPinPwmLeft);

	// Pin del enable
	handlerEnableLeft.pGPIOx                             	= GPIOC;
	handlerEnableLeft.GPIO_PinConfig.GPIO_PinNumber     	= PIN_11;
	handlerEnableLeft.GPIO_PinConfig.GPIO_PinMode        	= GPIO_MODE_OUT;
	handlerEnableLeft.GPIO_PinConfig.GPIO_PinOPType      	= GPIO_OTYPE_PUSHPULL;
	handlerEnableLeft.GPIO_PinConfig.GPIO_PinPuPdControl 	= GPIO_PUPDR_NOTHING;
	handlerEnableLeft.GPIO_PinConfig.GPIO_PinSpeed       	= GPIO_OSPEED_FAST;
	GPIO_Config(&handlerEnableLeft);
	GPIO_WritePin(&handlerEnableLeft, MOTOR_OFF);

	// Pin de direccion
	handlerDirLeft.pGPIOx                                	= GPIOD;
	handlerDirLeft.GPIO_PinConfig.GPIO_PinNumber         	= PIN_2;
	handlerDirLeft.GPIO_PinConfig.GPIO_PinMode           	= GPIO_MODE_OUT;
	handlerDirLeft.GPIO_PinConfig.GPIO_PinOPType         	= GPIO_OTYPE_PUSHPULL;
	handlerDirLeft.GPIO_PinConfig.GPIO_PinPuPdControl    	= GPIO_PUPDR_NOTHING;
	handlerDirLeft.GPIO_PinConfig.GPIO_PinSpeed          	= GPIO_OSPEED_FAST;
	GPIO_Config(&handlerDirLeft);
	GPIO_WritePin(&handlerDirLeft, MOTOR_FORWARD);


	/* Configuración de las EXTI */

	// Encoder derecha (amarillo)
	handlerPinIntRight.pGPIOx                            	= GPIOC;
	handlerPinIntRight.GPIO_PinConfig.GPIO_PinNumber      	= PIN_1;
	handlerPinIntRight.GPIO_PinConfig.GPIO_PinMode       	= GPIO_MODE_IN;
	handlerPinIntRight.GPIO_PinConfig.GPIO_PinPuPdControl 	= GPIO_PUPDR_NOTHING;
	GPIO_Config(&handlerPinIntRight);

	handlerIntRight.edgeType     	= EXTERNAL_INTERRUPT_RISING_EDGE;
	handlerIntRight.pGPIOHandler 	= &handlerPinIntRight;
	extInt_Config(&handlerIntRight);

	// Encoder izquierda (azul)
	handlerPinIntLeft.pGPIOx                      			= GPIOC;
	handlerPinIntLeft.GPIO_PinConfig.GPIO_PinNumber     	= PIN_3;
	handlerPinIntLeft.GPIO_PinConfig.GPIO_PinMode        	= GPIO_MODE_IN;
	handlerPinIntLeft.GPIO_PinConfig.GPIO_PinPuPdControl 	= GPIO_PUPDR_NOTHING;
	GPIO_Config(&handlerPinIntLeft);

	handlerIntLeft.edgeType     	= EXTERNAL_INTERRUPT_RISING_EDGE;
	handlerIntLeft.pGPIOHandler 	= &handlerPinIntLeft;
	extInt_Config(&handlerIntLeft);

}


/** Función encargada de modificar la frecuencia y %duttyCycle de ambos motores */
void setMotorSignals(uint8_t freqHz, uint8_t duttyPer){
	period = (uint16_t)(10000.0*(1.0/freqHz));
	dutty = (uint16_t)(period*(duttyPer/100.0));
	duttyChange = period*0.02;
	updatePeriod(&handlerPwmRight, period);
	updatePeriod(&handlerPwmLeft, period);
	updateDuttyCycle(&handlerPwmRight, dutty);
	updateDuttyCycle(&handlerPwmLeft, dutty);
}


/** Función que restaura la configuración a un estado conocido, dirección para linea recta y polaridad activa-alta */
void defaultMove(void){
	setPolarity(&handlerPwmRight, PWM_POLARITY_ACTIVE_HIGH);
	setPolarity(&handlerPwmLeft, PWM_POLARITY_ACTIVE_HIGH);
	GPIO_WritePin(&handlerDirRight, MOTOR_FORWARD);
	GPIO_WritePin(&handlerDirLeft, MOTOR_FORWARD);
}

/** Función para iniciar el movimiento de los motores */
void startMove(void){
	counterIntRight = 0;
	counterIntLeft = 0;
	startPwmSignal(&handlerPwmRight);
	startPwmSignal(&handlerPwmLeft);
	GPIO_WritePin(&handlerEnableRight, MOTOR_ON);
	GPIO_WritePin(&handlerEnableLeft, MOTOR_ON);
}

/** Función para detener el movimiento de los motores */
void stopMove(void){
	GPIO_WritePin(&handlerEnableRight, MOTOR_OFF);
	GPIO_WritePin(&handlerEnableLeft, MOTOR_OFF);
	stopPwmSignal(&handlerPwmRight);
	stopPwmSignal(&handlerPwmLeft);
	counterIntRight = 0;
	counterIntLeft = 0;
}

/** Función para realizar un recorrido en linea recta con control */
void straightLine(uint16_t distance_in_mm){
	defaultMove();
	uint32_t goalInterrupts = interruptsRev*((float)(distance_in_mm)/wheelDiameter);
	uint32_t ticksRight = 0;
	uint32_t ticksLeft = 0;
	uint32_t differenceRight = 0;
	uint32_t differenceLeft = 0;
	counterIntRight = 0;
	counterIntLeft = 0;
	uint16_t duttyRight = dutty;
	uint16_t duttyLeft = dutty;
	uint32_t previousTicksRight = counterIntRight;
	uint32_t previousTicksLeft = counterIntLeft;
	startMove();
	while((counterIntRight < goalInterrupts)&&(counterIntLeft < goalInterrupts)){
		// A la frecuencia que ocurre el ciclo while se muestrean las interrupciones
		ticksRight = counterIntRight;
		ticksLeft = counterIntLeft;

		updateDuttyCycle(&handlerPwmRight, duttyRight);
		updateDuttyCycle(&handlerPwmLeft, duttyLeft);

		differenceRight = ticksRight - previousTicksRight;
		differenceLeft = ticksLeft - previousTicksLeft;

		previousTicksRight = ticksRight;
		previousTicksLeft = ticksLeft;

		if(differenceLeft > differenceRight){
			duttyRight += duttyChange;
			duttyLeft -= duttyChange;
		}
		else if(differenceRight > differenceLeft){
			duttyRight -= duttyChange;
			duttyLeft += duttyChange;
		}
	}
	stopMove();
}

/** Función para rotar */
void rotation(uint8_t direction, uint16_t degress){
	stopMove();
	defaultMove();
	if(direction == MOVEMENT_CW){
		// Cambiamos la polaridad del motor del lado derecho (amarillo)
		setPolarity(&handlerPwmRight,PWM_POLARITY_ACTIVE_LOW);
		GPIO_WritePin(&handlerDirRight, MOTOR_BACK);
	}
	else if(direction == MOVEMENT_CCW){
		// Cambiamos la polaridad del motor del lado izquierdo (azul)
		setPolarity(&handlerPwmLeft,PWM_POLARITY_ACTIVE_LOW);
		GPIO_WritePin(&handlerDirLeft, MOTOR_BACK);
	}
	else{
		__NOP();
	}
	// Falta hacer lo demás
}

/** Función para realizar el cuadrado en la dirección y con la medida indicada */
void square(uint8_t direction, uint16_t side_in_mm){
	straightLine(side_in_mm);
	rotation(direction, 90);
	straightLine(side_in_mm);
	rotation(direction, 90);
	straightLine(side_in_mm);
	rotation(direction, 90);
	straightLine(side_in_mm);
	rotation(direction, 90);
}


void callback_extInt1 (void){
	counterIntRight++;
}

void callback_extInt3 (void){
	counterIntLeft++;
}

