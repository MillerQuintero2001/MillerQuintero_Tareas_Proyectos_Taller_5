/*
 * MotorDriver.c
 *
 *  Created on: 08/04/2024
 *      Author: MillerQuintero2001
 */

#include "MotorDriver.h"
#include "MPU6050.h"

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

// Definimos los handler para el seguidor de línea y el de distancia infrarroja
EXTI_Config_t handlerIntDistance 		= {0};
GPIO_Handler_t handlerPinIntDistance 	= {0};

// Variables generales del Oppy
bool flagMove = false;
uint32_t counterIntRight = 0;
uint32_t counterIntLeft = 0;
uint16_t period = 40000;
uint16_t duttyBaseRight = 12000;
uint16_t duttyBaseLeft = 12725;
uint8_t interruptsRev = 120;			// Interrupciones por revolución del encoder (Depende de las aberturas del encoder y los flancos)
float wheelDiameter = 51.725f;			// Diámetro promedio de las ruedas
float wheelPerimeter = M_PI*51.725f;	// Perímetro con promedio diámetro de las ruedas en milímetros
float distanceAxis = 106.50f;			// Distance entre ruedas (eje) (anteriormente era 109 mm)

// Variables relacionadas con el PID
float q0 = 0.0f;					// Constante de PID discreto
float q1 = 0.0f;					// Constante de PID discreto
float q2 = 0.0f;					// Constante de PID discreto
float u_control = 0.0f; 			// Acción de control actual
float u_1_control = 0.0f;			// Accioń de control previadutty
float error = 0.0f;					// Error actual
float error_1 = 0.0f;				// Error una muestra antes
float error_2 = 0.0f;				// Error dos muestras antes
float timeSample = 0.020f;			// Tiempo de muestreo [s]

/* Private functions prototypes */
void constraintControlPID(float* uControl, float maxChange);
void controlActionPID(void);

/** Función de hacer una configuración por defecto, esta es con motores en off, frecuencia 25Hz y Dutty de 20% */
void configMotors(void){

	/* Activamos el Coprocesador Matemático - FPU */
	SCB->CPACR |= (0XF << 20);

	/* Configuración del motor derecho (amarillo) */

	// Señal inicial PWM
	handlerPwmRight.ptrTIMx			    			= TIM2;
	handlerPwmRight.PWMx_Config.PWMx_Channel	    = PWM_CHANNEL_1;
	handlerPwmRight.PWMx_Config.PWMx_Prescaler 		= BTIMER_PLL_100MHz_SPEED_1us;
	handlerPwmRight.PWMx_Config.PWMx_Period	   		= period;
	handlerPwmRight.PWMx_Config.PWMx_DuttyCicle		= duttyBaseRight;
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
	handlerPwmLeft.PWMx_Config.PWMx_Prescaler		= BTIMER_PLL_100MHz_SPEED_1us;
	handlerPwmLeft.PWMx_Config.PWMx_Period	    	= period;
	handlerPwmLeft.PWMx_Config.PWMx_DuttyCicle		= duttyBaseLeft;
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
	handlerPinIntRight.GPIO_PinConfig.GPIO_PinPuPdControl 	= GPIO_PUPDR_PULLUP;
	handlerPinIntRight.GPIO_PinConfig.GPIO_PinSpeed			= GPIO_OSPEED_HIGH;
	GPIO_Config(&handlerPinIntRight);

	handlerIntRight.edgeType     		= EXTERNAL_INTERRUPT_BOTH_EDGE;
	handlerIntRight.priorityInterrupt	= 6;
	handlerIntRight.pGPIOHandler 		= &handlerPinIntRight;
	extInt_Config(&handlerIntRight);

	// Encoder izquierda (azul)
	handlerPinIntLeft.pGPIOx                      			= GPIOC;
	handlerPinIntLeft.GPIO_PinConfig.GPIO_PinNumber     	= PIN_3;
	handlerPinIntLeft.GPIO_PinConfig.GPIO_PinMode        	= GPIO_MODE_IN;
	handlerPinIntLeft.GPIO_PinConfig.GPIO_PinPuPdControl 	= GPIO_PUPDR_PULLUP;
	handlerPinIntLeft.GPIO_PinConfig.GPIO_PinSpeed			= GPIO_OSPEED_HIGH;
	GPIO_Config(&handlerPinIntLeft);

	handlerIntLeft.edgeType     		= EXTERNAL_INTERRUPT_BOTH_EDGE;
	handlerIntLeft.priorityInterrupt	= 6;
	handlerIntLeft.pGPIOHandler 		= &handlerPinIntLeft;
	extInt_Config(&handlerIntLeft);

	// Sensor de distancia infrarrrojo
	handlerPinIntDistance.pGPIOx								= GPIOC;
	handlerPinIntDistance.GPIO_PinConfig.GPIO_PinNumber			= PIN_0;
	handlerPinIntDistance.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_IN;
	handlerPinIntDistance.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
	GPIO_Config(&handlerPinIntDistance);

	handlerIntDistance.edgeType				= EXTERNAL_INTERRUPT_RISING_EDGE;
	handlerIntDistance.priorityInterrupt	= 6;
	handlerIntDistance.pGPIOHandler			= &handlerPinIntDistance;
	extInt_Config(&handlerIntDistance);

	configMPU6050();
}


/** Función encargada de modificar la frecuencia y %duttyCycle de ambos motores */
void setSignals(uint8_t freqHz, uint8_t duttyPer){
	period = (uint16_t)(1000000.00f*(1.00f/((float)freqHz)));
	duttyBaseRight = (uint16_t)(period*(((float)duttyPer)/100.00f));
	duttyBaseLeft = (uint16_t)(period*(((float)duttyPer)/100.00f));
	updatePeriod(&handlerPwmRight, period);
	updatePeriod(&handlerPwmLeft, period);
	updateDuttyCycle(&handlerPwmRight, duttyBaseRight);
	updateDuttyCycle(&handlerPwmLeft, duttyBaseLeft);
}

/** Function that changes dutty base value for each wheel */
void changeBaseDutty(uint16_t duttyRight, uint16_t duttyLeft){
	duttyBaseRight = duttyRight;
	duttyBaseLeft = duttyLeft;
	updateDuttyCycle(&handlerPwmRight, duttyBaseRight);
	updateDuttyCycle(&handlerPwmLeft, duttyBaseLeft);
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
	flagMove = true;
	counterIntRight = 0;
	counterIntLeft = 0;
	startPwmSignal(&handlerPwmRight);
	startPwmSignal(&handlerPwmLeft);
	GPIO_WritePin(&handlerEnableRight, MOTOR_ON);
	GPIO_WritePin(&handlerEnableLeft, MOTOR_ON);
}

/** Función para detener el movimiento de los motores */
void stopMove(void){
	flagMove = false;
	GPIO_WritePin(&handlerEnableRight, MOTOR_OFF);
	GPIO_WritePin(&handlerEnableLeft, MOTOR_OFF);
	stopPwmSignal(&handlerPwmRight);
	stopPwmSignal(&handlerPwmLeft);
	counterIntRight = 0;
	counterIntLeft = 0;
}


/** Función para desplazar el Oppy en un segmento indicado para el A* pathfinding */
void pathSegment(float distance_in_mm){
	defaultMove();
	updateDuttyCycle(&handlerPwmRight, DUTTY_RIGHT_BASE);
	updateDuttyCycle(&handlerPwmLeft, DUTTY_LEFT_BASE);
	uint32_t goalInterrupts = (uint32_t)(((float)interruptsRev)*(distance_in_mm/wheelPerimeter));
	counterIntRight = 0;
	counterIntLeft = 0;
	startMove();
	while((counterIntRight < goalInterrupts)&&(counterIntLeft < goalInterrupts)&&(flagMove)){
		__NOP();
	}
	stopMove();
}

/** Function that configures PID with proportional constant 'kp', Integrative Time 'ti', Derivative Time 'ti' and Time Sample 'ts' in seconds */
void configPID(float kp, float ti, float td, float ts){
	timeSample = ts;
	q0 = kp*(1.00f+(ts/(2.00f*ti))+(td/ts));
	q1 = -kp*(1.00f-(ts/(2.00f*ti))+((2.00f*td)/ts));
	q2 = (kp*td)/ts;
	handlerSampleTimer.TIMx_Config.TIMx_period = (uint32_t)(ts*10000.00f);
	BasicTimer_Config(&handlerSampleTimer);
}

/* Function to do a straight line with PID angle*/
float straightLinePID(uint16_t distance_in_mm){
	// First, initialize the varibles employeed
	//uint32_t goalInterrupts = interruptsRev*((float)(distance_in_mm)/wheelPerimeter);
	resetMPU6050();
	float dataAngle = 0.00f;
	float previousDataAngle = 0.00f;
	float offsetAngularVelocity = getGyroscopeOffset(200);
	float totalCurrentAngle = 0.00f;
	float differentialCurrentAngle = 0.00f;
	uint32_t counterPreviousRight = 0;
	uint32_t counterPreviousLeft = 0;
	float differentialDistance = 0.00f;
	float currentDistanceX = 0.00f;
	counterIntRight = 0;
	counterIntLeft = 0;
	uint32_t saveCounterIntRight = 0;
	uint32_t saveCounterIntLeft = 0;

	// Initial set-up
	defaultMove();
	updateDuttyCycle(&handlerPwmRight, DUTTY_RIGHT_BASE);
	updateDuttyCycle(&handlerPwmLeft, DUTTY_LEFT_BASE);
	startBasicTimer(&handlerSampleTimer);
	startMove();

	while((currentDistanceX < distance_in_mm)&&(flagMove)){
		if(flagData){
			// Take angular velocity in the time sample and multiply it by time sample to get the angle in that time
			dataAngle = (getGyroscopeData() - offsetAngularVelocity)*timeSample;
			// Sum with totalCurrentAngle to update the real current angle of the Oppy
			totalCurrentAngle += dataAngle;

			// Take the difference between previous and current data angle to get the differential angle and can calculate X and Y
			differentialCurrentAngle = previousDataAngle + dataAngle;

			differentialDistance = (((counterIntLeft - counterPreviousLeft)+(counterIntRight - counterPreviousRight))/2.0f)*(M_PI*51.725f/120.0f);
			currentDistanceX += differentialDistance*cosf(differentialCurrentAngle*M_PI/180.0f);

			previousDataAngle = dataAngle;

			counterPreviousRight = counterIntRight;
			counterPreviousLeft = counterIntLeft;

			// Calculate the error for input PID, is like this because the setPointAngle is equal to zero
			error = -totalCurrentAngle;
			if(fabsf(error) >= 10){
				saveCounterIntRight = counterIntRight;
				saveCounterIntLeft = counterIntLeft;
				flagData = false;
				stopBasicTimer(&handlerSampleTimer);
				stopMove();
				getGyroscopeOffset(50);
				rotation((error < 0) ? (MOVEMENT_CCW):(MOVEMENT_CW), (uint16_t)(fabsf(totalCurrentAngle)));
				resetMPU6050();
				totalCurrentAngle = 0.00f;
				offsetAngularVelocity = getGyroscopeOffset(200);
				defaultMove();
				updateDuttyCycle(&handlerPwmRight, duttyBaseRight);
				updateDuttyCycle(&handlerPwmLeft, duttyBaseLeft);
				startBasicTimer(&handlerSampleTimer);
				startMove();
				counterIntRight = saveCounterIntRight;
				counterIntLeft = saveCounterIntLeft;
				u_control = 0.0f;
				u_1_control = 0.0f;
				error = 0.0f;
				error_1 = 0.0f;
				error_2 = 0.0f;
			}
			else{
				controlActionPID();
				flagData = false;
			}
		}
		else{
			__NOP();
		}

	}
	// Return to the initial state
	stopMove();
	stopBasicTimer(&handlerSampleTimer);
	flagData = false;
	u_control = 0.0f;
	u_1_control = 0.0f;
	error = 0.0f;
	error_1 = 0.0f;
	error_2 = 0.0f;
	return totalCurrentAngle;
}


/** Función para rotar sin indicar dirección */
void rotateOppy(int16_t degrees){
	stopMove();
	defaultMove();
	updateDuttyCycle(&handlerPwmRight, DUTTY_RIGHT_ROTATION);
	updateDuttyCycle(&handlerPwmLeft, DUTTY_LEFT_ROTATION);
	if(degrees < 0){
		// Cambiamos la polaridad del motor del lado derecho (amarillo)
		setPolarity(&handlerPwmRight,PWM_POLARITY_ACTIVE_LOW);
		GPIO_WritePin(&handlerDirRight, MOTOR_BACK);
	}
	else if(degrees > 0){
		// Cambiamos la polaridad del motor del lado izquierdo (azul)
		setPolarity(&handlerPwmLeft,PWM_POLARITY_ACTIVE_LOW);
		GPIO_WritePin(&handlerDirLeft, MOTOR_BACK);
	}
	else{
		__NOP();
	}
	// Calculamos la cantidad de interrupciones para conseguir la rotación
	uint16_t goalInterrupts = (uint16_t)(((float)interruptsRev)*(((float)(abs((int)degrees)))/360.0f)*(distanceAxis/wheelDiameter));
	counterIntRight = 0;
	counterIntLeft = 0;
	startMove();
	// Hacemos un ciclo que no haga nada hasta alcanzar las interrupciones
	while((counterIntRight < goalInterrupts)&&(counterIntLeft < goalInterrupts)){
		__NOP();
	}
	stopMove();
}

/** Función para rotar */
void rotation(uint8_t direction, uint16_t degrees){
	stopMove();
	defaultMove();
	updateDuttyCycle(&handlerPwmRight, DUTTY_RIGHT_ROTATION);
	updateDuttyCycle(&handlerPwmLeft, DUTTY_LEFT_ROTATION);
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
	// Calculamos la cantidad de interrupciones para conseguir la rotación
	uint16_t goalInterrupts = (uint16_t)(((float)interruptsRev)*(((float)(degrees))/360.0f)*(distanceAxis/wheelDiameter));
	counterIntRight = 0;
	counterIntLeft = 0;
	startMove();
	// Hacemos un ciclo que no haga nada hasta alcanzar las interrupciones
	while((counterIntRight < goalInterrupts)&&(counterIntLeft < goalInterrupts)){
		__NOP();
	}
	stopMove();
}

/** Función para realizar el cuadrado en la dirección y con la medida indicada */
void square(uint8_t direction, uint16_t side_in_mm){
	float angleCorrection = 0.00f;
	for(uint8_t side = 0; side < 4; side++){
		angleCorrection = straightLinePID(side_in_mm);
		angleCorrection = (direction == MOVEMENT_CW) ? (angleCorrection):(-angleCorrection);
		// Here re-calculating the offset is just a way to wait until the inertia of the movement ends
		getGyroscopeOffset(100);
		rotation(direction, (uint16_t)(90.00f + angleCorrection));
	}
}

/** Función para limitar el cambio de dutty*/
void constraintControlPID(float* uControl, float maxChange){
	if(*uControl >= maxChange){
		*uControl = maxChange;
	}
	else if(*uControl <= -maxChange){
		*uControl = -maxChange;
	}
	else{
		__NOP();
	}
}

/** Función para calcular la acción de control */
void controlActionPID(void){
	u_control = (u_1_control)+(q0*error)+(q1*error_1)+(q2*error_2);
	// Control action is limited to a change of 10% Dutty Cycle
	constraintControlPID(&u_control, 4000.00f);

	updateDuttyCycle(&handlerPwmRight, (uint16_t)(duttyBaseRight + u_control));
	updateDuttyCycle(&handlerPwmLeft, (uint16_t)(duttyBaseLeft - u_control));

	// Update the values
	u_1_control = u_control;
	error_2 = error_1;
	error_1 = error;
}

void callback_extInt0 (void){
	stopMove();
}

void callback_extInt1 (void){
	counterIntRight++;
}

void callback_extInt3 (void){
	counterIntLeft++;
}
