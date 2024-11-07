/*
 * OppyDriver.c
 *
 *  Created on: 4/10/2024
 *      Author: MillerQuintero2001
 */

#include "OppyDriver.h"

/* Initialization of the driver's own variables and elements */
// Handlers for Right Wheel (Motor Yellow)
PWM_Handler_t handlerPwmRight 		= {0};
GPIO_Handler_t handlerPinPwmRight 	= {0};
GPIO_Handler_t handlerEnableRight	= {0};
GPIO_Handler_t handlerDirRight 		= {0};
EXTI_Config_t handlerIntRight  		= {0};
GPIO_Handler_t handlerPinIntRight   = {0};

// Handlers for Left Wheel (Motor Blue)
PWM_Handler_t handlerPwmLeft 		= {0};
GPIO_Handler_t handlerPinPwmLeft 	= {0};
GPIO_Handler_t handlerEnableLeft 	= {0};
GPIO_Handler_t handlerDirLeft 		= {0};
EXTI_Config_t handlerIntLeft  		= {0};
GPIO_Handler_t handlerPinIntLeft   	= {0};

// Handlers related with Infrared Sensor
EXTI_Config_t handlerIntDistance 		= {0};
GPIO_Handler_t handlerPinIntDistance 	= {0};

// General Variables of Oppy
bool flagMove = false;
uint32_t counterIntRight = 0;
uint32_t counterIntLeft = 0;
uint32_t period = PERIOD_1KHZ;					// To have a base signal of 10 kHz
uint32_t duttyBaseRight = DUTTY_RIGHT_BASE;
uint32_t duttyBaseLeft = DUTTY_LEFT_BASE;
float interruptsRev = 120.00f;					// Number of encoder's interrupts per revolution
float wheelDiameter = 51.725f;
float wheelPerimeter = M_PI*51.725f;
float wheelBase = 105.20f;

// Variables realted with PID controller
float q0 = 0.00f;
float q1 = 0.00f;
float q2 = 0.00f;
float u_control = 0.00f; 			// Current PID control action
float u_1_control = 0.00f;			// Previous PID control action
float error = 0.00f;				// Current error
float error_1 = 0.00f;				// Error one sample before
float error_2 = 0.00f;				// Error two samples before
float timeSample = 0.020f;			// Sample time in seconds

// Handlers I2C MPU6050
GPIO_Handler_t handlerPinSerialData = {0};
GPIO_Handler_t handlerPinSerialClock = {0};
I2C_Handler_t handlerMPU6050 = {0};

// Variables related with samples to calculate gyroscope offset
BasicTimer_Handler_t handlerSampleTimer = {0};
uint32_t counterSamples = 0;
float sumAngularVelocity = 0.0f;
bool flagTakeOffset = false;
bool flagDataOffset = false;
bool flagData = false;

/* Private functions prototypes */
void controlActionPID(void);
void constraintControlPID(float* uControl, float maxChange);


/** Functions that inits Oppy's hardware */
void configOppy(void){
	// Activation of Math Co-processor for the floating point unit (FPU)
	SCB->CPACR |= (0XF << 20);

	/* Configuration of Right Motor Yellow */
	// PWM signal
	handlerPwmRight.ptrTIMx			    			= TIM2;
	handlerPwmRight.PWMx_Config.PWMx_Channel	    = PWM_CHANNEL_1;
	handlerPwmRight.PWMx_Config.PWMx_Prescaler 		= BTIMER_PLL_100MHz_SPEED_10ns;
	handlerPwmRight.PWMx_Config.PWMx_Period	   		= PERIOD_1KHZ;
	handlerPwmRight.PWMx_Config.PWMx_DuttyCicle		= DUTTY_RIGHT_BASE;
	handlerPwmRight.PWMx_Config.PWMx_Polarity		= PWM_POLARITY_ACTIVE_HIGH;
	pwm_Config(&handlerPwmRight);

	// PWM pin
	handlerPinPwmRight.pGPIOx								= GPIOA;
	handlerPinPwmRight.GPIO_PinConfig.GPIO_PinNumber	    = PIN_0;
	handlerPinPwmRight.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_ALTFN;
	handlerPinPwmRight.GPIO_PinConfig.GPIO_PinOPType	    = GPIO_OTYPE_PUSHPULL;
	handlerPinPwmRight.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
	handlerPinPwmRight.GPIO_PinConfig.GPIO_PinSpeed			= GPIO_OSPEED_FAST;
	handlerPinPwmRight.GPIO_PinConfig.GPIO_PinAltFunMode	= AF1;
	GPIO_Config(&handlerPinPwmRight);

	// Enable pin
	handlerEnableRight.pGPIOx                             	= GPIOC;
	handlerEnableRight.GPIO_PinConfig.GPIO_PinNumber      	= PIN_10;
	handlerEnableRight.GPIO_PinConfig.GPIO_PinMode        	= GPIO_MODE_OUT;
	handlerEnableRight.GPIO_PinConfig.GPIO_PinOPType      	= GPIO_OTYPE_PUSHPULL;
	handlerEnableRight.GPIO_PinConfig.GPIO_PinPuPdControl 	= GPIO_PUPDR_NOTHING;
	handlerEnableRight.GPIO_PinConfig.GPIO_PinSpeed       	= GPIO_OSPEED_FAST;
	GPIO_Config(&handlerEnableRight);
	GPIO_WritePin(&handlerEnableRight, MOTOR_OFF);

	// Direction pin
	handlerDirRight.pGPIOx                                	= GPIOC;
	handlerDirRight.GPIO_PinConfig.GPIO_PinNumber         	= PIN_12;
	handlerDirRight.GPIO_PinConfig.GPIO_PinMode           	= GPIO_MODE_OUT;
	handlerDirRight.GPIO_PinConfig.GPIO_PinOPType         	= GPIO_OTYPE_PUSHPULL;
	handlerDirRight.GPIO_PinConfig.GPIO_PinPuPdControl    	= GPIO_PUPDR_NOTHING;
	handlerDirRight.GPIO_PinConfig.GPIO_PinSpeed          	= GPIO_OSPEED_FAST;
	GPIO_Config(&handlerDirRight);
	GPIO_WritePin(&handlerDirRight, MOTOR_FORWARD);


	/* Configuration of Left Motor Blue */
	// PWM signal
	handlerPwmLeft.ptrTIMx			    			= TIM2;
	handlerPwmLeft.PWMx_Config.PWMx_Channel	   		= PWM_CHANNEL_2;
	handlerPwmLeft.PWMx_Config.PWMx_Prescaler		= BTIMER_PLL_100MHz_SPEED_10ns;
	handlerPwmLeft.PWMx_Config.PWMx_Period	    	= PERIOD_1KHZ;
	handlerPwmLeft.PWMx_Config.PWMx_DuttyCicle		= DUTTY_LEFT_BASE;
	handlerPwmLeft.PWMx_Config.PWMx_Polarity		= PWM_POLARITY_ACTIVE_HIGH;
	pwm_Config(&handlerPwmLeft);

	// PWM pin
	handlerPinPwmLeft.pGPIOx								= GPIOA;
	handlerPinPwmLeft.GPIO_PinConfig.GPIO_PinNumber		    = PIN_1;
	handlerPinPwmLeft.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_ALTFN;
	handlerPinPwmLeft.GPIO_PinConfig.GPIO_PinOPType		    = GPIO_OTYPE_PUSHPULL;
	handlerPinPwmLeft.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
	handlerPinPwmLeft.GPIO_PinConfig.GPIO_PinSpeed			= GPIO_OSPEED_FAST;
	handlerPinPwmLeft.GPIO_PinConfig.GPIO_PinAltFunMode	    = AF1;
	GPIO_Config(&handlerPinPwmLeft);

	// Enable pin
	handlerEnableLeft.pGPIOx                             	= GPIOC;
	handlerEnableLeft.GPIO_PinConfig.GPIO_PinNumber     	= PIN_11;
	handlerEnableLeft.GPIO_PinConfig.GPIO_PinMode        	= GPIO_MODE_OUT;
	handlerEnableLeft.GPIO_PinConfig.GPIO_PinOPType      	= GPIO_OTYPE_PUSHPULL;
	handlerEnableLeft.GPIO_PinConfig.GPIO_PinPuPdControl 	= GPIO_PUPDR_NOTHING;
	handlerEnableLeft.GPIO_PinConfig.GPIO_PinSpeed       	= GPIO_OSPEED_FAST;
	GPIO_Config(&handlerEnableLeft);
	GPIO_WritePin(&handlerEnableLeft, MOTOR_OFF);

	// Direction pin
	handlerDirLeft.pGPIOx                                	= GPIOD;
	handlerDirLeft.GPIO_PinConfig.GPIO_PinNumber         	= PIN_2;
	handlerDirLeft.GPIO_PinConfig.GPIO_PinMode           	= GPIO_MODE_OUT;
	handlerDirLeft.GPIO_PinConfig.GPIO_PinOPType         	= GPIO_OTYPE_PUSHPULL;
	handlerDirLeft.GPIO_PinConfig.GPIO_PinPuPdControl    	= GPIO_PUPDR_NOTHING;
	handlerDirLeft.GPIO_PinConfig.GPIO_PinSpeed          	= GPIO_OSPEED_FAST;
	GPIO_Config(&handlerDirLeft);
	GPIO_WritePin(&handlerDirLeft, MOTOR_FORWARD);


	/* EXTI's Configuration */
	// Right Encoder
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

	// Left Encoder
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

	// Infrared Sensor
	handlerPinIntDistance.pGPIOx								= GPIOC;
	handlerPinIntDistance.GPIO_PinConfig.GPIO_PinNumber			= PIN_0;
	handlerPinIntDistance.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_IN;
	handlerPinIntDistance.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
	GPIO_Config(&handlerPinIntDistance);
	handlerIntDistance.edgeType				= EXTERNAL_INTERRUPT_RISING_EDGE;
	handlerIntDistance.priorityInterrupt	= 6;
	handlerIntDistance.pGPIOHandler			= &handlerPinIntDistance;
	extInt_Config(&handlerIntDistance);


	/* MPU6050 Configuration */
	// SCL pin
	handlerPinSerialClock.pGPIOx								= GPIOB;
	handlerPinSerialClock.GPIO_PinConfig.GPIO_PinNumber			= PIN_6;
	handlerPinSerialClock.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_ALTFN;
	handlerPinSerialClock.GPIO_PinConfig.GPIO_PinOPType			= GPIO_OTYPE_OPENDRAIN;
	handlerPinSerialClock.GPIO_PinConfig.GPIO_PinSpeed			= GPIO_OSPEED_HIGH;
	handlerPinSerialClock.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
	handlerPinSerialClock.GPIO_PinConfig.GPIO_PinAltFunMode 	= AF4;
	GPIO_Config(&handlerPinSerialClock);

	// SDA pin
	handlerPinSerialData.pGPIOx									= GPIOB;
	handlerPinSerialData.GPIO_PinConfig.GPIO_PinNumber			= PIN_7;
	handlerPinSerialData.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_ALTFN;
	handlerPinSerialData.GPIO_PinConfig.GPIO_PinOPType			= GPIO_OTYPE_OPENDRAIN;
	handlerPinSerialData.GPIO_PinConfig.GPIO_PinSpeed			= GPIO_OSPEED_HIGH;
	handlerPinSerialData.GPIO_PinConfig.GPIO_PinPuPdControl		= GPIO_PUPDR_NOTHING;
	handlerPinSerialData.GPIO_PinConfig.GPIO_PinAltFunMode 		= AF4;
	GPIO_Config(&handlerPinSerialData);

	// 12C config
	handlerMPU6050.ptrI2Cx		= I2C1;
	handlerMPU6050.modeI2C		= I2C_MODE_FM;
	handlerMPU6050.slaveAddress	= MPU6050_ADDRESS;
	i2c_config(&handlerMPU6050);

	// Reset of the sensor to ensure that it works the first time.
	resetMPU6050();

	// Default sample timer configuration is 20 miliseconds
	handlerSampleTimer.ptrTIMx								= TIM3;
	handlerSampleTimer.TIMx_Config.TIMx_mode				= BTIMER_MODE_UP;
	handlerSampleTimer.TIMx_Config.TIMx_speed				= BTIMER_PLL_100MHz_SPEED_100us;
	handlerSampleTimer.TIMx_Config.TIMx_period				= 200;
	handlerSampleTimer.TIMx_Config.TIMx_interruptEnable		= BTIMER_INTERRUP_ENABLE;
	handlerSampleTimer.TIMx_Config.TIMx_priorityInterrupt	= 6;
	BasicTimer_Config(&handlerSampleTimer);
}


/** Function responsible for modifying the frequency and %DuttyCycle of both motors */
void setSignals(uint8_t freqHz, uint8_t duttyPer){
	period = (uint32_t)(100000000.00f*(1.00f/((float)freqHz)));
	duttyBaseRight = (uint32_t)((float)period*(((float)duttyPer)/100.00f));
	duttyBaseLeft = (uint32_t)((float)period*(((float)duttyPer)/100.00f));
	updatePeriod(&handlerPwmRight, period);
	updatePeriod(&handlerPwmLeft, period);
	updateDuttyCycle(&handlerPwmRight, duttyBaseRight);
	updateDuttyCycle(&handlerPwmLeft, duttyBaseLeft);
}

/** Function that changes dutty base value for each wheel */
void changeBaseDutty(uint32_t duttyRight, uint32_t duttyLeft){
	duttyBaseRight = duttyRight;
	duttyBaseLeft = duttyLeft;
	updateDuttyCycle(&handlerPwmRight, duttyBaseRight);
	updateDuttyCycle(&handlerPwmLeft, duttyBaseLeft);
}


/** Function that restores the Oppy's movement configuration to default */
void defaultMove(void){
	setPolarity(&handlerPwmRight, PWM_POLARITY_ACTIVE_HIGH);
	setPolarity(&handlerPwmLeft, PWM_POLARITY_ACTIVE_HIGH);
	GPIO_WritePin(&handlerDirRight, MOTOR_FORWARD);
	GPIO_WritePin(&handlerDirLeft, MOTOR_FORWARD);
}

/** Function to start Oppy's movement */
void startMove(void){
	flagMove = true;
	counterIntRight = 0;
	counterIntLeft = 0;
	startPwmSignal(&handlerPwmRight);
	startPwmSignal(&handlerPwmLeft);
	GPIO_WritePin(&handlerEnableRight, MOTOR_ON);
	GPIO_WritePin(&handlerEnableLeft, MOTOR_ON);
}

/** Function to stop Oppy's movement */
void stopMove(void){
	flagMove = false;
	GPIO_WritePin(&handlerEnableRight, MOTOR_OFF);
	GPIO_WritePin(&handlerEnableLeft, MOTOR_OFF);
	stopPwmSignal(&handlerPwmRight);
	stopPwmSignal(&handlerPwmLeft);
	counterIntRight = 0;
	counterIntLeft = 0;
}

/** Function that configures PID with proportional constant 'kp', Integrative Time 'ti', Derivative Time 'ti' and Sample Time 'ts' in seconds */
void configPID(float kp, float ti, float td, float ts){
	timeSample = ts;
	q0 = kp*(1.00f+(ts/(2.00f*ti))+(td/ts));
	q1 = -kp*(1.00f-(ts/(2.00f*ti))+((2.00f*td)/ts));
	q2 = (kp*td)/ts;
	handlerSampleTimer.TIMx_Config.TIMx_period = (uint32_t)(ts*10000.00f);
	BasicTimer_Config(&handlerSampleTimer);
	// Reset MPU to configure the new sample divider
	resetMPU6050();
}


/** Function to do a straight line with PID controller according to the angle */
float straightLinePID(uint16_t distance_in_mm){
	// First, initialize the varibles employeed
	//uint32_t goalInterrupts = interruptsRev*((float)(distance_in_mm)/wheelPerimeter);
	resetMPU6050();
	float offsetAngularVelocity = getGyroscopeOffset(200);
	float dataAngle = 0.00f;
	float previousDataAngle = 0.00f;
	float totalCurrentAngle = 0.00f;
	float differentialCurrentAngle = 0.00f;
	uint32_t counterPreviousRight = 0;
	uint32_t counterPreviousLeft = 0;
	float differentialDistance = 0.00f;
	float currentDistanceX = 0.00f;
	uint32_t saveCounterIntRight = 0;
	uint32_t saveCounterIntLeft = 0;

	// Initial set-up
	defaultMove();
	updateDuttyCycle(&handlerPwmRight, duttyBaseRight);
	updateDuttyCycle(&handlerPwmLeft, duttyBaseLeft);
	startBasicTimer(&handlerSampleTimer);
	startMove();

	while((currentDistanceX < distance_in_mm)&&(flagMove)){
		if(flagData){
			// Take angular velocity in the time sample and multiply it by time sample to get the angle in that time
			dataAngle = (getGyroscopeData(Z_AXIS) - offsetAngularVelocity)*timeSample;
			// Sum with totalCurrentAngle to update the real current angle of the Oppy
			totalCurrentAngle += dataAngle;

			// Take the difference between previous and current data angle to get the differential angle and can calculate X and Y
			differentialCurrentAngle = previousDataAngle + dataAngle;

			differentialDistance = (((counterIntLeft - counterPreviousLeft)+(counterIntRight - counterPreviousRight))/2.0f)*(M_PI*51.725f/120.00f);
			currentDistanceX += differentialDistance*cosf(differentialCurrentAngle*M_PI/180.00f);

			previousDataAngle = dataAngle;

			counterPreviousRight = counterIntRight;
			counterPreviousLeft = counterIntLeft;

			// Calculate the error for input PID, is like this because the setPointAngle is equal to zero
			error = -totalCurrentAngle;

			// If the deviation is too much, it is necessary to correct it and reset the PID system
			if(fabsf(error) >= 10){
				// Here we save the values of interruptions in encoders
				saveCounterIntRight = counterIntRight;
				saveCounterIntLeft = counterIntLeft;

				stopBasicTimer(&handlerSampleTimer);
				flagData = false;
				stopMove();
				getGyroscopeOffset(50);

				// Rotation to restore orientation a little bit
				rotationMPU6050((int16_t)error);
				// Reset to mitigate the "drift"
				resetMPU6050();

				totalCurrentAngle = 0.00f;
				offsetAngularVelocity = getGyroscopeOffset(200);
				defaultMove();
				updateDuttyCycle(&handlerPwmRight, duttyBaseRight);
				updateDuttyCycle(&handlerPwmLeft, duttyBaseLeft);
				startBasicTimer(&handlerSampleTimer);
				startMove();

				// Restore values
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
	updateDuttyCycle(&handlerPwmRight, duttyBaseRight);
	updateDuttyCycle(&handlerPwmLeft, duttyBaseLeft);
	flagData = false;
	u_control = 0.0f;
	u_1_control = 0.0f;
	error = 0.0f;
	error_1 = 0.0f;
	error_2 = 0.0f;
	return totalCurrentAngle;
}


/** Function to rotate with odometry */
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
	uint32_t goalInterrupts = (uint32_t)(((float)interruptsRev)*(((float)(abs((int)degrees)))/360.0f)*(wheelBase/wheelDiameter));
	counterIntRight = 0;
	counterIntLeft = 0;
	startMove();
	// Hacemos un ciclo que no haga nada hasta alcanzar las interrupciones
	while((counterIntRight < goalInterrupts)&&(counterIntLeft < goalInterrupts)){
		__NOP();
	}
	stopMove();
}


/** Function to do a rotation with MPU6050 */
void rotationMPU6050(int16_t degrees){
	if(degrees != 0){
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
		float goalDegrees = fabsf((float)degrees);
		float offset = getGyroscopeOffset(100);
		float currentAngle = 0.00f;
		flagTakeOffset = false;
		startBasicTimer(&handlerSampleTimer);
		startMove();
		while(currentAngle < goalDegrees){
			if(flagData){
				currentAngle += fabsf((getGyroscopeData(Z_AXIS) - offset)*timeSample);
			}
			else{
				__NOP();
			}
		}
		stopMove();
	}
	// Otherwise, the angle is zero
	else{
		__NOP();
	}
}


/** Function to draw a square with Oppy */
void square(int16_t degrees, uint16_t side_in_mm){
	straightLinePID(side_in_mm);
	resetMPU6050();
	rotationMPU6050(degrees);
	straightLinePID(side_in_mm);
	resetMPU6050();
	rotationMPU6050(degrees);
	straightLinePID(side_in_mm);
	resetMPU6050();
	rotationMPU6050(degrees);
	straightLinePID(side_in_mm);
	resetMPU6050();
	rotationMPU6050(degrees);
}

/** Sensor reset function */
void resetMPU6050(void){
	// First, writing 0x80 we set DEVICE_RESET bit as 1
	i2c_writeSingleRegister(&handlerMPU6050, PWR_MGMT_1, 0x80);
	// Now, to avoid some kind of bugs, is necessary write 0x40, to set SLEEP bit as 1
	i2c_writeSingleRegister(&handlerMPU6050, PWR_MGMT_1, 0x40);
	// Finally, writing 0x00 the sensor is wake up
	i2c_writeSingleRegister(&handlerMPU6050, PWR_MGMT_1, 0x00);
	// Set  PLL with Z axis gyroscope as clock source
	i2c_writeSingleRegister(&handlerMPU6050, PWR_MGMT_1, 0x03);
	// Configuration of DLPF, it is with 188 Hz cut-off frequency and MPU6050's sample frequency of 1 kHz, take care that it introduces a 1.9 ms delay
	i2c_writeSingleRegister(&handlerMPU6050, CONFIG, 0x01);
	// Now, with a fs = 1000 Hz, if we want to sample at timeSample , is necessary have an ADC sampling of 10*freqSample
	uint8_t sampleDiv = (uint8_t)(100.00f*timeSample - 1.00f);
	i2c_writeSingleRegister(&handlerMPU6050, SMPRT_DIV, sampleDiv);
	// Reset of all signal paths
	i2c_writeSingleRegister(&handlerMPU6050, USER_CONTROL, 0x01);
	// Configure full scale range at 500 °/s, to ensure fast rotations
	i2c_writeSingleRegister(&handlerMPU6050, GYRO_CONFIG, 0x08);
}

/** Function to obtain the gyroscope data on the indicated axis */
float getGyroscopeData(uint8_t axis){
	uint8_t arraySaveData[2] = {0};
	switch (axis) {
		case X_AXIS: {
			i2c_readMultipleRegisters(&handlerMPU6050, GYRO_XOUT_H, 2, arraySaveData);
			break;
		}
		case Y_AXIS: {
			i2c_readMultipleRegisters(&handlerMPU6050, GYRO_YOUT_H, 2, arraySaveData);
			break;
		}
		case Z_AXIS: {
			i2c_readMultipleRegisters(&handlerMPU6050, GYRO_ZOUT_H, 2, arraySaveData);
			break;
		}
		default: {
			__NOP();
			break;
		}
	}
	uint8_t GyroHigh = arraySaveData[0];
	uint8_t GyroLow = arraySaveData[1];
	float Gyro = (float)((int16_t)(GyroHigh << 8 | GyroLow));
	float divValue = 0.00f;
	uint8_t GyroConfigValue = i2c_readSingleRegister(&handlerMPU6050, GYRO_CONFIG);
	switch (GyroConfigValue) {
		case 0b00000000: {
			divValue = 131.00f;
			break;
		}
		case 0b00001000: {
			divValue = 65.50f;
			break;
		}
		case 0b00010000: {
			divValue = 32.80f;
			break;
		}
		case 0b00011000: {
			divValue = 16.40f;
			break;
		}
		default: {
			divValue = 1.00f;
			break;
		}
	}
	return (Gyro/divValue);
}

/** Function that calculates and returns gyroscope offset */
float getGyroscopeOffset(uint32_t samples){
	counterSamples = 0;
	sumAngularVelocity = 0.0f;
	flagTakeOffset = true;
	float offsetAngularVelocity = 0.00f;
	startBasicTimer(&handlerSampleTimer);
	while(!(counterSamples >= samples)){
		if(flagDataOffset){
			counterSamples++;
			// Gyroscope data is in °/s
			sumAngularVelocity += getGyroscopeData(Z_AXIS);
			flagDataOffset = false;
		}
		else{
			__NOP();
		}
	}
	offsetAngularVelocity = (samples > 0) ? (sumAngularVelocity/((float)counterSamples)) : (0.00f);
	// Verify if the offset is appropiate
	if((offsetAngularVelocity >= -0.80f)&&(offsetAngularVelocity <= 0.80f)){
		// The offset angle is correct
		stopBasicTimer(&handlerSampleTimer);
		counterSamples = 0;
		sumAngularVelocity = 0.00f;
		flagTakeOffset = false;
		flagDataOffset = false;
		return offsetAngularVelocity;
	}
	else{
		stopBasicTimer(&handlerSampleTimer);
		flagTakeOffset = false;
		flagDataOffset = false;
		counterSamples = 0;
		sumAngularVelocity = 0.00f;
		return getGyroscopeOffset(samples);
	}
}

/** Function to calculate PID control action */
void controlActionPID(void){
	u_control = (u_1_control)+(q0*error)+(q1*error_1)+(q2*error_2);
	// Control action is limited to a change of 10% Dutty Cycle
	constraintControlPID(&u_control, 1000.0f);

	updateDuttyCycle(&handlerPwmRight, (uint32_t)(duttyBaseRight + u_control));
	updateDuttyCycle(&handlerPwmLeft, (uint32_t)(duttyBaseLeft - u_control));

	// Update the values
	u_1_control = u_control;
	error_2 = error_1;
	error_1 = error;
}

/** Function that constraints the maximum dutty change for PID */
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

/** Sample Timer Callback */
void BasicTimer3_Callback(void){
	if(flagTakeOffset){
		flagDataOffset = true;
	}
	else{
		flagData = true;
	}
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
