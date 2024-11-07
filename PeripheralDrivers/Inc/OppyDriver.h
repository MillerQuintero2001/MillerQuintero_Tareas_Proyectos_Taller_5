/*
 * OppyDriver.h
 *
 *  Created on: 4/10/2024
 *      Author: MillerQuintero2001
 */

#ifndef OPPYDRIVER_H_
#define OPPYDRIVER_H_

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>

#include <stm32f4xx.h>

#include "GPIOxDriver.h"
#include "BasicTimer.h"
#include "ExtiDriver.h"
#include "PwmDriver.h"
#include "I2CDriver.h"

/* Macrodefinitions for configurations and functions */
//Due to optocouplers with pull-up resistors, the direction and enable logics are inverted
#define MOTOR_ON		0
#define MOTOR_OFF		1
#define MOTOR_FORWARD	0
#define MOTOR_BACK		1
#define PERIOD_1KHZ				100000
#define DUTTY_RIGHT_BASE 		30000
#define DUTTY_LEFT_BASE			30000
#define DUTTY_RIGHT_ROTATION	30000
#define DUTTY_LEFT_ROTATION		30000
#define MPU6050_ADDRESS 		0b1101000 			// 0xD0 -> Dirección del sensor Acelerómetro con ADO=0

// Enum for MPU6050's registers
typedef enum {
	SMPRT_DIV = 25,
	CONFIG,
	GYRO_CONFIG,
	ACCEL_XOUT_H = 59,
	ACCEL_XOUT_L,
	ACCEL_YOUT_H,
	ACCEL_YOUT_L,
	ACCEL_ZOUT_H,
	ACCEL_ZOUT_L,
	TEMP_OUT_H,
	TEMP_OUT_L,
	GYRO_XOUT_H,
	GYRO_XOUT_L,
	GYRO_YOUT_H,
	GYRO_YOUT_L,
	GYRO_ZOUT_H,
	GYRO_ZOUT_L,
	SIGNAL_PATH_RESET = 104,
	USER_CONTROL = 106,
	PWR_MGMT_1 = 107,
	WHO_AM_I = 117
} MPU6050_Registers_t;

// Axis enum
typedef enum {
	X_AXIS = 0,
	Y_AXIS,
	Z_AXIS,
} Gyroscope_Axis_t;

/* Important variables that need to be global */
extern uint32_t counterIntRight;
extern uint32_t counterIntLeft;
extern PWM_Handler_t handlerPwmRight;
extern PWM_Handler_t handlerPwmLeft;
extern I2C_Handler_t handlerMPU6050;
extern BasicTimer_Handler_t handlerSampleTimer;
extern bool flagMove;
extern bool flagTakeOffset;
extern bool flagData;

/* Public Functions Proto-types */
void configOppy(void);
void setSignals(uint8_t freqHz, uint8_t dutty);
void changeBaseDutty(uint32_t duttyRight, uint32_t duttyLeft);
void defaultMove(void);
void startMove(void);
void stopMove(void);
//void pathSegment(float distance_in_mm);
void configPID(float kp, float ti, float td, float ts);
float straightLinePID(uint16_t distance_in_mm);
void rotateOppy(int16_t degrees);
void rotationMPU6050(int16_t degrees);
void square(int16_t degrees, uint16_t side_in_mm);
void resetMPU6050(void);
float getGyroscopeData(uint8_t axis);
float getGyroscopeOffset(uint32_t samples);

#endif /* OPPYDRIVER_H_ */
