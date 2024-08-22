/*
 * MPU6050.h
 *
 *  Created on: 24/07/2024
 *      Author: MillerQuintero2001
 */

#ifndef MPU6050_H_
#define MPU6050_H_

#include <stdint.h>
#include <stdbool.h>

#include <stm32f4xx.h>

#include "GPIOxDriver.h"
#include "I2CDriver.h"

/* Macrodefinicion para los registros */
#define ACCEL_ADDRESS 	0b1101000; // 0xD2 -> Dirección del sensor Acelerómetro con ADO=0
#define ACCEL_XOUT_H	59 // 0x3B
#define ACCEL_XOUT_L	60 // OX3C
#define ACCEL_YOUT_H	61 // 0x3D
#define ACCEL_YOUT_L 	62 // 0x3E
#define ACCEL_ZOUT_H 	63 // 0x3F
#define ACCEL_ZOUT_L 	64 // 0x40
#define GYRO_XOUT_H		67 // 0X43
#define GYRO_XOUT_L		68 // 0X44
#define GYRO_YOUT_H		69 // 0X45
#define GYRO_YOUT_L		70 // 0X46
#define GYRO_ZOUT_H		71 // 0X47
#define GYRO_ZOUT_L		71 // 0X48
#define PWR_MGMT_1		107
#define WHO_AM_I		117

/* Prototipos de las funciones */
void configMPU6050(void);
void resetMPU6050(void);
float getGyroscopeData(void);
float getGyroscopeOffset(uint32_t samples);

#endif /* MPU6050_H_ */
