/*
 * MotorDriver.h
 *
 *  Created on: 08/04/2024
 *      Author: MillerQuintero2001
 */

#ifndef MOTORDRIVER_H_
#define MOTORDRIVER_H_


#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>

#include <stm32f4xx.h>

#include "GPIOxDriver.h"
#include "BasicTimer.h"
#include "ExtiDriver.h"
#include "PwmDriver.h"

/* Macrodefiniciones para las configuraciones */
#define MOTOR_ON		0		// Debido a que los optoacopladores tienen resistencia Pull-Up la señal se invierte para el enable
#define MOTOR_OFF		1		// Debido a que los optoacopladores tienen resistencia Pull-Up la señal se invierte para el enable
#define MOTOR_FORWARD	0 		// Debido a que los optoacopladores tienen resistencia Pull-Up la señal se invierte para el direction
#define MOTOR_BACK		1 		// Debido a que los optoacopladores tienen resistencia Pull-Up la señal se invierte para el direction
#define MOVEMENT_CW		0		// Para usar con la función square que realiza un cuadrado, con este argumento en sentido anti-horario
#define MOVEMENT_CCW	1		// Para usar con la funcion square que realiza un cuadrado, con este argumento en sentido horario

/* Variables importantes */
extern uint32_t counterIntRight;	// Variable que guarda el número de interrupciones del encoder en la rueda derecha (Motor Amarillo)
extern uint32_t counterIntLeft;		// Variable que guarda el número de interrupciones del encoder en la rueda izquierda (Motor Azul)
extern bool flagMove;				// Variable bandera para controlar el movimiento
extern PWM_Handler_t handlerPwmRight;
extern PWM_Handler_t handlerPwmLeft;

/* Prototipos de las funciones */
void configMotors(void);
void setSignals(uint8_t freqHz, uint8_t dutty);
void changeBaseDutty(uint16_t duttyRight, uint16_t duttyLeft);
void defaultMove(void);
void startMove(void);
void stopMove(void);
void pathSegment(uint16_t distance_in_mm);
void configPID(float kp, float ti, float td, float ts);
void straightLinePID(uint16_t distance_in_mm);
void rotateOppy(int16_t degrees);
void rotation(uint8_t direction, uint16_t degrees);
void square(uint8_t direction, uint16_t side_in_mm);

#endif /* MOTORDRIVER_H_ */
