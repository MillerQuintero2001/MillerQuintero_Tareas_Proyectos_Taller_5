/*
 * PwmDriver.h
 *
 *  Created on: May 4, 2023
 *      Author: MillerQuintero2001
 */

#ifndef PWMDRIVER_H_
#define PWMDRIVER_H_

#include "stm32f4xx.h"
#include "SysTickDriver.h"

#define PWM_CHANNEL_1	0
#define PWM_CHANNEL_2	1
#define PWM_CHANNEL_3	2
#define PWM_CHANNEL_4	3

#define PWM_DUTTY_0_PERCENT		0
#define PWM_DUTTY_5_PERCENT		5
#define PWM_DUTTY_10_PERCENT	10
#define PWM_DUTTY_15_PERCENT	15
#define PWM_DUTTY_20_PERCENT	20
#define PWM_DUTTY_25_PERCENT	25
#define PWM_DUTTY_30_PERCENT	30
#define PWM_DUTTY_40_PERCENT	40
#define PWM_DUTTY_50_PERCENT	50
#define PWM_DUTTY_75_PERCENT	75
#define PWM_DUTTY_90_PERCENT	90
#define PWM_DUTTY_100_PERCENT	100

/**/
typedef struct
{
	uint8_t		PWMx_Channel; 		// Canal PWM relacionado con el TIMER
	uint32_t	PWMx_Prescaler;		// A qué velocidad se incrementa el Timer
	uint16_t	PWMx_Period;		// Indica el número de veces que el Timer se incrementa, el periodo de la frecuencia viene dado por Time_Fosc * PSC * ARR
	uint16_t	PWMx_DuttyCicle;	// Valor en porcentaje (%) del tiempo que la señal está en alto
}PWM_Config_t;

/**/
typedef struct
{
	TIM_TypeDef		*ptrTIMx;	// Timer al que esta asociado el PWM
	PWM_Config_t	PWMx_Config;	// Configuración inicial del PWM
}PWM_Handler_t;

/* Prototipos de las funciones */
void pwm_Config(PWM_Handler_t *ptrPwmHandler);
void setPeriod(PWM_Handler_t *ptrPwmHandler);
void updatePeriod(PWM_Handler_t *ptrPwmHandler, uint16_t newPeriod);
void setDuttyCycle(PWM_Handler_t *ptrPwmHandler);
void updateDuttyCycle(PWM_Handler_t *ptrPwmHandler, uint16_t newDutty);
void enableOutput(PWM_Handler_t *ptrPwmHandler);
void disableOutput(PWM_Handler_t *ptrPwmHandler);
void startPwmSignal(PWM_Handler_t *ptrPwmHandler);
void stopPwmSignal(PWM_Handler_t *ptrPwmHandler);

#endif /* PWMDRIVER_H_ */
