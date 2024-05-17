/*
 * BasicTimer.h
 *
 *  Created on: Apr 18, 2022
 *      Author: MillerQuintero2001
 */

#ifndef INC_BASICTIMER_H_
#define INC_BASICTIMER_H_

#include "stm32f4xx.h"

#define BTIMER_MODE_UP		0
#define BTIMER_MODE_DOWN	1

#define BTIMER_SPEED_1us	16
#define BTIMER_SPEED_10us	160
#define BTIMER_SPEED_100us	1600
#define BTIMER_SPEED_1ms	16000

#define BTIMER_PLL_80MHz_SPEED_100ns	8
#define BTIMER_PLL_80MHz_SPEED_1us		80
#define BTIMER_PLL_80MHz_SPEED_10us		800
#define BTIMER_PLL_80MHz_SPEED_100us	8000

#define BTIMER_PLL_100MHz_SPEED_10ns	1
#define BTIMER_PLL_100MHz_SPEED_100ns	10
#define BTIMER_PLL_100MHz_SPEED_1us		100
#define BTIMER_PLL_100MHz_SPEED_10us	1000
#define BTIMER_PLL_100MHz_SPEED_100us	10000

#define BTIMER_INTERRUP_DISABLE		0
#define BTIMER_INTERRUP_ENABLE 		1


/* Estructura que contiene la configuración mínima necesaria para el manejo del Timer.*/
typedef struct
{
	uint8_t		TIMx_mode; 					// Up or dowm
	uint32_t	TIMx_speed;					// A qué velocidad se incrementa el Timer
	uint32_t	TIMx_period;				// Valor en ms del periodo del Timer
	uint8_t		TIMx_interruptEnable;		// Activa o desactiva el modo interrupción del timer.
	uint8_t		TIMx_priorityInterrupt;		// Prioridad de la interrupción
}BasicTimer_Config_t;

/* Handler para el Timer*/
typedef struct
{
	TIM_TypeDef			*ptrTIMx;
	BasicTimer_Config_t	TIMx_Config;
}BasicTimer_Handler_t;

void BasicTimer_Config(BasicTimer_Handler_t *ptrBTimerHandler);
void startBasicTimer(BasicTimer_Handler_t *ptrBTimerHandler);
void stopBasicTimer(BasicTimer_Handler_t *ptrBTimerHandler);

void BasicTimer2_Callback(void); /* Esta función debe ser sobre-escrita en el main para que el sistema funcione*/
void BasicTimer3_Callback(void);
void BasicTimer4_Callback(void);
void BasicTimer5_Callback(void);

void TIM2_IRQHandler(void);
void TIM3_IRQHandler(void);
void TIM4_IRQHandler(void);
void TIM5_IRQHandler(void);

#endif /* INC_BASICTIMER_H_ */
