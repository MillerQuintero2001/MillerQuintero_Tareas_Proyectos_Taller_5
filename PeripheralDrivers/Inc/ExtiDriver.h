/*
 * ExtiDriver.h
 *
 *  Created on: May 10, 2022
 *      Author: MillerQuintero2001
 */

#ifndef INC_EXTIDRIVER_H_
#define INC_EXTIDRIVER_H_

#include "stm32f4xx.h"
#include "GPIOxDriver.h"

#define EXTERNAL_INTERRUPT_FALLING_EDGE		0
#define EXTERNAL_INTERRUPT_RISING_EDGE		1
#define EXTERNAL_INTERRUPT_BOTH_EDGE		2

typedef struct
{
	GPIO_Handler_t *pGPIOHandler;				// Canal ADC que será utilizado para la conversión ADC
	uint8_t			edgeType;					// Se selecciona si se desea un tipo de flanco subiendo o bajando
	uint8_t 		priorityInterrupt;			// Prioridad de la interrupción
}EXTI_Config_t;


void extInt_Config(EXTI_Config_t *extiConfig);

void callback_extInt0(void);
void callback_extInt1(void);
void callback_extInt2(void);
void callback_extInt3(void);
void callback_extInt4(void);
void callback_extInt5(void);
void callback_extInt6(void);
void callback_extInt7(void);
void callback_extInt8(void);
void callback_extInt9(void);
void callback_extInt10(void);
void callback_extInt11(void);
void callback_extInt12(void);
void callback_extInt13(void);
void callback_extInt14(void);
void callback_extInt15(void);

void EXTI0_IRQHandler(void);
void EXTI1_IRQHandler(void);
void EXTI2_IRQHandler(void);
void EXTI3_IRQHandler(void);
void EXTI4_IRQHandler(void);
void EXTI9_5_IRQHandler(void);
void EXTI15_10_IRQHandler(void);

#endif /* INC_EXTIDRIVER_H_ */
