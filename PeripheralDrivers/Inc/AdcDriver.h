/*
 * AdcDriver.h
 *
 *  Created on: May 25, 2023
 *      Author: MillerQuintero2001
 */

#ifndef INC_ADCDRIVER_H_
#define INC_ADCDRIVER_H_

#include <stdint.h>
#include "stm32f4xx.h"

#define ADC_CHANNEL_0		0
#define ADC_CHANNEL_1		1
#define ADC_CHANNEL_2		2
#define ADC_CHANNEL_3		3
#define ADC_CHANNEL_4		4
#define ADC_CHANNEL_5		5
#define ADC_CHANNEL_6		6
#define ADC_CHANNEL_7		7
#define ADC_CHANNEL_8		8
#define ADC_CHANNEL_9		9
#define ADC_CHANNEL_10		10
#define ADC_CHANNEL_11		11
#define ADC_CHANNEL_12		12
#define ADC_CHANNEL_13		13
#define ADC_CHANNEL_14		14
#define ADC_CHANNEL_15		15

#define ADC_RESOLUTION_12_BIT	0
#define ADC_RESOLUTION_10_BIT	1
#define ADC_RESOLUTION_8_BIT	2
#define ADC_RESOLUTION_6_BIT	3

#define ADC_ALIGNMENT_RIGHT		0
#define ADC_ALIGNMENT_LEFT		1

#define ADC_SAMPLING_PERIOD_3_CYCLES	0b000
#define ADC_SAMPLING_PERIOD_15_CYCLES	0b001
#define ADC_SAMPLING_PERIOD_28_CYCLES	0b010
#define ADC_SAMPLING_PERIOD_56_CYCLES	0b011
#define ADC_SAMPLING_PERIOD_84_CYCLES	0b100
#define ADC_SAMPLING_PERIOD_112_CYCLES	0b101
#define ADC_SAMPLING_PERIOD_144_CYCLES	0b110
#define ADC_SAMPLING_PERIOD_480_CYCLES	0b111

#define ADC_EXTEN_DISABLED			0
#define ADC_EXTEN_RISING_EDGE		1
#define ADC_EXTEN_FALLING_EDGE		2
#define ADC_EXTEN_BOTH_EDGES		3

#define ADC_EXTSEL_TIM1_CC1		0
#define ADC_EXTSEL_TIM1_CC2		1
#define ADC_EXTSEL_TIM1_CC3		2
#define ADC_EXTSEL_TIM2_CC2		3
#define ADC_EXTSEL_TIM2_CC3		4
#define ADC_EXTSEL_TIM2_CC4		5
#define ADC_EXTSEL_TIM2_TRGO	6
#define ADC_EXTSEL_TIM3_CC1		7
#define ADC_EXTSEL_TIM3_TRGO	8
#define ADC_EXTSEL_TIM4_CC4		9
#define ADC_EXTSEL_TIM5_CC1		10
#define ADC_EXTSEL_TIM5_CC2		11
#define ADC_EXTSEL_TIM5_CC3		12
#define ADC_EXTSEL_EXTI_11		15

typedef struct
{
	uint8_t		channel;				// Canal ADC que será utilizado para la conversión ADC
	uint8_t		resolution;				// Precisión con la que el ADC hace la adquisición del dato
	uint16_t	samplingPeriod;			// Tiempo deseado para hacer la adquisición del dato
	uint8_t		dataAlignment;			// Alineación a la izquierda o a la derecha
	uint16_t	adcData;				// Dato de la conversión
	uint8_t		priorityInterrupt;		// Prioridad de la interrupción
}ADC_Config_t;

typedef struct
{
	uint8_t		*orderADC;			// Arreglo con el orden de operación de los canales
	uint8_t 	resolution;			// Precisión con la que el ADC hace la adquisición del dato
	uint16_t 	*samplingPeriod;	// Tiempo deseado para hacer la adquisición del dato
	uint8_t		dataAlignment;		// Alineación a la izquierda o a la derecha
	uint16_t 	adcData;			// Dato de la conversión
	uint8_t		extTriggerEnable;	// Activar trigger externo para los canales
	uint8_t		extTriggerSelect;	// Seleccionar que evento vamos a usar como trigger del ADC
	uint8_t		priorityInterrupt;	// Prioridad de la interrupción
}ADC_Multichannel_Config_t;

void adc_Config(ADC_Config_t *adcConfig);
void configAnalogPin(uint8_t adcChannel);
void adcComplete_Callback(void);
void startSingleADC(void);
void startContinousADC(void);
uint16_t getADC(void);
void adc_ConfigMultichannel (ADC_Multichannel_Config_t *adcMultiConfig, uint8_t numChannels);

#endif /* INC_ADCDRIVER_H_ */
