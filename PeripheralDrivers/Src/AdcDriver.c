/*
 * AdcDriver.c
 *
 *  Created on: May 25, 2023
 *      Author: MillerQuintero2001
 */

#include <stdint.h>
#include "AdcDriver.h"
#include "GPIOxDriver.h"

GPIO_Handler_t handlerAdcPin = {0};
uint16_t adcRawData = 0;

void adc_ConfigMultichannel (ADC_Multichannel_Config_t *adcMultiConfig, uint8_t numChannels){

	/* 1. Configuramos todos los pines en el orden específicado*/
	for(uint8_t i = 0; i < numChannels; i++){
		configAnalogPin(adcMultiConfig->orderADC[i]);
	}

	/* 2. Activamos la señal de reloj para el periférico ADC1 (bus APB2)*/
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

	// Limpiamos los registros antes de comenzar a configurar
	ADC1->CR1 = 0;
	ADC1->CR2 = 0;

	/* Comenzamos la configuración del ADC1 */
	/* 3. Resolución del ADC */
	switch(adcMultiConfig->resolution){
	case ADC_RESOLUTION_12_BIT:
	{
		ADC1->CR1 &= ~ADC_CR1_RES;
		break;
	}

	case ADC_RESOLUTION_10_BIT:
	{
		ADC1->CR1 |= ADC_CR1_RES_0;
		break;
	}

	case ADC_RESOLUTION_8_BIT:
	{
		ADC1->CR1 |= ADC_CR1_RES_1;
		break;
	}

	case ADC_RESOLUTION_6_BIT:
	{
		ADC1->CR1 |= ADC_CR1_RES;
		break;
	}

	default:
	{	// 6 bits por defecto lo quiero dejar

		ADC1->CR1 |= ADC_CR1_RES;
		break;
	}
	}

	/* 4. ¡IMPORTANTE! Configuramos el modo Scan como activado */
	ADC1->CR1 |= ADC_CR1_SCAN;

	/* 5. Configuramos la alineación de los datos (derecha o izquierda) */
	if(adcMultiConfig->dataAlignment == ADC_ALIGNMENT_RIGHT){
		// Alineación a la derecha (esta es la forma "natural")
		ADC1->CR2 &= ~ADC_CR2_ALIGN;
	}
	else{

		// Alineación a la izquierda (para algunos cálculos matemáticos)
		ADC1->CR2 |= ADC_CR2_ALIGN;
	}

	/* 6. Configuración del Sampling y el orden de la secuencia */
	for(uint8_t j = 0; j < numChannels; j++){

		// 7. Configuramos el sampling
		if(adcMultiConfig->orderADC[j] < ADC_CHANNEL_10){
			ADC1->SMPR2 |= (adcMultiConfig->samplingPeriod[j] << 3*adcMultiConfig->orderADC[j]);
		}
		else{
			ADC1->SMPR1 |= (adcMultiConfig->samplingPeriod[j] << 3*(adcMultiConfig->orderADC[j]%10));
		}

		// Para las primeras 6 secuencias, el orden se carga en el SQR3
		if(j < 6){
			// Asignamos el canal de la conversión en los 5 bits que le corresponden
			ADC1->SQR3 |= (adcMultiConfig->orderADC[j] << j*5);
		}
		// Para las secuencias de la 7 a la 12, el orden se carga en el SQR2
		else if((j > 5)&&(j < 12)){
			// Asignamos el canal de la conversión en los 5 bits que le corresponden
			ADC1->SQR2 |= (adcMultiConfig->orderADC[j] << (j%6)*5);
		}
		// Para las primeras de la 13 a la 16, el orden se carga en el SQR3
		else if((j > 11)&&(j < 16)){
			// Asignamos el canal de la conversión en los 5 bits que le corresponden
			ADC1->SQR1 |= (adcMultiConfig->orderADC[j] << (j%12)*5);
		}
	}

	/* 8. Establecemos la longitud de la secuencia multicanal */
	ADC1->SQR1 |= ((numChannels-1) << ADC_SQR1_L_Pos);

	/* 9. ¡IMPORTANTE! El módulo ADC está conectado al APB2, y solo puede trabajar
	 * a una frecuencia máxima de 36MHz, así que debemos configurar según el reloj */
	if(RCC->CFGR & RCC_CFGR_SW){		// Si estamos con el PLL
		ADC->CCR &= ~ADC_CCR_ADCPRE;
		ADC->CCR |= ADC_CCR_ADCPRE_0;	// Con esto si está a 100MHz se divide por 4 y queda a 25MHz, o 80MHz queda en 20MHz
	}
	else{
		ADC->CCR &= ~ADC_CCR_ADCPRE;
	}

	/* 10. Activamos el external trigger para las conversiones */
	ADC1->CR2 |= (adcMultiConfig->extTriggerEnable << ADC_CR2_EXTEN_Pos);

	/* 11. Seleccionamos que evento queremos como trigger del ADC */
	ADC1->CR2 |= (adcMultiConfig->extTriggerSelect << ADC_CR2_EXTSEL_Pos);

	/* 10. Desactivamos las interrupciones globales */
	__disable_irq();

	/* 11. Activamos la interrupción debida a la finalización de una conversión EOC (CR1)*/
	ADC1->CR1 |= ADC_CR1_EOCIE;

	/* 12. Activamos EOCS para tener las interrupciones cada que un canal termina de convertir */
	ADC1->CR2 |= ADC_CR2_EOCS;

	/* 11a. Matriculamos la interrupción en el NVIC*/
	__NVIC_EnableIRQ(ADC_IRQn);

	/* 11b. Configuramos la prioridad para la interrupción ADC */
	__NVIC_SetPriority(ADC_IRQn, adcMultiConfig->priorityInterrupt);

	/* 12. Activamos el modulo ADC */
	ADC1->CR2 |= ADC_CR2_ADON;

	/* 14. Activamos las interrupciones globales */
	__enable_irq();

}

void adc_Config(ADC_Config_t *adcConfig){
	/* 1. Configuramos el PinX para que cumpla la función de canal análogo deseado. */
	configAnalogPin(adcConfig->channel);

	/* 2. Activamos la señal de reloj para el periférico ADC1 (bus APB2)*/
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

	// Limpiamos los registros antes de comenzar a configurar
	ADC1->CR1 = 0;
	ADC1->CR2 = 0;

	/* Comenzamos la configuración del ADC1 */
	/* 3. Resolución del ADC */
	switch(adcConfig->resolution){
	case ADC_RESOLUTION_12_BIT:
	{
		ADC1->CR1 &= ~ADC_CR1_RES;
		break;
	}

	case ADC_RESOLUTION_10_BIT:
	{
		ADC1->CR1 |= ADC_CR1_RES_0;
		break;
	}

	case ADC_RESOLUTION_8_BIT:
	{
		ADC1->CR1 |= ADC_CR1_RES_1;
		break;
	}

	case ADC_RESOLUTION_6_BIT:
	{
		ADC1->CR1 |= ADC_CR1_RES;
		break;
	}

	default:
	{	// 6 bits por defecto lo quiero dejar

		ADC1->CR1 |= ADC_CR1_RES;
		break;
	}
	}

	/* 4. Configuramos el modo Scan como desactivado */
	ADC1->CR1 &= ~ADC_CR1_SCAN;

	/* 5. Configuramos la alineación de los datos (derecha o izquierda) */
	if(adcConfig->dataAlignment == ADC_ALIGNMENT_RIGHT){
		// Alineación a la derecha (esta es la forma "natural")
		ADC1->CR2 &= ~ADC_CR2_ALIGN;
	}
	else{

		// Alineación a la izquierda (para algunos cálculos matemáticos)
		ADC1->CR2 |= ADC_CR2_ALIGN;
	}

	/* 6. Desactivamos el "continuos mode" */
	ADC1->CR2 &= ~ADC_CR2_CONT;

	/* 7. Acá se debería configurar el sampling...*/
	if(adcConfig->channel < ADC_CHANNEL_10){
		ADC1->SMPR2 |= (adcConfig->samplingPeriod << 3*adcConfig->channel);
	}
	else{
		ADC1->SMPR1 |= (adcConfig->samplingPeriod << 3*(adcConfig->channel%10));
	}

	/* 8. Configuramos la secuencia y cuantos elementos hay en la secuencia */
	// Al hacerlo todo 0, estamos seleccionando solo 1 elemento en el conteo de la secuencia
	ADC1->SQR1 = 0;

	// Asignamos el canal de la conversión a la primera posición en la secuencia
	ADC1->SQR3 |= (adcConfig->channel << 0);

	/* 9. ¡IMPORTANTE! El módulo ADC está conectado al APB2, y solo puede trabajar
	 * a una frecuencia máxima de 36MHz, así que debemos configurar según el reloj */
	if(RCC->CFGR & RCC_CFGR_SW){		// Si estamos con el PLL
		ADC->CCR &= ~ADC_CCR_ADCPRE;
		ADC->CCR |= ADC_CCR_ADCPRE_0;	// Con esto si está a 100MHz se divide por 4 y queda a 25MHz, o 80MHz queda en 20MHz
	}
	else{
		ADC->CCR &= ~ADC_CCR_ADCPRE;
	}

	/* 10. Desactivamos las interrupciones globales */
	__disable_irq();

	/* 11. Activamos la interrupción debida a la finalización de una conversión EOC (CR1)*/
	ADC1->CR1 |= ADC_CR1_EOCIE;

	/* 11a. Matriculamos la interrupción en el NVIC*/
	__NVIC_EnableIRQ(ADC_IRQn);

	/* 11b. Configuramos la prioridad para la interrupción ADC */
	__NVIC_SetPriority(ADC_IRQn, adcConfig->priorityInterrupt);

	/* 12. Activamos el modulo ADC */
	ADC1->CR2 |= ADC_CR2_ADON;

	/* 13. Activamos las interrupciones globales */
	__enable_irq();
}

/*
 * Esta función lanza la conversion ADC y si la configuración es la correcta, solo se hace
 * una conversion ADC.
 * Al terminar la conversion, el sistema lanza una interrupción y el dato es leido en la
 * función callback, utilizando la funciona getADC().
 *
 * */
void startSingleADC(void){
	/* Desactivamos el modo continuo de ADC */
	ADC1->CR2 &= ~ADC_CR2_CONT;

	/* Limpiamos el bit del overrun (CR1) */
	ADC1->CR1 &= ~ADC_CR1_OVRIE;

	/* Iniciamos un ciclo de conversión ADC (CR2)*/
	ADC1->CR2 |= ADC_CR2_SWSTART;

}

/*
 * Esta función habilita la conversion ADC de forma continua.
 * Una vez ejecutada esta función, el sistema lanza una nueva conversion ADC cada vez que
 * termina, sin necesidad de utilizar para cada conversion el bit SWSTART del registro CR2.
 * Solo es necesario activar una sola vez dicho bit y las conversiones posteriores se lanzan
 * automaticamente.
 * */
void startContinousADC(void){

	/* Activamos el modo continuo de ADC */
	ADC1->CR2 |= ADC_CR2_CONT;

	/* Iniciamos un ciclo de conversión ADC */
	ADC1->CR2 |= ADC_CR2_SWSTART;

}

/*
 * Función que retorna el ultimo dato adquirido por la ADC
 * La idea es que esta función es llamada desde la función callback, de forma que
 * siempre se obtiene el valor mas actual de la conversión ADC.
 * */
uint16_t getADC(void){
	// Esta variable es actualizada en la ISR de la conversión, cada vez que se obtiene
	// un nuevo valor.
	return adcRawData;
}

/*
 * Esta es la ISR de la interrupción por conversión ADC
 */
void ADC_IRQHandler(void){
	// Evaluamos que se dio la interrupción por conversión ADC
	if(ADC1->SR & ADC_SR_EOC){
		// Leemos el resultado de la conversión ADC y lo cargamos en una variale auxiliar
		// la cual es utilizada en la función getADC()
		adcRawData = ADC1->DR;

		// Hacemos el llamado a la función que se ejecutará en el main
		adcComplete_Callback();
	}

}

/* Función debil, que debe ser sobreescrita en el main. */
__attribute__ ((weak)) void adcComplete_Callback(void){
	__NOP();
}

/*
 * Con esta función configuramos que pin deseamos que funcione como canal ADC
 * Esta funcion trabaja con el GPIOxDriver, por lo cual requiere que se incluya
 * dicho driver.
 */
void configAnalogPin(uint8_t adcChannel) {

	// Con este switch seleccionamos el canal y lo configuramos como análogo.
	switch (adcChannel) {

	case ADC_CHANNEL_0: {
		// Es el pin PA0
		handlerAdcPin.pGPIOx 						= GPIOA;
		handlerAdcPin.GPIO_PinConfig.GPIO_PinNumber = PIN_0;
		// Nota: Para el ejercicio inicial solo se necesita este canal, los demas
		// se necesitan para trabajos posteriores.
		break;
	}

	case ADC_CHANNEL_1: {
		// Es el pin PA1
		handlerAdcPin.pGPIOx 						= GPIOA;
		handlerAdcPin.GPIO_PinConfig.GPIO_PinNumber = PIN_1;
		break;
	}

	case ADC_CHANNEL_2: {
		// Es el pin PA2
		handlerAdcPin.pGPIOx 						= GPIOA;
		handlerAdcPin.GPIO_PinConfig.GPIO_PinNumber = PIN_2;
		break;
	}

	case ADC_CHANNEL_3: {
		// Es el pin PA3
		handlerAdcPin.pGPIOx 						= GPIOA;
		handlerAdcPin.GPIO_PinConfig.GPIO_PinNumber = PIN_3;
		break;
	}

	case ADC_CHANNEL_4: {
		// Es el pin PA4
		handlerAdcPin.pGPIOx 						= GPIOA;
		handlerAdcPin.GPIO_PinConfig.GPIO_PinNumber = PIN_4;
		break;
	}

	case ADC_CHANNEL_5: {
		// Es el pin PA5
		handlerAdcPin.pGPIOx 						= GPIOA;
		handlerAdcPin.GPIO_PinConfig.GPIO_PinNumber = PIN_5;
		break;
	}
	case ADC_CHANNEL_6: {
		// Es el pin PA6
		handlerAdcPin.pGPIOx 						= GPIOA;
		handlerAdcPin.GPIO_PinConfig.GPIO_PinNumber = PIN_6;
		break;
	}
	case ADC_CHANNEL_7: {
		// Es el pin PA7
		handlerAdcPin.pGPIOx 						= GPIOA;
		handlerAdcPin.GPIO_PinConfig.GPIO_PinNumber = PIN_7;
		break;
	}
	case ADC_CHANNEL_8: {
		//Es el pin PB0
		handlerAdcPin.pGPIOx 						= GPIOB;
		handlerAdcPin.GPIO_PinConfig.GPIO_PinNumber = PIN_0;
		break;
	}
	case ADC_CHANNEL_9: {
		//Es el pin PB1
		handlerAdcPin.pGPIOx 						= GPIOB;
		handlerAdcPin.GPIO_PinConfig.GPIO_PinNumber = PIN_1;
		break;
	}
	case ADC_CHANNEL_10: {
		//Es el pin C0
		handlerAdcPin.pGPIOx 						= GPIOC;
		handlerAdcPin.GPIO_PinConfig.GPIO_PinNumber = PIN_0;
		break;
	}
	case ADC_CHANNEL_11: {
		//Es el pin PC1
		handlerAdcPin.pGPIOx 						= GPIOC;
		handlerAdcPin.GPIO_PinConfig.GPIO_PinNumber = PIN_1;
		break;
	}
	case ADC_CHANNEL_12: {
		//Es el pin PC2
		handlerAdcPin.pGPIOx 						= GPIOC;
		handlerAdcPin.GPIO_PinConfig.GPIO_PinNumber = PIN_2;
		break;
	}
	case ADC_CHANNEL_13: {
		//Es el pin PC3
		handlerAdcPin.pGPIOx 						= GPIOC;
		handlerAdcPin.GPIO_PinConfig.GPIO_PinNumber = PIN_3;
		break;
	}
	case ADC_CHANNEL_14: {
		//Es el pin PC4
		handlerAdcPin.pGPIOx 						= GPIOC;
		handlerAdcPin.GPIO_PinConfig.GPIO_PinNumber = PIN_4;
		break;
	}
	case ADC_CHANNEL_15: {
		//Es el pin PC5
		handlerAdcPin.pGPIOx 						= GPIOC;
		handlerAdcPin.GPIO_PinConfig.GPIO_PinNumber = PIN_5;
		break;
	}
	default: {
		break;
	}

	}
	// Despues de configurar el canal adecuadamente, se define este pin como Analogo y se
	// carga la configuración con el driver del GPIOx
	handlerAdcPin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ANALOG;
	GPIO_Config(&handlerAdcPin);
}


