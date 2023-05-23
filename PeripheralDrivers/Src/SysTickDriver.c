/*
 * SysTickDriver.c
 *
 *  Created on: 26/04/2023
 *      Author: MillerQuintero2001
 *
 * 	Este driver controla el TImer que trae por defecto todo procesador ARM Cortex Mx,
 * 	el cual hace parte del sistema independiente de la empresa fabricante del MCU.
 *
 * 	Para encontrar cuál es su registro de configuración, debemos utilizar el manuel
 * 	genérico del procesador Cortex-M4, ya que es allí donde se encuentra la documentación
 * 	para este periférico.
 *
 * 	En el archivo core_cm4.h, la estructura que define el periférico se llama SysTick_Type
 *
 */

#include <stm32f4xx.h>
#include "SysTickDriver.h"
#include "PLLDriver.h"

uint64_t ticks = 0;
uint64_t ticks_start = 0;
uint64_t ticks_counting = 0;
uint64_t freq_clock = 0;

/** Función de configuracíon del SysTick */
void config_SysTick_ms(uint8_t systemClock){

	// Reiniciamos el valor de la variable que cuenta tiempo
	ticks = 0;
	freq_clock = getConfigPLL();

	switch(systemClock){

	// Caso para el reloj HSI a 16MHz
	case 0: {
		SysTick->LOAD = SYSTICK_LOAD_VALUE_16MHz_1ms;
		break;
	}

	// Caso	para el reloj HSE
	case 1: {
		SysTick->LOAD = SYSTICK_LOAD_VALUE_16MHz_1ms;
		break;
	}

	// Caso para el reloj PLL
	case 2: {
		SysTick->LOAD = freq_clock/1000;
		break;
	}
	// Caso por defecto
	default: {
		SysTick->LOAD = SYSTICK_LOAD_VALUE_16MHz_1ms;
		break;
	}

	} // Fin del switch-case


	// Limpiamos el valor actual de SysTick
	SysTick->VAL = 0;

	// Configuramos el reloj interno como el reloj para el Timer
	SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;

	// Desactivamos las interrupciones globales
	__disable_irq();

	// Matriculamos la interrupcíon en el NVIC
	NVIC_EnableIRQ(SysTick_IRQn);

	// Activamos la interrupción debida al conteo a cero del SysTick
	SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;

	// Activamos el Timer
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;

	// Activamos de nuevo las interrupciones globales
	__enable_irq();

}

/** Función que retorna el valor actual de los ticks, según lo que aumenta con el Handler*/
uint64_t getTicks_ms(void){
	return ticks;
}

/** Funcíon para aplicar un tiempo de delay en milisegundos */
void delay_ms(uint32_t wait_time_ms){

	// Captura el primer valor de tiempo para comparar
	ticks_start = getTicks_ms();

	// Captura el segundo valor de tiempo para comparar
	ticks_counting = getTicks_ms();

	// Compara: si el valor "counting" es menor que el "strart + wait"
	// Actualiza el valor "counting"
	// Repite esta operación  hasta que counting sea mayor (se cumple el tiempo de espera)
	while(ticks_counting < (ticks_start + (uint64_t)wait_time_ms)){

		// Actualizar el valor
		ticks_counting = getTicks_ms();
	}
}

/** Handler del SysTick, código que se ejecuta cada vez que el counter llega al valor*/
void SysTick_Handler(void){
	// Verficamos que la interrupción se haya lanzado
	if(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk){

		// Limpiamos la bandera
		SysTick->CTRL &= ~SysTick_CTRL_COUNTFLAG_Msk;

		// Incrementamos en 1 el contador
		ticks++;
	}
}
