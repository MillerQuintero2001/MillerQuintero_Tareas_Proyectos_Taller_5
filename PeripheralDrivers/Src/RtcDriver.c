/*
 * RtcDriver.h
 *
 *  Created on: Jun 11, 2023
 *      Author: MillerQuintero2001
 *
 *  Este driver por defecto funciona con el LSE
 */

#include "RtcDriver.h"
#include "stm32f4xx.h"
#include <stdint.h>

/** Función principal de configuración */
void configRTC(RTC_Handler_t *ptrRtcHandler){

	/* OBSERVACIÓN: Hay unos pasos previos a todo esto lo cuales son activar el
	 * Power Interface Clock, setear el bit DPB para desactivar la protección
	 * contra escritura del RTC, luego activar el LSE, seleccionarlo como fuente
	 * para el RTC, y finalmente activar el RTC  */

	/* 0. Pasos previos */
	// Activamos el Power Interface Clock
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	// Permitimos escritura en el LSE
	PWR->CR |= PWR_CR_DBP;
	// Activamos el LSE
	RCC->BDCR |= RCC_BDCR_LSEON;
	// Esperamos hasta que sea estable
	while(!(RCC->BDCR & RCC_BDCR_LSERDY)){
		__NOP();
	}
	// Seleccionamos el LSE como RTC
	RCC ->BDCR &= ~RCC_BDCR_RTCSEL;
	RCC->BDCR |= RCC_BDCR_RTCSEL_0;
	// Activamos el RTC
	RCC->BDCR |= RCC_BDCR_RTCEN;


	/* 1. Retiramos la protección contra escritura usando las claves */
	RTC->WPR |= (0xCA << RTC_WPR_KEY_Pos);	// Primera clave según el manual
	RTC->WPR |= (0x53 << RTC_WPR_KEY_Pos);	// Segunda clave según el manual

	// Vamos a iniciar ahora el calendario

	/* 2. Activamos el modo de inicialización, aquí el calendar counter se detine */
	RTC->ISR |= RTC_ISR_INIT;

	/* 3. Esperamos hasta que se levante la bandera de inicialización, indicando que podemos configurar */
	while(!(RTC->ISR & RTC_ISR_INITF)){
		__NOP();
	}

	/* 4. Activamos la lectura directa desde los contadores del calendario para mayor precisión */
	RTC->CR |= RTC_CR_BYPSHAD;

	/* 6. Cargamos todos lo valores de calendario */

	// Limpiamos el Time Register y el Date Register
	RTC->TR = 0;
	RTC->DR = 0;

	/* Vamos a cargar ahora los tiempos y fechas, para esto vamos a emplear 2 variables auxiliares con las
	 * que guardaremos las decenas y unidades*/

	// 6.1 Cargamos el número de año
	// Saco el residuo de dividir por 2000, ya que mi RTC va a funcionar del 2000 para adelante
	RTC->DR |= ((conversionDECtoBCD(ptrRtcHandler->RTC_Year%2000)) << RTC_DR_YU_Pos);

	// 6.2 Cargamos el número de mes
	RTC->DR |= ((conversionDECtoBCD(ptrRtcHandler->RTC_Month)) << RTC_DR_MU_Pos);

	// 6.3 Cargamos el número de día
	RTC->DR |= ((conversionDECtoBCD(ptrRtcHandler->RTC_Day)) << RTC_DR_DU_Pos);

	// 6.4 Definimos primero el formato
	// Si es 24 h
	if(ptrRtcHandler->RTC_HourFormat == RTC_HOUR_FORMAT_24H){
		RTC->CR &= ~RTC_CR_FMT;
		RTC->TR &= ~RTC_TR_PM;
	}
	// Si es 12h AM
	else if(ptrRtcHandler->RTC_HourFormat == RTC_HOUR_FORMAT_12H_AM){
		RTC->CR |= RTC_CR_FMT;
		RTC->TR &= ~RTC_TR_PM;
	}
	// Si es 12h PM
	else if(ptrRtcHandler->RTC_HourFormat == RTC_HOUR_FORMAT_12H_PM){
		RTC->CR |= RTC_CR_FMT;
		RTC->TR |= RTC_TR_PM;
	}
	else{
		__NOP();
	}

	// 6.5 Cargamos las horas
	RTC->TR |= ((conversionDECtoBCD(ptrRtcHandler->RTC_Hours)) << RTC_TR_HU_Pos);

	// 6.6 Cargamos los minutos
	RTC->TR |= ((conversionDECtoBCD(ptrRtcHandler->RTC_Minutes)) << RTC_TR_MNU_Pos);

	// 6.7 Cargamos los segundos
	RTC->TR |= ((conversionDECtoBCD(ptrRtcHandler->RTC_Seconds)) << RTC_TR_SU_Pos);

	/* 7. Desactivamos el modo de inicialización */
	RTC->ISR &= ~RTC_ISR_INIT;

	/* 8. Verificamos que el calendario haya sido inicializado */
	while(!(RTC->ISR & RTC_ISR_INITS)){
		__NOP();
	}

	/* 9. Nuevamente ponemos la protección contra escritura */
	RTC->WPR = RTC_WPR_KEY;

	// Fin de la configuración del RTC

}

/** Función encargada de obtener los datos del calendario en estructura [YEAR, MOUNTH, DAY, HOUR, MINUTE, SEG] */
void readCalendar(uint8_t *arrayCalendarData){

	arrayCalendarData[0] = conversionBCDtoDEC(((RTC->DR >> RTC_DR_YU_Pos) & 0xFF)); 	// Años
	arrayCalendarData[1] = conversionBCDtoDEC(((RTC->DR >> RTC_DR_MU_Pos) & 0x1F));		// Meses
	arrayCalendarData[2] = conversionBCDtoDEC(((RTC->DR >> RTC_DR_DU_Pos) & 0x3F));		// Días
	arrayCalendarData[3] = conversionBCDtoDEC(((RTC->TR >> RTC_TR_HU_Pos) & 0x3F));		// Horas
	arrayCalendarData[4] = conversionBCDtoDEC(((RTC->TR >> RTC_TR_MNU_Pos) & 0x7F));	// Minutos
	arrayCalendarData[5] = conversionBCDtoDEC(((RTC->TR >> RTC_TR_SU_Pos) & 0x7F));		// Segundos
	arrayCalendarData[6] = conversionBCDtoDEC(((RTC->TR >> RTC_TR_PM_Pos) & 0x1));		// Formato AM o PM
}

/** Funcíon utilizada para convertir de Binary Coded Decimal a Decimal simple */
uint8_t conversionBCDtoDEC(uint8_t BCDdata){
	return ((BCDdata/16)*10) + (BCDdata%16);
}

/** Funcíon utilizada para convertir de Decimal simple a Binary Coded Decimal */
uint8_t conversionDECtoBCD(uint8_t DECdata){
	return ((DECdata/10)*16) + (DECdata%10);
}


