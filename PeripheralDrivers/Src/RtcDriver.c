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
	 * para el RTC, y finalmente activar el RTC, esto se omite por que en el main
	 * ya tenemos una función llamada initLSE que se encarga de todo esto  */

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

	/* 4. Activamos el Calibration Output para el RTC */
	RTC->CR |= RTC_CR_COE;

	/* 5. Seleccionamos el Calibration Output de 1Hz, los prescaler por defecto sirven */
	RTC->CR	|= RTC_CR_COSEL;

	/* 6. Cargamos todos lo valores de calendario */

	/* Vamos a cargar ahora los tiempos y fechas, para esto vamos a emplear 2 variables auxiliares con las
	 * que guardaremos las decenas y unidades*/
	uint8_t auxDec = 0;
	uint8_t auxUnd = 0;

	// Limpiamos el Time Register y el Date Register
	RTC->TR = 0;
	RTC->DR = 0;

	// 6.1 Cargamos los segundos
	auxDec = ptrRtcHandler->RTC_Seconds/10;
	auxUnd = ptrRtcHandler->RTC_Seconds%10;
	RTC->TR |= (auxUnd << RTC_TR_SU_Pos);
	RTC->TR |= (auxDec << RTC_TR_ST_Pos);

	// 6.2 Cargamos los minutos
	auxDec = ptrRtcHandler->RTC_Minutes/10;
	auxUnd = ptrRtcHandler->RTC_Minutes%10;
	RTC->TR |= (auxUnd << RTC_TR_MNU_Pos);
	RTC->TR |= (auxDec << RTC_TR_MNT_Pos);

	// 6.3 Cargamos las horas, pero definiendo primero el formato

	// Definimos si es formato de 24 horas o si es AM o PM
	if(ptrRtcHandler->RTC_HourFormat == RTC_HOUR_FORMAT_24H){
		RTC->CR &= ~RTC_CR_FMT;
		RTC->TR &= ~RTC_TR_PM;
	}
	else{
		RTC->CR |= RTC_CR_FMT;
		// Ahora miramos si la hora en AM/PM es AM
		if(ptrRtcHandler->RTC_HourAMxPM == RTC_HOUR_AM){
			RTC->CR &= ~RTC_TR_PM;
		}
		// Sino es PM
		else{
			RTC->CR |= RTC_TR_PM;
		}
	}

	auxDec = ptrRtcHandler->RTC_Hours/10;
	auxUnd = ptrRtcHandler->RTC_Hours%10;
	RTC->TR |= (auxUnd << RTC_TR_HU_Pos);
	RTC->TR |= (auxDec << RTC_TR_HT_Pos);


	// 6.5 Cargamos el número de día
	auxDec = ptrRtcHandler->RTC_Day/10;
	auxUnd = ptrRtcHandler->RTC_Day%10;
	RTC->DR |= (auxUnd << RTC_DR_DU_Pos);
	RTC->DR |= (auxDec << RTC_DR_DT_Pos);

	// 6.6 Cargamos el número de mes
	auxDec = ptrRtcHandler->RTC_Month/10;
	auxUnd = ptrRtcHandler->RTC_Month%10;
	RTC->DR |= (auxUnd << RTC_DR_MU_Pos);
	RTC->DR |= (auxDec << RTC_DR_MT_Pos);

	// 6.7 Cargamos el número de año
	auxDec = ptrRtcHandler->RTC_Year/10;
	auxUnd = ptrRtcHandler->RTC_Year%10;
	RTC->DR |= (auxUnd << RTC_DR_YU_Pos);
	RTC->DR |= (auxDec << RTC_DR_YT_Pos);

	/* 7. Desactivamos el modo de inicialización */
	RTC->ISR &= ~RTC_ISR_INIT;

	/* 8. Verificamos que el calendario haya sido inicializado */
	while(!(RTC->ISR & RTC_ISR_INITS)){
		__NOP();
	}

	// Fin de la configuración del RTC

}

/** Funcíon encargada de actualizar la hora */
void updateTime(RTC_Handler_t *ptrRtcHandler, uint8_t hour, uint8_t min, uint8_t seg){

	/* 1. Activamos el modo de inicialización, aquí el calendar counter se detine */
	RTC->ISR |= RTC_ISR_INIT;

	/* 2. Esperamos hasta que se levante la bandera de inicialización, indicando que podemos configurar */
	while(!(RTC->ISR & RTC_ISR_INITF)){
		__NOP();
	}

	// Variables auxiliares
	uint8_t auxDec = 0;
	uint8_t auxUnd = 0;

	// Limpiamos el Time Register y el Date Register
	RTC->TR = 0;
	RTC->DR = 0;

	// 3.1 Cargamos los segundos
	auxDec = seg/10;
	auxUnd = seg%10;
	RTC->TR |= (auxUnd << RTC_TR_SU_Pos);
	RTC->TR |= (auxDec << RTC_TR_ST_Pos);

	// 3.2 Cargamos los minutos
	auxDec = min/10;
	auxUnd = min%10;
	RTC->TR |= (auxUnd << RTC_TR_MNU_Pos);
	RTC->TR |= (auxDec << RTC_TR_MNT_Pos);

	// 3.3 Cargamos las horas, pero definiendo primero el formato
	auxDec = hour/10;
	auxUnd = hour%10;
	RTC->TR |= (auxUnd << RTC_TR_HU_Pos);
	RTC->TR |= (auxDec << RTC_TR_HT_Pos);

	/* 4. Desactivamos el modo de inicialización */
	RTC->ISR &= ~RTC_ISR_INIT;

	/* 5. Verificamos que el calendario haya sido inicializado */
	while(!(RTC->ISR & RTC_ISR_INITS)){
		__NOP();
	}
}

/** Funcíon encargada de actualizar la fecha */
void updateDate(RTC_Handler_t *ptrRtcHandler, uint8_t day, uint8_t month, uint16_t year){
	/* 1. Activamos el modo de inicialización, aquí el calendar counter se detine */
	RTC->ISR |= RTC_ISR_INIT;

	/* 2. Esperamos hasta que se levante la bandera de inicialización, indicando que podemos configurar */
	while(!(RTC->ISR & RTC_ISR_INITF)){
		__NOP();
	}

	// Variables auxiliares
	uint8_t auxDec = 0;
	uint8_t auxUnd = 0;

	// Limpiamos el Time Register y el Date Register
	RTC->TR = 0;
	RTC->DR = 0;

	// 3.1 Cargamos el número de día
	auxDec = day/10;
	auxUnd = day%10;
	RTC->DR |= (auxUnd << RTC_DR_DU_Pos);
	RTC->DR |= (auxDec << RTC_DR_DT_Pos);

	// 3.2 Cargamos el número de mes
	auxDec = month/10;
	auxUnd = month%10;
	RTC->DR |= (auxUnd << RTC_DR_MU_Pos);
	RTC->DR |= (auxDec << RTC_DR_MT_Pos);

	// 3.3 Cargamos el número de año
	auxDec = year/10;
	auxUnd = year%10;
	RTC->DR |= (auxUnd << RTC_DR_YU_Pos);
	RTC->DR |= (auxDec << RTC_DR_YT_Pos);


	/* 4. Desactivamos el modo de inicialización */
	RTC->ISR &= ~RTC_ISR_INIT;

	/* 5. Verificamos que el calendario haya sido inicializado */
	while(!(RTC->ISR & RTC_ISR_INITS)){
		__NOP();
	}

}

/** Función encargada de cambiar el formato de la hora */
void changeHourFormat(RTC_Handler_t *ptrRtcHandler, uint8_t format){
	/* 1. Activamos el modo de inicialización, aquí el calendar counter se detine */
	RTC->ISR |= RTC_ISR_INIT;

	/* 2. Esperamos hasta que se levante la bandera de inicialización, indicando que podemos configurar */
	while(!(RTC->ISR & RTC_ISR_INITF)){
		__NOP();
	}

	/* 3. Cambiamos el formato */
	if(format == RTC_HOUR_FORMAT_24H){
		RTC->CR &= ~RTC_CR_FMT;
		RTC->TR &= ~RTC_TR_PM;
	}
	else{
		RTC->CR |= RTC_CR_FMT;
		// Ahora miramos si la hora en AM/PM es AM
		if(format == RTC_HOUR_AM){
			RTC->CR &= ~RTC_TR_PM;
		}
		// Sino es PM
		else{
			RTC->CR |= RTC_TR_PM;
		}
	}

	/* 4. Desactivamos el modo de inicialización */
	RTC->ISR &= ~RTC_ISR_INIT;

	/* 5. Verificamos que el calendario haya sido inicializado */
	while(!(RTC->ISR & RTC_ISR_INITS)){
		__NOP();
	}
}

/** Función encargada de obtener los datos del calendario en estructura [YEAR, MOUNTH, DAY, HOUR, MINUTE, SEG] */
void readCalendar(uint16_t *arrayCalendarData){

	arrayCalendarData[1] = conversionBCDtoDEC(((RTC->DR & 0xFF0000) >> RTC_DR_YU_Pos)); 		// Años
	arrayCalendarData[2] = conversionBCDtoDEC(((RTC->DR & 0x1F00) >> RTC_DR_MU_Pos));			// Meses
	arrayCalendarData[3] = conversionBCDtoDEC(((RTC->DR & 0x3F) >> RTC_DR_DU_Pos));				// Días
	arrayCalendarData[4] = conversionBCDtoDEC(((RTC->TR & 0x3F0000) >> RTC_TR_HU_Pos));			// Horas
	arrayCalendarData[5] = conversionBCDtoDEC(((RTC->TR & 0x7F00) >> RTC_TR_MNU_Pos));			// Minutos
	arrayCalendarData[6] = conversionBCDtoDEC(((RTC->TR & 0x7F) >> RTC_TR_HU_Pos));				// Segundos
}

/** Funcíon utilizada para convertir de Binary Decoded */
uint16_t conversionBCDtoDEC(uint16_t BCDdata){
	return ((BCDdata/16)*10) + (BCDdata%16);
}

