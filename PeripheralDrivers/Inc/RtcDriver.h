/*
 * RtcDriver.h
 *
 *  Created on: Jun 11, 2023
 *      Author: MillerQuintero2001
 */

#ifndef RTCDRIVER_H_
#define RTCDRIVER_H_

#include "stm32f4xx.h"
#include <stdint.h>

#define RTC_WEEK_DAY_MONDAY			1
#define RTC_WEEK_DAY_TUESDAY		2
#define RTC_WEEK_DAY_WEDNESDAY		3
#define RTC_WEEK_DAY_THURSDAY		4
#define RTC_WEEK_DAY_FRYDAY			5
#define RTC_WEEK_DAY_SATURDAY		6
#define RTC_WEEK_DAY_SUNDAY			7

#define RTC_FORMAT_24H				0
#define RTC_FORMAT_12H				1

#define RTC_HOUR_FORMAT_24H			0
#define RTC_HOUR_FORMAT_12H_AM		1
#define RTC_HOUR_FORMAT_12H_PM		2

typedef struct
{
	uint8_t RTC_HourFormat;
	uint8_t RTC_Hours;
	uint8_t RTC_Minutes;
	uint8_t RTC_Seconds;
	uint8_t RTC_Day;
	uint8_t RTC_Month;
	uint16_t RTC_Year;
}RTC_Handler_t;

/* Prototipos de Funciones */
void configRTC(RTC_Handler_t *ptrRtcHandler);
//void updateTime(RTC_Handler_t *ptrRtcHandler, uint8_t hour, uint8_t min, uint8_t seg, uint8_t amOrpm);
//void updateDate(RTC_Handler_t *ptrRtcHandler, uint8_t day, uint8_t month, uint16_t year);
//void changeHourFormat(RTC_Handler_t *ptrRtcHandler, uint8_t format);
void readCalendar(uint8_t *arrayCalendarData);
uint8_t conversionBCDtoDEC(uint8_t BCDdata);
uint8_t conversionDECtoBCD(uint8_t DECdata);

#endif /* RTCDRIVER_H_ */
