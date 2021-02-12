/**
  ******************************************************************************
  * @file    rtc.h
  * @author  Juraj Lonc (juraj.lonc@gmail.com)
  * @brief   Header file for RTC utilities.
  ******************************************************************************
  */

#ifndef __RTC_H
#define __RTC_H

#include <stdint.h>
#include "utils.h"

typedef enum {
	RTC_SUNDAY=1,
	RTC_MONDAY=2,
	RTC_TUESDAY=3,
	RTC_WEDNESDAY=4,
	RTC_THURSDAY=5,
	RTC_FRIDAY=6,
	RTC_SATURDAY=7
} RTCDay;

//#define MONDAY  	1
//#define TUESDAY 	2
//#define WEDNESDAY 	3
//#define THURSDAY 	4
//#define FRIDAY  	5
//#define SATURDAY 	6
//#define SUNDAY   	7

typedef enum {
	RTC_AM=1,
	RTC_PM=2,
	RTC_H24=3
} RTCHoursFormat;

//#define AM		1
//#define PM		2
//#define H24		3

typedef union {
	struct {
		uint8_t DATA[8];
	};
	struct {
		uint8_t second;
		uint8_t minute;
		uint8_t hour;
		RTCDay day_of_week;
		uint8_t day_of_month;
		uint8_t month;
		uint8_t year;
		RTCHoursFormat hour_format;
	};
} RTC_time;

void printDate(RTC_time *rtc, uint8_t *buffer);
void printTime(RTC_time *rtc, uint8_t *buffer);

void rtc_convert_seconds(RTC_time *time, uint8_t val);
void rtc_convert_minutes(RTC_time *time, uint8_t val);
void rtc_convert_hours(RTC_time *time, uint8_t val);
void rtc_convert_day_of_week(RTC_time *time, uint8_t val);
void rtc_convert_date(RTC_time *time, uint8_t val);
void rtc_convert_month(RTC_time *time, uint8_t val);
void rtc_convert_year(RTC_time *time, uint8_t val);

uint8_t rtc_encode_hours(uint8_t val, RTCHoursFormat hour_format);
uint8_t rtc_weekday(uint16_t year, uint8_t month, uint8_t day);

#endif
