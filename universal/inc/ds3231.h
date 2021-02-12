/**
  ******************************************************************************
  * @file    ds3231.h
  * @author  Juraj Lonc (juraj.lonc@gmail.com)
  * @brief   Header file of DS3231 driver.
  ******************************************************************************
  */

#ifndef __DS3231_H
#define __DS3231_H

#include "rtc.h"
#include "stdbool.h"
#include "device.h"
//#include "utils.h"

#define DS3231_ADDRESS				0b11010000

#define DS3231_REG_SECOND				0x00
#define DS3231_REG_MINUTE				0x01
#define DS3231_REG_HOUR					0x02
#define DS3231_REG_DAY					0x03
#define DS3231_REG_DATE					0x04
#define DS3231_REG_MONTH				0x05
#define DS3231_REG_YEAR					0x06

#define DS3231_REG_ALARM1_SEC			0x07
#define DS3231_REG_ALARM1_MIN			0x08
#define DS3231_REG_ALARM1_HOUR			0x09
#define DS3231_REG_ALARM1_DATE			0x0A

#define DS3231_REG_ALARM2_MIN			0x0B
#define DS3231_REG_ALARM2_HOUR			0x0C
#define DS3231_REG_ALARM2_DATE			0x0D

#define DS3231_REG_CONTROL				0x0E
#define DS3231_REG_STATUS				0x0F
#define DS3231_REG_AGEOFFSET			0x10
#define DS3231_REG_TEMP_MSB				0x11
#define DS3231_REG_TEMP_LSB				0x12

typedef enum {
	DS3231_RATE_1HZ=0b00,
	DS3231_RATE_1024HZ=0b01,
	DS3231_RATE_4096HZ=0b10,
	DS3231_RATE_8192HZ=0b11
} DS3231_RateSelect;

typedef enum {
	DS3231_ALARM1_ONCE_PER_SECOND,
	DS3231_ALARM1_WHEN_SECONDS_MATCH,
	DS3231_ALARM1_WHEN_MINUTES_SECONDS_MATCH,
	DS3231_ALARM1_WHEN_HOURS_MINUTES_SECONDS_MATCH,
	DS3231_ALARM1_WHEN_DATE_HOURS_MINUTES_SECONDS_MATCH,
	DS3231_ALARM1_WHEN_DAY_HOURS_MINUTES_SECONDS_MATCH
} DS3231_Alarm1Rate;

typedef enum {
	DS3231_ALARM2_ONCE_PER_MINUTE,
	DS3231_ALARM2_WHEN_MINUTES_MATCH,
	DS3231_ALARM2_WHEN_HOURS_MINUTES_MATCH,
	DS3231_ALARM2_WHEN_DATE_HOURS_MINUTES_MATCH,
	DS3231_ALARM2_WHEN_DAY_HOURS_MINUTES_MATCH
} DS3231_Alarm2Rate;

typedef struct {
	device_write_ptr write_cmd;
	device_read_ptr read_cmd;
} DS3231_Object;

typedef union {
	struct {
		uint8_t DATA[4];
	};
	struct {
		struct {
			uint8_t seconds :7;
			uint8_t A1M1 :1;
		};
		struct {
			uint8_t minutes :7;
			uint8_t A1M2 :1;
		};
		struct {
			uint8_t hours :7;
			uint8_t A1M3 :1;
		};
		struct {
			uint8_t date_day :6;
			uint8_t DY :1;
			uint8_t A1M4 :1;
		};

	};
} DS3231_Alarm1;

typedef union {
	struct {
		uint8_t DATA[3];
	};
	struct {
		struct {
			uint8_t minutes :7;
			uint8_t A2M2 :1;
		};
		struct {
			uint8_t hours :7;
			uint8_t A2M3 :1;
		};
		struct {
			uint8_t date_day :6;
			uint8_t DY :1;
			uint8_t A2M4 :1;
		};

	};
} DS3231_Alarm2;

typedef union {
	struct {
		uint8_t BYTE;
	};
	struct {
		uint8_t A1IE :1;
		uint8_t A2IE :1;
		uint8_t INTCN :1;
		uint8_t RS1 :1;
		uint8_t RS2 :1;
		uint8_t CONV :1;
		uint8_t BBSQW :1;
		uint8_t EOSC_N :1;
	};
	struct {
		uint8_t :3;
		uint8_t RS :2;
	};
} DS3231_Control;

typedef struct {
	uint8_t A1F :1;
	uint8_t A2F :1;
	uint8_t BSY :1;
	uint8_t EN32KHZ :1;
	uint8_t :3;
	uint8_t OSF :1;
} DS3231_Status;


void DS3231_init(DS3231_Object * obj, device_write_ptr write_cmd, device_read_ptr read_cmd);
//void DS3231_get_seconds(DS3231_Object *obj, RTC_time * time);
//void DS3231_get_minutes(DS3231_Object *obj, RTC_time *time);
//void DS3231_get_hours(DS3231_Object *obj, RTC_time *time);
//void DS3231_get_day_of_week(DS3231_Object *obj, RTC_time *time);
//void DS3231_get_date(DS3231_Object *obj, RTC_time *time);
//void DS3231_get_month(DS3231_Object *obj, RTC_time *time);
//void DS3231_get_year(DS3231_Object *obj, RTC_time *time);
void DS3231_read_time(DS3231_Object *obj, RTC_time * time);
//void DS3231_set_seconds(DS3231_Object *obj, uint8_t val);
//void DS3231_set_minutes(DS3231_Object *obj, uint8_t val);
//void DS3231_set_hours(DS3231_Object *obj, uint8_t val, RTCHoursFormat hour_format);
void DS3231_set_day_of_week(DS3231_Object *obj, RTCDay val);
//void DS3231_set_date(DS3231_Object *obj, uint8_t val);
//void DS3231_set_month(DS3231_Object *obj, uint8_t val);
//void DS3231_set_year(DS3231_Object *obj, uint8_t val);
int16_t DS3231_get_temperature_int(DS3231_Object *obj);
float DS3231_get_temperature(DS3231_Object *obj);
void DS3231_write_time(DS3231_Object *obj, RTC_time * time);
void DS3231_set_time_values(DS3231_Object *obj, uint8_t hours, uint8_t minutes, uint8_t seconds);
void DS3231_set_date_values(DS3231_Object *obj, uint8_t year, uint8_t month, uint8_t day_of_month);
void DS3231_read_alarm1(DS3231_Object *obj, DS3231_Alarm1 * alarm1);
void DS3231_write_alarm1(DS3231_Object *obj, DS3231_Alarm1 * alarm1);
void DS3231_set_alarm1(DS3231_Object *obj, uint8_t date_day, uint8_t hours, uint8_t minutes, uint8_t seconds, DS3231_Alarm1Rate rate);
void DS3231_disable_alarm1(DS3231_Object *obj);
void DS3231_set_alarm2(DS3231_Object *obj, uint8_t date_day, uint8_t hours, uint8_t minutes, DS3231_Alarm2Rate rate);
void DS3231_disable_alarm2(DS3231_Object *obj);
void DS3231_read_alarm2(DS3231_Object *obj, DS3231_Alarm2 * alarm2);
void DS3231_write_alarm2(DS3231_Object *obj, DS3231_Alarm2 * alarm2);
void DS3231_read_control(DS3231_Object *obj, DS3231_Control * control);
void DS3231_write_control(DS3231_Object *obj, DS3231_Control * control);
void DS3231_read_status(DS3231_Object *obj, DS3231_Status * status);
void DS3231_write_status(DS3231_Object *obj, DS3231_Status * status);
int8_t DS3231_read_aging_offset(DS3231_Object *obj);
void DS3231_start_conversion(DS3231_Object *obj);
bool DS3231_is_conversion_done(DS3231_Object *obj);

#endif
