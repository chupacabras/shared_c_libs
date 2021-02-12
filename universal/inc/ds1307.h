/**
  ******************************************************************************
  * @file    ds1307.h
  * @author  Juraj Lonc (juraj.lonc@gmail.com)
  * @brief   Header file of DS1307 driver.
  ******************************************************************************
  */

#ifndef __DS1307_H
#define __DS1307_H

#include "rtc.h"
#include "stdbool.h"
#include "device.h"

#define DS1307_ADDRESS			0b11010000

#define DS1307_REG_SEC 			0x00
#define DS1307_REG_MIN 			0x01
#define DS1307_REG_HOUR 		0x02
#define DS1307_REG_DAY 			0x03
#define DS1307_REG_DATE 		0x04
#define DS1307_REG_MONTH 		0x05
#define DS1307_REG_YEAR 		0x06
#define DS1307_REG_CONTROL 		0x07

#define DS1307_RAM_START		0x08

#define DS1307_RAM_LENGTH		56

typedef enum {
	DS1307_OUT_VAL0=0b00000000,
	DS1307_OUT_VAL1=0b10000000,
	DS1307_OUT_1HZ=0b00010000,
	DS1307_OUT_4096HZ=0b00010001,
	DS1307_OUT_8192HZ=0b00010010,
	DS1307_OUT_32768HZ=0b00010011
} DS1307_OutputControl;



typedef struct {
	device_write_ptr write_cmd;
	device_read_ptr read_cmd;
} DS1307_Object;

void DS1307_init(DS1307_Object * obj, device_write_ptr write_cmd, device_read_ptr read_cmd);

void DS1307_write_ram(DS1307_Object *obj, uint8_t offset, uint8_t data);
void DS1307_write_ram_burst(DS1307_Object *obj, uint8_t *buf, uint8_t len);
uint8_t DS1307_read_ram(DS1307_Object *obj, uint8_t offset);
void DS1307_read_ram_burst(DS1307_Object *obj, uint8_t *buf, uint8_t len);
uint8_t DS1307_read(DS1307_Object *obj, uint8_t reg);
void DS1307_write(DS1307_Object *obj, uint8_t reg, uint8_t data);

//void DS1307_get_seconds(DS1307_Object *obj, RTC_time * time);
//void DS1307_get_minutes(DS1307_Object *obj, RTC_time * time);
//void DS1307_get_hours(DS1307_Object *obj, RTC_time * time);
void DS1307_get_day_of_week(DS1307_Object *obj, RTC_time * time);
//void DS1307_get_date(DS1307_Object *obj, RTC_time * time);
//void DS1307_get_month(DS1307_Object *obj, RTC_time * time);
//void DS1307_get_year(DS1307_Object *obj, RTC_time * time);
void DS1307_read_time(DS1307_Object *obj, RTC_time * time);
void DS1307_write_time(DS1307_Object *obj, RTC_time *time);

//void DS1307_set_seconds(DS1307_Object *obj, uint8_t val);
//void DS1307_set_minutes(DS1307_Object *obj, uint8_t val);
//void DS1307_set_hours(DS1307_Object *obj, uint8_t val, uint8_t hour_format);
//void DS1307_set_day_of_week(DS1307_Object *obj, uint8_t val);
//void DS1307_set_date(DS1307_Object *obj, uint8_t val);
//void DS1307_set_month(DS1307_Object *obj, uint8_t val);
//void DS1307_set_year(DS1307_Object *obj, uint8_t val);

bool DS1307_clock_halted(DS1307_Object *obj);
void DS1307_clock_halt(DS1307_Object *obj);
void DS1307_clock_start(DS1307_Object *obj);

DS1307_OutputControl DS1307_get_output_control(DS1307_Object *obj);
void DS1307_set_output_control(DS1307_Object *obj, DS1307_OutputControl output);


#endif
