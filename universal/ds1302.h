/**
  ******************************************************************************
  * @file    ds1302.h
  * @author  Juraj Lonc (juraj.lonc@gmail.com)
  * @brief   Header file of DS1302 driver.
  ******************************************************************************
  */

#ifndef __DS1302_H
#define __DS1302_H

#ifdef __cplusplus
extern "C" {
#endif

#include "rtc.h"
#include "utils.h"
#include "stdbool.h"
#include "device.h"

#define DS1302_REG_SEC_W 		0x80
#define DS1302_REG_SEC_R 		0x81
#define DS1302_REG_MIN_W 		0x82
#define DS1302_REG_MIN_R 		0x83
#define DS1302_REG_HOUR_W 		0x84
#define DS1302_REG_HOUR_R 		0x85
#define DS1302_REG_DATE_W 		0x86
#define DS1302_REG_DATE_R 		0x87
#define DS1302_REG_MONTH_W 		0x88
#define DS1302_REG_MONTH_R 			0x89
#define DS1302_REG_DAY_W 			0x8a
#define DS1302_REG_DAY_R 			0x8b
#define DS1302_REG_YEAR_W 			0x8c
#define DS1302_REG_YEAR_R 			0x8d
#define DS1302_REG_W_PROTECT_W 		0x8e
#define DS1302_REG_W_PROTECT_R 		0x8f
#define DS1302_REG_TRICKE_CHARGE_W 	0x90
#define DS1302_REG_TRICKE_CHARGE_R 	0x91
#define DS1302_REG_CLOCK_BURST_W 	0xbe
#define DS1302_REG_CLOCK_BURST_R 	0xbf
#define DS1302_REG_RAM_BURST_W 		0xfe
#define DS1302_REG_RAM_BURST_R 		0xff

#define DS1302_RAM_START_R	0xc1
#define DS1302_RAM_START_W	0xc0

#define DS1302_RAM_LENGTH	31

typedef enum {
	DS1302_TRICKLE_DISABLED=0b01011100,
	DS1302_TRICKLE_1DIODE_2KOHM=0b10100101,
	DS1302_TRICKLE_1DIODE_4KOHM=0b10100110,
	DS1302_TRICKLE_1DIODE_8KOHM=0b10100111,
	DS1302_TRICKLE_2DIODES_2KOHM=0b10101001,
	DS1302_TRICKLE_2DIODES_4KOHM=0b10101010,
	DS1302_TRICKLE_2DIODES_8KOHM=0b10101011
} DS1302_TrickleCharge;


/**
 * @brief  Set value of pin. Hardware dependent function.
 * @param  bool: value of pin
 */
typedef void (*device_write_pin_ptr)(bool);

/**
 * @brief  Read value of pin. Hardware dependent function.
 * @retval  value of pin
 */
typedef bool (*device_read_pin_ptr)(void);


typedef struct {
	device_write_pin_ptr write_ce;
	device_write_pin_ptr write_clk;
	device_write_pin_ptr write_io;
	device_read_pin_ptr read_io;
	device_write_pin_ptr set_io_output;
	device_delay_us_ptr delay_us;
} DS1302_Handle;



//void DS1302_write_byte(DS1302_Handle *handle, uint8_t byte);
//uint8_t DS1302_read_byte(DS1302_Handle *handle);

//void DS1302_write(DS1302_Handle *handle, uint8_t reg, uint8_t data);
//uint8_t DS1302_read(DS1302_Handle *handle, uint8_t reg);

void DS1302_init(DS1302_Handle *handle, device_write_pin_ptr write_ce, device_write_pin_ptr write_clk, device_write_pin_ptr write_io, device_read_pin_ptr read_io, device_delay_us_ptr delay_us, device_write_pin_ptr set_io_output);

void DS1302_write_ram(DS1302_Handle *handle, uint8_t offset, uint8_t data);
void DS1302_write_ram_burst(DS1302_Handle *handle, uint8_t *buf, uint8_t len);
uint8_t DS1302_read_ram(DS1302_Handle *handle, uint8_t offset);
void DS1302_read_ram_burst(DS1302_Handle *handle, uint8_t *buf, uint8_t len);
uint8_t DS1302_read(DS1302_Handle *handle, uint8_t reg);
void DS1302_write(DS1302_Handle *handle, uint8_t reg, uint8_t data);

//void DS1302_get_seconds(DS1302_Handle *handle, RTC_time * time);
//void DS1302_get_minutes(DS1302_Handle *handle, RTC_time * time);
//void DS1302_get_hours(DS1302_Handle *handle, RTC_time * time);
//void DS1302_get_day_of_week(DS1302_Handle *handle, RTC_time * time);
//void DS1302_get_date(DS1302_Handle *handle, RTC_time * time);
//void DS1302_get_month(DS1302_Handle *handle, RTC_time * time);
//void DS1302_get_year(DS1302_Handle *handle, RTC_time * time);
void DS1302_read_time(DS1302_Handle *handle, RTC_time * time);
void DS1302_write_time(DS1302_Handle *handle, RTC_time * time);

//void DS1302_set_seconds(DS1302_Handle *handle, uint8_t val);
//void DS1302_set_minutes(DS1302_Handle *handle, uint8_t val);
//void DS1302_set_hours(DS1302_Handle *handle, uint8_t val, uint8_t hour_format);
void DS1302_set_day_of_week(DS1302_Handle *handle, RTCDay val);
//void DS1302_set_date(DS1302_Handle *handle, uint8_t val);
//void DS1302_set_month(DS1302_Handle *handle, uint8_t val);
//void DS1302_set_year(DS1302_Handle *handle, uint8_t val);

bool DS1302_clock_halted(DS1302_Handle *handle);
void DS1302_clock_halt(DS1302_Handle *handle);
void DS1302_clock_start(DS1302_Handle *handle);

DS1302_TrickleCharge DS1302_get_trickle_charge(DS1302_Handle *handle);
void DS1302_set_trickle_charge(DS1302_Handle *handle, DS1302_TrickleCharge trickle);
//void DS1302_set_write_protect(DS1302_Handle *handle, bool write_protect_on);

#ifdef __cplusplus
}
#endif

#endif
