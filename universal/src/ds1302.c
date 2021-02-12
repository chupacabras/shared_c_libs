/**
 ******************************************************************************
 * @file    ds1302.c
 * @author  Juraj Lonc (juraj.lonc@gmail.com)
 * @brief   DS1302 RTC driver.
 ******************************************************************************
 */

#include "ds1302.h"

static uint8_t temp;
//static uint8_t temp_arr[7];

/**
 * @brief  Initialize DS1302 RTC and DS1302 object/handler.
 * @param  obj:       Pointer to a DS1302_Object structure that contains
 *                    the information for the display.
 * @param  write_ce:		The function that sets CE (chip enable) pin. Hardware dependent.
 * @param  write_clk:		The function that writes clock. Hardware dependent.
 * @param  write_io:		The function that writes data/command to the module. Hardware dependent.
 * @param  read_io:			The function that reads data from the module. Hardware dependent.
 * @param  delay_us:		The function that makes delay in microseconds. Hardware dependent.
 * @param  set_io_output:	The function that sets port as output or input. Hardware dependent.
 */
void DS1302_init(DS1302_Object *obj, device_write_pin_ptr write_ce, device_write_pin_ptr write_clk, device_write_pin_ptr write_io, device_read_pin_ptr read_io,
		device_delay_us_ptr delay_us, device_write_pin_ptr set_io_output) {
	obj->write_ce = write_ce;
	obj->write_clk = write_clk;
	obj->write_io = write_io;
	obj->read_io = read_io;
	obj->delay_us = delay_us;
	obj->set_io_output = set_io_output;

	obj->write_ce(false);
}

/**
 * @brief  Send a single-byte to DS1302 RTC.
 * @param  obj:       Pointer to a DS1302_Object structure that contains
 *                    the information for the display.
 * @param  data: byte to send.
 */
static void DS1302_write_byte(DS1302_Object *obj, uint8_t data) {
	uint8_t i;
	obj->set_io_output(true);

	for (i = 0; i < 8; ++i) {
		obj->write_io(data & 0x01);

		obj->write_clk(false);
		obj->delay_us(2);
		obj->write_clk(true);
		obj->delay_us(2);
		data >>= 1;
	}
}

/**
 * @brief  Read a single-byte from DS1302 RTC.
 * @param  obj:       Pointer to a DS1302_Object structure that contains
 *                    the information for the display.
 * @retval byte from DS1302.
 */
static uint8_t DS1302_read_byte(DS1302_Object *obj) {
	uint8_t i, data;
	obj->set_io_output(false);

	data = 0;
	for (i = 0; i < 8; i++) {
		obj->write_clk(true);
		obj->delay_us(2);
		obj->write_clk(false);
		obj->delay_us(2);

		if (obj->read_io()) {
			data |= 0x80;
		}
		if (i < 7)
			data >>= 1;
	}

	return data;
}

/**
 * @brief  Reset and start DS1302 lines for communication. Sets all lines low. Chip enable high.
 * @param  obj:       Pointer to a DS1302_Object structure that contains
 *                    the information for the display.
 */
static void DS1302_restart_communication(DS1302_Object *obj) {
	obj->write_clk(false);
	obj->write_ce(false);
	obj->write_ce(true);
	obj->delay_us(4);

}
/**
 * @brief  End DS1302 communication. Sets all lines low. Chip enable low.
 * @param  obj:       Pointer to a DS1302_Object structure that contains
 *                    the information for the display.
 */
static void DS1302_close_communication(DS1302_Object *obj) {
	obj->write_clk(false);
	obj->write_ce(false);

}

/**
 * @brief  Write data to register of DS1302 RTC.
 * @param  obj:       Pointer to a DS1302_Object structure that contains
 *                    the information for the display.
 * @param  reg: register to write to.
 * @param  data: data to write.
 */
void DS1302_write(DS1302_Object *obj, uint8_t reg, uint8_t data) {
	DS1302_restart_communication(obj);
	DS1302_write_byte(obj, reg);
	DS1302_write_byte(obj, data);
	DS1302_close_communication(obj);
}

/**
 * @brief  Read data from register of DS1302 RTC.
 * @param  obj:       Pointer to a DS1302_Object structure that contains
 *                    the information for the display.
 * @param  reg: register to read from.
 * @retval data from register.
 */
uint8_t DS1302_read(DS1302_Object *obj, uint8_t reg) {
	DS1302_restart_communication(obj);
	DS1302_write_byte(obj, reg);
	temp = DS1302_read_byte(obj);
	DS1302_close_communication(obj);

	return temp;
}

/**
 * @brief  Write data to RAM in DS1302 RTC. (31 bytes available)
 * @param  obj:       Pointer to a DS1302_Object structure that contains
 *                    the information for the display.
 * @param  offset: address offset to write to. Values from 0 to 30
 * @param  data: data to write.
 */
void DS1302_write_ram(DS1302_Object *obj, uint8_t offset, uint8_t data) {
	uint8_t addr = DS1302_RAM_START_W + offset * 2;

	DS1302_restart_communication(obj);
	DS1302_write_byte(obj, addr);
	DS1302_write_byte(obj, data);
	DS1302_close_communication(obj);
}

/**
 * @brief  Write data to RAM in DS1302 RTC in burst mode. (31 bytes available)
 * @param  obj:       Pointer to a DS1302_Object structure that contains
 *                    the information for the display.
 * @param  buf: buffer to read from
 * @param  len: number of bytes to write. max 31
 */
void DS1302_write_ram_burst(DS1302_Object *obj, uint8_t *buf, uint8_t len) {
	DS1302_restart_communication(obj);
	DS1302_write_byte(obj, DS1302_REG_RAM_BURST_W);
	for (temp = 0; temp < len; temp++) {
		DS1302_write_byte(obj, buf[temp]);
	}
	DS1302_close_communication(obj);
}

/**
 * @brief  Read data from RAM in DS1302 RTC. (31 bytes available)
 * @param  obj:       Pointer to a DS1302_Object structure that contains
 *                    the information for the display.
 * @param  offset: address offset to write to. Values from 0 to 30
 * @retval data from RAM.
 */
uint8_t DS1302_read_ram(DS1302_Object *obj, uint8_t offset) {
	uint8_t d = DS1302_RAM_START_R + offset * 2;

	DS1302_restart_communication(obj);
	DS1302_write_byte(obj, d);
	d = DS1302_read_byte(obj);
	DS1302_close_communication(obj);

	return d;
}

/**
 * @brief  Read data from RAM in DS1302 RTC in burst mode. (31 bytes available)
 * @param  obj:       Pointer to a DS1302_Object structure that contains
 *                    the information for the display.
 * @param  buf: buffer to write to
 * @param  len: number of bytes to read. max 31
 */
void DS1302_read_ram_burst(DS1302_Object *obj, uint8_t *buf, uint8_t len) {
	DS1302_restart_communication(obj);
	DS1302_write_byte(obj, DS1302_REG_RAM_BURST_R);
	for (temp = 0; temp < len; temp++) {
		buf[temp] = DS1302_read_byte(obj);
	}
	DS1302_close_communication(obj);
}

///**
// * @brief  Read seconds register and update value in RTC_time object.
// * @param  obj:       Pointer to a DS1302_Object structure that contains
// *                    the information for the display.
// * @param  time:      Pointer to a RTC_time structure that contains time values.
// */
//void DS1302_get_seconds(DS1302_Object *obj, RTC_time *time) {
//	temp = DS1302_read(obj, DS1302_REG_SEC_R);
//	rtc_convert_seconds(time, temp);
//}
//
///**
// * @brief  Read minutes register and update value in RTC_time object.
// * @param  obj:       Pointer to a DS1302_Object structure that contains
// *                    the information for the display.
// * @param  time:      Pointer to a RTC_time structure that contains time values.
// */
//void DS1302_get_minutes(DS1302_Object *obj, RTC_time *time) {
//	temp = DS1302_read(obj, DS1302_REG_MIN_R);
//	rtc_convert_minutes(time, temp);
//}
//
///**
// * @brief  Read hours register and update value in RTC_time object.
// * @param  obj:       Pointer to a DS1302_Object structure that contains
// *                    the information for the display.
// * @param  time:      Pointer to a RTC_time structure that contains time values.
// */
//void DS1302_get_hours(DS1302_Object *obj, RTC_time *time) {
//	temp = DS1302_read(obj, DS1302_REG_HOUR_R);
//	rtc_convert_hours(time, temp);
//}
//
///**
// * @brief  Read day of week register and update value in RTC_time object.
// * @param  obj:       Pointer to a DS1302_Object structure that contains
// *                    the information for the display.
// * @param  time:      Pointer to a RTC_time structure that contains time values.
// */
//void DS1302_get_day_of_week(DS1302_Object *obj, RTC_time *time) {
//	temp = DS1302_read(obj, DS1302_REG_DAY_R);
//	rtc_convert_day_of_week(time, temp);
//}
//
///**
// * @brief  Read day of month (date) register and update value in RTC_time object.
// * @param  obj:       Pointer to a DS1302_Object structure that contains
// *                    the information for the display.
// * @param  time:      Pointer to a RTC_time structure that contains time values.
// */
//void DS1302_get_date(DS1302_Object *obj, RTC_time *time) {
//	temp = DS1302_read(obj, DS1302_REG_DATE_R);
//	rtc_convert_date(time, temp);
//}
//
///**
// * @brief  Read month register and update value in RTC_time object.
// * @param  obj:       Pointer to a DS1302_Object structure that contains
// *                    the information for the display.
// * @param  time:      Pointer to a RTC_time structure that contains time values.
// */
//void DS1302_get_month(DS1302_Object *obj, RTC_time *time) {
//	temp = DS1302_read(obj, DS1302_REG_MONTH_R);
//	rtc_convert_month(time, temp);
//}
//
///**
// * @brief  Read year register and update value in RTC_time object.
// * @param  obj:       Pointer to a DS1302_Object structure that contains
// *                    the information for the display.
// * @param  time:      Pointer to a RTC_time structure that contains time values.
// */
//void DS1302_get_year(DS1302_Object *obj, RTC_time *time) {
//	temp = DS1302_read(obj, DS1302_REG_YEAR_R);
//	rtc_convert_year(time, temp);
//}

/**
 * @brief  Read all registers and update values in RTC_time object.
 * @param  obj:       Pointer to a DS1302_Object structure that contains
 *                    the information for the display.
 * @param  time:      Pointer to a RTC_time structure that contains time values.
 */
void DS1302_read_time(DS1302_Object *obj, RTC_time *time) {
//	DS1302_get_seconds(obj, time);
//	DS1302_get_minutes(obj, time);
//	DS1302_get_hours(obj, time);
//	DS1302_get_day_of_week(obj, time);
//	DS1302_get_date(obj, time);
//	DS1302_get_month(obj, time);
//	DS1302_get_year(obj, time);

	DS1302_restart_communication(obj);
	DS1302_write_byte(obj, DS1302_REG_CLOCK_BURST_R);

	temp = DS1302_read_byte(obj);
	rtc_convert_seconds(time, temp);
	temp = DS1302_read_byte(obj);
	rtc_convert_minutes(time, temp);
	temp = DS1302_read_byte(obj);
	rtc_convert_hours(time, temp);
	temp = DS1302_read_byte(obj);
	rtc_convert_date(time, temp);
	temp = DS1302_read_byte(obj);
	rtc_convert_month(time, temp);
	temp = DS1302_read_byte(obj);
	rtc_convert_day_of_week(time, temp); // 1=sunday, 2=monday, etc.
	temp = DS1302_read_byte(obj);
	rtc_convert_year(time, temp);

	DS1302_close_communication(obj);

}

/**
 * @brief  Write all RTC registers from RTC_time object.
 * @param  obj:       Pointer to a DS1302_Object structure that contains
 *                    the information for the display.
 * @param  time:      Pointer to a RTC_time structure that contains time values.
 */
void DS1302_write_time(DS1302_Object *obj, RTC_time *time) {
	// disable write protection
	DS1302_write(obj, DS1302_REG_W_PROTECT_W, 0);

	DS1302_restart_communication(obj);
	DS1302_write_byte(obj, DS1302_REG_CLOCK_BURST_W);

	// eight bytes must be written

	temp=num_to_bcd(time->second & 0x7f);
	DS1302_write_byte(obj, temp);

	temp=num_to_bcd(time->minute & 0x7f);
	DS1302_write_byte(obj, temp);

	temp=rtc_encode_hours(time->hour, time->hour_format);
	DS1302_write_byte(obj, temp);

	temp=num_to_bcd(time->day_of_month & 0x3f);
	DS1302_write_byte(obj, temp);

	temp=num_to_bcd(time->month & 0x1f);
	DS1302_write_byte(obj, temp);

	temp=time->day_of_week & 0x07;
	DS1302_write_byte(obj, temp);

	temp=num_to_bcd(time->year);
	DS1302_write_byte(obj, temp);

	DS1302_write_byte(obj, 0);

	DS1302_close_communication(obj);

}

///**
// * @brief  Write seconds to register in DS1302 register.
// * @param  obj:       Pointer to a DS1302_Object structure that contains
// *                    the information for the display.
// * @param  val:      Seconds.
// */
//void DS1302_set_seconds(DS1302_Object *obj, uint8_t val) {
//	DS1302_write(obj, DS1302_REG_SEC_W, num_to_bcd(val & 0x7f));
//}
//
///**
// * @brief  Write minutes to register in DS1302 register.
// * @param  obj:       Pointer to a DS1302_Object structure that contains
// *                    the information for the display.
// * @param  val:      Minutes.
// */
//void DS1302_set_minutes(DS1302_Object *obj, uint8_t val) {
//	DS1302_write(obj, DS1302_REG_MIN_W, num_to_bcd(val & 0x7f));
//}
//
///**
// * @brief  Write hours to register in DS1302 register.
// * @param  obj:       Pointer to a DS1302_Object structure that contains
// *                    the information for the display.
// * @param  val:      	Hours.
// * @param  hour_format: Hours format (AM, PM, H24).
// */
//void DS1302_set_hours(DS1302_Object *obj, uint8_t val, RTCHoursFormat hour_format) {
//	temp=rtc_encode_hours(val, hour_format);
//	DS1302_write(obj, DS1302_REG_HOUR_W, temp);
//
//}
//
/**
 * @brief  Write day of week to register in DS1302 register.
 * @param  obj:       Pointer to a DS1302_Object structure that contains
 *                    the information for the display.
 * @param  val:      Day of week. 1=sunday, 2=monday, etc.
 */
void DS1302_set_day_of_week(DS1302_Object *obj, RTCDay val) {
	DS1302_write(obj, DS1302_REG_DAY_W, val & 0x07);
}
//
///**
// * @brief  Write day of month (date) to register in DS1302 register.
// * @param  obj:       Pointer to a DS1302_Object structure that contains
// *                    the information for the display.
// * @param  val:      Day of month.
// */
//void DS1302_set_date(DS1302_Object *obj, uint8_t val) {
//	DS1302_write(obj, DS1302_REG_DATE_W, num_to_bcd(val & 0x3f));
//}
//
///**
// * @brief  Write month to register in DS1302 register.
// * @param  obj:       Pointer to a DS1302_Object structure that contains
// *                    the information for the display.
// * @param  val:      Month.
// */
//void DS1302_set_month(DS1302_Object *obj, uint8_t val) {
//	DS1302_write(obj, DS1302_REG_MONTH_W, num_to_bcd(val & 0x1f));
//}
//
/**
 * @brief  Write year to register in DS1302 register (year 2000 to 2100).
 * @param  obj:       Pointer to a DS1302_Object structure that contains
 *                    the information for the display.
 * @param  val:      Year. 00-99 (00=2000, 99=2099)
 */
void DS1302_set_year(DS1302_Object *obj, uint8_t val) {
	DS1302_write(obj, DS1302_REG_YEAR_W, num_to_bcd(val));
}

/**
 * @brief  Check whether clock is halted or running. Read register and return true if clock is halted.
 * @param  obj:       Pointer to a DS1302_Object structure that contains
 *                    the information for the display.
 * @retval  true if clock is halted.
 */
bool DS1302_clock_halted(DS1302_Object *obj) {
	temp = DS1302_read(obj, DS1302_REG_SEC_R);
	return ((temp & 0x80) != 0);
}

/**
 * @brief  Halt (stop) clock.
 * @param  obj:       Pointer to a DS1302_Object structure that contains
 *                    the information for the display.
 */
void DS1302_clock_halt(DS1302_Object *obj) {
	temp = DS1302_read(obj, DS1302_REG_SEC_R);
	temp = temp | 0x80;
	DS1302_write(obj, DS1302_REG_SEC_W, temp);
}

/**
 * @brief  Start halted clock.
 * @param  obj:       Pointer to a DS1302_Object structure that contains
 *                    the information for the display.
 */
void DS1302_clock_start(DS1302_Object *obj) {
	temp = DS1302_read(obj, DS1302_REG_SEC_R);
	temp = temp & 0x7f;
	DS1302_write(obj, DS1302_REG_SEC_W, temp);
}

/**
 * @brief  Set trickle charging (combination of diodes and resistor).
 * @param  obj:       Pointer to a DS1302_Object structure that contains
 *                    the information for the display.
 * @param  trickle:   Value of DS1302_TrickleCharge
 */
void DS1302_set_trickle_charge(DS1302_Object *obj, DS1302_TrickleCharge trickle) {
	DS1302_write(obj, DS1302_REG_TRICKE_CHARGE_W, trickle);
}

/**
 * @brief  Read trickle charging register.
 * @param  obj:       Pointer to a DS1302_Object structure that contains
 *                    the information for the display.
 * @retval  trickle:   Value of DS1302_TrickleCharge
 */
DS1302_TrickleCharge DS1302_get_trickle_charge(DS1302_Object *obj) {
	temp = DS1302_read(obj, DS1302_REG_TRICKE_CHARGE_R);
	return temp;
}

/**
 * @brief  Set write protect.
 * @param  obj:       Pointer to a DS1302_Object structure that contains
 *                    the information for the display.
 * @param  write_protect_on:   true = write protect on
 */
//void DS1302_set_write_protect(DS1302_Object *obj, bool write_protect_on) {
//	DS1302_write(obj, DS1302_REG_W_PROTECT_W, write_protect_on?0x80:0);
//}

//uint8_t DS1302_get_write_protect(DS1302_Object *obj) {
//	return DS1302_read(obj, DS1302_REG_W_PROTECT_R);
//}
