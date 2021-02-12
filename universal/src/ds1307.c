/**
 ******************************************************************************
 * @file    ds1307.c
 * @author  Juraj Lonc (juraj.lonc@gmail.com)
 * @brief   DS1307 RTC driver.
 ******************************************************************************
 */

// UNTESTED YET

#include "ds1307.h"

static uint8_t temp;
static uint8_t temp_arr[7];

/**
 * @brief  Initialize DS1307 RTC and DS1307 object/handler.
 * @param  obj:       Pointer to a DS1307_Object structure that contains
 *                    the information for the display.
 * @param  write_cmd:		The function that writes data to DS1307. Hardware dependent.
 * @param  read_cmd:		The function that reads data from DS1307. Hardware dependent.
 */
void DS1307_init(DS1307_Object * obj, device_write_ptr write_cmd, device_read_ptr read_cmd) {
	obj->write_cmd=write_cmd;
	obj->read_cmd=read_cmd;
}

/**
 * @brief  Write data to RAM in DS1307 RTC. (56 bytes available)
 * @param  obj:       Pointer to a DS1307_Object structure that contains
 *                    the information for the display.
 * @param  offset: address offset to write to. Values from 0 to 55
 * @param  data: data to write.
 */
void DS1307_write_ram(DS1307_Object *obj, uint8_t offset, uint8_t data) {
	obj->write_cmd(obj, DS1307_RAM_START + offset, &data, 1);
}

/**
 * @brief  Read data from RAM in DS1307 RTC. (56 bytes available)
 * @param  obj:       Pointer to a DS1307_Object structure that contains
 *                    the information for the display.
 * @param  offset: address offset to write to. Values from 0 to 55
 * @retval data from RAM.
 */
uint8_t DS1307_read_ram(DS1307_Object *obj, uint8_t offset) {
	obj->read_cmd(obj, DS1307_RAM_START + offset, &temp, 1);
	return temp;
}

/**
 * @brief  Write data to RAM in DS1307 RTC in burst mode. (56 bytes available)
 * @param  obj:       Pointer to a DS1307_Object structure that contains
 *                    the information for the display.
 * @param  buf: buffer to read from
 * @param  len: number of bytes to write. max 56
 */
void DS1307_write_ram_burst(DS1307_Object *obj, uint8_t *buf, uint8_t len) {
	obj->write_cmd(obj, DS1307_RAM_START, buf, len);
}

/**
 * @brief  Read data from RAM in DS1307 RTC in burst mode. (56 bytes available)
 * @param  obj:       Pointer to a DS1307_Object structure that contains
 *                    the information for the display.
 * @param  buf: buffer to write to
 * @param  len: number of bytes to read. max 56
 */
void DS1307_read_ram_burst(DS1307_Object *obj, uint8_t *buf, uint8_t len) {
	obj->read_cmd(obj, DS1307_RAM_START, buf, len);
}

///**
// * @brief  Read seconds register and update value in RTC_time object.
// * @param  obj:       Pointer to a DS1307_Object structure that contains
// *                    the information for the display.
// * @param  time:      Pointer to a RTC_time structure that contains time values.
// */
//void DS1307_get_seconds(DS1307_Object *obj, RTC_time * time) {
//	obj->read_cmd(obj, DS1307_REG_SEC, &temp, 1);
//	rtc_convert_seconds(time, temp);
//}
//
///**
// * @brief  Read minutes register and update value in RTC_time object.
// * @param  obj:       Pointer to a DS1307_Object structure that contains
// *                    the information for the display.
// * @param  time:      Pointer to a RTC_time structure that contains time values.
// */
//void DS1307_get_minutes(DS1307_Object *obj, RTC_time *time) {
//	obj->read_cmd(obj, DS1307_REG_MIN, &temp, 1);
//	rtc_convert_minutes(time, temp);
//}
//
///**
// * @brief  Read hours register and update value in RTC_time object.
// * @param  obj:       Pointer to a DS1307_Object structure that contains
// *                    the information for the display.
// * @param  time:      Pointer to a RTC_time structure that contains time values.
// */
//void DS1307_get_hours(DS1307_Object *obj, RTC_time *time) {
//	obj->read_cmd(obj, DS1307_REG_HOUR, &temp, 1);
//	rtc_convert_hours(time, temp);
//}
//
///**
// * @brief  Read day of week register and update value in RTC_time object.
// * @param  obj:       Pointer to a DS1307_Object structure that contains
// *                    the information for the display.
// * @param  time:      Pointer to a RTC_time structure that contains time values.
// */
//void DS1307_get_day_of_week(DS1307_Object *obj, RTC_time *time) {
//	obj->read_cmd(obj, DS1307_REG_DAY, &temp, 1);
//	rtc_convert_day_of_week(time, temp);
//}
//
///**
// * @brief  Read day of month (date) register and update value in RTC_time object.
// * @param  obj:       Pointer to a DS1307_Object structure that contains
// *                    the information for the display.
// * @param  time:      Pointer to a RTC_time structure that contains time values.
// */
//void DS1307_get_date(DS1307_Object *obj, RTC_time *time) {
//	obj->read_cmd(obj, DS1307_REG_DATE, &temp, 1);
//	rtc_convert_date(time, temp);
//}
//
///**
// * @brief  Read month register and update value in RTC_time object.
// * @param  obj:       Pointer to a DS1307_Object structure that contains
// *                    the information for the display.
// * @param  time:      Pointer to a RTC_time structure that contains time values.
// */
//void DS1307_get_month(DS1307_Object *obj, RTC_time *time) {
//	obj->read_cmd(obj, DS1307_REG_MONTH, &temp, 1);
//	rtc_convert_month(time, temp);
//}
//
///**
// * @brief  Read year register and update value in RTC_time object.
// * @param  obj:       Pointer to a DS1307_Object structure that contains
// *                    the information for the display.
// * @param  time:      Pointer to a RTC_time structure that contains time values.
// */
//void DS1307_get_year(DS1307_Object *obj, RTC_time *time) {
//	obj->read_cmd(obj, DS1307_REG_YEAR, &temp, 1);
//	rtc_convert_year(time, temp);
//}

/**
 * @brief  Read all registers and update values in RTC_time object.
 * @param  obj:       Pointer to a DS1307_Object structure that contains
 *                    the information for the display.
 * @param  time:      Pointer to a RTC_time structure that contains time values.
 */
void DS1307_read_time(DS1307_Object *obj, RTC_time *time) {
	obj->read_cmd(obj, DS1307_REG_SEC, temp_arr, 7);

	rtc_convert_seconds(time, temp_arr[0]);
	rtc_convert_minutes(time, temp_arr[1]);
	rtc_convert_hours(time, temp_arr[2]);
	rtc_convert_day_of_week(time, temp_arr[3]);  // 1=sunday, 2=monday, etc.
	rtc_convert_date(time, temp_arr[4]);
	rtc_convert_month(time, temp_arr[5]);
	rtc_convert_year(time, temp_arr[6]);
}


/**
 * @brief  Write all RTC registers from RTC_time object.
 * @param  obj:       Pointer to a DS1307_Object structure that contains
 *                    the information for the display.
 * @param  time:      Pointer to a RTC_time structure that contains time values.
 */
void DS1307_write_time(DS1307_Object *obj, RTC_time *time) {
	temp_arr[0]=num_to_bcd(time->second & 0x7f);
	temp_arr[1]=num_to_bcd(time->minute & 0x7f);
	temp_arr[2]=rtc_encode_hours(time->hour, time->hour_format);
	temp_arr[3]=time->day_of_week & 0x07;
	temp_arr[4]=num_to_bcd(time->day_of_month & 0x3f);
	temp_arr[5]=num_to_bcd(time->month & 0x1f);
	temp_arr[6]=num_to_bcd(time->year);

	obj->write_cmd(obj, DS1307_REG_SEC, temp_arr, 7);
}

///**
// * @brief  Write seconds to register in DS1307 register.
// * @param  obj:       Pointer to a DS1307_Object structure that contains
// *                    the information for the display.
// * @param  val:      Seconds.
// */
//void DS1307_set_seconds(DS1307_Object *obj, uint8_t val) {
//	temp=num_to_bcd(val & 0x7f);
//	obj->write_cmd(obj, DS1307_REG_SEC, &temp, 1);
//}
//
///**
// * @brief  Write minutes to register in DS1307 register.
// * @param  obj:       Pointer to a DS1307_Object structure that contains
// *                    the information for the display.
// * @param  val:      Minutes.
// */
//void DS1307_set_minutes(DS1307_Object *obj, uint8_t val) {
//	temp=num_to_bcd(val & 0x7f);
//	obj->write_cmd(obj, DS1307_REG_MIN, &temp, 1);
//}
//
///**
// * @brief  Write hours to register in DS1307 register.
// * @param  obj:       Pointer to a DS1307_Object structure that contains
// *                    the information for the display.
// * @param  val:      	Hours.
// * @param  hour_format: Hours format (AM, PM, H24).
// */
//void DS1307_set_hours(DS1307_Object *obj, uint8_t val, RTCHoursFormat hour_format) {
//	temp=rtc_encode_hours(val, hour_format);
//
//	obj->write_cmd(obj, DS1307_REG_HOUR, &temp, 1);
//}
//
/**
 * @brief  Write day of week to register in DS1307 register.
 * @param  obj:       Pointer to a DS1307_Object structure that contains
 *                    the information for the display.
 * @param  val:      Day of week. 1=sunday, 2=monday, etc.
 */
void DS1307_set_day_of_week(DS1307_Object *obj, RTCDay val) {
	temp=val & 0x07;
	obj->write_cmd(obj, DS1307_REG_DAY, &temp, 1);
}
//
///**
// * @brief  Write day of month (date) to register in DS1307 register.
// * @param  obj:       Pointer to a DS1307_Object structure that contains
// *                    the information for the display.
// * @param  val:      Day of month.
// */
//void DS1307_set_date(DS1307_Object *obj, uint8_t val) {
//	temp=num_to_bcd(val & 0x3f);
//	obj->write_cmd(obj, DS1307_REG_DATE, &temp, 1);
//}
//
///**
// * @brief  Write month to register in DS1307 register.
// * @param  obj:       Pointer to a DS1307_Object structure that contains
// *                    the information for the display.
// * @param  val:      Month.
// */
//void DS1307_set_month(DS1307_Object *obj, uint8_t val) {
//	temp=num_to_bcd(val & 0x1f);
//	obj->write_cmd(obj, DS1307_REG_MONTH, &temp, 1);
//}
//
///**
// * @brief  Write year to register in DS1307 register (year 2000 to 2100).
// * @param  obj:       Pointer to a DS1307_Object structure that contains
// *                    the information for the display.
// * @param  val:      Year. 00-99 (00=2000, 99=2099)
// */
//void DS1307_set_year(DS1307_Object *obj, uint8_t val) {
//	temp=num_to_bcd(val);
//	obj->write_cmd(obj, DS1307_REG_YEAR, &temp, 1);
//}

/**
 * @brief  Check whether clock is halted or running. Read register and return true if clock is halted.
 * @param  obj:       Pointer to a DS1307_Object structure that contains
 *                    the information for the display.
 * @retval  true if clock is halted.
 */
bool DS1307_clock_halted(DS1307_Object *obj) {
	obj->read_cmd(obj, DS1307_REG_SEC, &temp, 1);
	return ((temp & 0x80) != 0);
}

/**
 * @brief  Halt (stop) clock.
 * @param  obj:       Pointer to a DS1307_Object structure that contains
 *                    the information for the display.
 */
void DS1307_clock_halt(DS1307_Object *obj) {
	obj->read_cmd(obj, DS1307_REG_SEC, &temp, 1);
	temp = temp | 0x80;
	obj->write_cmd(obj, DS1307_REG_SEC, &temp, 1);
}

/**
 * @brief  Start halted clock.
 * @param  obj:       Pointer to a DS1307_Object structure that contains
 *                    the information for the display.
 */
void DS1307_clock_start(DS1307_Object *obj) {
	obj->read_cmd(obj, DS1307_REG_SEC, &temp, 1);
	temp = temp & 0x7f;
	obj->write_cmd(obj, DS1307_REG_SEC, &temp, 1);
}

/**
 * @brief  Set output control register.
 * @param  obj:       Pointer to a DS1307_Object structure that contains
 *                    the information for the display.
 * @param  output:    Value of DS1307_OutputControl
 */
void DS1307_set_output_control(DS1307_Object *obj, DS1307_OutputControl output) {
	obj->write_cmd(obj, DS1307_REG_CONTROL, &output, 1);
}

/**
 * @brief  Read output control register.
 * @param  obj:       Pointer to a DS1307_Object structure that contains
 *                    the information for the display.
 * @retval  output:   Value of DS1307_OutputControl
 */
DS1307_OutputControl DS1307_get_output_control(DS1307_Object *obj) {
	obj->read_cmd(obj, DS1307_REG_CONTROL, &temp, 1);
	return temp;
}
