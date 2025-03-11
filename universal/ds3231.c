/**
 ******************************************************************************
 * @file    ds3231.c
 * @author  Juraj Lonc (juraj.lonc@gmail.com)
 * @brief   DS3231 RTC driver.
 ******************************************************************************
 */

#include "ds3231.h"

static uint8_t temp;
static uint8_t temp_arr[7];

/**
 * @brief  Initialize DS3231 RTC and DS3231 object/handler.
 * @param  handle:       Pointer to a DS3231_Handle structure that contains
 *                    the information for the display.
 * @param  write_cmd:		The function that writes data to DS3231. Hardware dependent.
 * @param  read_cmd:		The function that reads data from DS3231. Hardware dependent.
 */
void DS3231_init(DS3231_Handle * handle, device_write_ptr write_cmd, device_read_ptr read_cmd) {
	handle->write_cmd=write_cmd;
	handle->read_cmd=read_cmd;
}

///**
// * @brief  Read seconds register and update value in RTC_time object.
// * @param  handle:       Pointer to a DS3231_Handle structure that contains
// *                    the information for the display.
// * @param  time:      Pointer to a RTC_time structure that contains time values.
// */
//void DS3231_get_seconds(DS3231_Handle *handle, RTC_time * time) {
//	handle->read_cmd(handle, DS3231_REG_SECOND, &temp, 1);
//	rtc_convert_seconds(time, temp);
//}
//
///**
// * @brief  Read minutes register and update value in RTC_time object.
// * @param  handle:       Pointer to a DS3231_Handle structure that contains
// *                    the information for the display.
// * @param  time:      Pointer to a RTC_time structure that contains time values.
// */
//void DS3231_get_minutes(DS3231_Handle *handle, RTC_time *time) {
//	handle->read_cmd(handle, DS3231_REG_MINUTE, &temp, 1);
//	rtc_convert_minutes(time, temp);
//}
//
///**
// * @brief  Read hours register and update value in RTC_time object.
// * @param  handle:       Pointer to a DS3231_Handle structure that contains
// *                    the information for the display.
// * @param  time:      Pointer to a RTC_time structure that contains time values.
// */
//void DS3231_get_hours(DS3231_Handle *handle, RTC_time *time) {
//	handle->read_cmd(handle, DS3231_REG_HOUR, &temp, 1);
//	rtc_convert_hours(time, temp);
//}
//
///**
// * @brief  Read day of week register and update value in RTC_time object.
// * @param  handle:       Pointer to a DS3231_Handle structure that contains
// *                    the information for the display.
// * @param  time:      Pointer to a RTC_time structure that contains time values.
// */
//void DS3231_get_day_of_week(DS3231_Handle *handle, RTC_time *time) {
//	handle->read_cmd(handle, DS3231_REG_DAY, &temp, 1);
//	rtc_convert_day_of_week(time, temp);
//}
//
///**
// * @brief  Read day of month (date) register and update value in RTC_time object.
// * @param  handle:       Pointer to a DS3231_Handle structure that contains
// *                    the information for the display.
// * @param  time:      Pointer to a RTC_time structure that contains time values.
// */
//void DS3231_get_date(DS3231_Handle *handle, RTC_time *time) {
//	handle->read_cmd(handle, DS3231_REG_DATE, &temp, 1);
//	rtc_convert_date(time, temp);
//}
//
///**
// * @brief  Read month register and update value in RTC_time object.
// * @param  handle:       Pointer to a DS3231_Handle structure that contains
// *                    the information for the display.
// * @param  time:      Pointer to a RTC_time structure that contains time values.
// */
//void DS3231_get_month(DS3231_Handle *handle, RTC_time *time) {
//	handle->read_cmd(handle, DS3231_REG_MONTH, &temp, 1);
//	rtc_convert_month(time, temp);
//}
//
///**
// * @brief  Read year register and update value in RTC_time object.
// * @param  handle:       Pointer to a DS3231_Handle structure that contains
// *                    the information for the display.
// * @param  time:      Pointer to a RTC_time structure that contains time values.
// */
//void DS3231_get_year(DS3231_Handle *handle, RTC_time *time) {
//	handle->read_cmd(handle, DS3231_REG_YEAR, &temp, 1);
//	rtc_convert_year(time, temp);
//}

/**
 * @brief  Read all registers and update values in RTC_time object.
 * @param  handle:       Pointer to a DS3231_Handle structure that contains
 *                    the information for the display.
 * @param  time:      Pointer to a RTC_time structure that contains time values.
 */
void DS3231_read_time(DS3231_Handle *handle, RTC_time * time) {
	handle->read_cmd(handle, DS3231_REG_SECOND, temp_arr, 7);
	rtc_convert_seconds(time, temp_arr[0]);
	rtc_convert_minutes(time, temp_arr[1]);
	rtc_convert_hours(time, temp_arr[2]);
	rtc_convert_day_of_week(time, temp_arr[3]); // 1=sunday, 2=monday, etc.
	rtc_convert_date(time, temp_arr[4]);
	rtc_convert_month(time, temp_arr[5]);
	rtc_convert_year(time, temp_arr[6]);
}

///**
// * @brief  Write seconds to register in DS3231 register.
// * @param  handle:       Pointer to a DS3231_Handle structure that contains
// *                    the information for the display.
// * @param  val:      Seconds.
// */
//void DS3231_set_seconds(DS3231_Handle *handle, uint8_t val) {
//	temp=num_to_bcd(val & 0x7f);
//	handle->write_cmd(handle, DS3231_REG_SECOND, &temp, 1);
//}
//
///**
// * @brief  Write minutes to register in DS3231 register.
// * @param  handle:       Pointer to a DS3231_Handle structure that contains
// *                    the information for the display.
// * @param  val:      Minutes.
// */
//void DS3231_set_minutes(DS3231_Handle *handle, uint8_t val) {
//	temp=num_to_bcd(val & 0x7f);
//	handle->write_cmd(handle, DS3231_REG_MINUTE, &temp, 1);
//}
//
///**
// * @brief  Write hours to register in DS3231 register.
// * @param  handle:       Pointer to a DS3231_Handle structure that contains
// *                    the information for the display.
// * @param  val:      	Hours.
// * @param  hour_format: Hours format (AM, PM, H24).
// */
//void DS3231_set_hours(DS3231_Handle *handle, uint8_t val, RTCHoursFormat hour_format) {
//	temp=rtc_encode_hours(val, hour_format);
//
//	handle->write_cmd(handle, DS3231_REG_HOUR, &temp, 1);
//}
//
/**
 * @brief  Write day of week to register in DS3231 register.
 * @param  handle:       Pointer to a DS3231_Handle structure that contains
 *                    the information for the display.
 * @param  val:      Day of week. 1=sunday, 2=monday, etc.
 */
void DS3231_set_day_of_week(DS3231_Handle *handle, RTCDay val) {
	temp=val & 0x07;
	handle->write_cmd(handle, DS3231_REG_DAY, &temp, 1);
}
//
///**
// * @brief  Write day of month (date) to register in DS3231 register.
// * @param  handle:       Pointer to a DS3231_Handle structure that contains
// *                    the information for the display.
// * @param  val:      Day of month.
// */
//void DS3231_set_date(DS3231_Handle *handle, uint8_t val) {
//	temp=num_to_bcd(val & 0x3f);
//	handle->write_cmd(handle, DS3231_REG_DATE, &temp, 1);
//}
//
///**
// * @brief  Write month to register in DS3231 register.
// * @param  handle:       Pointer to a DS3231_Handle structure that contains
// *                    the information for the display.
// * @param  val:      Month.
// */
//void DS3231_set_month(DS3231_Handle *handle, uint8_t val) {
//	temp=num_to_bcd(val & 0x1f);
//	handle->write_cmd(handle, DS3231_REG_MONTH, &temp, 1);
//}
//
///**
// * @brief  Write year to register in DS3231 register (year 2000 to 2100).
// * @param  handle:       Pointer to a DS3231_Handle structure that contains
// *                    the information for the display.
// * @param  val:      Year. 00-99 (00=2000, 99=2099)
// */
//void DS3231_set_year(DS3231_Handle *handle, uint8_t val) {
//	temp=num_to_bcd(val);
//	handle->write_cmd(handle, DS3231_REG_YEAR, &temp, 1);
//}

/**
 * @brief  Read integer temperature from DS3231.
 * @param  handle:       Pointer to a DS3231_Handle structure that contains
 *                    the information for the display.
 * @retval  Temperature in hundredths of deg.C. (2500 == 25.00 deg.C)
 */
int16_t DS3231_get_temperature_int(DS3231_Handle *handle) {
	// arr[0]=MSB, arr[1]=LSB
	handle->read_cmd(handle, DS3231_REG_TEMP_MSB, temp_arr, 2);

	uint16_t t;
	t=(int16_t)(int8_t)temp_arr[0] << 2 | temp_arr[1] >> 6;
	t*=25;

	return t;
}

/**
 * @brief  Read float temperature from DS3231.
 * @param  handle:       Pointer to a DS3231_Handle structure that contains
 *                    the information for the display.
 * @retval  Temperature in deg.C.
 */
float DS3231_get_temperature(DS3231_Handle *handle) {
	return DS3231_get_temperature_int(handle)*0.01;
}

/**
 * @brief  Write all RTC registers from RTC_time object.
 * @param  handle:       Pointer to a DS3231_Handle structure that contains
 *                    the information for the display.
 * @param  time:      Pointer to a RTC_time structure that contains time values.
 */
void DS3231_write_time(DS3231_Handle *handle, RTC_time * time) {
	temp_arr[0]=num_to_bcd(time->second & 0x7f);
	temp_arr[1]=num_to_bcd(time->minute & 0x7f);
	temp_arr[2]=rtc_encode_hours(time->hour, time->hour_format);
	temp_arr[3]=time->day_of_week & 0x07;
	temp_arr[4]=num_to_bcd(time->day_of_month & 0x3f);
	temp_arr[5]=num_to_bcd(time->month & 0x1f);
	temp_arr[6]=num_to_bcd(time->year);
	
	handle->write_cmd(handle, DS3231_REG_SECOND, temp_arr, 7);
}

/**
 * @brief  Set time.
 * @param  handle:       Pointer to a DS3231_Handle structure that contains
 *                    the information for the display.
 * @param  hours:     Hours value in 24hr format.
 * @param  minutes:   Minutes value.
 * @param  seconds:   Seconds value.
 */
void DS3231_set_time_values(DS3231_Handle *handle, uint8_t hours, uint8_t minutes, uint8_t seconds) {
	temp_arr[0] = num_to_bcd(seconds & 0x7f);
	temp_arr[1] = num_to_bcd(minutes & 0x7f);
	temp_arr[2] = rtc_encode_hours(hours, RTC_H24);

	handle->write_cmd(handle, DS3231_REG_SECOND, temp_arr, 3);
}

/**
 * @brief  Set date.
 * @param  handle:       Pointer to a DS3231_Handle structure that contains
 *                    the information for the display.
 * @param  year:     	Year value (00-99).
 * @param  month:   	Month value.
 * @param  day_of_month: Day of month value.
 */
void DS3231_set_date_values(DS3231_Handle *handle, uint8_t year, uint8_t month, uint8_t day_of_month) {
	temp_arr[0] = num_to_bcd(day_of_month & 0x3f);
	temp_arr[1] = num_to_bcd(month & 0x1f);
	temp_arr[2] = num_to_bcd(year);

	handle->write_cmd(handle, DS3231_REG_DATE, temp_arr, 3);
}

/**
 * @brief  Read alarm1 registers from DS3231.
 * @param  handle:       Pointer to a DS3231_Handle structure that contains
 *                    the information for the display.
 * @param  alarm1:    Pointer to DS3231_Alarm1 object
 */
void DS3231_read_alarm1(DS3231_Handle *handle, DS3231_Alarm1 * alarm1) {
	handle->read_cmd(handle, DS3231_REG_ALARM1_SEC, alarm1->DATA, 4);
}

/**
 * @brief  Write alarm1 registers to DS3231.
 * @param  handle:       Pointer to a DS3231_Handle structure that contains
 *                    the information for the display.
 * @param  alarm1:    Pointer to DS3231_Alarm1 object
 */
void DS3231_write_alarm1(DS3231_Handle *handle, DS3231_Alarm1 * alarm1) {
	handle->write_cmd(handle, DS3231_REG_ALARM1_SEC, alarm1->DATA, 4);
}

/**
 * @brief  Set and enable alarm1 in DS3231.
 * @param  handle:       Pointer to a DS3231_Handle structure that contains
 *                    the information for the display.
 * @param  date_day:  Day (or date) value
 * @param  hours:     Hours value, in 24hr format
 * @param  minutes:   Minutes value
 * @param  seconds:   Seconds value
 * @param  rate:      Rate of alarm1
 */
void DS3231_set_alarm1(DS3231_Handle *handle, uint8_t date_day, uint8_t hours, uint8_t minutes, uint8_t seconds, DS3231_Alarm1Rate rate) {
	DS3231_Alarm1 * alarm1;
	alarm1=(DS3231_Alarm1*)temp_arr;

	alarm1->seconds=num_to_bcd(seconds);
	alarm1->minutes=num_to_bcd(minutes);
	alarm1->hours=rtc_encode_hours(hours, RTC_H24);
	alarm1->date_day=num_to_bcd(date_day);

	alarm1->A1M1=(rate==DS3231_ALARM1_ONCE_PER_SECOND?1:0);
	alarm1->A1M2=((rate==DS3231_ALARM1_ONCE_PER_SECOND || rate==DS3231_ALARM1_WHEN_SECONDS_MATCH)?1:0);
	alarm1->A1M3=((rate==DS3231_ALARM1_ONCE_PER_SECOND || rate==DS3231_ALARM1_WHEN_SECONDS_MATCH || rate==DS3231_ALARM1_WHEN_MINUTES_SECONDS_MATCH)?1:0);
	alarm1->A1M4=((rate==DS3231_ALARM1_ONCE_PER_SECOND || rate==DS3231_ALARM1_WHEN_SECONDS_MATCH || rate==DS3231_ALARM1_WHEN_MINUTES_SECONDS_MATCH || rate==DS3231_ALARM1_WHEN_HOURS_MINUTES_SECONDS_MATCH)?1:0);
	alarm1->DY=(rate==DS3231_ALARM1_WHEN_DAY_HOURS_MINUTES_SECONDS_MATCH?1:0);

	DS3231_read_control(handle, (DS3231_Control*)&temp);
	((DS3231_Control*)&temp)->A1IE=1;
	DS3231_write_control(handle, (DS3231_Control*)&temp);
}

/**
 * @brief  Disable alarm1 in DS3231.
 * @param  handle:       Pointer to a DS3231_Handle structure that contains
 *                    the information for the display.
 */
void DS3231_disable_alarm1(DS3231_Handle *handle) {
	DS3231_read_control(handle, (DS3231_Control*)&temp);
	((DS3231_Control*)&temp)->A1IE=0;
	DS3231_write_control(handle, (DS3231_Control*)&temp);
}

/**
 * @brief  Set and enable alarm2 in DS3231.
 * @param  handle:       Pointer to a DS3231_Handle structure that contains
 *                    the information for the display.
 * @param  date_day:  Day (or date) value
 * @param  hours:     Hours value, in 24hr format
 * @param  minutes:   Minutes value
 * @param  rate:      Rate of alarm2
 */
void DS3231_set_alarm2(DS3231_Handle *handle, uint8_t date_day, uint8_t hours, uint8_t minutes, DS3231_Alarm2Rate rate) {
	DS3231_Alarm2 * alarm2;
	alarm2=(DS3231_Alarm2*)temp_arr;

	alarm2->minutes=num_to_bcd(minutes);
	alarm2->hours=rtc_encode_hours(hours, RTC_H24);
	alarm2->date_day=num_to_bcd(date_day);

	alarm2->A2M2=((rate==DS3231_ALARM2_ONCE_PER_MINUTE)?1:0);
	alarm2->A2M3=((rate==DS3231_ALARM2_ONCE_PER_MINUTE || rate==DS3231_ALARM2_WHEN_MINUTES_MATCH)?1:0);
	alarm2->A2M4=((rate==DS3231_ALARM2_ONCE_PER_MINUTE || rate==DS3231_ALARM2_WHEN_MINUTES_MATCH || rate==DS3231_ALARM2_WHEN_HOURS_MINUTES_MATCH)?1:0);
	alarm2->DY=(rate==DS3231_ALARM2_WHEN_DAY_HOURS_MINUTES_MATCH?1:0);

	DS3231_read_control(handle, (DS3231_Control*)&temp);
	((DS3231_Control*)&temp)->A2IE=1;
	DS3231_write_control(handle, (DS3231_Control*)&temp);
}

/**
 * @brief  Disable alarm2 in DS3231.
 * @param  handle:       Pointer to a DS3231_Handle structure that contains
 *                    the information for the display.
 */
void DS3231_disable_alarm2(DS3231_Handle *handle) {
	DS3231_read_control(handle, (DS3231_Control*)&temp);
	((DS3231_Control*)&temp)->A2IE=0;
	DS3231_write_control(handle, (DS3231_Control*)&temp);
}

/**
 * @brief  Read alarm2 registers from DS3231.
 * @param  handle:       Pointer to a DS3231_Handle structure that contains
 *                    the information for the display.
 * @param  alarm2:    Pointer to DS3231_Alarm2 object
 */
void DS3231_read_alarm2(DS3231_Handle *handle, DS3231_Alarm2 * alarm2) {
	handle->read_cmd(handle, DS3231_REG_ALARM2_MIN, alarm2->DATA, 3);
}

/**
 * @brief  Write alarm2 registers to DS3231.
 * @param  handle:       Pointer to a DS3231_Handle structure that contains
 *                    the information for the display.
 * @param  alarm2:    Pointer to DS3231_Alarm2 object
 */
void DS3231_write_alarm2(DS3231_Handle *handle, DS3231_Alarm2 * alarm2) {
	handle->write_cmd(handle, DS3231_REG_ALARM2_MIN, alarm2->DATA, 3);
}

/**
 * @brief  Read Control register from DS3231.
 * @param  handle:       Pointer to a DS3231_Handle structure that contains
 *                    the information for the display.
 * @param  control:    Pointer to DS3231_Control object
 */
void DS3231_read_control(DS3231_Handle *handle, DS3231_Control * ctrl) {
	handle->read_cmd(handle, DS3231_REG_CONTROL, &(ctrl->BYTE), 1);
}

/**
 * @brief  Write Control register to DS3231.
 * @param  handle:       Pointer to a DS3231_Handle structure that contains
 *                    the information for the display.
 * @param  control:   Pointer to DS3231_Control object
 */
void DS3231_write_control(DS3231_Handle *handle, DS3231_Control * ctrl) {
	handle->write_cmd(handle, DS3231_REG_CONTROL, &(ctrl->BYTE), 1);
}

/**
 * @brief  Read Status register from DS3231.
 * @param  handle:       Pointer to a DS3231_Handle structure that contains
 *                    the information for the display.
 * @param  status:    Pointer to DS3231_Status object
 */
void DS3231_read_status(DS3231_Handle *handle, DS3231_Status * status) {
	handle->read_cmd(handle, DS3231_REG_STATUS, (uint8_t*)&status, 1);
}

/**
 * @brief  Write Status register to DS3231.
 * @param  handle:       Pointer to a DS3231_Handle structure that contains
 *                    the information for the display.
 * @param  status:    Pointer to DS3231_Status object
 */
void DS3231_write_status(DS3231_Handle *handle, DS3231_Status * status) {
	handle->write_cmd(handle, DS3231_REG_STATUS, (uint8_t*)&status, 1);
}

/**
 * @brief  Read Aging Offset register from DS3231.
 * @param  handle:       Pointer to a DS3231_Handle structure that contains
 *                    the information for the display.
 * @retval  aging offset
 */
int8_t DS3231_read_aging_offset(DS3231_Handle *handle) {
	handle->read_cmd(handle, DS3231_REG_AGEOFFSET, &temp, 1);

	return (int8_t)temp;
}

/**
 * @brief  Start temperature conversion, if it is not running.
 * @param  handle:       Pointer to a DS3231_Handle structure that contains
 *                    the information for the display.
 */
void DS3231_start_conversion(DS3231_Handle *handle) {
	handle->read_cmd(handle, DS3231_REG_STATUS, &temp, 1);
	if (((DS3231_Status*)&temp)->BSY==1) {
		return;
	}
	handle->read_cmd(handle, DS3231_REG_CONTROL, &temp, 1);
	((DS3231_Control*)&temp)->CONV=1;
	handle->write_cmd(handle, DS3231_REG_CONTROL, &temp, 1);
}

/**
 * @brief  Check whether conversion is done or still running (busy).
 * @param  handle:       Pointer to a DS3231_Handle structure that contains
 *                    the information for the display.
 * @retval true if conversion is done
 */
bool DS3231_is_conversion_done(DS3231_Handle *handle) {
	handle->read_cmd(handle, DS3231_REG_CONTROL, &temp, 1);
	if (((DS3231_Control*)&temp)->CONV==1) {
		return false;
	}
	handle->read_cmd(handle, DS3231_REG_STATUS, &temp, 1);
	if (((DS3231_Status*)&temp)->BSY==1) {
		return false;
	}
	return true;
}
