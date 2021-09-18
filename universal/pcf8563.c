/**
 ******************************************************************************
 * @file    pcf8563.c
 * @author  Juraj Lonc (juraj.lonc@gmail.com)
 * @brief   PCF8563 RTC driver.
 ******************************************************************************
 */

#include "pcf8563.h"

static uint8_t temp;
static uint8_t temp_arr[7];

/**
 * @brief  Initialize PCF8563 RTC and PCF8563 object/handler.
 * @param  obj:       Pointer to a PCF8563_Object structure that contains
 *                    the information for the display.
 * @param  write_cmd:		The function that writes data to PCF8563. Hardware dependent.
 * @param  read_cmd:		The function that reads data from PCF8563. Hardware dependent.
 */
void PCF8563_init(PCF8563_Object * obj, device_write_ptr write_cmd, device_read_ptr read_cmd) {
	obj->write_cmd=write_cmd;
	obj->read_cmd=read_cmd;
}

/**
 * @brief  Read all registers and update values in RTC_time object.
 * @param  obj:       Pointer to a PCF8563_Object structure that contains
 *                    the information for the display.
 * @param  time:      Pointer to a RTC_time structure that contains time values.
 */
void PCF8563_read_time(PCF8563_Object *obj, RTC_time * time) {
	obj->read_cmd(obj, PCF8563_REG_SECONDS, temp_arr, 7);

	rtc_convert_seconds(time, temp_arr[0]);
	rtc_convert_minutes(time, temp_arr[1]);
	rtc_convert_hours(time, temp_arr[2]);
	rtc_convert_date(time, temp_arr[4]);
	rtc_convert_day_of_week(time, temp_arr[3]+1); // PCF8563: 0=sunday, 1=monday, etc. -> RTC: 1=sunday, 2=monday, etc.
	rtc_convert_month(time, temp_arr[5]);
	rtc_convert_year(time, temp_arr[6]);
}

/**
 * @brief  Write all RTC registers from RTC_time object.
 * @param  obj:       Pointer to a PCF8563_Object structure that contains
 *                    the information for the display.
 * @param  time:      Pointer to a RTC_time structure that contains time values.
 */
void PCF8563_write_time(PCF8563_Object *obj, RTC_time * time) {
	temp_arr[0]=num_to_bcd(time->second & 0x7f);
	temp_arr[1]=num_to_bcd(time->minute & 0x7f);
	temp_arr[2]=rtc_encode_hours(time->hour, time->hour_format);
	temp_arr[3]=num_to_bcd(time->day_of_month & 0x3f);
	temp_arr[4]=(time->day_of_week-1) & 0x07;  // RTC: 1=sunday, 2=monday, etc. -> PCF8563: 0=sunday, 1=monday, etc.
	temp_arr[5]=num_to_bcd(time->month & 0x1f);
	temp_arr[6]=num_to_bcd(time->year);

	obj->write_cmd(obj, PCF8563_REG_SECONDS, temp_arr, 7);
}

/**
 * @brief  Read control status 1 register.
 * @param  obj:       Pointer to a PCF8563_Object structure that contains
 *                    the information for the display.
 * @retval  output:   Value of PCF8563_ControlStatus1
 */
PCF8563_ControlStatus1 PCF8563_get_control_status1(PCF8563_Object *obj) {
	obj->read_cmd(obj, PCF8563_REG_CONTROL_STATUS_1, &temp, 1);
	return *((PCF8563_ControlStatus1*)&temp);
}

/**
 * @brief  Read control status 2 register.
 * @param  obj:       Pointer to a PCF8563_Object structure that contains
 *                    the information for the display.
 * @retval  output:   Value of PCF8563_ControlStatus2
 */
PCF8563_ControlStatus2 PCF8563_get_control_status2(PCF8563_Object *obj) {
	obj->read_cmd(obj, PCF8563_REG_CONTROL_STATUS_2, &temp, 1);
	return *((PCF8563_ControlStatus2*)&temp);
}

/**
 * @brief  Read CLKOUT control register.
 * @param  obj:       Pointer to a PCF8563_Object structure that contains
 *                    the information for the display.
 * @retval  output:   Value of PCF8563_CLKOUT_Control
 */
PCF8563_CLKOUT_Control PCF8563_get_clkout_control(PCF8563_Object *obj) {
	obj->read_cmd(obj, PCF8563_REG_CLKOUT_CONTROL, &temp, 1);
	return *((PCF8563_CLKOUT_Control*)&temp);
}

/**
 * @brief  Read alarm registers from PCF8563.
 * @param  obj:       Pointer to a PCF8563_Object structure that contains
 *                    the information for the display.
 * @param  alarm:     Pointer to PCF8563_Alarm object
 */
void PCF8563_read_alarm(PCF8563_Object *obj, PCF8563_Alarm * alarm) {
	obj->read_cmd(obj, PCF8563_REG_ALARM_MINUTE, alarm->DATA, 4);
}

/**
 * @brief  Set and enable alarm in PCF8563.
 * @param  obj:       Pointer to a PCF8563_Object structure that contains
 *                    the information for the display.
 * @param  date:  	  Date value
 * @param  weekday:	  Weekday value
 * @param  hours:     Hours value, in 24hr format
 * @param  minutes:   Minutes value
 * @param  alarm_flags:      Alarm flags (PCF8563_ALARM_FLAG_MINUTES, PCF8563_ALARM_FLAG_HOURS, PCF8563_ALARM_FLAG_DAY, PCF8563_ALARM_FLAG_WEEKDAY)
 */
void PCF8563_set_alarm(PCF8563_Object *obj, uint8_t date, uint8_t weekday, uint8_t hours, uint8_t minutes, uint8_t alarm_flags) {
	PCF8563_Alarm * alarm;
	alarm=(PCF8563_Alarm*)temp_arr;

	alarm->minutes=num_to_bcd(minutes);
	alarm->hours=num_to_bcd(hours);
	alarm->day=num_to_bcd(date);
	alarm->weekday=weekday;

	alarm->AE_M=(alarm_flags | PCF8563_ALARM_FLAG_MINUTES)?0:1;
	alarm->AE_H=(alarm_flags | PCF8563_ALARM_FLAG_HOURS)?0:1;
	alarm->AE_D=(alarm_flags | PCF8563_ALARM_FLAG_DAY)?0:1;
	alarm->AE_W=(alarm_flags | PCF8563_ALARM_FLAG_WEEKDAY)?0:1;

	obj->write_cmd(obj, PCF8563_REG_ALARM_MINUTE, temp_arr, 4);

	// enable alarm interrupt
	obj->read_cmd(obj, PCF8563_REG_CONTROL_STATUS_2, &temp, 1);
	((PCF8563_ControlStatus2*)&temp)->AIE=1;
	obj->write_cmd(obj, PCF8563_REG_CONTROL_STATUS_2, &temp, 1);
}

/**
 * @brief  Disable alarm interrupt in PCF8563.
 * @param  obj:       Pointer to a PCF8563_Object structure that contains
 *                    the information for the display.
 */
void DS3231_disable_alarm(PCF8563_Object *obj) {
	obj->read_cmd(obj, PCF8563_REG_CONTROL_STATUS_2, &temp, 1);
	((PCF8563_ControlStatus2*)&temp)->AIE=0;
	obj->write_cmd(obj, PCF8563_REG_CONTROL_STATUS_2, &temp, 1);
}

/**
 * @brief  Get alarm interrupt status.
 * @param  obj:       Pointer to a PCF8563_Object structure that contains
 *                    the information for the display.
 * @retval  alarm flag set, alarm was triggered
 */
bool DS3231_get_alarm_flag(PCF8563_Object *obj) {
	obj->read_cmd(obj, PCF8563_REG_CONTROL_STATUS_2, &temp, 1);
	return ((PCF8563_ControlStatus2*)&temp)->AF==1;
}

/**
 * @brief  Clear alarm interrupt flag in PCF8563.
 * @param  obj:       Pointer to a PCF8563_Object structure that contains
 *                    the information for the display.
 */
void DS3231_clear_alarm_flag(PCF8563_Object *obj) {
	obj->read_cmd(obj, PCF8563_REG_CONTROL_STATUS_2, &temp, 1);
	((PCF8563_ControlStatus2*)&temp)->AF=0;
	obj->write_cmd(obj, PCF8563_REG_CONTROL_STATUS_2, &temp, 1);
}

/**
 * @brief  Set and enable timer in PCF8563.
 * @param  obj:       Pointer to a PCF8563_Object structure that contains
 *                    the information for the display.
 * @param  timer:  	  Timer value
 * @param  clock_source:	PCF8563_TimerSource, clock source
 */
void PCF8563_set_timer(PCF8563_Object *obj, uint8_t timer, PCF8563_TimerSource clock_source) {
	obj->write_cmd(obj, PCF8563_REG_TIMER, &timer, 1);

	temp=0;
	((PCF8563_Timer_Control*)&temp)->TD=clock_source;
	((PCF8563_Timer_Control*)&temp)->TE=1;

	obj->write_cmd(obj, PCF8563_REG_TIMER_CONTROL, &temp, 1);
}

/**
 * @brief  Disable alarm interrupt in PCF8563.
 * @param  obj:       Pointer to a PCF8563_Object structure that contains
 *                    the information for the display.
 */
void DS3231_disable_timer(PCF8563_Object *obj) {
	obj->read_cmd(obj, PCF8563_REG_TIMER_CONTROL, &temp, 1);
	((PCF8563_Timer_Control*)&temp)->TE=0;
	obj->write_cmd(obj, PCF8563_REG_TIMER_CONTROL, &temp, 1);
}

/**
 * @brief  Get alarm interrupt status.
 * @param  obj:       Pointer to a PCF8563_Object structure that contains
 *                    the information for the display.
 * @retval  alarm flag set, alarm was triggered
 */
bool DS3231_get_timer_flag(PCF8563_Object *obj) {
	obj->read_cmd(obj, PCF8563_REG_CONTROL_STATUS_2, &temp, 1);
	return ((PCF8563_ControlStatus2*)&temp)->TF==1;
}

/**
 * @brief  Clear alarm interrupt flag in PCF8563.
 * @param  obj:       Pointer to a PCF8563_Object structure that contains
 *                    the information for the display.
 */
void DS3231_clear_timer_flag(PCF8563_Object *obj) {
	obj->read_cmd(obj, PCF8563_REG_CONTROL_STATUS_2, &temp, 1);
	((PCF8563_ControlStatus2*)&temp)->TF=0;
	obj->write_cmd(obj, PCF8563_REG_CONTROL_STATUS_2, &temp, 1);
}

/**
 * @brief  Get clock integrity guaranteed (low voltage flag).
 * @param  obj:       Pointer to a PCF8563_Object structure that contains
 *                    the information for the display.
 * @retval  clock integrity guaranteed
 */
bool DS3231_clock_integrity_guaranteed(PCF8563_Object *obj) {
	obj->read_cmd(obj, PCF8563_REG_SECONDS, &temp, 1);
	return (temp & 0x80)==0;
}
