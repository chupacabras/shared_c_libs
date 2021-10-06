/**
  ******************************************************************************
  * @file    rtc.c
  * @author  Juraj Lonc (juraj.lonc@gmail.com)
  * @brief   RTC utilities.
  ******************************************************************************
  */

#include "rtc.h"

/**
 * @brief  Print RTC date value to string.
 * @param  rtc:		Pointer to a RTC_time structure that contains timedate values
 * @param  buffer:	buffer where to write string
 */
void printDate(RTC_time *rtc, uint8_t *buffer) {
	buffer[0] = '2';
	buffer[1] = '0' + (rtc->year / 100);
	buffer[2] = '0' + ((rtc->year % 100) / 10);
	buffer[3] = '0' + (rtc->year % 10);
	buffer[4] = '-';
	buffer[5] = '0' + (rtc->month / 10);
	buffer[6] = '0' + (rtc->month % 10);
	buffer[7] = '-';
	buffer[8] = '0' + (rtc->day_of_month / 10);
	buffer[9] = '0' + (rtc->day_of_month % 10);
	buffer[10] = 0;
}

/**
 * @brief  Print RTC time value to string.
 * @param  rtc:		Pointer to a RTC_time structure that contains timedate values
 * @param  buffer:	buffer where to write string
 */
void printTime(RTC_time *rtc, uint8_t *buffer) {
	buffer[0] = '0' + (rtc->hour / 10);
	buffer[1] = '0' + (rtc->hour % 10);
	buffer[2] = ':';
	buffer[3] = '0' + (rtc->minute / 10);
	buffer[4] = '0' + (rtc->minute % 10);
	buffer[5] = ':';
	buffer[6] = '0' + (rtc->second / 10);
	buffer[7] = '0' + (rtc->second % 10);
	buffer[8] = 0;
}

/**
 * @brief  Convert value from register and update RTC_time object.
 * @param  time:      Pointer to a RTC_time structure that contains time values.
 * @param  val:       BCD value of seconds.
 */
void rtc_convert_seconds(RTC_time *time, uint8_t val) {
	time->second = bcd_to_num(val & 0x7f);
}

/**
 * @brief  Convert value from register and update RTC_time object.
 * @param  time:      Pointer to a RTC_time structure that contains time values.
 * @param  val:       BCD value of minutes.
 */
void rtc_convert_minutes(RTC_time *time, uint8_t val) {
	time->minute = bcd_to_num(val & 0x7f);
}

/**
 * @brief  Convert value from register and update RTC_time object.
 * @param  time:      Pointer to a RTC_time structure that contains time values.
 * @param  val:       BCD value of hours. Including AM/PM/H24 flags (bits 6, 5)
 */
void rtc_convert_hours(RTC_time *time, uint8_t val) {
	if ((val & 0x80) > 0) {
		// 12 format
		if ((val) == 1) // PM
			time->hour_format = RTC_PM;
		else
			// AM
			time->hour_format = RTC_AM;

		time->hour = bcd_to_num(val & 0x1f);
	} else {
		// 24 format
		time->hour_format = RTC_H24;
		time->hour = bcd_to_num(val & 0x3f);
	}
}

/**
 * @brief  Convert value from register and update RTC_time object. For internal use only.
 * @param  time:      Pointer to a RTC_time structure that contains time values.
 * @param  val:       Value of day of week.
 */
void rtc_convert_day_of_week(RTC_time *time, uint8_t val) {
	time->day_of_week = (val & 0x07);
}

/**
 * @brief  Convert value from register and update RTC_time object. For internal use only.
 * @param  time:      Pointer to a RTC_time structure that contains time values.
 * @param  val:       BCD value of day of month.
 */
void rtc_convert_date(RTC_time *time, uint8_t val) {
	time->day_of_month = bcd_to_num(val & 0x3f);
}

/**
 * @brief  Convert value from register and update RTC_time object. For internal use only.
 * @param  time:      Pointer to a RTC_time structure that contains time values.
 * @param  val:       BCD value of month.
 */
void rtc_convert_month(RTC_time *time, uint8_t val) {
	time->month = bcd_to_num(val & 0x1f);
}

/**
 * @brief  Convert value from register and update RTC_time object. For internal use only.
 * @param  time:      Pointer to a RTC_time structure that contains time values.
 * @param  val:       BCD value of year.
 */
void rtc_convert_year(RTC_time *time, uint8_t val) {
	time->year = bcd_to_num(val);
}

/**
 * @brief  Encode hours to BCD format with AM/PM/H24 flag (for DS1302, DS1307, DS3231).
 * @param  hours:      		Hours value.
 * @param  hour_format:     Hours format (AM/PM/H24).
 * @retval	BCD encoded hours with AM/PM/H24 flag
 */
uint8_t rtc_encode_hours(uint8_t hours, RTCHoursFormat hour_format) {
	if (hour_format == RTC_AM)
		hours = num_to_bcd(hours & 0x1f) | 0x01000000;
	else if (hour_format == RTC_PM)
		hours = num_to_bcd(hours & 0x1f) | 0b01100000;
	else if (hour_format == RTC_H24)
		hours = num_to_bcd(hours & 0x3f);

	return hours;
}

/**
 * @brief  Calculate day of week in proleptic Gregorian calendar. Sunday == 1.
 * @param  year:   Years value.
 * @param  month:  Months value.
 * @param  day:    Days value.
 * @retval	Weekday, 1=Sunday, 2=Monday, etc.
 */
uint8_t rtc_weekday(uint16_t year, uint8_t month, uint8_t day) {
	uint8_t adjustment, mm;
	uint16_t yy;
	if (year < 2000) year += 2000;
	adjustment = (14 - month) / 12;
	mm = month + 12 * adjustment - 2;
	yy = year - adjustment;

	return ((day + (13 * mm - 1) / 5 + yy + yy / 4 - yy / 100 + yy / 400) % 7)+1;
}
