/**
  ******************************************************************************
  * @file    pcf8563.h
  * @author  Juraj Lonc (juraj.lonc@gmail.com)
  * @brief   Header file of PCF8563 driver.
  ******************************************************************************
  */

#ifndef INC_PCF8563_H_
#define INC_PCF8563_H_

#include "rtc.h"
#include "stdbool.h"
#include "device.h"


#define PCF8563_ADDRESS					0b10100010

// Control and status registers
#define PCF8563_REG_CONTROL_STATUS_1	0x00
#define PCF8563_REG_CONTROL_STATUS_2	0x01

// Time and date registers
#define PCF8563_REG_SECONDS				0x02
#define PCF8563_REG_MINUTES				0x03
#define PCF8563_REG_HOURS				0x04
#define PCF8563_REG_DAYS				0x05
#define PCF8563_REG_WEEKDAYS			0x06
#define PCF8563_REG_MONTHS				0x07
#define PCF8563_REG_YEARS				0x08

// Alarm registers
#define PCF8563_REG_ALARM_MINUTE		0x09
#define PCF8563_REG_ALARM_HOUR			0x0a
#define PCF8563_REG_ALARM_DAY			0x0b
#define PCF8563_REG_ALARM_WEEKDAY		0x0c

// CLKOUT control register
#define PCF8563_REG_CLKOUT_CONTROL		0x0d

// Timer registers
#define PCF8563_REG_TIMER_CONTROL		0x0e
#define PCF8563_REG_TIMER				0x0f

// Alarm flags
#define PCF8563_ALARM_FLAG_MINUTES		0x01
#define PCF8563_ALARM_FLAG_HOURS		0x02
#define PCF8563_ALARM_FLAG_DAY			0x04
#define PCF8563_ALARM_FLAG_WEEKDAY		0x08


typedef enum {
	PCF8563_SUNDAY=0b000,
	PCF8563_MONDAY=0b001,
	PCF8563_TUESDAY=0b010,
	PCF8563_WEDNESDAY=0b011,
	PCF8563_THURSDAY=0b100,
	PCF8563_FRIDAY=0b101,
	PCF8563_SATURDAY=0b110
} PCF8563_Weekday;

typedef enum {
	PCF8563_TIMER_CLOCK_4096HZ=0b00,
	PCF8563_TIMER_CLOCK_64HZ=0b01,
	PCF8563_TIMER_CLOCK_1HZ=0b10,
	PCF8563_TIMER_CLOCK_1_60HZ=0b11
} PCF8563_TimerSource;

//typedef enum {
//	PCF8563_JANUARY=0b00001,
//	PCF8563_FEBRUARY=0b00010,
//	PCF8563_MARCH=0b00011,
//	PCF8563_APRIL=0b00100,
//	PCF8563_MAY=0b00101,
//	PCF8563_JUNE=0b00110,
//	PCF8563_JULY=0b00111,
//	PCF8563_AUGUST=0b01000,
//	PCF8563_SEPTEMBER=0b01001,
//	PCF8563_OCTOBER=0b10000,
//	PCF8563_NOVEMBER=0b10001,
//	PCF8563_DECEMBER=0b10010,
//
//} PCF8563_Month;

typedef struct {
	device_write_ptr write_cmd;
	device_read_ptr read_cmd;
} PCF8563_Object;

typedef union {
	struct {
		uint8_t BYTE;
	};
	struct {
		uint8_t :2;
		uint8_t TESTC :1;
		uint8_t :1;
		uint8_t STOP :1;
		uint8_t :1;
		uint8_t TEST1 :1;
	};
} PCF8563_ControlStatus1;

typedef struct {
	uint8_t TIE :1;
	uint8_t AIE :1;
	uint8_t TF :1;
	uint8_t AF :1;
	uint8_t TI_TP :1;
	uint8_t :3;
} PCF8563_ControlStatus2;

typedef struct {
	uint8_t SECONDS :7;
	uint8_t VL :1;
} PCF8563_Seconds;

typedef union {
	struct {
		uint8_t DATA[4];
	};
	struct {
		struct {
			uint8_t minutes :7;
			uint8_t AE_M :1;
		};
		struct {
			uint8_t hours :6;
			uint8_t :1;
			uint8_t AE_H :1;
		};
		struct {
			uint8_t day :6;
			uint8_t :1;
			uint8_t AE_D :1;
		};
		struct {
			uint8_t weekday :3;
			uint8_t :4;
			uint8_t AE_W :1;
		};
	};
} PCF8563_Alarm;

typedef union {
	struct {
		uint8_t BYTE;
	};
	struct {
		uint8_t FD1 :1;
		uint8_t FD2 :1;
		uint8_t :5;
		uint8_t FE :1;
	};
	struct {
		uint8_t FD :2;
		uint8_t :6;
	};
} PCF8563_CLKOUT_Control;

typedef union {
	struct {
		uint8_t BYTE;
	};
	struct {
		uint8_t TD1 :1;
		uint8_t TD2 :1;
		uint8_t :5;
		uint8_t TE :1;
	};
	struct {
		uint8_t TD :2;
		uint8_t :6;
	};
} PCF8563_Timer_Control;



void PCF8563_init(PCF8563_Object * obj, device_write_ptr write_cmd, device_read_ptr read_cmd);
void PCF8563_read_time(PCF8563_Object *obj, RTC_time * time);
void PCF8563_write_time(PCF8563_Object *obj, RTC_time * time);
PCF8563_ControlStatus1 PCF8563_get_control_status1(PCF8563_Object *obj);
PCF8563_ControlStatus2 PCF8563_get_control_status2(PCF8563_Object *obj);
PCF8563_CLKOUT_Control PCF8563_get_clkout_control(PCF8563_Object *obj);
void PCF8563_read_alarm(PCF8563_Object *obj, PCF8563_Alarm * alarm);
void PCF8563_set_alarm(PCF8563_Object *obj, uint8_t date, uint8_t weekday, uint8_t hours, uint8_t minutes, uint8_t alarm_flags);
void DS3231_disable_alarm(PCF8563_Object *obj);
bool DS3231_get_alarm_flag(PCF8563_Object *obj);
void DS3231_clear_alarm_flag(PCF8563_Object *obj);
void PCF8563_set_timer(PCF8563_Object *obj, uint8_t timer, PCF8563_TimerSource clock_source);
void DS3231_disable_timer(PCF8563_Object *obj);
bool DS3231_get_timer_flag(PCF8563_Object *obj);
void DS3231_clear_timer_flag(PCF8563_Object *obj);
bool DS3231_clock_integrity_guaranteed(PCF8563_Object *obj);


#endif /* INC_PCF8563_H_ */
