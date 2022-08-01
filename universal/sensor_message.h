/**
  ******************************************************************************
  * @file    sensor_message.h
  * @author  Juraj Lonc (juraj.lonc@gmail.com)
  * @brief   Header file of Sensor message.
  ******************************************************************************
  */
#ifndef __SENSOR_MESSAGE_H
#define __SENSOR_MESSAGE_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#ifndef DISABLE_MSG_CRC
#include "crc16.h"
#endif


#define MESSAGE_VERSION_1		1
#define MESSAGE_VERSION_2		2	// added message number
#ifndef DISABLE_MSG_CRC
#define MESSAGE_VERSION_3		3	// added CRC check
#define MESSAGE_VERSION_4		4	// added options byte
#endif
#define SENSOR_ID_LENGTH	6

#define MSG_VALUE_TEMPERATURE_PCB1				1
#define MSG_VALUE_TEMPERATURE_PCB2				2
#define MSG_VALUE_TEMPERATURE_AMBIENT1			4
#define MSG_VALUE_TEMPERATURE_AMBIENT2			5
#define MSG_VALUE_AIR_HUMIDITY					7
#define MSG_VALUE_AIR_PRESSURE					8
#define MSG_VALUE_AIR_HUMIDITY2					9
#define MSG_VALUE_WATER_LEVEL					10
#define MSG_VALUE_DC_VOLTAGE_BATTERY			20
#define MSG_VALUE_DC_VOLTAGE_INPUT				21
#define MSG_VALUE_DC_VOLTAGE_PRESENT			22
#define MSG_VALUE_BATTERY_LEVEL					25
#define MSG_VALUE_AC_VOLTAGE_MAINS				30
#define MSG_VALUE_AC_CURRENT_MAINS				31
#define MSG_VALUE_BUTTON1_PRESSED				41
#define MSG_VALUE_BUTTON2_PRESSED				42
#define MSG_VALUE_BUTTON3_PRESSED				43
#define MSG_VALUE_BUTTON4_PRESSED				44
#define MSG_VALUE_BUTTON5_PRESSED				45
#define MSG_VALUE_IPV4							50
#define MSG_VALUE_STATUS						51
#define MSG_VALUE_STARTING						52
#define MSG_VALUE_RESET_CAUSE					53
#define MSG_VALUE_RSSI							60
#define MSG_VALUE_SNR							61

#define MSG_OPTION_COUNTER						0b10000000
#define MSG_OPTION_CRC16						0b01000000


#define STR_TEMPERATURE_PCB			"temperature_pcb"
#define STR_TEMPERATURE_PCB2		"temperature_pcb2"
#define STR_TEMPERATURE_AMBIENT		"temperature_ambient"
#define STR_TEMPERATURE_AMBIENT2	"temperature_ambient2"
#define STR_AIR_HUMIDITY			"air_humidity"
#define STR_AIR_HUMIDITY2			"air_humidity2"
#define STR_AIR_PRESSURE			"air_pressure"
#define STR_WATER_LEVEL				"water_level"
#define STR_BATTERY_VOLTAGE			"battery_voltage"
#define STR_DC_POWER_PRESENT		"dc_power_present"
#define STR_BATTERY_LEVEL			"battery_level"
#define STR_BUTTON1					"button1"
#define STR_BUTTON2					"button2"
#define STR_BUTTON3					"button3"
#define STR_BUTTON4					"button4"
#define STR_IPV4					"ipv4"
#define STR_STATUS					"status"
#define STR_STARTING				"starting"
#define STR_RESET_CAUSE				"reset_cause"
#define STR_RSSI					"rssi"
#define STR_SNR						"snr"



typedef struct tag_SensorMessage {
	uint8_t *buffer;
	uint16_t pos;
	uint8_t counter;
	uint8_t version;
} SensorMessage;

void SensorMessage_init_v1(SensorMessage *handle, uint8_t *sensor_id, uint8_t *buffer);
void SensorMessage_init(SensorMessage *handle, uint8_t *sensor_id, uint8_t *buffer, uint8_t version, uint8_t options);
void SensorMessage_finish(SensorMessage *handle);

void SensorMessage_clear_data(SensorMessage *handle);
void SensorMessage_add_1byte(SensorMessage *handle, uint8_t message_type, uint8_t data);
void SensorMessage_add_2byte(SensorMessage *handle, uint8_t message_type, uint16_t data);
void SensorMessage_add_3byte(SensorMessage *handle, uint8_t message_type, uint32_t data);
void SensorMessage_add_4byte(SensorMessage *handle, uint8_t message_type, uint8_t *data);

#endif
