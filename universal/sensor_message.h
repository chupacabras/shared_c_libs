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

#define MESSAGE_VERSION		1
#define SENSOR_ID_LENGTH	6

#define MSG_VALUE_TEMPERATURE_PCB1				1
#define MSG_VALUE_TEMPERATURE_PCB2				2
#define MSG_VALUE_TEMPERATURE_AMBIENT1			4
#define MSG_VALUE_TEMPERATURE_AMBIENT2			5
#define MSG_VALUE_HUMIDITY						7
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
#define MSG_VALUE_RSSI							60




typedef struct tag_SensorMessage {
	uint8_t *buffer;
	uint16_t pos;
} SensorMessage;

void SensorMessage_init(SensorMessage *handle, uint8_t *sensor_id, uint8_t *buffer);

void SensorMessage_clear_data(SensorMessage *handle);
void SensorMessage_add_1byte(SensorMessage *handle, uint8_t message_type, uint8_t data);
void SensorMessage_add_2byte(SensorMessage *handle, uint8_t message_type, uint16_t data);

#endif
