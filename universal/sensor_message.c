/**
  ******************************************************************************
  * @file    sensor_message.c
  * @author  Juraj Lonc (juraj.lonc@gmail.com)
  * @brief   Sensor message.
  ******************************************************************************
  */

#include "sensor_message.h"

void SensorMessage_init(SensorMessage *handle, uint8_t *sensor_id, uint8_t *buffer) {
    handle->buffer=buffer;
    handle->buffer[0]=MESSAGE_VERSION;
    memcpy(&handle->buffer[1], sensor_id, SENSOR_ID_LENGTH);
    handle->pos=1+SENSOR_ID_LENGTH;
}

void SensorMessage_clear_data(SensorMessage *handle) {
    handle->pos=1+SENSOR_ID_LENGTH;
}

void SensorMessage_add_1byte(SensorMessage *handle, uint8_t message_type, uint8_t data) {
    handle->buffer[handle->pos++]=message_type;
    handle->buffer[handle->pos++]=data;
}

void SensorMessage_add_2byte(SensorMessage *handle, uint8_t message_type, uint16_t data) {
    handle->buffer[handle->pos++]=message_type;
    handle->buffer[handle->pos++]=(data >> 8) & 0xff;
    handle->buffer[handle->pos++]=(data) & 0xff;
}