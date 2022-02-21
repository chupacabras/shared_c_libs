/**
  ******************************************************************************
  * @file    sensor_message.c
  * @author  Juraj Lonc (juraj.lonc@gmail.com)
  * @brief   Sensor message.
  ******************************************************************************
  */

#include "sensor_message.h"

void SensorMessage_init_v1(SensorMessage *handle, uint8_t *sensor_id, uint8_t *buffer) {
    handle->buffer=buffer;
    handle->buffer[0]=MESSAGE_VERSION_1;
    memcpy(&handle->buffer[1], sensor_id, SENSOR_ID_LENGTH);
    handle->pos=1+SENSOR_ID_LENGTH;
}

void SensorMessage_init(SensorMessage *handle, uint8_t *sensor_id, uint8_t *buffer, uint8_t version, uint8_t options) {
    handle->buffer=buffer;
    handle->buffer[0]=version;
    handle->version=version;
    memcpy(&handle->buffer[1], sensor_id, SENSOR_ID_LENGTH);
    handle->pos=1+SENSOR_ID_LENGTH;
    if (version==MESSAGE_VERSION_2) {
    	handle->buffer[handle->pos++]=0; // counter
    }
#ifndef DISABLE_MSG_CRC
    if (version==MESSAGE_VERSION_4) {
    	handle->buffer[handle->pos++]=options;
    	if (options & MSG_OPTION_COUNTER) {
    		handle->buffer[handle->pos++]=0; // counter
    	}
    }
#endif

}

void SensorMessage_clear_data(SensorMessage *handle) {
    handle->pos=1+SENSOR_ID_LENGTH;
    handle->counter++;
    
#ifndef DISABLE_MSG_CRC
    if ((handle->version==MESSAGE_VERSION_2) || (handle->version==MESSAGE_VERSION_3)) {
    	handle->buffer[handle->pos++]=handle->counter;
    }
    if (handle->version==MESSAGE_VERSION_4) {
    	handle->pos++; // options byte
    	if (handle->buffer[7] & MSG_OPTION_COUNTER) {
    		handle->buffer[handle->pos++]=handle->counter;
    	}
    }
#else
    if (handle->version==MESSAGE_VERSION_2) {
    	handle->buffer[handle->pos++]=handle->counter;
    }
#endif
}

void SensorMessage_finish(SensorMessage *handle) {
#ifndef DISABLE_MSG_CRC
	if (handle->version==MESSAGE_VERSION_3) {
		// add CRC
		uint8_t crc=0;
		for (uint8_t q=0; q<handle->pos; q++) {
			crc^=handle->buffer[q]; // XOR
		}
		handle->buffer[handle->pos++]=crc;
	}
	if (handle->version==MESSAGE_VERSION_4) {
		if (handle->buffer[7] && MSG_OPTION_CRC16) {
			// add CRC
			uint16_t crc=crc16(handle->buffer, handle->pos);
			handle->buffer[handle->pos++]=(crc >> 8) & 0xff;
			handle->buffer[handle->pos++]=crc & 0xff;
		}

	}
#endif
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

void SensorMessage_add_4byte(SensorMessage *handle, uint8_t message_type, uint8_t *data) {
    handle->buffer[handle->pos++]=message_type;
    
    memcpy(&handle->buffer[handle->pos], data, 4);
    handle->pos+=4;
//    handle->buffer[handle->pos++]=data[0];
//    handle->buffer[handle->pos++]=data[1];
//    handle->buffer[handle->pos++]=data[2];
//    handle->buffer[handle->pos++]=data[3];
}
