/**
  ******************************************************************************
  * @file    dht.c
  * @author  Juraj Lonc (juraj.lonc@gmail.com)
  * @brief   DHT temperature & humidity sensor driver.
  ******************************************************************************
  */

#include "dht.h"

void DHT_init(DHT_Handle *dht, dht_device_pin_output pin_output, dht_device_pin_input pin_input, dht_device_pin_low pin_low, dht_device_pin_high pin_high, dht_device_pin_read pin_read, dht_device_reset_timer reset_timer, dht_device_read_timer read_timer) {
	dht->pin_output=pin_output;
	dht->pin_input=pin_input;
	dht->pin_low=pin_low;
	dht->pin_high=pin_high;
	dht->pin_read=pin_read;
	dht->reset_timer=reset_timer;
	dht->read_timer=read_timer;
}

void DHT_start_reading(DHT_Handle *dht) {
	dht->pin_output();
	dht->pin_high();
	dht->stage=DHT_STAGE_HOST_START;
	dht->bytes_cnt=0;
	memset(dht->data, 0, 5);
	dht->cur_mask=0b10000000;
	dht->cur_index=0;
	dht->reset_timer();
	dht->pin_low();

	while (true) {
		if (dht->read_timer()>6000) {
			dht->stage=DHT_STAGE_ERROR;
			break;
		}
		if (dht->stage==DHT_STAGE_HOST_START) {
			if (dht->read_timer()>DHT_HOST_START_SIGNAL_DOWN_TIME) {
				// 1ms, host start signal
				dht->pin_high();
				dht->reset_timer();
				while (dht->read_timer()<DHT_BUS_MASTER_RELEASED_TIME); // 30us bus master release time
				dht->reset_timer();
				dht->pin_low();
				while (dht->read_timer()<5);
				dht->pin_input();
				dht->stage=DHT_STAGE_SLAVE_RESPONSE;
			}
		} else if (dht->stage==DHT_STAGE_SLAVE_RESPONSE || dht->stage==DHT_STAGE_DATA) {
			if (dht->read_timer()>DHT_HOST_START_SIGNAL_DOWN_TIME) {
				dht->stage=DHT_STAGE_ERROR;
			}
		} else if (dht->stage==DHT_STAGE_ERROR) {
			break;
		} else if (dht->stage==DHT_STAGE_FINISHED) {
			break;
		}

	}

}

void DHT_interrupt_handler_io(DHT_Handle *dht) {

	if (dht->stage==DHT_STAGE_DATA) {
		uint8_t t=dht->read_timer();
		dht->reset_timer();
		if (t<60 || t>200) {
			dht->stage=DHT_STAGE_ERROR;
		} else {
			if (t>90) {
				// value = 1
				dht->data[dht->cur_index] |=dht->cur_mask;
			}

			if (dht->cur_mask==0b00000001) {
				dht->cur_mask=0b10000000;
				dht->cur_index++;
				if (dht->cur_index>=5) {
					dht->stage=DHT_STAGE_FINISHED;
				}
			} else {
				dht->cur_mask >>= 1;
			}
		}
	} else if (dht->stage==DHT_STAGE_SLAVE_RESPONSE) {
		uint8_t t=dht->read_timer();
		dht->reset_timer();
		if (t<130 || t>200) {
			dht->stage=DHT_STAGE_ERROR;
		} else {
			dht->stage=DHT_STAGE_DATA;
		}
	}
}

bool DHT_get_value(DHT_Handle *dht, uint16_t *humidity, int16_t *temperature) {
	if (dht->stage!=DHT_STAGE_FINISHED) return false;
	// check parity byte
	if (((dht->data[0]+dht->data[1]+dht->data[2]+dht->data[3]) & 0xff)!=dht->data[4]) {
		dht->stage=DHT_STAGE_INVALID_PARITY;
		return false;
	}

	*humidity=(dht->data[0] << 8) + dht->data[1];

	*temperature=(((uint16_t)dht->data[2] & 0b01111111) << 8) + dht->data[3];
	if (dht->data[2] & 0b10000000) {
		*temperature=-*temperature;
	}

	return true;
}
