/**
  ******************************************************************************
  * @file    dht.h
  * @author  Juraj Lonc (juraj.lonc@gmail.com)
  * @brief   Header file of DHT temperature & humidity sensor driver.
  ******************************************************************************
  */

#ifndef INC_DHT_H_
#define INC_DHT_H_

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define DHT_HOST_START_SIGNAL_DOWN_TIME		1000	// us
#define DHT_BUS_MASTER_RELEASED_TIME		30	// us


typedef enum {
	DHT_STAGE_IDLE,
	DHT_STAGE_HOST_START,
	DHT_STAGE_SLAVE_RESPONSE,
	DHT_STAGE_DATA,
	DHT_STAGE_FINISHED,
	DHT_STAGE_ERROR,
	DHT_STAGE_INVALID_PARITY
} DHT_Stage;

typedef void(*dht_device_pin_output) (void);
typedef void(*dht_device_pin_input) (void);
typedef void(*dht_device_pin_high) (void);
typedef void(*dht_device_pin_low) (void);
typedef uint8_t(*dht_device_pin_read) (void);
typedef void(*dht_device_reset_timer) (void);	// timer with 1us period
typedef uint16_t(*dht_device_read_timer) (void);


typedef struct {
	dht_device_pin_output pin_output;
	dht_device_pin_input pin_input;
	dht_device_pin_low pin_low;
	dht_device_pin_high pin_high;
	dht_device_pin_read pin_read;
	dht_device_reset_timer reset_timer;
	dht_device_read_timer read_timer;

	DHT_Stage stage;
	uint8_t bytes_cnt;
	uint8_t data[5];

	uint8_t cur_mask;
	uint8_t cur_index;

//	uint16_t tmp;
//	uint16_t tmp2;
//	uint16_t i;
//	uint8_t tt[200];
} DHT_Handle;


void DHT_init(DHT_Handle *dht, dht_device_pin_output pin_output, dht_device_pin_input pin_input, dht_device_pin_low pin_low, dht_device_pin_high pin_high, dht_device_pin_read pin_read, dht_device_reset_timer reset_timer, dht_device_read_timer read_timer);
void DHT_start_reading(DHT_Handle *dht);
bool DHT_decode_value(DHT_Handle *dht, float *humidity, float *temperature);
// humidity in per mille (630 = 63.0%), temperature in tenths of degree (239 = 23.9C)
bool DHT_get_value(DHT_Handle *dht, uint16_t *humidity, uint16_t *temperature);

void DHT_interrupt_handler_io(DHT_Handle *dht);

#endif /* INC_DHT_H_ */
