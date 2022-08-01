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



/**
 * @brief  DHT initialization.
 * @param	handle*		pointer to DHT_Handle
 * @param	pin_output	The function that configures pin to output mode. Hardware dependent.
 * @param	pin_input	The function that configures pin to input mode. Hardware dependent.
 * @param	pin_low		The function that sets pin to low. Hardware dependent.
 * @param	pin_high	The function that sets pin to high. Hardware dependent.
 * @param	pin_read	The function that reads pin value. Hardware dependent.
 * @param	reset_timer	The function that resets timer value. Hardware dependent. 1us timer.
 * @param	read_timer	The function that reads timer value. Hardware dependent.
 */
void DHT_init(DHT_Handle *dht, dht_device_pin_output pin_output, dht_device_pin_input pin_input, dht_device_pin_low pin_low, dht_device_pin_high pin_high, dht_device_pin_read pin_read, dht_device_reset_timer reset_timer, dht_device_read_timer read_timer);

/**
 * @brief  DHT start read. Blocking function.
 * @param	handle*		pointer to DHT_Handle
 */
void DHT_start_reading(DHT_Handle *dht);

/**
 * @brief  Get received values of temperature and humidity from DHT.
 * 		   Humidity in tenths of percent (630 = 63.0%), temperature in tenths of degree (239 = 23.9C)
 * @param	handle*			pointer to DHT_Handle
 * @param	humidity*		pointer to humidity variable.
 * @param	temperature*	pointer to temperature variable.
 * @retval	retval			true = values are present
 */
bool DHT_get_value(DHT_Handle *dht, uint16_t *humidity, uint16_t *temperature);

/**
 * @brief  Get received values of temperature and humidity from DHT in float format.
 * @param	handle*			pointer to DHT_Handle
 * @param	humidity*		pointer to humidity variable.
 * @param	temperature*	pointer to temperature variable.
 * @retval	retval			true = values are present
 */
bool DHT_decode_value(DHT_Handle *dht, float *humidity, float *temperature);

/**
 * @brief  DHT interrupt handler. To be called in IO change (falling edge) interrupt
 * @param	handle*		pointer to DHT_Handle
 */
void DHT_interrupt_handler_io(DHT_Handle *dht);


#endif /* INC_DHT_H_ */
