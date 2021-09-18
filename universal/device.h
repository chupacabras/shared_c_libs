/**
  ******************************************************************************
  * @file    device.h
  * @author  Juraj Lonc (juraj.lonc@gmail.com)
  * @brief   Hardware device specific functions.
  ******************************************************************************
  */

#ifndef INC_DEVICE_H_
#define INC_DEVICE_H_

/**
 * @brief  Delay in milliseconds.
 * @param  uint16_t: milliseconds
 */
typedef void (*device_delay_ms_ptr)(uint16_t);

/**
 * @brief  Delay in microseconds.
 * @param  uint16_t: microseconds
 */
typedef void (*device_delay_us_ptr)(uint16_t);

/**
 * @brief  Function that writes to register.
 * @param	void*		pointer to customizable handle
 * @param	uint8_t		register
 * @param	uint8_t*	data buffer
 * @param	uint16_t	length of data to write
 * @retval 0 = no problem
 */
typedef int8_t(*device_write_ptr)(void *, uint8_t, uint8_t *, uint16_t);

/**
 * @brief  Function that reads register.
 * @param	void*		pointer to customizable handle
 * @param	uint8_t		register
 * @param	uint8_t*	data buffer
 * @param	uint16_t	length of data to read
 * @retval 0 = no problem
 */
typedef int8_t(*device_read_ptr) (void *, uint8_t, uint8_t *, uint16_t);


#endif /* INC_DEVICE_H_ */
