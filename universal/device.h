/**
  ******************************************************************************
  * @file    device.h
  * @author  Juraj Lonc (juraj.lonc@gmail.com)
  * @brief   Hardware device specific functions.
  ******************************************************************************
  */

#ifndef INC_DEVICE_H_
#define INC_DEVICE_H_

#ifdef __cplusplus
extern "C" {
#endif

#ifndef __STM8S_H
#include <stdint.h>
#include <stdbool.h>
#endif /* __STM8S_H */

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
 * @brief  Function that writes to register.
 * @param	void*		pointer to customizable handle
 * @param	uint8_t*	data buffer
 * @param	uint16_t	length of data to write
 * @retval 0 = no problem
 */
typedef int8_t(*device_spi_write_ptr)(void *, uint8_t *, uint16_t);

/**
 * @brief  Function that reads register.
 * @param	void*		pointer to customizable handle
 * @param	uint8_t		register
 * @param	uint8_t*	data buffer
 * @param	uint16_t	length of data to read
 * @retval 0 = no problem
 */
typedef int8_t(*device_read_ptr) (void *, uint8_t, uint8_t *, uint16_t);

/**
 * @brief  Set value of pin. Hardware dependent function.
 * @param  bool: value of pin
 */
typedef void (*device_write_pin_ptr)(bool);

/**
 * @brief  Read value of pin. Hardware dependent function.
 * @retval  value of pin
 */
typedef bool (*device_read_pin_ptr)(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_DEVICE_H_ */
