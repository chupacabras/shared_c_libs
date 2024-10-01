/**
  ******************************************************************************
  * @file    bl094x_utils.h
  * @author  Juraj Lonc (juraj.lonc@gmail.com)
  * @brief   Header file of BL094x utilities.
  ******************************************************************************
  */

#ifndef INC_BL094X_UTILS_H_
#define INC_BL094X_UTILS_H_

#include <stdbool.h>
#include <stdint.h>

typedef int8_t(*bl094x_write_ptr)(uint8_t *, uint8_t);
typedef int8_t(*bl094x_read_ptr)(uint8_t *, uint8_t);
typedef int8_t(*bl094x_cs_ptr)(bool);

uint8_t BL094X_calculate_checksum(uint8_t *data1, uint8_t len1, uint8_t *data2, uint8_t len2);

void BL094X_convert_signed24_value(uint8_t *data, int32_t *val);
void BL094X_convert_signed20_value(uint8_t *data, int32_t *val);
void BL094X_convert_unsigned24_value(uint8_t *data, uint32_t *val);
void BL094X_convert_unsigned16_value(uint8_t *data, uint16_t *val);

#endif /* INC_BL094X_UTILS_H_ */
