/**
  ******************************************************************************
  * @file    bl094x_utils.c
  * @author  Juraj Lonc (juraj.lonc@gmail.com)
  * @brief   Utilities for BL0940 and BL0942.
  ******************************************************************************
  */

#include "bl094x_utils.h"

uint8_t BL094X_calculate_checksum(uint8_t *data1, uint8_t len1, uint8_t *data2, uint8_t len2) {
	uint8_t tmp=0;
	uint8_t q=0;

	for (q=0; q<len1; q++) {
		tmp+=data1[q];
	}

	for (q=0; q<len2; q++) {
		tmp+=data2[q];
	}

	tmp^=0xff;

	return tmp;
}

void BL094X_convert_signed24_value(uint8_t *data, int32_t *val) {
	if (data[2] & 0x80) {
		*val=0xff;
		*val<<=8;
	} else {
		*val=0;
	}

	*val|=data[2];
	*val<<=8;
	*val|=data[1];
	*val<<=8;
	*val|=data[0];

}

void BL094X_convert_signed20_value(uint8_t *data, int32_t *val) {
	if (data[2] & 0x08) {
		*val=0xff;
		*val<<=8;
		data[2]|=0xf0;
	} else {
		data[2]&=0x0f;
		*val=0;
	}

	*val|=data[2];
	*val<<=8;
	*val|=data[1];
	*val<<=8;
	*val|=data[0];

}

void BL094X_convert_unsigned24_value(uint8_t *data, uint32_t *val) {
	*val=data[2];
	*val<<=8;
	*val|=data[1];
	*val<<=8;
	*val|=data[0];
}

void BL094X_convert_unsigned16_value(uint8_t *data, uint16_t *val) {
	*val=data[1];
	*val<<=8;
	*val|=data[0];
}
