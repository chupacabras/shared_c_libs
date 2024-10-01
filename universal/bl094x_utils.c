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
	bool neg=false;
	if (data[0] & 0x80) {
		neg=true;
		data[0]&=0x7f;
		data[0]^=0xff;
		data[1]^=0xff;
		data[2]^=0xff;
	}

	*val=data[0];
	*val<<=8;
	*val|=data[1];
	*val<<=8;
	*val|=data[2];

	if (neg) {
		*val=-*val;
	}
}

void BL094X_convert_signed20_value(uint8_t *data, int32_t *val) {
	bool neg=false;
	if (data[0] & 0x08) {
		neg=true;
		data[0]&=0x07;
		data[0]^=0xff;
		data[1]^=0xff;
		data[2]^=0xff;
	}

	*val=data[0];
	*val<<=8;
	*val|=data[1];
	*val<<=8;
	*val|=data[2];

	if (neg) {
		*val=-*val;
	}
}

void BL094X_convert_unsigned24_value(uint8_t *data, uint32_t *val) {
	*val=data[0];
	*val<<=8;
	*val|=data[1];
	*val<<=8;
	*val|=data[2];
}

void BL094X_convert_unsigned16_value(uint8_t *data, uint16_t *val) {
	*val=data[0];
	*val<<=8;
	*val|=data[1];
}
