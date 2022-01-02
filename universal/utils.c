/**
  ******************************************************************************
  * @file    utils.c
  * @author  Juraj Lonc (juraj.lonc@gmail.com)
  * @brief   Utilities.
  ******************************************************************************
  */

#include "utils.h"


// maximal length of string (for internal processing)
#define MAX_STRING_LENGTH	12


/**
 * @brief  Convert number to string as binary number. For internal use only.
 * @param  buffer: Buffer where to build the string.
 * @param  i: number to convert
 * @param  separated: true = place spaces between groups of 4 bits
 * @param  parts: number of separated parts.
 */
static void _2binary(uint8_t *buffer, uint32_t i, bool separated, uint8_t parts) {
	uint8_t q;
	uint8_t t = 4 * parts;
	uint8_t index = t;

	if (separated) {
		index += (parts - 1);
	}
	buffer[index] = 0;
	index--;

	for (q = 0; q < t; q++) {
		if ((i & 1) == 0) {
			buffer[index] = '0';
		} else {
			buffer[index] = '1';
		}
		index--;
		i = i >> 1;

		if (separated && ((q + 1) % 4 == 0) && q < t - 1) {
			buffer[index] = ' ';
			index--;
		}
	}

}

/**
 * @brief  Convert 32-bit number to string as binary number.
 * @param  buffer: Buffer where to build the string.
 * @param  i: number to convert
 * @param  separated: true = place spaces between groups of 4 bits
 */
void long2binary(uint8_t *buffer, uint32_t i, bool separated) {
	_2binary(buffer, i, separated, 8);
}

/**
 * @brief  Convert 16-bit number to string as binary number.
 * @param  buffer: Buffer where to build the string.
 * @param  i: number to convert
 * @param  separated: true = place spaces between groups of 4 bits
 */
void int2binary(uint8_t *buffer, uint16_t i, bool separated) {
	_2binary(buffer, i, separated, 4);
}

/**
 * @brief  Convert 8-bit number to string as binary number.
 * @param  buffer: Buffer where to build the string.
 * @param  i: number to convert
 * @param  separated: true = place spaces between groups of 4 bits
 */
void char2binary(uint8_t *buffer, uint8_t c, bool separated) {
	_2binary(buffer, c, separated, 2);
}




/**
 * @brief  Convert number to string as hex number. For internal use only.
 * @param  buffer: Buffer where to build the string.
 * @param  i: number to convert
 * @param  parts: number of separated parts.
 */
static void _2hex(uint8_t *buffer, uint32_t i, uint8_t parts) {
	uint8_t q;
	uint8_t c;
	for (q = 0; q < parts; q++) {
		c = i & 0b1111;
		if (c < 10) {
			c += '0';
		} else {
			c += 'A' - 10;
		}
		buffer[parts - 1 - q] = c;
		i = i >> 4;
	}
	buffer[parts] = 0;

}

/**
 * @brief  Convert 8-bit number to string as hex number.
 * @param  buffer: Buffer where to build the string.
 * @param  i: number to convert
 */
void char2hex(uint8_t *buffer, uint8_t i) {
	_2hex(buffer, i, 2);
}

/**
 * @brief  Convert 16-bit number to string as hex number.
 * @param  buffer: Buffer where to build the string.
 * @param  i: number to convert
 */
void int2hex(uint8_t *buffer, uint16_t i) {
	_2hex(buffer, i, 4);
}

/**
 * @brief  Convert 32-bit number to string as hex number.
 * @param  buffer: Buffer where to build the string.
 * @param  i: number to convert
 */
void long2hex(uint8_t *buffer, uint32_t i) {
	_2hex(buffer, i, 8);
}



/**
 * @brief  Convert signed number to string.
 * @param  buffer: Buffer where to build the string. Buffer must be at least 12 bytes long.
 * @param  i: signed number to convert
 * @retval  output:   Length of string
 */
uint8_t long2string(uint8_t *buffer, int32_t i) {
	int32_t n;
	bool negate = false;
	uint8_t c = MAX_STRING_LENGTH;

	if (i < 0) {
		negate = true;
		n = -i;
	} else if (i == 0) {
		buffer[0] = '0';
		buffer[1] = 0;
		return 1;
	} else {
		n = i;
	}
	buffer[c--] = 0;
	do {
		buffer[c--] = (n % 10) + '0';
		n = n / 10;
	} while (n);

	if (negate) {
		buffer[c--] = '-';
	}

	memcpy(buffer, &buffer[c+1], MAX_STRING_LENGTH-c);
	return 11-c;
}

/**
 * @brief  Convert double to string as decimal number. Fixed string length.
 * @param  buffer: Buffer where to build the string.
 * @param  f: number to convert
 * @param  decimals: number of places after decimal point
 * @param  length: number of all characters including minus sign. Filled with spaces at beginning.
 */
void double2string_fixed(uint8_t *buffer, double d, uint8_t decimals, uint8_t length) {
	long mul = 1;
	uint8_t q;
	for (q = 0; q < decimals; q++) {
		mul = mul * 10;
	}
	long n = (long)(d*mul);
	int c = length;
	bool negate = false;

	if (d < 0) {
		negate = true;
		n = -n;
	}


	buffer[c--] = 0;
	if (decimals > 0) {
		for (q = 0; q < decimals; q++) {
			buffer[c--] = (n % 10) + '0';
			n = n / 10;
		}
		buffer[c--] = '.';
	}

	for (q = 0; q < length - decimals; q++) {
		buffer[c--] = (n % 10) + '0';
		n = n / 10;

		if (n < 1) break;
	}

	if (negate && c >= 0) {
		buffer[c--] = '-';
	}
	while (c >= 0) {
		buffer[c--] = ' ';
	}

}

/**
 * @brief  Convert double to string as decimal number.
 * @param  buffer: Buffer where to build the string.
 * @param  f: number to convert
 * @param  decimals: number of places after decimal point
 */
void double2string(uint8_t *buffer, double d, uint8_t decimals) {
	long mul = 1;
	uint8_t q;
	for (q = 0; q < decimals; q++) {
		mul = mul * 10;
	}

	int32_t n;
	roundDouble(d*mul, &n);

	int c = MAX_STRING_LENGTH;
	bool negate = false;

	if (d < 0) {
		negate = true;
		n = -n;
	}


	buffer[c--] = 0;
	if (decimals > 0) {
		for (q = 0; q < decimals; q++) {
			buffer[c--] = (n % 10) + '0';
			n = n / 10;
		}
		buffer[c--] = '.';
	}

	for (q = 0; q < MAX_STRING_LENGTH - decimals; q++) {
		buffer[c--] = (n % 10) + '0';
		n = n / 10;

		if (n < 1) break;
	}

	if (negate && c >= 0) {
		buffer[c--] = '-';
	}
	memcpy(buffer, &buffer[c+1], MAX_STRING_LENGTH-c);

}





/**
 * @brief  Convert number from BCD format.
 * @param  num:	number
 * @retval converted number in BCD format.
 */
uint8_t bcd_to_num(uint8_t bcd) {
	return ((bcd & 0x0f) + ((bcd & 0xf0) >> 4)*10);
}

/**
 * @brief  Convert number to BCD format.
 * @param  num:	number in BCD format
 * @retval converted number.
 */
uint8_t num_to_bcd(uint8_t num) {
	return ((((num / 10) << 4) & 0xf0) | (num % 10));
}


/**
 * @brief  Round double to integer.
 * @param  d: number to round.
 * @param  rounded: pointer where to place rounded number
 */
void roundDouble(double d, int32_t* rounded) {
	if ((d - (double) (long) d) < 0.5) {
		*rounded=(long) d;
	} else {
		*rounded=(long) (d + 1);
	}
}

/**
 * @brief  Ceil double to integer.
 * @param  d: number to ceil.
 * @param  ceiled: pointer where to place ceiled number
 */
void ceilDouble(double d, int32_t* ceiled) {
	if ((d - (double) (long) d) == 0) {
		*ceiled=(long) d;
	} else {
		*ceiled=(long) (d + 1);
	}
}
