#ifndef AT24CXX_H
#define	AT24CXX_H

#include <stdint.h>
#include <stdbool.h>
#include "device.h"

#define AT24CXX_ADDR		  0b10100000
#define AT24CXX_READ_ADDR	 0b10100001
#define AT24CXX_WRITE_ADDR	0b10100000

#define AT24C02_PAGE_SIZE	8
#define AT24C04_PAGE_SIZE	16
#define AT24C08_PAGE_SIZE	16
#define AT24C16_PAGE_SIZE	16

#define AT24C32_PAGE_SIZE	32
#define AT24C64_PAGE_SIZE	32
#define AT24C512_PAGE_SIZE	128
#define AT24C1024_PAGE_SIZE	256

/**
 * @brief  Function that writes (I2C) to memory address.
 * @param	void*		pointer to customizable handle
 * @param	uint8_t		device I2C address
 * @param	uint16_t	memory address
 * @param	uint8_t		length of memory address (1 or 2 bytes)
 * @param	uint8_t*	data buffer
 * @param	uint16_t	length of data to write
 * @retval 0 = no problem
 */
typedef int8_t(*at24cxx_write_ptr)(void *, uint8_t, uint16_t, uint8_t, uint8_t *, uint16_t);

typedef struct tag_AT24CXX_Object {
	at24cxx_write_ptr write_reg;
	device_read_ptr read_reg;
	device_delay_ms_ptr delay_ms;

	uint8_t m_address_length;	// 1 or 2 bytes
	uint8_t address_pins_as_page_mapping;
	uint8_t address;
	uint16_t page_size;
	uint32_t eeprom_size; // bytes

} AT24CXX_Object;

void AT24C02_init(AT24CXX_Object *obj, at24cxx_write_ptr write_reg, device_read_ptr read_reg, device_delay_ms_ptr delay_ms, bool a2, bool a1, bool a0);
void AT24C04_init(AT24CXX_Object *obj, at24cxx_write_ptr write_reg, device_read_ptr read_reg, device_delay_ms_ptr delay_ms, bool a2, bool a1);
void AT24C08_init(AT24CXX_Object *obj, at24cxx_write_ptr write_reg, device_read_ptr read_reg, device_delay_ms_ptr delay_ms, bool a2);
void AT24C16_init(AT24CXX_Object *obj, at24cxx_write_ptr write_reg, device_read_ptr read_reg, device_delay_ms_ptr delay_ms);
void AT24C32_init(AT24CXX_Object *obj, at24cxx_write_ptr write_reg, device_read_ptr read_reg, device_delay_ms_ptr delay_ms, bool a2, bool a1, bool a0);
void AT24C64_init(AT24CXX_Object *obj, at24cxx_write_ptr write_reg, device_read_ptr read_reg, device_delay_ms_ptr delay_ms, bool a2, bool a1, bool a0);
void AT24C512_init(AT24CXX_Object *obj, at24cxx_write_ptr write_reg, device_read_ptr read_reg, device_delay_ms_ptr delay_ms, bool a2, bool a1, bool a0);
void AT24C1024_init(AT24CXX_Object *obj, at24cxx_write_ptr write_reg, device_read_ptr read_reg, device_delay_ms_ptr delay_ms, bool a2, bool a1, bool a0);

uint16_t AT24CXX_read_int(AT24CXX_Object *obj, uint16_t m_address, uint8_t *buf);
uint32_t AT24CXX_read_long(AT24CXX_Object *obj, uint16_t m_address, uint8_t *buf);
float AT24CXX_read_float(AT24CXX_Object *obj, uint16_t m_address, uint8_t *buf);
double AT24CXX_read_double(AT24CXX_Object *obj, uint16_t m_address, uint8_t *buf);

void AT24CXX_write_int(AT24CXX_Object *obj, uint16_t m_address, uint16_t data);
void AT24CXX_write_long(AT24CXX_Object *obj, uint16_t m_address, uint32_t data);
void AT24CXX_write_float(AT24CXX_Object *obj, uint16_t m_address, float data);
void AT24CXX_write_double(AT24CXX_Object *obj, uint16_t m_address, double data);

uint8_t AT24CXX_read_byte(AT24CXX_Object *obj, uint16_t m_address);
uint8_t AT24CXX_read_next_byte(AT24CXX_Object *obj);
void AT24CXX_write_byte(AT24CXX_Object *obj, uint16_t m_address, uint8_t data);
void AT24CXX_write_data(AT24CXX_Object *obj, uint16_t m_address, uint8_t *data, uint16_t length);
uint16_t AT24CXX_read_data(AT24CXX_Object *obj, uint16_t m_address, uint8_t *data, uint16_t length);


#endif	/* AT24C32_64_H */

