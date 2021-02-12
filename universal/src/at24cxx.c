#include "at24cxx.h"

static uint8_t prepared_address;
static uint16_t prepared_m_address;

/**
 * @brief  Initialize EEPROM object/handler.
 * @param  obj:       Pointer to a AT24CXX_Object structure that contains
 *                    the information for the EEPROM.
 * @param  write_reg: 	The function that writes data/command to the module. Hardware dependent.
 * @param  read_reg: 	The function that reads data fromo the module. Hardware dependent.
 * @param  delay_ms: 	The function that makes delay in milliseconds. Hardware dependent.
 * @param  a2: 			I2C address input value
 * @param  a1: 			I2C address input value
 * @param  a0: 			I2C address input value
 */
void AT24C02_init(AT24CXX_Object *obj, at24cxx_write_ptr write_reg, device_read_ptr read_reg, device_delay_ms_ptr delay_ms, bool a2, bool a1, bool a0) {
	obj->eeprom_size = 256; // 2kbit
	obj->page_size = AT24C02_PAGE_SIZE;
	obj->address = AT24CXX_ADDR | (a0 ? 0b00000010 : 0) | (a1 ? 0b00000100 : 0) | (a2 ? 0b00001000 : 0);
	obj->address_pins_as_page_mapping = 0;
	obj->m_address_length = 1;
	obj->write_reg=write_reg;
	obj->read_reg=read_reg;
}

/**
 * @brief  Initialize EEPROM object/handler.
 * @param  obj:       Pointer to a AT24CXX_Object structure that contains
 *                    the information for the EEPROM.
 * @param  write_reg: 	The function that writes data/command to the module. Hardware dependent.
 * @param  read_reg: 	The function that reads data fromo the module. Hardware dependent.
 * @param  delay_ms: 	The function that makes delay in milliseconds. Hardware dependent.
 * @param  a2: 			I2C address input value
 * @param  a1: 			I2C address input value
 */
void AT24C04_init(AT24CXX_Object *obj, at24cxx_write_ptr write_reg, device_read_ptr read_reg, device_delay_ms_ptr delay_ms, bool a2, bool a1) {
	obj->eeprom_size = 512; // 4kbit
	obj->page_size = AT24C04_PAGE_SIZE;
	obj->address = AT24CXX_ADDR | (a1 ? 0b00000100 : 0) | (a2 ? 0b00001000 : 0);
	obj->address_pins_as_page_mapping = 1;
	obj->m_address_length = 1;
	obj->write_reg=write_reg;
	obj->read_reg=read_reg;
}

/**
 * @brief  Initialize EEPROM object/handler.
 * @param  obj:       Pointer to a AT24CXX_Object structure that contains
 *                    the information for the EEPROM.
 * @param  write_reg: 	The function that writes data/command to the module. Hardware dependent.
 * @param  read_reg: 	The function that reads data fromo the module. Hardware dependent.
 * @param  delay_ms: 	The function that makes delay in milliseconds. Hardware dependent.
 * @param  a2: 			I2C address input value
 */
void AT24C08_init(AT24CXX_Object *obj, at24cxx_write_ptr write_reg, device_read_ptr read_reg, device_delay_ms_ptr delay_ms, bool a2) {
	obj->eeprom_size = 1024; // 8kbit
	obj->page_size = AT24C08_PAGE_SIZE;
	obj->address = AT24CXX_ADDR | (a2 ? 0b00001000 : 0);
	obj->address_pins_as_page_mapping = 2;
	obj->m_address_length = 1;
	obj->write_reg=write_reg;
	obj->read_reg=read_reg;
}

/**
 * @brief  Initialize EEPROM object/handler.
 * @param  obj:       Pointer to a AT24CXX_Object structure that contains
 *                    the information for the EEPROM.
 * @param  write_reg: 	The function that writes data/command to the module. Hardware dependent.
 * @param  read_reg: 	The function that reads data fromo the module. Hardware dependent.
 * @param  delay_ms: 	The function that makes delay in milliseconds. Hardware dependent.
 */
void AT24C16_init(AT24CXX_Object *obj, at24cxx_write_ptr write_reg, device_read_ptr read_reg, device_delay_ms_ptr delay_ms) {
	obj->eeprom_size = 2048; // 16kbit
	obj->page_size = AT24C16_PAGE_SIZE;
	obj->address = AT24CXX_ADDR;
	obj->address_pins_as_page_mapping = 3;
	obj->m_address_length = 1;
	obj->write_reg=write_reg;
	obj->read_reg=read_reg;
}

/**
 * @brief  Initialize EEPROM object/handler.
 * @param  obj:       Pointer to a AT24CXX_Object structure that contains
 *                    the information for the EEPROM.
 * @param  write_reg: 	The function that writes data/command to the module. Hardware dependent.
 * @param  read_reg: 	The function that reads data fromo the module. Hardware dependent.
 * @param  delay_ms: 	The function that makes delay in milliseconds. Hardware dependent.
 * @param  a2: 			I2C address input value
 * @param  a1: 			I2C address input value
 * @param  a0: 			I2C address input value
 */
void AT24C32_init(AT24CXX_Object *obj, at24cxx_write_ptr write_reg, device_read_ptr read_reg, device_delay_ms_ptr delay_ms, bool a2, bool a1, bool a0) {
	obj->eeprom_size = 4096; // 32kbit
	obj->page_size = AT24C32_PAGE_SIZE;
	obj->address = AT24CXX_ADDR | (a0 ? 0b00000010 : 0) | (a1 ? 0b00000100 : 0) | (a2 ? 0b00001000 : 0);
	obj->address_pins_as_page_mapping = 0;
	obj->m_address_length = 2;
	obj->write_reg=write_reg;
	obj->read_reg=read_reg;
}

/**
 * @brief  Initialize EEPROM object/handler.
 * @param  obj:       Pointer to a AT24CXX_Object structure that contains
 *                    the information for the EEPROM.
 * @param  write_reg: 	The function that writes data/command to the module. Hardware dependent.
 * @param  read_reg: 	The function that reads data fromo the module. Hardware dependent.
 * @param  delay_ms: 	The function that makes delay in milliseconds. Hardware dependent.
 * @param  a2: 			I2C address input value
 * @param  a1: 			I2C address input value
 * @param  a0: 			I2C address input value
 */
void AT24C64_init(AT24CXX_Object *obj, at24cxx_write_ptr write_reg, device_read_ptr read_reg, device_delay_ms_ptr delay_ms, bool a2, bool a1, bool a0) {
	obj->eeprom_size = 8192; // 64kbit
	obj->page_size = AT24C64_PAGE_SIZE;
	obj->address = AT24CXX_ADDR | (a0 ? 0b00000010 : 0) | (a1 ? 0b00000100 : 0) | (a2 ? 0b00001000 : 0);
	obj->address_pins_as_page_mapping = 0;
	obj->m_address_length = 2;
	obj->write_reg=write_reg;
	obj->read_reg=read_reg;
}

/**
 * @brief  Initialize EEPROM object/handler.
 * @param  obj:       Pointer to a AT24CXX_Object structure that contains
 *                    the information for the EEPROM.
 * @param  write_reg: 	The function that writes data/command to the module. Hardware dependent.
 * @param  read_reg: 	The function that reads data fromo the module. Hardware dependent.
 * @param  delay_ms: 	The function that makes delay in milliseconds. Hardware dependent.
 * @param  a2: 			I2C address input value
 * @param  a1: 			I2C address input value
 * @param  a0: 			I2C address input value
 */
void AT24C512_init(AT24CXX_Object *obj, at24cxx_write_ptr write_reg, device_read_ptr read_reg, device_delay_ms_ptr delay_ms, bool a2, bool a1, bool a0) {
	obj->eeprom_size = 65536L; // 512kbit
	obj->page_size = AT24C512_PAGE_SIZE;
	obj->address = AT24CXX_ADDR | (a0 ? 0b00000010 : 0) | (a1 ? 0b00000100 : 0) | (a2 ? 0b00001000 : 0);
	obj->address_pins_as_page_mapping = 0;
	obj->m_address_length = 2;
	obj->write_reg=write_reg;
	obj->read_reg=read_reg;
}

/**
 * @brief  Initialize EEPROM object/handler.
 * @param  obj:       Pointer to a AT24CXX_Object structure that contains
 *                    the information for the EEPROM.
 * @param  write_reg: 	The function that writes data/command to the module. Hardware dependent.
 * @param  read_reg: 	The function that reads data fromo the module. Hardware dependent.
 * @param  delay_ms: 	The function that makes delay in milliseconds. Hardware dependent.
 * @param  a2: 			I2C address input value
 * @param  a1: 			I2C address input value
 * @param  a0: 			I2C address input value
 */
void AT24C1024_init(AT24CXX_Object *obj, at24cxx_write_ptr write_reg, device_read_ptr read_reg, device_delay_ms_ptr delay_ms, bool a2, bool a1, bool a0) {
	obj->eeprom_size = 131072L; // 1024kbit
	obj->page_size = AT24C1024_PAGE_SIZE;
	obj->address = AT24CXX_ADDR | (a0 ? 0b00000010 : 0) | (a1 ? 0b00000100 : 0) | (a2 ? 0b00001000 : 0);
	obj->address_pins_as_page_mapping = 0;
	obj->m_address_length = 2;
	obj->write_reg=write_reg;
	obj->read_reg=read_reg;
}

/**
 * @brief  Calculate I2C address and memory address. Some bits of memory address can be transferred in I2C address.
 * @param  obj:       Pointer to a AT24CXX_Object structure that contains
 *                    the information for the EEPROM.
 * @param  m_address: 	Requested memory address.
 */
static void AT24CXX_prepare_address(AT24CXX_Object *obj, uint16_t m_address) {
	if (obj->m_address_length==2) {
		prepared_address=obj->address;
		prepared_m_address=m_address;
	} else {
		uint8_t a = (m_address >> 8) & ((obj->address_pins_as_page_mapping == 0) ? 0b0 : ((obj->address_pins_as_page_mapping == 1) ? 0b1 : ((obj->address_pins_as_page_mapping == 2) ? 0b11 : 0b111)));

		prepared_address=obj->address | a;
		prepared_m_address=m_address & 0xff;
	}
}

/**
 * @brief  Read 1 byte from EEPROM.
 * @param  obj:       Pointer to a AT24CXX_Object structure that contains
 *                    the information for the EEPROM.
 * @param  m_address: 	Requested memory address.
 * @retval value from memory.
 */
uint8_t AT24CXX_read_byte(AT24CXX_Object *obj, uint16_t m_address) {
	uint8_t value = 0;

	AT24CXX_prepare_address(obj, m_address);
	obj->write_reg(obj, prepared_address, prepared_m_address, obj->m_address_length, 0, 0);
	obj->read_reg(obj, prepared_address, &value, 1);

	return value;
}


/**
 * @brief  Read next 1 byte from EEPROM.
 * @param  obj:       Pointer to a AT24CXX_Object structure that contains
 *                    the information for the EEPROM.
 * @retval value from memory
 */
uint8_t AT24CXX_read_next_byte(AT24CXX_Object *obj) {
	uint8_t value = 0;

	obj->read_reg(obj, prepared_address, &value, 1);

	return value;
}

/**
 * @brief  Read data from EEPROM.
 * @param  obj:       Pointer to a AT24CXX_Object structure that contains
 *                    the information for the EEPROM.
 * @param  m_address: 	Requested memory address.
 * @param  data: 		Data buffer.
 * @param  length: 		Count of bytes to read.
 * @retval Count of bytes that were actually read
 */
uint16_t AT24CXX_read_data(AT24CXX_Object *obj, uint16_t m_address, uint8_t *data, uint16_t length) {
	if (length > (obj->eeprom_size - m_address - 1)) {
		length = obj->eeprom_size - m_address - 1;
	}

	AT24CXX_prepare_address(obj, m_address);
	obj->write_reg(obj, prepared_address, prepared_m_address, obj->m_address_length, 0, 0);
	obj->read_reg(obj, prepared_address, data, length);

	return length;
}

/**
 * @brief  Read 2-byte integer from EEPROM.
 * @param  obj:       Pointer to a AT24CXX_Object structure that contains
 *                    the information for the EEPROM.
 * @param  m_address: 	Requested memory address.
 * @param  buf: 		Data buffer.
 * @retval value from memory
 */
uint16_t AT24CXX_read_int(AT24CXX_Object *obj, uint16_t m_address, uint8_t *buf) {
	AT24CXX_read_data(obj, m_address, buf, 2);

	return *(uint16_t*) & buf[0];
}

/**
 * @brief  Read 4-byte integer from EEPROM.
 * @param  obj:       Pointer to a AT24CXX_Object structure that contains
 *                    the information for the EEPROM.
 * @param  m_address: 	Requested memory address.
 * @param  buf: 		Data buffer.
 * @retval value from memory
 */
uint32_t AT24CXX_read_long(AT24CXX_Object *obj, uint16_t m_address, uint8_t *buf) {
	AT24CXX_read_data(obj, m_address, buf, 4);

	return *(uint32_t*) & buf[0];
}

/**
 * @brief  Read float from EEPROM.
 * @param  obj:       Pointer to a AT24CXX_Object structure that contains
 *                    the information for the EEPROM.
 * @param  m_address: 	Requested memory address.
 * @param  buf: 		Data buffer.
 * @retval value from memory
 */
float AT24CXX_read_float(AT24CXX_Object *obj, uint16_t m_address, uint8_t *buf) {
	AT24CXX_read_data(obj, m_address, buf, sizeof(float));

	return *(float*) &buf[0];
}

/**
 * @brief  Read double from EEPROM.
 * @param  obj:       Pointer to a AT24CXX_Object structure that contains
 *                    the information for the EEPROM.
 * @param  m_address: 	Requested memory address.
 * @param  buf: 		Data buffer.
 * @retval value from memory
 */
double AT24CXX_read_double(AT24CXX_Object *obj, uint16_t m_address, uint8_t *buf) {
	AT24CXX_read_data(obj, m_address, buf, sizeof(double));

	return *(double*) &buf[0];
}

/**
 * @brief  Write 1 byte to EEPROM.
 * @param  obj:       Pointer to a AT24CXX_Object structure that contains
 *                    the information for the EEPROM.
 * @param  m_address: 	Target memory address.
 * @param  data: 		Value to store.
 */
void AT24CXX_write_byte(AT24CXX_Object *obj, uint16_t m_address, uint8_t data) {
	AT24CXX_prepare_address(obj, m_address);
	obj->write_reg(obj, prepared_address, prepared_m_address, obj->m_address_length, &data, 1);
}

/**
 * @brief  Write 2-byte integer to EEPROM.
 * @param  obj:       Pointer to a AT24CXX_Object structure that contains
 *                    the information for the EEPROM.
 * @param  m_address: 	Target memory address.
 * @param  data: 		Value to store.
 */
void AT24CXX_write_int(AT24CXX_Object *obj, uint16_t m_address, uint16_t data) {
	AT24CXX_write_data(obj, m_address, (uint8_t*) & data, 2);
}

/**
 * @brief  Write 4-byte integer to EEPROM.
 * @param  obj:       Pointer to a AT24CXX_Object structure that contains
 *                    the information for the EEPROM.
 * @param  m_address: 	Target memory address.
 * @param  data: 		Value to store.
 */
void AT24CXX_write_long(AT24CXX_Object *obj, uint16_t m_address, uint32_t data) {
	AT24CXX_write_data(obj, m_address, (uint8_t*) & data, 4);
}

/**
 * @brief  Write float to EEPROM.
 * @param  obj:       Pointer to a AT24CXX_Object structure that contains
 *                    the information for the EEPROM.
 * @param  m_address: 	Target memory address.
 * @param  data: 		Value to store.
 */
void AT24CXX_write_float(AT24CXX_Object *obj, uint16_t m_address, float data) {
	AT24CXX_write_data(obj, m_address, (uint8_t*) & data, sizeof(float));
}

/**
 * @brief  Write double to EEPROM.
 * @param  obj:       Pointer to a AT24CXX_Object structure that contains
 *                    the information for the EEPROM.
 * @param  m_address: 	Target memory address.
 * @param  data: 		Value to store.
 */
void AT24CXX_write_double(AT24CXX_Object *obj, uint16_t m_address, double data) {
	AT24CXX_write_data(obj, m_address, (uint8_t*) & data, sizeof(double));
}

/**
 * @brief  Write page to EEPROM. For internal purposes only.
 * @param  obj:       Pointer to a AT24CXX_Object structure that contains
 *                    the information for the EEPROM.
 * @param  m_address: 	Target memory address.
 * @param  data: 		Data to store.
 * @param  length: 		Length of data to store.
 */
static void AT24CXX_write_page(AT24CXX_Object *obj, uint16_t m_address, uint8_t *data, uint8_t length) {
	AT24CXX_prepare_address(obj, m_address);
	obj->write_reg(obj, prepared_address, prepared_m_address, obj->m_address_length, data, length);

	// tWR - see datasheet - Write Cycle Time
	obj->delay_ms(10);
}

/**
 * @brief  Write data to EEPROM. Writes data by pages.
 * @param  obj:       Pointer to a AT24CXX_Object structure that contains
 *                    the information for the EEPROM.
 * @param  m_address: 	Target memory address.
 * @param  data: 		Data to store.
 * @param  length: 		Length of data to store.
 */
void AT24CXX_write_data(AT24CXX_Object *obj, uint16_t m_address, uint8_t *data, uint16_t length) {
	// write first page if not aligned
	uint8_t first_page_offset = m_address % obj->page_size;
	uint8_t first_page_not_aligned_length = 0;
	if (first_page_offset > 0) {
		first_page_not_aligned_length = obj->page_size - first_page_offset;
		if (length < first_page_not_aligned_length) {
			first_page_not_aligned_length = length;
		}
		AT24CXX_write_page(obj, m_address, data, first_page_not_aligned_length);
		length -= first_page_not_aligned_length;
	}

	// write following pages
	if (length > 0) {
		m_address += first_page_not_aligned_length;
		data += first_page_not_aligned_length;

		// write complete and aligned pages
		uint16_t whole_pages_count = length / obj->page_size;
		uint16_t q;
		for (q = 0; q < whole_pages_count; q++) {
			AT24CXX_write_page(obj, m_address, data, obj->page_size);

			m_address += obj->page_size;
			data += obj->page_size;
			length -= obj->page_size;
		}

		// write remaining incomplete page
		if (length > 0) {
			AT24CXX_write_page(obj, m_address, data, length);
		}

	}

}
