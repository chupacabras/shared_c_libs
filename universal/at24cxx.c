#include "at24cxx.h"

/**
 * @brief  Calculate I2C address and memory address. Some bits of memory address can be transferred in I2C address.
 * @param  handle:       Pointer to a AT24CXX_Object structure that contains
 *                    the information for the EEPROM.
 * @param  m_address: 	Requested memory address.
 */
static void AT24CXX_prepare_address(AT24CXX_Handle *handle, uint16_t m_address) {
	if (handle->m_address_length==2) {
		handle->prepared_address=handle->address;
		handle->prepared_m_address=m_address;
	} else {
		uint8_t a = (m_address >> 8) & ((handle->address_pins_as_page_mapping == 0) ? 0b0 : ((handle->address_pins_as_page_mapping == 1) ? 0b1 : ((handle->address_pins_as_page_mapping == 2) ? 0b11 : 0b111)));

		handle->prepared_address=handle->address | a;
		handle->prepared_m_address=m_address & 0xff;
	}
}

/**
 * @brief  Write page to EEPROM. For internal purposes only.
 * @param  handle:       Pointer to a AT24CXX_Object structure that contains
 *                    the information for the EEPROM.
 * @param  m_address: 	Target memory address.
 * @param  data: 		Data to store.
 * @param  length: 		Length of data to store.
 */
static void AT24CXX_write_page(AT24CXX_Handle *handle, uint16_t m_address, uint8_t *data, uint8_t length) {
	AT24CXX_prepare_address(handle, m_address);
	handle->write_reg(handle, handle->prepared_address, handle->prepared_m_address, handle->m_address_length, data, length);

	// tWR - see datasheet - Write Cycle Time
	handle->delay_ms(10);
}

void AT24C02_init(AT24CXX_Handle *handle, at24cxx_write_ptr write_reg, device_read_ptr read_reg, device_delay_ms_ptr delay_ms, bool a2, bool a1, bool a0) {
	handle->eeprom_size = 256; // 2kbit
	handle->page_size = AT24C02_PAGE_SIZE;
	handle->address = AT24CXX_ADDR | (a0 ? 0b00000010 : 0) | (a1 ? 0b00000100 : 0) | (a2 ? 0b00001000 : 0);
	handle->address_pins_as_page_mapping = 0;
	handle->m_address_length = 1;
	handle->write_reg=write_reg;
	handle->read_reg=read_reg;
    handle->delay_ms=delay_ms;
}

void AT24C04_init(AT24CXX_Handle *handle, at24cxx_write_ptr write_reg, device_read_ptr read_reg, device_delay_ms_ptr delay_ms, bool a2, bool a1) {
	handle->eeprom_size = 512; // 4kbit
	handle->page_size = AT24C04_PAGE_SIZE;
	handle->address = AT24CXX_ADDR | (a1 ? 0b00000100 : 0) | (a2 ? 0b00001000 : 0);
	handle->address_pins_as_page_mapping = 1;
	handle->m_address_length = 1;
	handle->write_reg=write_reg;
	handle->read_reg=read_reg;
    handle->delay_ms=delay_ms;
}

void AT24C08_init(AT24CXX_Handle *handle, at24cxx_write_ptr write_reg, device_read_ptr read_reg, device_delay_ms_ptr delay_ms, bool a2) {
	handle->eeprom_size = 1024; // 8kbit
	handle->page_size = AT24C08_PAGE_SIZE;
	handle->address = AT24CXX_ADDR | (a2 ? 0b00001000 : 0);
	handle->address_pins_as_page_mapping = 2;
	handle->m_address_length = 1;
	handle->write_reg=write_reg;
	handle->read_reg=read_reg;
    handle->delay_ms=delay_ms;
}

void AT24C16_init(AT24CXX_Handle *handle, at24cxx_write_ptr write_reg, device_read_ptr read_reg, device_delay_ms_ptr delay_ms) {
	handle->eeprom_size = 2048; // 16kbit
	handle->page_size = AT24C16_PAGE_SIZE;
	handle->address = AT24CXX_ADDR;
	handle->address_pins_as_page_mapping = 3;
	handle->m_address_length = 1;
	handle->write_reg=write_reg;
	handle->read_reg=read_reg;
    handle->delay_ms=delay_ms;
}

void AT24C32_init(AT24CXX_Handle *handle, at24cxx_write_ptr write_reg, device_read_ptr read_reg, device_delay_ms_ptr delay_ms, bool a2, bool a1, bool a0) {
	handle->eeprom_size = 4096; // 32kbit
	handle->page_size = AT24C32_PAGE_SIZE;
	handle->address = AT24CXX_ADDR | (a0 ? 0b00000010 : 0) | (a1 ? 0b00000100 : 0) | (a2 ? 0b00001000 : 0);
	handle->address_pins_as_page_mapping = 0;
	handle->m_address_length = 2;
	handle->write_reg=write_reg;
	handle->read_reg=read_reg;
    handle->delay_ms=delay_ms;
}

void AT24C64_init(AT24CXX_Handle *handle, at24cxx_write_ptr write_reg, device_read_ptr read_reg, device_delay_ms_ptr delay_ms, bool a2, bool a1, bool a0) {
	handle->eeprom_size = 8192; // 64kbit
	handle->page_size = AT24C64_PAGE_SIZE;
	handle->address = AT24CXX_ADDR | (a0 ? 0b00000010 : 0) | (a1 ? 0b00000100 : 0) | (a2 ? 0b00001000 : 0);
	handle->address_pins_as_page_mapping = 0;
	handle->m_address_length = 2;
	handle->write_reg=write_reg;
	handle->read_reg=read_reg;
    handle->delay_ms=delay_ms;
}

void AT24C512_init(AT24CXX_Handle *handle, at24cxx_write_ptr write_reg, device_read_ptr read_reg, device_delay_ms_ptr delay_ms, bool a2, bool a1, bool a0) {
	handle->eeprom_size = 65536L; // 512kbit
	handle->page_size = AT24C512_PAGE_SIZE;
	handle->address = AT24CXX_ADDR | (a0 ? 0b00000010 : 0) | (a1 ? 0b00000100 : 0) | (a2 ? 0b00001000 : 0);
	handle->address_pins_as_page_mapping = 0;
	handle->m_address_length = 2;
	handle->write_reg=write_reg;
	handle->read_reg=read_reg;
    handle->delay_ms=delay_ms;
}

void AT24C1024_init(AT24CXX_Handle *handle, at24cxx_write_ptr write_reg, device_read_ptr read_reg, device_delay_ms_ptr delay_ms, bool a2, bool a1, bool a0) {
	handle->eeprom_size = 131072L; // 1024kbit
	handle->page_size = AT24C1024_PAGE_SIZE;
	handle->address = AT24CXX_ADDR | (a0 ? 0b00000010 : 0) | (a1 ? 0b00000100 : 0) | (a2 ? 0b00001000 : 0);
	handle->address_pins_as_page_mapping = 0;
	handle->m_address_length = 2;
	handle->write_reg=write_reg;
	handle->read_reg=read_reg;
    handle->delay_ms=delay_ms;
}

uint8_t AT24CXX_read_byte(AT24CXX_Handle *handle, uint16_t m_address) {
	uint8_t value = 0;

	AT24CXX_prepare_address(handle, m_address);
	handle->write_reg(handle, handle->prepared_address, handle->prepared_m_address, handle->m_address_length, 0, 0);
	handle->read_reg(handle, handle->prepared_address | 0x01, &value, 1);

	return value;
}


uint8_t AT24CXX_read_next_byte(AT24CXX_Handle *handle) {
	uint8_t value = 0;

	handle->read_reg(handle, handle->prepared_address | 0x01, &value, 1);

	return value;
}

uint16_t AT24CXX_read_data(AT24CXX_Handle *handle, uint16_t m_address, uint8_t *data, uint16_t length) {
	if (length > (handle->eeprom_size - m_address - 1)) {
		length = handle->eeprom_size - m_address - 1;
	}

	AT24CXX_prepare_address(handle, m_address);
	handle->write_reg(handle, handle->prepared_address, handle->prepared_m_address, handle->m_address_length, 0, 0);
	handle->read_reg(handle, handle->prepared_address | 0x01, data, length);

	return length;
}

uint16_t AT24CXX_read_int(AT24CXX_Handle *handle, uint16_t m_address, uint8_t *buf) {
	AT24CXX_read_data(handle, m_address, buf, 2);

	return *(uint16_t*) & buf[0];
}

uint32_t AT24CXX_read_long(AT24CXX_Handle *handle, uint16_t m_address, uint8_t *buf) {
	AT24CXX_read_data(handle, m_address, buf, 4);

	return *(uint32_t*) & buf[0];
}

float AT24CXX_read_float(AT24CXX_Handle *handle, uint16_t m_address, uint8_t *buf) {
	AT24CXX_read_data(handle, m_address, buf, sizeof(float));

	return *(float*) &buf[0];
}

double AT24CXX_read_double(AT24CXX_Handle *handle, uint16_t m_address, uint8_t *buf) {
	AT24CXX_read_data(handle, m_address, buf, sizeof(double));

	return *(double*) &buf[0];
}

void AT24CXX_write_byte(AT24CXX_Handle *handle, uint16_t m_address, uint8_t data) {
	AT24CXX_prepare_address(handle, m_address);
	handle->write_reg(handle, handle->prepared_address, handle->prepared_m_address, handle->m_address_length, &data, 1);
}

void AT24CXX_write_int(AT24CXX_Handle *handle, uint16_t m_address, uint16_t data) {
	AT24CXX_write_data(handle, m_address, (uint8_t*) & data, 2);
}

void AT24CXX_write_long(AT24CXX_Handle *handle, uint16_t m_address, uint32_t data) {
	AT24CXX_write_data(handle, m_address, (uint8_t*) & data, 4);
}

void AT24CXX_write_float(AT24CXX_Handle *handle, uint16_t m_address, float data) {
	AT24CXX_write_data(handle, m_address, (uint8_t*) & data, sizeof(float));
}

void AT24CXX_write_double(AT24CXX_Handle *handle, uint16_t m_address, double data) {
	AT24CXX_write_data(handle, m_address, (uint8_t*) & data, sizeof(double));
}

void AT24CXX_write_data(AT24CXX_Handle *handle, uint16_t m_address, uint8_t *data, uint16_t length) {
	// write first page if not aligned
	uint8_t first_page_offset = m_address % handle->page_size;
	uint8_t first_page_not_aligned_length = 0;
	if (first_page_offset > 0) {
		first_page_not_aligned_length = handle->page_size - first_page_offset;
		if (length < first_page_not_aligned_length) {
			first_page_not_aligned_length = length;
		}
		AT24CXX_write_page(handle, m_address, data, first_page_not_aligned_length);
		length -= first_page_not_aligned_length;
	}

	// write following pages
	if (length > 0) {
		m_address += first_page_not_aligned_length;
		data += first_page_not_aligned_length;

		// write complete and aligned pages
		uint16_t whole_pages_count = length / handle->page_size;
		uint16_t q;
		for (q = 0; q < whole_pages_count; q++) {
			AT24CXX_write_page(handle, m_address, data, handle->page_size);

			m_address += handle->page_size;
			data += handle->page_size;
			length -= handle->page_size;
		}

		// write remaining incomplete page
		if (length > 0) {
			AT24CXX_write_page(handle, m_address, data, length);
		}

	}

}
