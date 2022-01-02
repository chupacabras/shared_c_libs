#include "at24cxx.h"

AT24CXX_Handle eeprom;

void hw_delay_ms(uint16_t ms) {
	HAL_Delay(ms);
}

int8_t at24cxx_i2c_read(void *handle, uint8_t address, uint8_t *data, uint16_t count) {
	return HAL_I2C_Master_Receive(&hi2c1, address, data, count, 100);
}

int8_t at24cxx_i2c_write(void *handle, uint8_t address, uint16_t m_address, uint8_t m_address_length, uint8_t *data, uint16_t count) {
	return HAL_I2C_Mem_Write(&hi2c1, address, m_address, m_address_length, data, count, 100);
}

int main(void) {
	// standard HAL initializations here
	...

	AT24C08_init(&eeprom, at24cxx_i2c_write, at24cxx_i2c_read, hw_delay_ms, 0);

	// read data
	AT24CXX_read_data(&eeprom, EEPROM_PASSWORDS_ADDR, passwords, 128);

	// write data
	AT24CXX_write_data(&eeprom, EEPROM_PASSWORDS_ADDR, passwords, 128);


	while (1) {
		
	}
}
