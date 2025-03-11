/**
  ******************************************************************************
  * @file    pcf8574.c
  * @author  Juraj Lonc (juraj.lonc@gmail.com)
  * @brief   PCF8574 I/O expander driver.
  ******************************************************************************
  */

#include "pcf8574.h"

/**
 * @brief  PCF8574 initialization.
 * @param	handle*		pointer to PCF8574_Handle
 * @param	addr_pins	A0, A1, A2 address pins. 0b000 to 0b111
 * @param	write_reg	The function that writes command to the module. Hardware dependent.
 * @param	read_reg	The function that reads data from the module. Hardware dependent.
 */
void PCF8574_init(PCF8574_Handle *handle, uint8_t addr_pins, device_write_ptr write_reg, device_read_ptr read_reg) {
	handle->write_reg=write_reg;
	handle->read_reg=read_reg;
	handle->addr=PCF8574_ADDRESS | ((addr_pins<<1) & 0b1110);
}

/**
 * @brief  PCF8574 read port.
 * @param	handle*			pointer to PCF8574_Handle
 * @retval	port values
 */
uint8_t PCF8574_read_port(PCF8574_Handle *handle) {
	uint8_t v;
	return handle->read_reg(handle, 0, &v, 1);
}

/**
 * @brief  PCF8574 write port.
 * @param	handle*			pointer to PCF8574_Handle
 * @param	port_data		port values
 */
void PCF8574_write_port(PCF8574_Handle *handle, uint8_t port_data) {
	handle->write_reg(handle, 0, &port_data, 1);
}
