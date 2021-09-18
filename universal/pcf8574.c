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
 * @param	obj*		pointer to PCF8574_Object
 * @param	addr_pins	A0, A1, A2 address pins. 0b000 to 0b111
 * @param	write_reg	The function that writes command to the module. Hardware dependent.
 * @param	read_reg	The function that reads data from the module. Hardware dependent.
 */
void PCF8574_init(PCF8574_Object *obj, uint8_t addr_pins, device_write_ptr write_reg, device_read_ptr read_reg) {
	obj->write_reg=write_reg;
	obj->read_reg=read_reg;
	obj->addr=PCF8574_ADDRESS | ((addr_pins<<1) & 0b1110);
}

/**
 * @brief  PCF8574 read port.
 * @param	obj*			pointer to PCF8574_Object
 * @retval	port values
 */
uint8_t PCF8574_read_port(PCF8574_Object *obj) {
	uint8_t v;
	return obj->read_reg(obj, 0, &v, 1);
}

/**
 * @brief  PCF8574 write port.
 * @param	obj*			pointer to PCF8574_Object
 * @param	port_data		port values
 */
void PCF8574_write_port(PCF8574_Object *obj, uint8_t port_data) {
	obj->write_reg(obj, 0, &port_data, 1);
}
