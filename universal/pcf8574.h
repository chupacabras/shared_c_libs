/**
  ******************************************************************************
  * @file    pcf8574.h
  * @author  Juraj Lonc (juraj.lonc@gmail.com)
  * @brief   Header file of PCF8574 driver.
  ******************************************************************************
  */

#ifndef INC_PCF8574_H_
#define INC_PCF8574_H_

#include <stdint.h>
#include "device.h"


#define PCF8574_ADDRESS			0b01000000


typedef struct {
	device_write_ptr write_reg;
	device_read_ptr read_reg;
	uint8_t addr;
} PCF8574_Object;

void PCF8574_init(PCF8574_Object *obj, uint8_t addr_pins, device_write_ptr write_reg, device_read_ptr read_reg);
uint8_t PCF8574_read_port(PCF8574_Object *obj);
void PCF8574_write_port(PCF8574_Object *obj, uint8_t port_data);


#endif /* INC_PCF8574_H_ */
