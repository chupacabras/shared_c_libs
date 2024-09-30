/**
  ******************************************************************************
  * @file    bl0942.c
  * @author  Juraj Lonc (juraj.lonc@gmail.com)
  * @brief   BL0942 driver.
  ******************************************************************************
  */

#include "bl0942.h"

/**
 * @brief  BL0942 initialization.
 * @param	obj*		pointer to BL0942_Handle
 * @param	write_reg	The function that writes command to the module. Hardware dependent.
 * @param	read_reg	The function that reads data from the module. Hardware dependent.
 * @param	delay_ms	The function that makes delay in milliseconds. Hardware dependent.
 * @retval	status		0=device found
 */
uint8_t BL0942_init_SPI(BL0942_Handle *obj, device_write_ptr write_reg, device_read_ptr read_reg, device_delay_ms_ptr delay_ms) {
	obj->write_reg=write_reg;
	obj->read_reg=read_reg;
	obj->delay_ms=delay_ms;
	obj->com_type=BL0942_COM_SPI;

//	obj->addr=addrpin==0?BL0942_I2C_ADDRESS_LOW:BL0942_I2C_ADDRESS_HIGH;
//
//	// after power-up: it takes 20 milliseconds at most to enter idle state
//	obj->delay_ms(20);
//	uint8_t ret=obj->write_reg(obj, BL0942_CMD_INIT, 0, 0);
//	if (ret!=0) return ret;
//
//	obj->delay_ms(20);
//	BL0942_soft_reset(obj);
//	// Soft reset takes no more than 20 milliseconds
//	obj->delay_ms(20);
	return 0;
}

uint8_t BL0942_init_UART(BL0942_Handle *obj, uint8_t addr1, uint8_t addr2, device_write_ptr write_reg, device_read_ptr read_reg, device_delay_ms_ptr delay_ms) {
	obj->write_reg=write_reg;
	obj->read_reg=read_reg;
	obj->delay_ms=delay_ms;
	obj->com_type=BL0942_COM_UART;
	obj->addr=BL0942_UART_ADDR;
	if (addr1) obj->addr|=0b01;
	if (addr2) obj->addr|=0b10;

//	obj->addr=addrpin==0?BL0942_I2C_ADDRESS_LOW:BL0942_I2C_ADDRESS_HIGH;
//
//	// after power-up: it takes 20 milliseconds at most to enter idle state
//	obj->delay_ms(20);
//	uint8_t ret=obj->write_reg(obj, BL0942_CMD_INIT, 0, 0);
//	if (ret!=0) return ret;
//
//	obj->delay_ms(20);
//	BL0942_soft_reset(obj);
//	// Soft reset takes no more than 20 milliseconds
//	obj->delay_ms(20);
	return 0;
}

void BL0942_read_watt(BL0942_Handle *obj, uint8_t *data) {
	obj->read_reg(obj, BL0942_REG_WATT, data, 3);
}
