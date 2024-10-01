/**
  ******************************************************************************
  * @file    bl0940.c
  * @author  Juraj Lonc (juraj.lonc@gmail.com)
  * @brief   BL0940 driver.
  ******************************************************************************
  */

#include "bl0940.h"

static uint8_t tmp_data[3];

/**
 * @brief  BL0940 initialization.
 * @param	handle*		pointer to BL0940_Handle
 * @param	write_reg	The function that writes command to the module. Hardware dependent.
 * @param	read_reg	The function that reads data from the module. Hardware dependent.
 * @retval	status		0=device found
 */
void BL0940_init(BL0940_Handle *handle, bl094x_write_ptr write_reg, bl094x_read_ptr read_reg, bl094x_cs_ptr chip_select) {
	handle->write_reg=write_reg;
	handle->read_reg=read_reg;
	handle->chip_select=chip_select;
}

static uint8_t BL0940_read_reg(BL0940_Handle *handle, uint8_t reg_addr, uint8_t *data, uint8_t count) {
	uint8_t tmp[3];
	tmp[0]=BL0940_ADDR_READ;
	tmp[1]=reg_addr;

	if (handle->chip_select!=NULL) handle->chip_select(true);
	handle->write_reg(tmp, 2);
	uint8_t stat=handle->read_reg(data, count);
	stat=handle->read_reg(&tmp[2], 1);
	if (handle->chip_select!=NULL) handle->chip_select(false);

	uint8_t chksum=BL094X_calculate_checksum(tmp, 2, data, count);
	if (tmp[2]!=chksum) return 10;

	return stat;
}

static uint8_t BL0940_write_reg(BL0940_Handle *handle, uint8_t reg_addr, uint8_t *data, uint8_t count) {
	uint8_t tmp[3];
	tmp[0]=BL0940_ADDR_WRITE;
	tmp[1]=reg_addr;

	uint8_t chksum=BL094X_calculate_checksum(tmp, 2, data, count);

	if (handle->chip_select!=NULL) handle->chip_select(true);
	handle->write_reg(tmp, 2);
	handle->write_reg(data, count);
	uint8_t stat=handle->write_reg(&chksum, 1);
	if (handle->chip_select!=NULL) handle->chip_select(false);

	return stat;
}
