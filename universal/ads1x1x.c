/**
  ******************************************************************************
  * @file    ads1x1x.c
  * @author  Juraj Lonc (juraj.lonc@gmail.com)
  * @brief   ADS1013/ADS1014/ADS1015/ADS1113/ADS1114/ADS1115 driver.
  ******************************************************************************
  */

#include "ads1x1x.h"

/**
 * @brief  ADS1x1x initialization.
 * @param	handle*		pointer to ADS1X1X_Handle
 * @param	addrpin		ADDR_Pin address selection.
 * @param	write_reg	The function that writes command to the module. Hardware dependent.
 * @param	read_reg	The function that reads data from the module. Hardware dependent.
 * @retval	status		0=device found
 */
uint8_t ADS1X1X_init(ADS1X1X_Handle *handle, ADS1X1X_Variant variant, ADS1X1X_ADDR_Pin addrpin, device_write_ptr write_reg, device_read_ptr read_reg) {
	handle->write_reg=write_reg;
	handle->read_reg=read_reg;
	handle->addr=ADS1X1X_I2C_ADDRESS+addrpin;
	handle->variant=variant;
	handle->config.DATA=0x8583; // default value

	return 0;
}

uint8_t ADS1X1X_set_lo_threshold(ADS1X1X_Handle *handle, int16_t val) {
	if (handle) val*=16; // add 4b padding to 12bit ADC
	return handle->write_reg(handle, ADS1X1X_REG_LO_THRESHOLD, (uint8_t*)&val, 2);
}

uint8_t ADS1X1X_set_hi_threshold(ADS1X1X_Handle *handle, int16_t val) {
	if (handle->variant<=ADS1015) val*=16; // add 4b padding to 12bit ADC
	return handle->write_reg(handle, ADS1X1X_REG_HI_THRESHOLD, (uint8_t*)&val, 2);
}

uint8_t ADS1X1X_set_config(ADS1X1X_Handle *handle) {
	return handle->write_reg(handle, ADS1X1X_REG_CONFIG, (uint8_t*)&handle->config.DATA, 2);
}

uint8_t ADS1X1X_read_lo_threshold(ADS1X1X_Handle *handle, int16_t *val) {
	return handle->read_reg(handle, ADS1X1X_REG_LO_THRESHOLD, (uint8_t*)val, 2);
}

uint8_t ADS1X1X_read_hi_threshold(ADS1X1X_Handle *handle, int16_t *val) {
	return handle->read_reg(handle, ADS1X1X_REG_HI_THRESHOLD, (uint8_t*)val, 2);
}

uint8_t ADS1X1X_read_config(ADS1X1X_Handle *handle) {
	return handle->read_reg(handle, ADS1X1X_REG_CONFIG, (uint8_t*)&handle->config.DATA, 2);
}

void ADS1X1X_set_data_rate(ADS1X1X_Handle *handle, ADS1X1X_DataRate dr) {
	handle->config.DR=dr;
}
void ADS1X1X_set_pga(ADS1X1X_Handle *handle, ADS1X1X_PGA pga) {
	handle->config.PGA=pga;
}
void ADS1X1X_set_mux(ADS1X1X_Handle *handle, ADS1X1X_MUX mux) {
	handle->config.MUX=mux;
}
void ADS1X1X_set_mode(ADS1X1X_Handle *handle, ADS1X1X_Mode mode) {
	handle->config.MODE=mode;
}
void ADS1X1X_set_comparator(ADS1X1X_Handle *handle, ADS1X1X_ComparatorMode mode, ADS1X1X_ComparatorPolarity pol, ADS1X1X_ComparatorLatching lat, ADS1X1X_ComparatorQueue queue) {
	handle->config.COMP_MODE=mode;
	handle->config.COMP_POL=pol;
	handle->config.COMP_LAT=lat;
	handle->config.COMP_QUE=queue;
}

uint8_t ADS1X1X_start_one_shot(ADS1X1X_Handle *handle) {
	handle->config.OS=ADS1X1X_STATUS_BEGIN_CONVERSION;
	handle->config.MODE=ADS1X1X_MODE_SINGLE_SHOT;

	return ADS1X1X_set_config(handle);
}
uint8_t ADS1X1X_start_continuous(ADS1X1X_Handle *handle) {
	handle->config.MODE=ADS1X1X_MODE_CONTINOUS;

	return ADS1X1X_set_config(handle);
}
