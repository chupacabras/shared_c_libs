/**
  ******************************************************************************
  * @file    aht10.c
  * @author  Juraj Lonc (juraj.lonc@gmail.com)
  * @brief   AHT10 driver.
  ******************************************************************************
  */

#include "aht10.h"

/**
 * @brief  AHT10 initialization.
 * @param	handle*		pointer to AHT10_Handle
 * @param	write_reg	The function that writes command to the module. Hardware dependent.
 * @param	read_reg	The function that reads data from the module. Hardware dependent.
 * @param	delay_ms	The function that makes delay in milliseconds. Hardware dependent.
 * @retval	status		0=device found
 */
uint8_t AHT10_init(AHT10_Handle *handle, uint8_t addrpin, device_write_ptr write_reg, device_read_ptr read_reg, device_delay_ms_ptr delay_ms) {
	handle->write_reg=write_reg;
	handle->read_reg=read_reg;
	handle->delay_ms=delay_ms;

	handle->addr=addrpin==0?AHT10_I2C_ADDRESS_LOW:AHT10_I2C_ADDRESS_HIGH;

	// after power-up: it takes 20 milliseconds at most to enter idle state
	handle->delay_ms(20);
	uint8_t ret=handle->write_reg(handle, AHT10_CMD_INIT, 0, 0);
	if (ret!=0) return ret;

	handle->delay_ms(20);
	AHT10_soft_reset(handle);
	// Soft reset takes no more than 20 milliseconds
	handle->delay_ms(20);
	return 0;
}

/**
 * @brief  AHT10 soft restart.
 * @param	handle*		pointer to AHT10_Handle
 */
void AHT10_soft_reset(AHT10_Handle *handle) {
	handle->write_reg(handle, AHT10_CMD_SOFT_RESET, 0, 0);
}

/**
 * @brief  AHT10 trigger measurement.
 * @param	handle*		pointer to AHT10_Handle
 */
void AHT10_trigger_measurement(AHT10_Handle *handle) {
	uint8_t data[]={0b00110011, 0};
	handle->write_reg(handle, AHT10_CMD_TRIGGER_MEASUREMENT, data, 2);
}

/**
 * @brief  AHT10 read measurement.
 * @param	handle*			pointer to AHT10_Handle
 * @param	measurement*	pointer to AHT10_Measurement where to store data to
 * @retval	bool			measurement is done and measurement data is correct
 */
bool AHT10_read_measurement(AHT10_Handle *handle, AHT10_Measurement *measurement) {
	handle->read_reg(handle, 0, measurement->data, 6);

	return (~measurement->data[0] & 0x80) && !(measurement->data[1] == 0 && measurement->data[2] == 0 && measurement->data[3] == 0);
}

/**
 * @brief  Convert raw data to temperature (in deg. Celsius).
 * @param	measurement*	pointer to AHT10_Measurement where to store data to
 * @retval	float			temperature (in deg. Celsius)
 */
float AHT10_convert_temperature_float(AHT10_Measurement *measurement) {
	float f = 0;

	uint32_t raw = (((uint32_t)measurement->data[3] & 15) << 16) | ((uint32_t)measurement->data[4] << 8) | measurement->data[5];
	f = (float)(raw * 200.0 / 1048576.0) - 50;

	return f;
}


/**
 * @brief  Convert raw data to humidity (in percent).
 * @param	measurement*	pointer to AHT10_Measurement where to store data to
 * @retval	float			relative humidity (in percent)
 */
float AHT10_convert_humidity_float(AHT10_Measurement *measurement) {
	float f = 0;

	uint32_t raw = ((uint32_t) measurement->data[1] << 12) | ((uint32_t) measurement->data[2] << 4) | (measurement->data[3] >> 4);
	f = (float) (raw * 100.0 / 1048576.0);

	return f;
}

/**
 * @brief  Convert raw data to temperature (in deg. Celsius).
 * @param	measurement*	pointer to AHT10_Measurement where to store data to
 * @retval	int16_t			temperature (hundreths of deg C (10000 = 100.00 C))
 */
int16_t AHT10_convert_temperature(AHT10_Measurement *measurement) {
	uint32_t raw = ((((uint32_t)measurement->data[3] & 0x0f)) << 16) | (((uint32_t)measurement->data[4]) << 8) | measurement->data[5];
    // * 20000 / 1048576 (20bits)
    // * 625 / 32768 (15bits)
    raw*=625;
    raw>>=15;
//	f = (float)(raw * 200.0 / 1048576.0) - 50;

	return ((int16_t)raw)-5000;
}


/**
 * @brief  Convert raw data to relative humidity.
 * @param	measurement*	pointer to AHT10_Measurement where to store data to
 * @retval	uint16_t		relative humidity (per mill value (890 = 89.0%))
 */
uint16_t AHT10_convert_humidity(AHT10_Measurement *measurement) {
	uint32_t raw = (((uint32_t) measurement->data[1]) << 12) | (((uint32_t) measurement->data[2]) << 4) | (measurement->data[3] >> 4);
	// * 1000 / 1048576 (20bits)
	// * 125 / 131072 (17bits))
	raw *= 125;
	raw >>= 17;

	return raw;
}
