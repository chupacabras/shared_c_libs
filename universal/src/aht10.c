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
 * @param	obj*		pointer to AHT10_Object
 * @param	write_reg	The function that writes command to the module. Hardware dependent.
 * @param	read_reg	The function that reads data from the module. Hardware dependent.
 * @param	delay_ms	The function that makes delay in milliseconds. Hardware dependent.
 */
void AHT10_init(AHT10_Object *obj, device_write_ptr write_reg, device_read_ptr read_reg, device_delay_ms_ptr delay_ms) {
	obj->write_reg=write_reg;
	obj->read_reg=read_reg;
	obj->delay_ms=delay_ms;

	// after power-up: it takes 20 milliseconds at most to enter idle state
	obj->delay_ms(20);
	obj->write_reg(obj, AHT10_CMD_INIT, 0, 0);
	obj->delay_ms(20);
	AHT10_soft_reset(obj);
	// Soft reset takes no more than 20 milliseconds
	obj->delay_ms(20);
}

/**
 * @brief  AHT10 soft restart.
 * @param	obj*		pointer to AHT10_Object
 */
void AHT10_soft_reset(AHT10_Object *obj) {
	obj->write_reg(obj, AHT10_CMD_SOFT_RESET, 0, 0);
}

/**
 * @brief  AHT10 trigger measurement.
 * @param	obj*		pointer to AHT10_Object
 */
void AHT10_trigger_measurement(AHT10_Object *obj) {
	uint8_t data[]={0b00110011, 0};
	obj->write_reg(obj, AHT10_CMD_TRIGGER_MEASUREMENT, data, 2);
}

/**
 * @brief  AHT10 read measurement.
 * @param	obj*			pointer to AHT10_Object
 * @param	measurement*	pointer to AHT10_Measurement where to store data to
 */
void AHT10_read_measurement(AHT10_Object *obj, AHT10_Measurement *measurement) {
	obj->read_reg(obj, 0, measurement->data, 6);
}

/**
 * @brief  Convert raw data to temperature (in deg. Celsius).
 * @param	measurement*	pointer to AHT10_Measurement where to store data to
 * @retval	float			temperature (in deg. Celsius)
 */
float AHT10_convert_temperature(AHT10_Measurement *measurement) {
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
float AHT10_convert_humidity(AHT10_Measurement *measurement) {
	float f = 0;

	uint32_t raw = ((uint32_t) measurement->data[1] << 12) | ((uint32_t) measurement->data[2] << 4) | (measurement->data[3] >> 4);
	f = (float) (raw * 100.0 / 1048576.0);

	return f;
}
