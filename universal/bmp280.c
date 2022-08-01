/**
  ******************************************************************************
  * @file    bmp280.c
  * @author  Juraj Lonc (juraj.lonc@gmail.com)
  * @brief   BMP280 driver.
  ******************************************************************************
  */

#include "bmp280.h"

bool BMP280_init(BMP280_Handle *handle, uint8_t cspin, device_write_ptr write_reg, device_read_ptr read_reg, device_delay_ms_ptr delay_ms) {
	handle->write_reg=write_reg;
	handle->read_reg=read_reg;
	handle->delay_ms=delay_ms;

	handle->addr=cspin==0?BMP280_I2C_ADDRESS_LOW:BMP280_I2C_ADDRESS_HIGH;

	if (!BMP280_verify_id(handle)) return false;

	BMP280_load_compensation_values(handle);
	BMP280_set_config(handle, BMP280_STANDBYTIME_0M5, BMP280_FILTER_COEFF_OFF);
	BMP280_set_control_measurement(handle, BMP280_OVERSAMPLING_16X, BMP280_OVERSAMPLING_16X, BMP280_MODE_NORMAL);

	return true;
}

bool BMP280_verify_id(BMP280_Handle *handle) {
	handle->read_reg(handle, BMP280_REG_ID, &(handle->sensor_id), 1);
	return (handle->sensor_id==BMP280_CHIP_ID1) || (handle->sensor_id==BMP280_CHIP_ID2) || (handle->sensor_id==BMP280_CHIP_ID3) || (handle->sensor_id==BME280_CHIP_ID) || (handle->sensor_id==BME680_CHIP_ID);
}

void BMP280_read_raw_temperature(BMP280_Handle *handle) {
	uint8_t data[3];
	handle->read_reg(handle, BMP280_REG_TEMP_MSB, data, 3);

	handle->raw_temperature=(data[0]<<12) | (data[1]<<4) | ((data[2] >> 4) & 0x0f);
}
int32_t BMP280_calc_temperature(BMP280_Handle *handle) {
	int32_t var1, var2, T;
	var1 = (((((handle->raw_temperature)>>3) - ((int32_t)(handle->dig_T1)<<1))) * ((int32_t)handle->dig_T2)) >> 11;
	var2 = ((((((handle->raw_temperature)>>4) - ((int32_t)handle->dig_T1)) * (((handle->raw_temperature)>>4) - ((int32_t)handle->dig_T1)))>> 12) *((int32_t)handle->dig_T3)) >> 14;
	handle->t_fine=var1 + var2;

	T = ((handle->t_fine) * 5 + 128) >> 8;

	return T;
}

void BMP280_read_raw_pressure(BMP280_Handle *handle) {
	uint8_t data[3];
	handle->read_reg(handle, BMP280_REG_PRESS_MSB, data, 3);

	handle->raw_pressure=(((int32_t)data[0])<<12) | (((int32_t)data[1])<<4) | ((data[2] >> 4) & 0x0f);
}
//uint32_t BMP280_calc_pressure0(BMP280_Handle *handle) {
//	int64_t var1, var2, p;
//
//	var1 = ((int64_t)handle->t_fine) - 128000;
//	var2 = var1 * var1 * ((int64_t)handle->dig_P6);
//	var2 = var2 + ((var1* ((int64_t)(handle->dig_P5)))<<17);
//	var2 = var2 + (((int64_t)handle->dig_P4)<<35);
//	var1 = ((var1 * var1 * ((int64_t)handle->dig_P3))>>8) + ((var1 * ((int64_t)handle->dig_P2))<<12);
//	var1 = (((((int64_t)1)<<47)+var1))*(handle->dig_P1)>>33;
//	if (var1 == 0) {
//		return (uint32_t)0; // avoid exception caused by division by zero
//	}
//	p = 1048576-(handle->raw_pressure);
//	p = (((p<<31)-var2)*3125)/var1;
//	var1 = (((int64_t)handle->dig_P9) * (p>>13) * (p>>13)) >> 25;
//	var2 = (((int64_t)(handle->dig_P8)) * p) >> 19;
//	p = ((p + var1 + var2) >> 8) + (((int64_t)handle->dig_P7)<<4);
//	p=p>>8;
//	return (uint64_t)p;
//}


uint32_t BMP280_calc_pressure(BMP280_Handle *handle) {
	int32_t var1, var2;
	uint32_t p;
	var1 = (((int32_t)handle->t_fine)>>1) - (int32_t)64000;
	var2 = (((var1>>2) * (var1>>2)) >> 11 ) * ((int32_t)handle->dig_P6);
	var2 = var2 + ((var1*((int32_t)handle->dig_P5))<<1);
	var2 = (var2>>2)+(((int32_t)handle->dig_P4)<<16);
	var1 = (((handle->dig_P3 * (((var1>>2) * (var1>>2)) >> 13 )) >> 3) + ((((int32_t)handle->dig_P2) * var1)>>1))>>18;
	var1 =((((32768+var1))*((int32_t)handle->dig_P1))>>15);
	if (var1 == 0)
	{
		return 0; // avoid exception caused by division by zero
	}
	p = (((uint32_t)(((int32_t)1048576)-handle->raw_pressure)-(var2>>12)))*3125;
	if (p < 0x80000000)
	{
		p = (p << 1) / ((uint32_t)var1);
	}
	else
	{
		p = (p / (uint32_t)var1) * 2;
	}
	var1 = (((int32_t)handle->dig_P9) * ((int32_t)(((p>>3) * (p>>3))>>13)))>>12;
	var2 = (((int32_t)(p>>2)) * ((int32_t)handle->dig_P8))>>13;
	p = (uint32_t)((int32_t)p + ((var1 + var2 + handle->dig_P7) >> 4));
	return p;
}

void BMP280_load_compensation_values(BMP280_Handle *handle) {
	uint8_t data[6];

	// temperature coefficients
	handle->read_reg(handle, BMP280_REG_DIG_T1, data, 6);
	handle->dig_T1=(data[0] | (data[1] << 8)); // unsigned
	handle->dig_T2=(data[2] | (data[3] << 8)); // signed
	handle->dig_T3=(data[4] | (data[5] << 8)); // signed

	// pressure coefficients
	handle->read_reg(handle, BMP280_REG_DIG_P1, data, 6);
	handle->dig_P1=(data[0] | (data[1] << 8)); // unsigned
	handle->dig_P2=(data[2] | (data[3] << 8)); // signed
	handle->dig_P3=(data[4] | (data[5] << 8)); // signed
	handle->read_reg(handle, BMP280_REG_DIG_P4, data, 6);
	handle->dig_P4=(data[0] | (data[1] << 8)); // signed
	handle->dig_P5=(data[2] | (data[3] << 8)); // signed
	handle->dig_P6=(data[4] | (data[5] << 8)); // signed
	handle->read_reg(handle, BMP280_REG_DIG_P7, data, 6);
	handle->dig_P7=(data[0] | (data[1] << 8)); // signed
	handle->dig_P8=(data[2] | (data[3] << 8)); // signed
	handle->dig_P9=(data[4] | (data[5] << 8)); // signed
//	handle->dig_P9=6000;
}

//void BMP280_load_compensation_values(BMP280_Handle *handle) {
//	uint8_t data[24];
//
////	handle->read_reg(handle, BMP280_REG_DIG_T1, data, 24);
//	handle->read_reg(handle, BMP280_REG_DIG_T1, data, 22); // freezing, when loading all 24 bytes
//
//	// temperature coefficients
//	handle->dig_T1=(data[0] | (data[1] << 8)); // unsigned
//	handle->dig_T2=(data[2] | (data[3] << 8)); // signed
//	handle->dig_T3=(data[4] | (data[5] << 8)); // signed
//
//	// pressure coefficients
//	handle->dig_P1=(data[6] | (data[7] << 8)); // unsigned
//	handle->dig_P2=(data[8] | (data[9] << 8)); // signed
//	handle->dig_P3=(data[10] | (data[11] << 8)); // signed
//	handle->dig_P4=(data[12] | (data[13] << 8)); // signed
//	handle->dig_P5=(data[14] | (data[15] << 8)); // signed
//	handle->dig_P6=(data[16] | (data[17] << 8)); // signed
//	handle->dig_P7=(data[18] | (data[19] << 8)); // signed, default 15500
//	handle->dig_P8=(data[20] | (data[21] << 8)); // signed, default -14600
////	handle->dig_P8=-14600;
////	handle->dig_P9=(data[22] | (data[23] << 8)); // signed, default 6000
//	handle->dig_P9=6000;
//
//}

BMP280_Register_Status BMP280_get_status(BMP280_Handle *handle) {
	BMP280_Register_Status status;
	handle->read_reg(handle, BMP280_REG_STATUS, &status.DATA, 1);
	return status;
}
void BMP280_reset(BMP280_Handle *handle) {
	uint8_t data=BMP280_SOFT_RESET;
	handle->write_reg(handle, BMP280_REG_RESET, &data, 1);
}

uint16_t BMP280_get_altitude(BMP280_Handle *handle, float sea_level_hpa) {

	BMP280_read_raw_temperature(handle);
	BMP280_calc_temperature(handle);

	BMP280_read_raw_pressure(handle);
	uint32_t pressure=BMP280_calc_pressure(handle);
	float a=pressure/100.0;

	uint16_t altitude = 44330 * (1.0 - pow(a / sea_level_hpa, 0.1903));

	return altitude;
}
void BMP280_set_power_mode(BMP280_Handle *handle, uint8_t power_mode) {
	BMP280_Register_ControlMeas ctrl;
	handle->read_reg(handle, BMP280_REG_CTRL_MEAS, &ctrl.DATA, 1);
	ctrl.mode=power_mode;
	handle->write_reg(handle, BMP280_REG_CTRL_MEAS, &ctrl.DATA, 1);
}
void BMP280_set_oversamp_temperature(BMP280_Handle *handle, uint8_t oversample_temp) {
	BMP280_Register_ControlMeas ctrl;
	handle->read_reg(handle, BMP280_REG_CTRL_MEAS, &ctrl.DATA, 1);
	ctrl.osrs_t=oversample_temp;
	handle->write_reg(handle, BMP280_REG_CTRL_MEAS, &ctrl.DATA, 1);
}
void BMP280_set_oversamp_pressure(BMP280_Handle *handle, uint8_t oversample_pressure) {
	BMP280_Register_ControlMeas ctrl;
	handle->read_reg(handle, BMP280_REG_CTRL_MEAS, &ctrl.DATA, 1);
	ctrl.osrs_p=oversample_pressure;
	handle->write_reg(handle, BMP280_REG_CTRL_MEAS, &ctrl.DATA, 1);
}
void BMP280_set_control_measurement(BMP280_Handle *handle, uint8_t oversample_temp, uint8_t oversample_pressure, uint8_t power_mode) {
	BMP280_Register_ControlMeas ctrl;
	ctrl.osrs_t=oversample_temp;
	ctrl.osrs_p=oversample_pressure;
	ctrl.mode=power_mode;
	handle->write_reg(handle, BMP280_REG_CTRL_MEAS, &ctrl.DATA, 1);
}
void BMP280_set_filter(BMP280_Handle *handle, uint8_t iir_filter) {
	BMP280_Register_Config config;
	handle->read_reg(handle, BMP280_REG_CONFIG, &config.DATA, 1);
	config.filter=iir_filter;
	handle->write_reg(handle, BMP280_REG_CONFIG, &config.DATA, 1);
}
void BMP280_set_standby_time(BMP280_Handle *handle, uint8_t standby_time) {
	BMP280_Register_Config config;
	handle->read_reg(handle, BMP280_REG_CONFIG, &config.DATA, 1);
	config.t_sb=standby_time;
	handle->write_reg(handle, BMP280_REG_CONFIG, &config.DATA, 1);
}
void BMP280_set_config(BMP280_Handle *handle, uint8_t standby_time, uint8_t iir_filter) {
	BMP280_Register_Config config;
	config.filter=iir_filter;
	config.t_sb=standby_time;
	handle->write_reg(handle, BMP280_REG_CONFIG, &config.DATA, 1);
}
void BMP280_set_work_mode(BMP280_Handle *handle, uint8_t work_mode) {
	BMP280_Register_ControlMeas ctrl;
	handle->read_reg(handle, BMP280_REG_CTRL_MEAS, &ctrl.DATA, 1);
	switch (work_mode) {
	/* write work mode*/
	case BMP280_ULTRA_LOW_POWER_MODE:
		ctrl.osrs_t = BMP280_ULTRALOWPOWER_OVERSAMP_TEMPERATURE;
		ctrl.osrs_p = BMP280_ULTRALOWPOWER_OVERSAMP_PRESSURE;
		break;
	case BMP280_LOW_POWER_MODE:
		ctrl.osrs_t = BMP280_LOWPOWER_OVERSAMP_TEMPERATURE;
		ctrl.osrs_p = BMP280_LOWPOWER_OVERSAMP_PRESSURE;
		break;
	case BMP280_STANDARD_RESOLUTION_MODE:
		ctrl.osrs_t = BMP280_STANDARDRESOLUTION_OVERSAMP_TEMPERATURE;
		ctrl.osrs_p = BMP280_STANDARDRESOLUTION_OVERSAMP_PRESSURE;
		break;
	case BMP280_HIGH_RESOLUTION_MODE:
		ctrl.osrs_t = BMP280_HIGHRESOLUTION_OVERSAMP_TEMPERATURE;
		ctrl.osrs_p = BMP280_HIGHRESOLUTION_OVERSAMP_PRESSURE;
		break;
	case BMP280_ULTRA_HIGH_RESOLUTION_MODE:
		ctrl.osrs_t = BMP280_ULTRAHIGHRESOLUTION_OVERSAMP_TEMPERATURE;
		ctrl.osrs_p = BMP280_ULTRAHIGHRESOLUTION_OVERSAMP_PRESSURE;
		break;
	}
	handle->write_reg(handle, BMP280_REG_CTRL_MEAS, &ctrl.DATA, 1);
}
