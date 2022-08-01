/**
  ******************************************************************************
  * @file    bmp280.h
  * @author  Juraj Lonc (juraj.lonc@gmail.com)
  * @brief   Header file of BMP280 driver.
  ******************************************************************************
  */

#ifndef INC_BMP280_H_
#define INC_BMP280_H_

#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "device.h"

#define BMP280_I2C_ADDRESS_LOW			0b11101100	// SDO pulled low
#define BMP280_I2C_ADDRESS_HIGH			0b11101110	// SDO pulled high

#define BMP280_REG_ID					0xD0
#define BMP280_REG_RESET				0xE0
#define BMP280_REG_STATUS				0xF3
#define BMP280_REG_CTRL_MEAS			0xF4
#define BMP280_REG_CONFIG				0xF5
#define BMP280_REG_PRESS_MSB			0xF7
#define BMP280_REG_PRESS_LSB			0xF8
#define BMP280_REG_PRESS_XLSB			0xF9
#define BMP280_REG_TEMP_MSB				0xFA
#define BMP280_REG_TEMP_LSB				0xFB
#define BMP280_REG_TEMP_XLSB			0xFC

#define BMP280_REG_DIG_T1				0x88
#define BMP280_REG_DIG_T2				0x8A
#define BMP280_REG_DIG_T3				0x8C
#define BMP280_REG_DIG_P1				0x8E
#define BMP280_REG_DIG_P2				0x90
#define BMP280_REG_DIG_P3				0x92
#define BMP280_REG_DIG_P4				0x94
#define BMP280_REG_DIG_P5				0x96
#define BMP280_REG_DIG_P6				0x98
#define BMP280_REG_DIG_P7				0x9A
#define BMP280_REG_DIG_P8				0x9C
#define BMP280_REG_DIG_P9				0x9E

#define BMP280_CHIP_ID1					0x56
#define BMP280_CHIP_ID2					0x57
#define BMP280_CHIP_ID3					0x58
#define BMP280_SOFT_RESET				0xB6

#define BMP280_MODE_SLEEP				0b00
#define BMP280_MODE_FORCED				0b01	// same as 0b10
#define BMP280_MODE_NORMAL				0b11

#define BMP280_STANDBYTIME_0M5			0b000
#define BMP280_STANDBYTIME_62M5			0b001
#define BMP280_STANDBYTIME_125M			0b010
#define BMP280_STANDBYTIME_250M			0b011
#define BMP280_STANDBYTIME_500M			0b100
#define BMP280_STANDBYTIME_1000M		0b101
#define BMP280_STANDBYTIME_2000M		0b110
#define BMP280_STANDBYTIME_4000M		0b111

#define BMP280_OVERSAMPLING_SKIPPED		0b000
#define BMP280_OVERSAMPLING_1X			0b001
#define BMP280_OVERSAMPLING_2X			0b010
#define BMP280_OVERSAMPLING_4X			0b011
#define BMP280_OVERSAMPLING_8X			0b100
#define BMP280_OVERSAMPLING_16X			0b101

#define BMP280_FILTER_COEFF_OFF			0b000
#define BMP280_FILTER_COEFF_2			0b001
#define BMP280_FILTER_COEFF_4			0b010
#define BMP280_FILTER_COEFF_8			0b011
#define BMP280_FILTER_COEFF_16			0b100

#define BMP280_ULTRA_LOW_POWER_MODE			0x00
#define BMP280_LOW_POWER_MODE				0x01
#define BMP280_STANDARD_RESOLUTION_MODE		0x02
#define BMP280_HIGH_RESOLUTION_MODE			0x03
#define BMP280_ULTRA_HIGH_RESOLUTION_MODE	0x04

#define BMP280_ULTRALOWPOWER_OVERSAMP_PRESSURE			BMP280_OVERSAMPLING_1X
#define BMP280_ULTRALOWPOWER_OVERSAMP_TEMPERATURE		BMP280_OVERSAMPLING_1X
#define BMP280_LOWPOWER_OVERSAMP_PRESSURE				BMP280_OVERSAMPLING_2X
#define BMP280_LOWPOWER_OVERSAMP_TEMPERATURE			BMP280_OVERSAMPLING_1X
#define BMP280_STANDARDRESOLUTION_OVERSAMP_PRESSURE		BMP280_OVERSAMPLING_4X
#define BMP280_STANDARDRESOLUTION_OVERSAMP_TEMPERATURE	BMP280_OVERSAMPLING_1X
#define BMP280_HIGHRESOLUTION_OVERSAMP_PRESSURE			BMP280_OVERSAMPLING_8X
#define BMP280_HIGHRESOLUTION_OVERSAMP_TEMPERATURE		BMP280_OVERSAMPLING_1X
#define BMP280_ULTRAHIGHRESOLUTION_OVERSAMP_PRESSURE	BMP280_OVERSAMPLING_16X
#define BMP280_ULTRAHIGHRESOLUTION_OVERSAMP_TEMPERATURE	BMP280_OVERSAMPLING_2X

// BME280
#define BME280_CHIP_ID					0x60

// BME680
#define BME680_CHIP_ID					0x61


typedef struct {
	device_write_ptr write_reg;
	device_read_ptr read_reg;
	device_delay_ms_ptr delay_ms;
	uint8_t addr;

	uint8_t sensor_id;
	int32_t t_fine;
	int32_t p_fine;
	int32_t raw_temperature;
	int32_t raw_pressure;

	uint16_t dig_T1;
	int16_t dig_T2;
	int16_t dig_T3;

	uint16_t dig_P1;
	int16_t dig_P2;
	int16_t dig_P3;
	int16_t dig_P4;
	int16_t dig_P5;
	int16_t dig_P6;
	int16_t dig_P7;
	int16_t dig_P8;
	int16_t dig_P9;


} BMP280_Handle;

// 0xF3; status register
typedef struct tag_BMP280_Register_Status {
	union {
		struct {
			uint8_t DATA;
		};
		struct {
			uint8_t im_update :1;	// bit 0; Automatically set to ‘1’ whenever a conversion is running and back to ‘0’ when the results have been transferred to the data registers.
			uint8_t :2;				// bits 1-2;
			uint8_t measuring :1;	// bits 3; Controls oversampling of temperature data. See chapter 3.3.2 for details.
		};
	};
} BMP280_Register_Status;

// 0xF4; control measurement register
typedef struct tag_BMP280_Register_ControlMeas {
	union {
		struct {
			uint8_t DATA;
		};
		struct {
			uint8_t mode :2;	// bits 0-1; Controls the power mode of the device. See chapter 3.6 for details.
			uint8_t osrs_p :3;	// bits 2-4; Controls oversampling of pressure data. See chapter 3.3.1 for details.
			uint8_t osrs_t :3;	// bits 5-7; Controls oversampling of temperature data. See chapter 3.3.2 for details.
		};
	};
} BMP280_Register_ControlMeas;

// 0xF5; configuration register
typedef struct tag_BMP280_Register_Config {
	union {
		struct {
			uint8_t DATA;
		};
		struct {
			uint8_t spi3w_en :1;	// bit 0; Enables 3-wire SPI interface when set to ‘1’. See chapter 5.3 for details.
			uint8_t :1;			// bit 1;
			uint8_t filter :3;		// bits 2-4; Controls the time constant of the IIR filter. See chapter 3.3.3 for details.
			uint8_t t_sb :3;		// bits 5-7; Controls inactive duration t_standby in normal mode. See chapter 3.6.3 for details.
		};
	};
} BMP280_Register_Config;


/**
 * @brief  BMP280 initialization.
 * @param	handle*		pointer to BMP280_Handle
 * @param	cspin		cspin, address selector. Can be 0 or 1.
 * @param	write_reg	The function that writes command to the module. Hardware dependent.
 * @param	read_reg	The function that reads data from the module. Hardware dependent.
 * @param	delay_ms	The function that makes delay in milliseconds. Hardware dependent.
 * @retval	retval		true = device found and ID is OK
 */
bool BMP280_init(BMP280_Handle *handle, uint8_t cspin, device_write_ptr write_reg, device_read_ptr read_reg, device_delay_ms_ptr delay_ms);

/**
 * @brief  Verify ID of device.
 * @param	handle*		pointer to BMP280_Handle
 * @retval	retval		true = device ID is OK
 */
bool BMP280_verify_id(BMP280_Handle *handle);

/**
 * @brief  Soft reset of device.
 * @param	handle*		pointer to BMP280_Handle
 */
void BMP280_reset(BMP280_Handle *handle);

/**
 * @brief  Get status of device.
 * @param	handle*		pointer to BMP280_Handle
 * @retval	retval		value of status register
 */
BMP280_Register_Status BMP280_get_status(BMP280_Handle *handle);

/**
 * @brief  Load compensation/calibration values and store it into the handle.
 * @param	handle*		pointer to BMP280_Handle
 */
void BMP280_load_compensation_values(BMP280_Handle *handle);

/**
 * @brief  Read raw temperature and store it into the handle.
 * @param	handle*		pointer to BMP280_Handle
 */
void BMP280_read_raw_temperature(BMP280_Handle *handle);

/**
 * @brief  Calculate temperature from stored raw temperature.
 * @param	handle*		pointer to BMP280_Handle
 * @retval	retval		temperature in hundreths of C (value 2546 = 25.46 C)
 */
int32_t BMP280_calc_temperature(BMP280_Handle *handle);

/**
 * @brief  Read raw pressure and store it into the handle.
 * @param	handle*		pointer to BMP280_Handle
 */
void BMP280_read_raw_pressure(BMP280_Handle *handle);

/**
 * @brief  Calculate pressure from stored raw pressure.
 * @param	handle*		pointer to BMP280_Handle
 * @retval	retval		pressure in Pascals
 */
uint32_t BMP280_calc_pressure(BMP280_Handle *handle);

/**
 * @brief  Set power mode.
 * @param	handle*		pointer to BMP280_Handle
 * @param	power_mode	power mode (sleep, forced, normal)
 */
void BMP280_set_power_mode(BMP280_Handle *handle, uint8_t power_mode);

/**
 * @brief  Set oversampling for temperature measurement.
 * @param	handle*				pointer to BMP280_Handle
 * @param	oversample_temp		oversampling (off, 1x to 16x)
 */
void BMP280_set_oversamp_temperature(BMP280_Handle *handle, uint8_t oversample_temp);

/**
 * @brief  Set oversampling for pressure measurement.
 * @param	handle*					pointer to BMP280_Handle
 * @param	oversample_pressure		oversampling (off, 1x to 16x)
 */
void BMP280_set_oversamp_pressure(BMP280_Handle *handle, uint8_t oversample_pressure);

/**
 * @brief  Set control measurement.
 * @param	handle*					pointer to BMP280_Handle
 * @param	oversample_temp			oversampling (off, 1x to 16x)
 * @param	oversample_pressure		oversampling (off, 1x to 16x)
 * @param	power_mode				power mode (sleep, forced, normal)
 */
void BMP280_set_control_measurement(BMP280_Handle *handle, uint8_t oversample_temp, uint8_t oversample_pressure, uint8_t power_mode);

/**
 * @brief  Set filter.
 * @param	handle*			pointer to BMP280_Handle
 * @param	iir_filter		filter coefficient (off, 2 to 16)
 */
void BMP280_set_filter(BMP280_Handle *handle, uint8_t iir_filter);

/**
 * @brief  Set standby time.
 * @param	handle*			pointer to BMP280_Handle
 * @param	standby_time	standby time between measurements (0.5ms to 4000ms)
 */
void BMP280_set_standby_time(BMP280_Handle *handle, uint8_t standby_time);

/**
 * @brief  Set config.
 * @param	handle*			pointer to BMP280_Handle
 * @param	standby_time	standby time between measurements (0.5ms to 4000ms)
 * @param	iir_filter		filter coefficient (off, 2 to 16)
 */
void BMP280_set_config(BMP280_Handle *handle, uint8_t standby_time, uint8_t iir_filter);

/**
 * @brief  Set work mode.
 * @param	handle*		pointer to BMP280_Handle
 * @param	work_mode	from ultra low power to ultra high resolution
 */
void BMP280_set_work_mode(BMP280_Handle *handle, uint8_t work_mode);

/**
 * @brief  Calculate altitude from sea level pressure.
 * @param	handle*			pointer to BMP280_Handle
 * @param	sea_level_hpa	sea level pressure in hPa (get from weather forecast)
 * @retval	retval			altitude in meters
 */
uint16_t BMP280_get_altitude(BMP280_Handle *handle, float sea_level_hpa);

#endif /* INC_BMP280_H_ */
