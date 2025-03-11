/**
  ******************************************************************************
  * @file    aht10.h
  * @author  Juraj Lonc (juraj.lonc@gmail.com)
  * @brief   Header file of AHT10 driver.
  ******************************************************************************
  */

#ifndef INC_AHT10_H_
#define INC_AHT10_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "device.h"


#define AHT10_I2C_ADDRESS_LOW			0b01110000
#define AHT10_I2C_ADDRESS_HIGH			0b01110010

#define AHT10_CMD_INIT					0b11100001
#define AHT10_CMD_TRIGGER_MEASUREMENT	0b10101100
#define AHT10_CMD_SOFT_RESET			0b10111010


typedef struct {
	device_write_ptr write_reg;
	device_read_ptr read_reg;
	device_delay_ms_ptr delay_ms;
	uint8_t addr;
} AHT10_Handle;


typedef struct {
	uint8_t data[6];
} AHT10_Measurement;


uint8_t AHT10_init(AHT10_Handle *handle, uint8_t addrpin, device_write_ptr write_reg, device_read_ptr read_reg, device_delay_ms_ptr delay_ms);
void AHT10_soft_reset(AHT10_Handle *handle);
void AHT10_trigger_measurement(AHT10_Handle *handle);
bool AHT10_read_measurement(AHT10_Handle *handle, AHT10_Measurement *measurement);

float AHT10_convert_temperature_float(AHT10_Measurement *measurement);
float AHT10_convert_humidity_float(AHT10_Measurement *measurement);

int16_t AHT10_convert_temperature(AHT10_Measurement *measurement);
uint16_t AHT10_convert_humidity(AHT10_Measurement *measurement);


#ifdef __cplusplus
}
#endif

#endif /* INC_AHT10_H_ */
