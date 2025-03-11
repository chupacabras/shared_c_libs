/**
  ******************************************************************************
  * @file    max5216.h
  * @author  Juraj Lonc (juraj.lonc@gmail.com)
  * @brief   Header file for MAX5216 DAC driver.
  ******************************************************************************
  */

#ifndef INC_MAX5216_H_
#define INC_MAX5216_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "device.h"

typedef enum {
	MAX5216_POWER_UP=0b0000,
    MAX5216_POWER_DOWN_HIGH_Z=0b0100,
    MAX5216_POWER_DOWN_100K=0b1000,
    MAX5216_POWER_DOWN_1K=0b1100,
} MAX5216_Power;

#define MAX5216_CTRL_NO_OP  0b00000000
#define MAX5216_CTRL_POWER  0b10000000
#define MAX5216_CTRL_WRITE  0b01000000

typedef struct {
    device_write_pin_ptr write_cs;
	device_spi_write_ptr write_cmd;
} MAX5216_Handle;

void MAX5216_init(MAX5216_Handle *handle, device_write_pin_ptr write_cs, device_spi_write_ptr write_cmd);
void MAX5216_set_dac_value(MAX5216_Handle *handle, uint16_t val);
void MAX5216_power(MAX5216_Handle *handle, MAX5216_Power power);

#ifdef __cplusplus
}
#endif

#endif