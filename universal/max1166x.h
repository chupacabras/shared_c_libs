/**
  ******************************************************************************
  * @file    max1166x.h
  * @author  Juraj Lonc (juraj.lonc@gmail.com)
  * @brief   Header file for MAX11661-MAX11666 ADC driver.
  ******************************************************************************
  */

#ifndef INC_MAX1166X_H_
#define INC_MAX1166X_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "device.h"

typedef enum {
    MAX11661,
    MAX11662,
    MAX11663,
    MAX11664,
    MAX11665,
    MAX11666
} MAX1166X_Type;

typedef enum {
    MAX1166X_CHANNEL1=0,
    MAX1166X_CHANNEL2=1
} MAX1166X_Channel;


typedef struct {
    MAX1166X_Type type;
    uint8_t bits;
    uint8_t channels;
    MAX1166X_Channel current_channel;
    
    device_write_pin_ptr write_cs;
    device_spi_write_ptr write_cmd;
    device_write_pin_ptr write_chsel;
} MAX1166X_Handle;

void MAX1166X_init(MAX1166X_Handle *handle, MAX1166X_Type type, device_write_pin_ptr write_cs, device_spi_write_ptr write_cmd, device_write_pin_ptr write_chsel);
void MAX1166X_read_next_channel(MAX1166X_Handle *handle, uint16_t *val);
void MAX1166X_power_down(MAX1166X_Handle *handle);
void MAX1166X_wake_up(MAX1166X_Handle *handle);

#ifdef __cplusplus
}
#endif

#endif