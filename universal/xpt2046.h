/**
  ******************************************************************************
  * @file    xpt2046.h
  * @author  Juraj Lonc (juraj.lonc@gmail.com)
  * @brief   Header file of XPT2046 driver.
  ******************************************************************************
  */

#ifndef __XPT2046_H
#define	__XPT2046_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "device.h"

/*
	1 - Always 1 (Command enable)
	2, 3, 4 - A2, A1, A0 - Input select pin
	5 - Mode, 12(L) or 8(H) bit mode select
	6 - SER(H)/DFR(L) | Single-Ended (HIGH) / Differential reference Mode (LOW)
	7 - PD1, Power down mode
	8 - PD0 (LSB)
	A2,	A1,	A0
	0	0	0	= TEMP-0
	0	0	1	= Y
	0	1	0	= VBat
	0	1	1	= Z1
	1	0	0	= Z2
	1	0	1	= X
	1	1	0	= AUX-IN
	1	1	1	= TEMP-1
	
	# Request: B1XXX0000 00000000
	# Answer:  B0XXXXXXX XXXXX000
	R-touch = Rx-plate * X-Pos/4096 * (Z2 / Z1 - 1)
	R-touch = ((X-res * X-pos) / 4096) * (4096 / Z1 - 1) - Y-res * (1- (Y-Pos / 4096))
	*/


#define XPT_START	   0b10000000
#define XPT_MODE_8     0b00001000
#define XPT_MODE_12    0b00000000
#define XPT_SER        0b00000100
#define XPT_DFR        0b00000000
#define XPT_PDOWN	   0b00000000
#define XPT_PUP        0b00000011

#define XPT_TEMP0	   0b00000000
#define XPT_TEMP1	   0b01110000
#define XPT_Z1		   0b00110000
#define XPT_Z2		   0b01000000
#define XPT_X		   0b01010000
#define XPT_Y		   0b00010000

#define XPT_Z_THRESHOLD 3500

typedef struct {
	device_write_pin_ptr write_cs;
    device_spi_write_ptr write_cmd;
} XPT2046_Handle;

void xpt2046_init(XPT2046_Handle * handle, device_write_pin_ptr write_cs, device_spi_write_ptr write_cmd);

// returns 12bits value
uint16_t xpt2046_read_value(XPT2046_Handle * handle, uint8_t reg);
bool xpt2046_is_touched(XPT2046_Handle * handle);
uint16_t xpt2046_touched_x(XPT2046_Handle * handle);
uint16_t xpt2046_touched_y(XPT2046_Handle * handle);
int16_t xpt2046_pressure(XPT2046_Handle * handle);

// hundreths of deg C (2501 = 25.01C)
int16_t xpt2046_get_temperature(XPT2046_Handle * handle);

#ifdef __cplusplus
}
#endif

#endif	/* __XPT2046_H */