/**
 ******************************************************************************
 * @file    xpt2046.c
 * @author  Juraj Lonc (juraj.lonc@gmail.com)
 * @brief   XPT2046 driver.
 ******************************************************************************
 */

#include "xpt2046.h"

void xpt2046_init(XPT2046_Handle * handle, device_write_pin_ptr write_cs, device_spi_write_ptr write_cmd) {
	handle->write_cs=write_cs;
    handle->write_cmd=write_cmd;
}

// 12bits value
uint16_t xpt2046_read_value(XPT2046_Handle * handle, uint8_t reg) {
    uint8_t buf[3];
    
    buf[0]=reg;
    buf[1]=0;
    buf[2]=0;
    
    handle->write_cs(true);
    handle->write_cmd(handle, buf, 3);
	handle->write_cs(false);
    
    uint16_t value;
    
    value=buf[1];
    
	value = (value << 5) | ((buf[2] >> 3) & 0b11111);
    
    return value;
}

bool xpt2046_is_touched(XPT2046_Handle * handle) {
	if (xpt2046_pressure(handle) < XPT_Z_THRESHOLD) {
		return true;
	}
	
	return false;
}

uint16_t xpt2046_touched_x(XPT2046_Handle * handle) {
	return xpt2046_read_value(handle, XPT_START | XPT_X | XPT_MODE_12 | XPT_DFR | XPT_PUP);
}

uint16_t xpt2046_touched_y(XPT2046_Handle * handle) {
	return xpt2046_read_value(handle, XPT_START | XPT_Y | XPT_MODE_12 | XPT_DFR | XPT_PUP);
}

int16_t xpt2046_pressure(XPT2046_Handle * handle) {
	unsigned int z1=xpt2046_read_value(handle, XPT_START | XPT_Z1 | XPT_MODE_12 | XPT_DFR | XPT_PUP);
	unsigned int z2=xpt2046_read_value(handle, XPT_START | XPT_Z2 | XPT_MODE_12 | XPT_DFR | XPT_PUP);
	
	return (z2 - z1);
}

// hundreths of deg C (2501 = 25.01C)
int16_t xpt2046_get_temperature(XPT2046_Handle * handle) {
	uint16_t v0=xpt2046_read_value(handle, XPT_START | XPT_TEMP0 | XPT_MODE_12 | XPT_SER | XPT_PUP);
	uint16_t v1=xpt2046_read_value(handle, XPT_START | XPT_TEMP1 | XPT_MODE_12 | XPT_SER | XPT_PUP);
	
	int32_t tmp=v1-v0;
	
    // temperature = 1.5704345*(delta V)-273.15; [C]
    // temperature = 157.04345*(delta V)-27315; [0.01 C]
    // temperature = 160812.4928*(delta V)/1024-27315; [0.01 C]
    // temperature = 321625*(delta V)/2048-27315; [0.01 C]
    
	tmp=321625*tmp;
    tmp=tmp >> 11;
    tmp-=27315;
	
	return tmp;
}
