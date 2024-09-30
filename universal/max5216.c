#include "max5216.h"

void MAX5216_init(MAX5216_Handle *handle, device_write_pin_ptr write_cs, device_spi_write_ptr write_cmd) {
    handle->write_cs=write_cs;
    handle->write_cmd=write_cmd;
}

void MAX5216_set_dac_value(MAX5216_Handle *handle, uint16_t val) {
    uint8_t data[3];
    data[0]=(val>>10) & 0b00111111;
    data[0]|=MAX5216_CTRL_WRITE;
    
    data[1]=(val>>2) & 0xff;
    data[2]=(val<<6) & 0b11000000;
    
    handle->write_cs(true);
    handle->write_cmd(handle, data, 3);
    // wait 100ns
    handle->write_cs(false);
    
}

void MAX5216_power(MAX5216_Handle *handle, MAX5216_Power power) {
    uint8_t data[3];
    data[0]=MAX5216_CTRL_POWER | power;
    
    handle->write_cs(true);
    handle->write_cmd(handle, data, 3);
    // wait 100ns
    handle->write_cs(false);
}