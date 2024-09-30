#include "max1120x.h"

void MAX1120X_init(MAX1120X_Handle *handle, device_write_pin_ptr write_cs, device_spi_write_ptr write_cmd, device_read_pin_ptr read_data_pin, device_delay_ms_ptr delay_ms) {
    handle->write_cs=write_cs;
    handle->write_cmd=write_cmd;
    handle->read_data_pin=read_data_pin;
    handle->delay_ms=delay_ms;
}

void MAX1120X_send_command(MAX1120X_Handle *handle, MAX1120X_Command command) {
	uint8_t data[1];
    data[0]=command;
    
    handle->write_cs(true);
    handle->write_cmd(handle, data, 1);
    // wait 3ns
    handle->write_cs(false);
}

void MAX1120X_write_register_byte(MAX1120X_Handle *handle, MAX1120X_Register reg, uint8_t val) {
	uint8_t data[2];
    data[0]=reg | MAX1120X_REG_WRITE;
    data[1]=val;
    
    handle->write_cs(true);
    handle->write_cmd(handle, data, 2);
    // wait 3ns
    handle->write_cs(false);
}

void MAX1120X_write_register_3byte(MAX1120X_Handle *handle, MAX1120X_Register reg, uint32_t val) {
	uint8_t data[4];
    data[0]=reg | MAX1120X_REG_WRITE;
    data[1]=(val>>16) & 0xff;
    data[2]=(val>>8) & 0xff;
    data[3]=(val) & 0xff;
    
    handle->write_cs(true);
    handle->write_cmd(handle, data, 4);
    // wait 3ns
    handle->write_cs(false);
}

void MAX1120X_read_register_byte(MAX1120X_Handle *handle, MAX1120X_Register reg, uint8_t *val) {
	uint8_t data[2];
    data[0]=reg | MAX1120X_REG_READ;
    data[1]=0;
    
    handle->write_cs(true);
    handle->write_cmd(handle, data, 2);
    // wait 3ns
    handle->write_cs(false);
    
    *val=data[1];
}

void MAX1120X_read_register_3byte(MAX1120X_Handle *handle, MAX1120X_Register reg, uint32_t *val) {
	uint8_t data[4];
    data[0]=reg | MAX1120X_REG_READ;
    data[1]=0;
    data[2]=0;
    data[3]=0;
    
    handle->write_cs(true);
    handle->write_cmd(handle, data, 4);
    // wait 3ns
    handle->write_cs(false);
    
    *val=data[1];
    *val<<=8;
    *val=*val | data[2];
    *val<<=8;
    *val=*val | data[3];
    *val>>=4;
}

bool MAX1120X_wait_for_conversion_done(MAX1120X_Handle *handle) {
    // wait for data pin to be low
    uint16_t c=0;
    while (handle->read_data_pin()) {
        c++;
        if (c>65000) return false;
    }
    return true;
}

void MAX1120X_system_calibration(MAX1120X_Handle *handle, device_set_input_signal_ptr set_zero_signal, device_set_input_signal_ptr set_full_signal, device_set_input_signal_ptr set_normal_signal) {
    MAX1120X_Ctrl3 ctrl3;
    ctrl3.DATA=0;
    
    // self calibration SCOC and SCGC
    ctrl3.NOSYSG=1;
    ctrl3.NOSYSO=1;
    MAX1120X_write_register_byte(handle, MAX1120X_REGISTER_CTRL3, ctrl3.DATA);
    MAX1120X_send_command(handle, MAX1120X_CMD_SELF_CAL);
    // wait at least 200ms
    handle->delay_ms(250);
    
    // system calibration
    
    // set 0V - relay_ADC1 off, relay_ADC2 on
    set_zero_signal();
    
    ctrl3.DATA=0;
    ctrl3.NOSYSG=1;
    MAX1120X_write_register_byte(handle, MAX1120X_REGISTER_CTRL3, ctrl3.DATA);
    MAX1120X_send_command(handle, MAX1120X_CMD_SYSTEM_OFFSET_CAL);
    // wait at least 100ms
    handle->delay_ms(150);
    
    // set full scale 3V - relay_ADC1 off, relay_ADC2 off
	set_full_signal();
    
    ctrl3.DATA=0;
    MAX1120X_write_register_byte(handle, MAX1120X_REGISTER_CTRL3, ctrl3.DATA);
    MAX1120X_send_command(handle, MAX1120X_CMD_SYSTEM_GAIN_CAL);
    // wait at least 100ms
    handle->delay_ms(150);
    
    // switch to normal input signal
	set_normal_signal();
    
    
}

//void MAX1120X_startConversion() {
//	unsigned char c1;
//	while (MAX1120X_SPIRBE==0) c1=MAX1120X_SPIBUF;
//	
////	SPI1BUFL = 0b10000101; // 25/30sps
//	MAX1120X_SPIBUF = 0b10000111; // 100/120sps
//	while (!MAX1120X_SPIRBF);
//	c1 = MAX1120X_SPIBUF;
//	while (MAX1120X_SPIMISO==1) {} // wait for conversion finish
//}

