/**
 ******************************************************************************
 * @file    cs1237.c
 * @author  Juraj Lonc (juraj.lonc@gmail.com)
 * @brief   CS1237 ADC driver.
 ******************************************************************************
 */

#include "cs1237.h"

/**
 * @brief  Initialize CS1237 ADC and CS1237 object/handler.
 * @param  handle:       Pointer to a CS1237_Handle structure that contains
 *                    the information for the display.
 * @param  write_cmd:		The function that writes data to CS1237. Hardware dependent.
 * @param  read_cmd:		The function that reads data from CS1237. Hardware dependent.
 */
void CS1237_init(CS1237_Handle * handle, cs1237_device_pin set_pin_input, cs1237_device_pin set_pin_output, cs1237_device_pin set_clk_low, cs1237_device_pin set_clk_high, cs1237_device_pin set_dat_low, cs1237_device_pin set_dat_high, cs1237_device_pin_read read_dat, device_delay_us_ptr delay_us) {
	handle->set_pin_input = set_pin_input;
	handle->set_pin_output = set_pin_output;
	handle->set_clk_low = set_clk_low;
	handle->set_clk_high = set_clk_high;
	handle->set_dat_low = set_dat_low;
	handle->set_dat_high = set_dat_high;
	handle->read_dat = read_dat;
	handle->delay_us = delay_us;
	
	handle->set_pin_input();
	
	// reset ADC (put to sleep and then wake up)
	CS1237_power_down(handle);
	handle->delay_us(140);	// wait at least 100us to power down
	CS1237_wake_up(handle);
	
	
}

void CS1237_power_down(CS1237_Handle * handle) {
	handle->set_clk_high();
	// it takes at least 100us to power down
}

void CS1237_wake_up(CS1237_Handle * handle) {
	handle->set_clk_low();
	// it takes at least 10us to wake up
	handle->delay_us(12);
}

bool CS1237_data_ready(CS1237_Handle * handle) {
	return handle->read_dat()==0;
}

bool CS1237_read_bit(CS1237_Handle * handle) {
	bool data;
	
	handle->set_clk_high();
//	handle->delay_us(1);		// wait at least 455ns to read data
	
	data=handle->read_dat();
	
	handle->set_clk_low();
//	handle->delay_us(1);
	
	return data;
}

void CS1237_write_bit(CS1237_Handle * handle, bool high) {
	if (high) handle->set_dat_high();
	else handle->set_dat_low();
	
	handle->set_clk_high();
//	handle->delay_us(1);		// wait at least 455ns to read data
	
	handle->set_clk_low();
//	handle->delay_us(1);
	
}

bool CS1237_adc_read(CS1237_Handle * handle) {
	handle->value=0;
	
	for (uint8_t q=0; q<24; q++) {
		handle->value= (handle->value <<1) | CS1237_read_bit(handle);
	}
	
	// convert 24bit 2's complement value
	handle->value = (handle->value ^ 0x800000) - 0x800000;
	
	return true;
}

bool CS1237_poll(CS1237_Handle * handle) {
	CS1237_wake_up(handle);
	if (!CS1237_data_ready(handle)) return false;
	
	if (!CS1237_adc_read(handle)) return false;
	
	// make sure data line is high
	CS1237_read_bit(handle);
	
	return true;
}

bool CS1237_rw_config(CS1237_Handle * handle, bool write) {
	if (!CS1237_adc_read(handle)) return false;
	
	// bits: 25 (update1), 26 (update2), 27-29 (device switched to input)
	for (uint8_t q=0; q<5; q++) {
		CS1237_read_bit(handle);
	}
	
	handle->set_pin_output();
	
	// bits: 30-36 (command: read or write)
	uint8_t cmd=write?CS1237_CMD_WRITE_CONFIG:CS1237_CMD_READ_CONFIG;
	for (uint8_t q=0; q<7; q++) {
//		CS1237_write_bit(handle, (cmd & 0b00000001));
//		cmd=cmd>>1;
		
		CS1237_write_bit(handle, (cmd & 0b01000000));
		cmd<<=1;
	}
	
	if (write) {
		// bit: 37
		CS1237_read_bit(handle);

		// bits: 38-45 (config data)
		uint8_t config=handle->config.DATA;

		for (uint8_t q=0; q<8; q++) {
			CS1237_write_bit(handle, config & 0b10000000);
			config<<=1;
		}
		
		handle->set_pin_input();
		
	} else {
		handle->set_pin_input();
	
		// bit: 37 (device switched to output)
		CS1237_read_bit(handle);

		// bits: 38-45 (config data)
		handle->config.DATA=0;

		for (uint8_t q=0; q<8; q++) {
			handle->config.DATA= (uint8_t)((handle->config.DATA) <<1) | CS1237_read_bit(handle);
		}
	}
	
	// bit: 46
	CS1237_read_bit(handle);
	
	return true;
}

bool CS1237_read_config(CS1237_Handle * handle) {
//	if (!CS1237_adc_read(handle)) return false;
//	
//	// bits: 25 (update1), 26 (update2), 27-29 (device switched to input)
//	for (uint8_t q=0; q<5; q++) {
//		CS1237_read_bit(handle);
//	}
//	
//	handle->set_pin_output();
//	
//	// bits: 30-36 (command: read or write)
//	uint8_t cmd=CS1237_CMD_READ_CONFIG;
//	for (uint8_t q=0; q<7; q++) {
////		CS1237_write_bit(handle, (cmd & 0b00000001));
////		cmd=cmd>>1;
//		
//		CS1237_write_bit(handle, (cmd & 0b01000000));
//		cmd=cmd<<1;
//	}
//	
//	handle->set_pin_input();
//	
//	// bit: 37 (device switched to output)
//	CS1237_read_bit(handle);
//	
//	// bits: 38-45 (config data)
//	handle->config.DATA=0;
//	
//	for (uint8_t q=0; q<8; q++) {
//		handle->config.DATA= (uint8_t)((handle->config.DATA) <<1) | CS1237_read_bit(handle);
//	}
//	
//	return true;
	
	return CS1237_rw_config(handle, false);
}

bool CS1237_write_config(CS1237_Handle * handle) {
	return CS1237_rw_config(handle, true);
}
