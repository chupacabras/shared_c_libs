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
 * @param  obj:       Pointer to a CS1237_Object structure that contains
 *                    the information for the display.
 * @param  write_cmd:		The function that writes data to CS1237. Hardware dependent.
 * @param  read_cmd:		The function that reads data from CS1237. Hardware dependent.
 */
void CS1237_init(CS1237_Handle * obj, cs1237_device_pin set_pin_input, cs1237_device_pin set_pin_output, cs1237_device_pin set_clk_low, cs1237_device_pin set_clk_high, cs1237_device_pin set_dat_low, cs1237_device_pin set_dat_high, cs1237_device_pin_read read_dat, device_delay_us_ptr delay_us) {
	obj->set_pin_input = set_pin_input;
	obj->set_pin_output = set_pin_output;
	obj->set_clk_low = set_clk_low;
	obj->set_clk_high = set_clk_high;
	obj->set_dat_low = set_dat_low;
	obj->set_dat_high = set_dat_high;
	obj->read_dat = read_dat;
	obj->delay_us = delay_us;
	
	obj->set_pin_input();
	
	// reset ADC (put to sleep and then wake up)
	CS1237_power_down(obj);
	obj->delay_us(140);	// wait at least 100us to power down
	CS1237_wake_up(obj);
	
	
}

void CS1237_power_down(CS1237_Handle * obj) {
	obj->set_clk_high();
	// it takes at least 100us to power down
}

void CS1237_wake_up(CS1237_Handle * obj) {
	obj->set_clk_low();
	// it takes at least 10us to wake up
	obj->delay_us(12);
}

bool CS1237_data_ready(CS1237_Handle * obj) {
	return obj->read_dat()==0;
}

bool CS1237_read_bit(CS1237_Handle * obj) {
	bool data;
	
	obj->set_clk_high();
//	obj->delay_us(1);		// wait at least 455ns to read data
	
	data=obj->read_dat();
	
	obj->set_clk_low();
//	obj->delay_us(1);
	
	return data;
}

void CS1237_write_bit(CS1237_Handle * obj, bool high) {
	if (high) obj->set_dat_high();
	else obj->set_dat_low();
	
	obj->set_clk_high();
//	obj->delay_us(1);		// wait at least 455ns to read data
	
	obj->set_clk_low();
//	obj->delay_us(1);
	
}

bool CS1237_adc_read(CS1237_Handle * obj) {
	obj->value=0;
	
	for (uint8_t q=0; q<24; q++) {
		obj->value= (obj->value <<1) | CS1237_read_bit(obj);
	}
	
	// convert 24bit 2's complement value
	obj->value = (obj->value ^ 0x800000) - 0x800000;
	
	return true;
}

bool CS1237_poll(CS1237_Handle * obj) {
	CS1237_wake_up(obj);
	if (!CS1237_data_ready(obj)) return false;
	
	if (!CS1237_adc_read(obj)) return false;
	
	// make sure data line is high
	CS1237_read_bit(obj);
	
	return true;
}

bool CS1237_rw_config(CS1237_Handle * obj, bool write) {
	if (!CS1237_adc_read(obj)) return false;
	
	// bits: 25 (update1), 26 (update2), 27-29 (device switched to input)
	for (uint8_t q=0; q<5; q++) {
		CS1237_read_bit(obj);
	}
	
	obj->set_pin_output();
	
	// bits: 30-36 (command: read or write)
	uint8_t cmd=write?CS1237_CMD_WRITE_CONFIG:CS1237_CMD_READ_CONFIG;
	for (uint8_t q=0; q<7; q++) {
//		CS1237_write_bit(obj, (cmd & 0b00000001));
//		cmd=cmd>>1;
		
		CS1237_write_bit(obj, (cmd & 0b01000000));
		cmd<<=1;
	}
	
	if (write) {
		// bit: 37
		CS1237_read_bit(obj);

		// bits: 38-45 (config data)
		uint8_t config=obj->config.DATA;

		for (uint8_t q=0; q<8; q++) {
			CS1237_write_bit(obj, config & 0b10000000);
			config<<=1;
		}
		
		obj->set_pin_input();
		
	} else {
		obj->set_pin_input();
	
		// bit: 37 (device switched to output)
		CS1237_read_bit(obj);

		// bits: 38-45 (config data)
		obj->config.DATA=0;

		for (uint8_t q=0; q<8; q++) {
			obj->config.DATA= (uint8_t)((obj->config.DATA) <<1) | CS1237_read_bit(obj);
		}
	}
	
	// bit: 46
	CS1237_read_bit(obj);
	
	return true;
}

bool CS1237_read_config(CS1237_Handle * obj) {
//	if (!CS1237_adc_read(obj)) return false;
//	
//	// bits: 25 (update1), 26 (update2), 27-29 (device switched to input)
//	for (uint8_t q=0; q<5; q++) {
//		CS1237_read_bit(obj);
//	}
//	
//	obj->set_pin_output();
//	
//	// bits: 30-36 (command: read or write)
//	uint8_t cmd=CS1237_CMD_READ_CONFIG;
//	for (uint8_t q=0; q<7; q++) {
////		CS1237_write_bit(obj, (cmd & 0b00000001));
////		cmd=cmd>>1;
//		
//		CS1237_write_bit(obj, (cmd & 0b01000000));
//		cmd=cmd<<1;
//	}
//	
//	obj->set_pin_input();
//	
//	// bit: 37 (device switched to output)
//	CS1237_read_bit(obj);
//	
//	// bits: 38-45 (config data)
//	obj->config.DATA=0;
//	
//	for (uint8_t q=0; q<8; q++) {
//		obj->config.DATA= (uint8_t)((obj->config.DATA) <<1) | CS1237_read_bit(obj);
//	}
//	
//	return true;
	
	return CS1237_rw_config(obj, false);
}

bool CS1237_write_config(CS1237_Handle * obj) {
	return CS1237_rw_config(obj, true);
}
