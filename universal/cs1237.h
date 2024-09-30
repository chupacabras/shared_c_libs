/**
  ******************************************************************************
  * @file    cs1237.h
  * @author  Juraj Lonc (juraj.lonc@gmail.com)
  * @brief   Header file of CS1237 driver.
  ******************************************************************************
  */

#ifndef INC_CS1237_H_
#define INC_CS1237_H_

#include "stdbool.h"
#include <stdint.h>
#include "device.h"

typedef void(*cs1237_device_pin) (void);
typedef uint8_t(*cs1237_device_pin_read) (void);

typedef struct tag_CS1237_Register_Config {
	union {
		struct {
			uint8_t DATA;
		};
		struct {
			uint8_t ch_sel :2;	// bits 0-1; Channel selection
			uint8_t pga_sel :2;	// bits 2-3; PGA selection
			uint8_t speed_sel :2;	// bits 4-5; ADC output rate selection
			uint8_t refo_off :1;	// bit 6; REF output switch (1 = Turn off REF output, 0 = REF Normal output)
			uint8_t reserved :1;	// bit 7; Unused, set to 0
		};
	};
} CS1237_Register_Config;

typedef struct {
	cs1237_device_pin set_pin_input;
	cs1237_device_pin set_pin_output;
	cs1237_device_pin set_clk_low;
	cs1237_device_pin set_clk_high;
	cs1237_device_pin set_dat_low;
	cs1237_device_pin set_dat_high;
	cs1237_device_pin_read read_dat;
	device_delay_us_ptr delay_us;
	
	int32_t value;
	CS1237_Register_Config config;
} CS1237_Handle;

#define CS1237_CMD_WRITE_CONFIG	0x65
#define CS1237_CMD_READ_CONFIG	0x56

// Config Register values
#define CS1237_SPEED_SEL_1280HZ	0b11
#define CS1237_SPEED_SEL_640HZ	0b10
#define CS1237_SPEED_SEL_40HZ		0b01
#define CS1237_SPEED_SEL_10HZ		0b00

#define CS1237_PGA_SEL_128		0b11
#define CS1237_PGA_SEL_64		0b10
#define CS1237_PGA_SEL_2		0b01
#define CS1237_PGA_SEL_1		0b00

#define CS1237_CH_SEL_CHANNEL_A		0b00
#define CS1237_CH_SEL_TEMPERATURE		0b10
#define CS1237_CH_SEL_INTERNAL_SHORT	0b11

#define CS1237_REF_OUT_OFF		1
#define CS1237_REF_OUT_ON		0

/*
CS1237lib Status Register
  Bit 7 bit26 of last ADC response
  Bit 6 bit25 of last ADC response
  Bit 5 initialize sucessfull
  Bit 4 new data is ready at ADCread
  
  Bit 3 sleeping
  Bit 2 reserved
  Bit 1 config write cycle is pending
  Bit 0 config read cycle is pending
*/

#define CS1237_STATUS_INIT_SUCCESS    0b00000001

//mask
#define SRM_BIT26_UPDATE2     0x80  
#define SRM_BIT25_UPDATE1     0x40
//#define SRM_INIT_SUCCESS    0x20
#define SRM_NEW_DATA_READY    0x10
#define SRM_ADC_SLEEPING     0x08
//#define SRM_RESERVED       0x04
#define SRM_WRITE_CNF_PENDING  0x02
#define SRM_READ_CNF_PENDING   0x01


void CS1237_init(CS1237_Handle * obj, cs1237_device_pin set_pin_input, cs1237_device_pin set_pin_output, cs1237_device_pin set_clk_low, cs1237_device_pin set_clk_high, cs1237_device_pin set_dat_low, cs1237_device_pin set_dat_high, cs1237_device_pin_read read_dat, device_delay_us_ptr delay_us);
void CS1237_power_down(CS1237_Handle * obj);
void CS1237_wake_up(CS1237_Handle * obj);
bool CS1237_data_ready(CS1237_Handle * obj);
bool CS1237_adc_read(CS1237_Handle * obj);
bool CS1237_poll(CS1237_Handle * obj);
bool CS1237_read_config(CS1237_Handle * obj);
bool CS1237_write_config(CS1237_Handle * obj);


#endif /* INC_CS1237_H_ */
