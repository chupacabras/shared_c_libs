/**
  ******************************************************************************
  * @file    max1120x.h
  * @author  Juraj Lonc (juraj.lonc@gmail.com)
  * @brief   Header file for MAX11206/MAX1120X ADC driver.
  ******************************************************************************
  */

#ifndef INC_MAX1120X_H_
#define INC_MAX1120X_H_

#include <stdbool.h>
#include <stdint.h>
#include "device.h"

// MODE=0
typedef enum {
	MAX1120X_CMD_SELF_CAL           =0b10010000,
    MAX1120X_CMD_SYSTEM_OFFSET_CAL  =0b10100000,
    MAX1120X_CMD_SYSTEM_GAIN_CAL    =0b10110000,
    MAX1120X_CMD_POWER_DOWN         =0b10001000,
    MAX1120X_CMD_CONVERT_1SPS       =0b10000000,
    MAX1120X_CMD_CONVERT_2_5SPS     =0b10000001,
    MAX1120X_CMD_CONVERT_5SPS       =0b10000010,
    MAX1120X_CMD_CONVERT_10SPS      =0b10000011,
    MAX1120X_CMD_CONVERT_15SPS      =0b10000100,
    MAX1120X_CMD_CONVERT_30SPS      =0b10000101,
    MAX1120X_CMD_CONVERT_60SPS      =0b10000110,
    MAX1120X_CMD_CONVERT_120SPS     =0b10000111
} MAX1120X_Command;

// MODE=1
typedef enum {
    MAX1120X_REGISTER_STAT1     =0b11000000,
    MAX1120X_REGISTER_CTRL1     =0b11000010,
    MAX1120X_REGISTER_CTRL2     =0b11000100,
    MAX1120X_REGISTER_CTRL3     =0b11000110,
    MAX1120X_REGISTER_DATA      =0b11001000,
    MAX1120X_REGISTER_SOC       =0b11001010,
    MAX1120X_REGISTER_SGC       =0b11001100,
    MAX1120X_REGISTER_SCOC      =0b11001110,
    MAX1120X_REGISTER_SCGC      =0b11010000
} MAX1120X_Register;

#define MAX1120X_REG_READ       0b00000001
#define MAX1120X_REG_WRITE      0b00000000

typedef union {
	struct {
		uint8_t DATA;
	};
	struct {
		uint8_t RDY :1;     // The ready bit; 1=conversion result is available
        uint8_t MSTAT :1;   // The measurement status bit; 1=converting in progress, 0=not converting
        uint8_t UR :1;      // The under-range bit
        uint8_t OR :1;      // The over-range bit
        uint8_t RATE0 :1;   // The data rate
        uint8_t RATE1 :1;
        uint8_t RATE2 :1;
        uint8_t SYSOR :1;   // The system gain over-range bit
	};
} MAX1120X_Stat1;

typedef union {
	struct {
		uint8_t DATA;
	};
	struct {
		uint8_t :1;
        uint8_t SCYCLE :1;  // The single-cycle bit; 1=single conversion, 0=continuous conversion
        uint8_t FORMAT :1;  // The format bit; 1=offset binary, 0=two's complement
        uint8_t SIGBUF :1;  // The signal buffer; 1=enable signal buffer, 0=disable
        uint8_t REFBUF :1;  // The reference buffer bit; 1=enable reference buffer, 0=disable
        uint8_t EXTCLK :1;  // The external clock bit; 1=external clock, 0=internal clock
        uint8_t U :1;       // The unipolar/bipolar bit; 1=unipolar input range, 0=bipolar input range
        uint8_t LINEF :1;   // The line frequency bit; 1=50Hz, 0=60Hz
	};
} MAX1120X_Ctrl1;

typedef union {
	struct {
		uint8_t DATA;
	};
	struct {
		uint8_t DIO1 :1;    // The data input/output bits
        uint8_t DIO2 :1;
        uint8_t DIO3 :1;
        uint8_t DIO4 :1;
        uint8_t DIR1 :1;    // The direction bits; 1=output, 0=input
        uint8_t DIR2 :1;
        uint8_t DIR3 :1;
        uint8_t DIR4 :1;
	};
} MAX1120X_Ctrl2;

typedef union {
	struct {
		uint8_t DATA;
	};
	struct {
		uint8_t :1;
        uint8_t NOSCO :1;
        uint8_t NOSCG :1;
        uint8_t NOSYSO :1;
        uint8_t NOSYSG :1;
        uint8_t DGAIN0 :1;
        uint8_t DGAIN1 :1;
        uint8_t DGAIN2 :1;
	};
} MAX1120X_Ctrl3;

typedef struct {
    device_write_pin_ptr write_cs;
	device_spi_write_ptr write_cmd;
    device_read_pin_ptr read_data_pin;
    device_delay_ms_ptr delay_ms;
} MAX1120X_Handle;

typedef void (*device_set_input_signal_ptr)(void);

void MAX1120X_init(MAX1120X_Handle *handle, device_write_pin_ptr write_cs, device_spi_write_ptr write_cmd, device_read_pin_ptr read_data_pin, device_delay_ms_ptr delay_ms);
void MAX1120X_send_command(MAX1120X_Handle *handle, MAX1120X_Command command);
void MAX1120X_write_register_byte(MAX1120X_Handle *handle, MAX1120X_Register reg, uint8_t val);
void MAX1120X_write_register_3byte(MAX1120X_Handle *handle, MAX1120X_Register reg, uint32_t val);
void MAX1120X_read_register_byte(MAX1120X_Handle *handle, MAX1120X_Register reg, uint8_t *val);
void MAX1120X_read_register_3byte(MAX1120X_Handle *handle, MAX1120X_Register reg, uint32_t *val);

// true = done, false = problem
bool MAX1120X_wait_for_conversion_done(MAX1120X_Handle *handle);

void MAX1120X_system_calibration(MAX1120X_Handle *handle, device_set_input_signal_ptr set_zero_signal, device_set_input_signal_ptr set_full_signal, device_set_input_signal_ptr set_normal_signal);

#endif