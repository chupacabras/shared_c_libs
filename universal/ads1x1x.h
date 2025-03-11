/**
  ******************************************************************************
  * @file    ads1x1x.h
  * @author  Juraj Lonc (juraj.lonc@gmail.com)
  * @brief   Header file of ADS1013/ADS1014/ADS1015/ADS1113/ADS1114/ADS1115 driver.
  ******************************************************************************
  */

#ifndef INC_ADS1X1X_H_
#define INC_ADS1X1X_H_

#ifdef __cplusplus
extern "C" {
#endif

#ifndef __STM8S_H
#include <stdint.h>
#include <stdbool.h>
#endif /* __STM8S_H */

#include "device.h"

#define ADS1X1X_I2C_ADDRESS			0b10010000

typedef enum {
	ADS1013=13,		// 12bit, 3300sps, 1 (1) channels
	ADS1014=14,		// 12bit, 3300sps, 1 (1) channels, PGA, comparator
	ADS1015=15,		// 12bit, 3300sps, 2 (4) channels, PGA, comparator

	ADS1113=113,	// 16bit, 860sps, 1 (1) channels
	ADS1114=114,	// 16bit, 860sps, 1 (1) channels, PGA, comparator
	ADS1115=115		// 16bit, 860sps, 2 (4) channels, PGA, comparator
} ADS1X1X_Variant;

typedef enum {
	ADS1X1X_ADDR_GND=0b000,
	ADS1X1X_ADDR_VDD=0b010,
	ADS1X1X_ADDR_SDA=0b100,
	ADS1X1X_ADDR_SCL=0b110
} ADS1X1X_ADDR_Pin;

#define ADS1X1X_REG_CONVERSION			0b00000000
#define ADS1X1X_REG_CONFIG				0b00000001
#define ADS1X1X_REG_LO_THRESHOLD		0b00000010
#define ADS1X1X_REG_HI_THRESHOLD		0b00000011

/*
This bit determines the operational status of the device.
This bit can only be written when in power-down mode.
 */
#define ADS1X1X_STATUS_BEGIN_CONVERSION				1	// Start a single conversion (when in power-down state)
#define ADS1X1X_STATUS_PERFORMING_CONVERSION		0
#define ADS1X1X_STATUS_NOT_PERFORMING_CONVERSION	1

// ADS1x15 only
/*
These bits configure the input multiplexer. They serve no function on the ADS1x13/4
 */
typedef enum {
	ADS1X1X_MUX_AIN0_AIN1=0b000,	// differential AIN0-AIN1 (default)
	ADS1X1X_MUX_AIN0_AIN3=0b001,	// differential AIN0-AIN3
	ADS1X1X_MUX_AIN1_AIN3=0b010,	// differential AIN1-AIN3
	ADS1X1X_MUX_AIN2_AIN3=0b011,	// differential AIN2-AIN3
	ADS1X1X_MUX_AIN0_GND=0b100,		// single-ended AIN0
	ADS1X1X_MUX_AIN1_GND=0b101,		// single-ended AIN1
	ADS1X1X_MUX_AIN2_GND=0b110,		// single-ended AIN2
	ADS1X1X_MUX_AIN3_GND=0b111		// single-ended AIN3
} ADS1X1X_MUX;

/*
Programmable gain amplifier configuration (ADS1x14 and ADS1x15 only)
These bits configure the programmable gain amplifier. They serve no function on the ADS1x13.
(*1) This parameter expresses the full-scale range of the ADC scaling. Do not apply more than VDD + 0.3V to the analog inputs of the device
 */
typedef enum {
	ADS1X1X_PGA_6_144V=0b000,	// FS = ±6.144V; gain = 2/3 (*1)
	ADS1X1X_PGA_4_096V=0b001,	// FS = ±4.096V; gain = 1 (*1)
	ADS1X1X_PGA_2_048V=0b010,	// FS = ±2.048V; gain = 2 (default)
	ADS1X1X_PGA_1_024V=0b011,	// FS = ±1.024V; gain = 4
	ADS1X1X_PGA_0_512V=0b100,	// FS = ±0.512V; gain = 8
	ADS1X1X_PGA_0_256V=0b101	// FS = ±0.256V; gain = 16
} ADS1X1X_PGA;

/*
Device operating mode
This bit controls the current operational mode of the ADS1x13/4/5.
 */
typedef enum {
	ADS1X1X_MODE_CONTINOUS=0,	// Continuous conversion mode
	ADS1X1X_MODE_SINGLE_SHOT=1	// Power-down single-shot mode (default)
} ADS1X1X_Mode;

/*
These bits control the data rate setting.
 */
typedef enum {
	ADS101X_DR_128SPS=0b000,
	ADS101X_DR_250SPS=0b001,
	ADS101X_DR_490SPS=0b010,
	ADS101X_DR_920SPS=0b011,
	ADS101X_DR_1600SPS=0b100,	// (default)
	ADS101X_DR_2400SPS=0b101,
	ADS101X_DR_3300SPS=0b110,

	ADS111X_DR_8SPS=0b000,
	ADS111X_DR_16SPS=0b001,
	ADS111X_DR_32SPS=0b010,
	ADS111X_DR_64SPS=0b011,
	ADS111X_DR_128SPS=0b100,	// (default)
	ADS111X_DR_250SPS=0b101,
	ADS111X_DR_475SPS=0b110,
	ADS111X_DR_860SPS=0b111
} ADS1X1X_DataRate;

// ADS1x14 and ADS1x15 only
/*
This bit controls the comparator mode of operation. It changes whether the comparator is implemented as a
traditional comparator (COMP_MODE = '0') or as a window comparator (COMP_MODE = '1'). It serves no
function on the ADS1x13
*/
typedef enum {
	ADS1X1X_COMP_MODE_TRADITIONAL=0,	// Traditional comparator with hysteresis (default)
	ADS1X1X_COMP_MODE_WINDOW=1			// Window comparator
} ADS1X1X_ComparatorMode;

// ADS1x14 and ADS1x15 only
/*
This bit controls the polarity of the ALERT/RDY pin. When COMP_POL = '0' the comparator output is active
low. When COMP_POL='1' the ALERT/RDY pin is active high. It serves no function on the ADS1x13.
*/
typedef enum {
	ADS1X1X_COMP_POL_ACTIVE_LOW=0,		// Active low (default)
	ADS1X1X_COMP_POL_ACTIVE_HIGH=1		// Active high
} ADS1X1X_ComparatorPolarity;

// ADS1x14 and ADS1x15 only
/*
This bit controls whether the ALERT/RDY pin latches once asserted or clears once conversions are within the
margin of the upper and lower threshold values. When COMP_LAT = '0', the ALERT/RDY pin does not latch
when asserted. When COMP_LAT = '1', the asserted ALERT/RDY pin remains latched until conversion data
are read by the master or an appropriate SMBus alert response is sent by the master, the device responds with
its address, and it is the lowest address currently asserting the ALERT/RDY bus line. This bit serves no
function on the ADS1x13.
 */
typedef enum {
	ADS1X1X_COMP_LAT_NON_LATCHING=0,	// Non-latching comparator (default)
	ADS1X1X_COMP_LAT_LATCHING=1			// Latching comparator
} ADS1X1X_ComparatorLatching;

// ADS1x14 and ADS1x15 only
/*
These bits perform two functions. When set to '11', they disable the comparator function and put the
ALERT/RDY pin into a high state. When set to any other value, they control the number of successive
conversions exceeding the upper or lower thresholds required before asserting the ALERT/RDY pin. They
serve no function on the ADS1x13.
 */
typedef enum {
	ADS1X1X_COMP_QUE_1CONV=0b00,	// Assert after one conversion
	ADS1X1X_COMP_QUE_2CONV=0b01,	// Assert after two conversions
	ADS1X1X_COMP_QUE_4CONV=0b10,	// Assert after four conversions
	ADS1X1X_COMP_QUE_DISABLE=0b11	// Disable comparator and set ALERT/RDY pin to high-impedance (default)
} ADS1X1X_ComparatorQueue;


typedef struct {
	union {
		struct {
			uint16_t DATA;
		};
		struct {
			uint8_t COMP_QUE :2;	// bit 0-1; Comparator queue and disable (ADS1x14/ADS1x15 only)
			uint8_t COMP_LAT :1;	// bit 2; Latching comparator (ADS1x14/ADS1x15 only)
			uint8_t COMP_POL :1;	// bit 3; Comparator polarity (ADS1x14/ADS1x15 only)
			uint8_t COMP_MODE :1;	// bit 4; Comparator mode (ADS1x14/ADS1x15 only)
			uint8_t DR :3;			// bit 5-7; Data rate
			uint8_t MODE :1;		// bit 8; Device operating mode
			uint8_t PGA :3;			// bit 9-11; Programmable gain amplifier configuration (ADS1x14/ADS1x15 only)
			uint8_t MUX :3;			// bit 12-14; Input multiplexer configuration (ADS1x15 only)
			uint8_t OS :1;			// bit 15; Operational status or single-shot conversion start
		};
	};
} ADS1X1X_Config;


typedef struct {
	device_write_ptr write_reg;
	device_read_ptr read_reg;

	uint8_t addr;
	ADS1X1X_Variant variant;
	ADS1X1X_Config config;
} ADS1X1X_Handle;

uint8_t ADS1X1X_init(ADS1X1X_Handle *handle, ADS1X1X_Variant variant, ADS1X1X_ADDR_Pin addrpin, device_write_ptr write_reg, device_read_ptr read_reg);

uint8_t ADS1X1X_set_lo_threshold(ADS1X1X_Handle *handle, int16_t val);
uint8_t ADS1X1X_set_hi_threshold(ADS1X1X_Handle *handle, int16_t val);
uint8_t ADS1X1X_set_config(ADS1X1X_Handle *handle);
uint8_t ADS1X1X_read_lo_threshold(ADS1X1X_Handle *handle, int16_t *val);
uint8_t ADS1X1X_read_hi_threshold(ADS1X1X_Handle *handle, int16_t *val);
uint8_t ADS1X1X_read_config(ADS1X1X_Handle *handle);

void ADS1X1X_set_data_rate(ADS1X1X_Handle *handle, ADS1X1X_DataRate dr);
void ADS1X1X_set_pga(ADS1X1X_Handle *handle, ADS1X1X_PGA pga);
void ADS1X1X_set_mux(ADS1X1X_Handle *handle, ADS1X1X_MUX mux);
void ADS1X1X_set_mode(ADS1X1X_Handle *handle, ADS1X1X_Mode mode);
void ADS1X1X_set_comparator(ADS1X1X_Handle *handle, ADS1X1X_ComparatorMode mode, ADS1X1X_ComparatorPolarity pol, ADS1X1X_ComparatorLatching lat, ADS1X1X_ComparatorQueue queue);

uint8_t ADS1X1X_start_one_shot(ADS1X1X_Handle *handle);
uint8_t ADS1X1X_start_continuous(ADS1X1X_Handle *handle);

#ifdef __cplusplus
}
#endif

#endif /* INC_ADS1X1X_H_ */
