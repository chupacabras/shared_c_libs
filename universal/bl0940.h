/**
  ******************************************************************************
  * @file    bl0940.h
  * @author  Juraj Lonc (juraj.lonc@gmail.com)
  * @brief   Header file of BL0940 driver.
  ******************************************************************************
  */

#ifndef INC_BL0940_H_
#define INC_BL0940_H_

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "bl094x_utils.h"

// read-only registers
#define BL0940_REG_I_FAST_RMS		0x00
#define BL0940_REG_I_WAVE 			0x01
#define BL0940_REG_V_WAVE 			0x03
#define BL0940_REG_I_RMS			0x04
#define BL0940_REG_V_RMS 			0x06
#define BL0940_REG_WATT 			0x08
#define BL0940_REG_CF_CNT 			0x0a
#define BL0940_REG_CORNER 			0x0c
#define BL0940_REG_TPS1 			0x0e
#define BL0940_REG_TPS2 			0x0f

#define BL0940_REG_READ_ALL			0xaa

// read-write registers
#define BL0940_REG_I_FAST_RMS_CTRL 	0x10
#define BL0940_REG_I_RMSOS 			0x13
#define BL0940_REG_WATTOS 			0x15
#define BL0940_REG_WA_CREEP 		0x17
#define BL0940_REG_MODE 			0x18
#define BL0940_REG_SOFT_RESET 		0x19
#define BL0940_REG_USR_WRPROT 		0x1a
#define BL0940_REG_TPS_CTRL 		0x1b
#define BL0940_REG_TPS2_A 			0x1c
#define BL0940_REG_TPS2_B 			0x1d





#define BL0940_ADDR_READ		 	0x58
#define BL0940_ADDR_WRITE		 	0xA8

#define BL0940_UNLOCK_CODE		 	0x55
#define BL0940_SOFT_RESET_CODE	 	0x5A


#define BL0940_MODE_RMS_UPDATE_SEL_400MS		0
#define BL0940_MODE_RMS_UPDATE_SEL_800MS		1
#define BL0940_MODE_AC_FREQ_SEL_50HZ			0
#define BL0940_MODE_AC_FREQ_SEL_60HZ			1
#define BL0940_MODE_CF_ENERGY_PULSES			0
#define BL0940_MODE_CF_TPS_ALARM				1
#define BL0940_FAST_RMS_REFRESH_TIME_HALFCYCLE	0
#define BL0940_FAST_RMS_REFRESH_TIME_CYCLE		1


typedef struct {
	bl094x_write_ptr write_reg;
	bl094x_read_ptr read_reg;
	bl094x_cs_ptr chip_select;
} BL0940_Handle;


typedef union {
	struct {
		uint8_t  :8;
		uint8_t RMS_UPDATE_SEL :1;
		uint8_t AC_FREQ_SEL :1;
		uint8_t :2;
		uint8_t CF_UNABLE :1;
	};
	struct {
		uint8_t DATA[2];
	};
} BL0940_Mode;

typedef union {
	struct {
		uint16_t EXT_TEMP_ALARM_THRESHOLS :9;
		uint8_t TEMP_MESUREMENT_INTERVAL :2;
		uint8_t TEMP_MEASUREMENT_SEL :2;
		uint8_t ALARM_SWITCH :1;
		uint8_t TEMP_SWITCH :1;
	};
	struct {
		uint8_t DATA[2];
	};
} BL0940_TpsControl;

typedef union {
	struct {
		uint16_t FAST_RMS_THRESHOLD :15;
		uint8_t FAST_RMS_REFRESH_TIME :1;
	};
	struct {
		uint8_t DATA[2];
	};
} BL0940_FastRmsControl;

typedef enum {
	BL0940_TEMP_MEAS_INTERVAL_50MS=0b00,
	BL0940_TEMP_MEAS_INTERVAL_100MS=0b01,
	BL0940_TEMP_MEAS_INTERVAL_200MS=0b10,
	BL0940_TEMP_MEAS_INTERVAL_400MS=0b11
} BL0940_TempInterval;

typedef enum {
	BL0940_TEMP_MEAS_SEL_AUTO=0b00,
//	BL0940_TEMP_MEAS_SEL_=0b01, // same as 0b00
	BL0940_TEMP_MEAS_SEL_INTERNAL=0b10,
	BL0940_TEMP_MEAS_SEL_EXTERNAL=0b11
} BL0940_TempSelection;

typedef enum {
	BL0940_ALARM_TEMPERATURE=0b0,
	BL0940_ALARM_OVERCURRENT=0b1
} BL0940_AlarmSwitch;

typedef enum {
	BL0940_TEMP_SWITCH_ON=0b0,
	BL0940_TEMP_SWITCH_OFF=0b1
} BL0940_TemperatureSwitch;

typedef union {
	struct {
		uint8_t HEAD;
		uint8_t I_FAST_RMS[3];
		uint8_t I_RMS[3];
		uint32_t :24;
		uint8_t V_RMS[3];
		uint32_t :24;
		uint8_t WATT[3];
		uint32_t :24;
		uint8_t CF_CNT[3];
		uint32_t :24;
		uint8_t TPS1[3];
		uint8_t TPS2[3];
		uint8_t CHECKSUM;
	};
	struct {
		uint8_t DATA[35];
	};
} BL0940_AllData;



void BL0940_init(BL0940_Handle *handle, bl094x_write_ptr write_reg, bl094x_read_ptr read_reg, bl094x_cs_ptr chip_select);

uint8_t BL0940_read_i_fast_rms(BL0940_Handle *handle, uint32_t *i_fast_rms);
uint8_t BL0940_read_i_wave(BL0940_Handle *handle, int32_t *val);
uint8_t BL0940_read_v_wave(BL0940_Handle *handle, int32_t *val);
uint8_t BL0940_read_i_rms(BL0940_Handle *handle, uint32_t *val);
uint8_t BL0940_read_v_rms(BL0940_Handle *handle, uint32_t *val);
uint8_t BL0940_read_watt(BL0940_Handle *handle, int32_t *val);
uint8_t BL0940_read_cf_count(BL0940_Handle *handle, uint32_t *val);
uint8_t BL0940_read_corner(BL0940_Handle *handle, uint16_t *val);
uint8_t BL0940_read_tps1(BL0940_Handle *handle, uint16_t *val);
uint8_t BL0940_read_tps2(BL0940_Handle *handle, uint16_t *val);



uint8_t BL0940_read_all(BL0940_Handle *handle, BL0940_AllData *data);


uint8_t BL0940_read_fast_rms_control(BL0940_Handle *handle, BL0940_FastRmsControl *ctrl);
uint8_t BL0940_set_fast_rms_control(BL0940_Handle *handle, BL0940_FastRmsControl *ctrl);
uint8_t BL0940_read_i_rms_offset(BL0940_Handle *handle, uint8_t *offset);
uint8_t BL0940_set_i_rms_offset(BL0940_Handle *handle, uint8_t offset);
uint8_t BL0940_read_watt_offset(BL0940_Handle *handle, uint8_t *offset);
uint8_t BL0940_set_watt_offset(BL0940_Handle *handle, uint8_t offset);
uint8_t BL0940_read_anticreep_threshold(BL0940_Handle *handle, uint8_t *wa_creep);
uint8_t BL0940_set_anticreep_threshold(BL0940_Handle *handle, uint8_t wa_creep);
uint8_t BL0940_read_mode(BL0940_Handle *handle, BL0940_Mode *mode);
uint8_t BL0940_set_mode(BL0940_Handle *handle, BL0940_Mode *mode);
uint8_t BL0940_read_tps_control(BL0940_Handle *handle, BL0940_TpsControl *ctrl);
uint8_t BL0940_set_tps_control(BL0940_Handle *handle, BL0940_TpsControl *ctrl);
uint8_t BL0940_read_tps2a_gain(BL0940_Handle *handle, uint8_t *gain);
uint8_t BL0940_set_tps2a_gain(BL0940_Handle *handle, uint8_t gain);
uint8_t BL0940_read_tps2b_offset(BL0940_Handle *handle, uint8_t *offset);
uint8_t BL0940_set_tps2b_offset(BL0940_Handle *handle, uint8_t offset);


uint8_t BL0940_soft_reset(BL0940_Handle *handle);

uint8_t BL0940_disable_write_protection(BL0940_Handle *handle);

#endif /* INC_BL0940_H_ */
