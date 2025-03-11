/**
  ******************************************************************************
  * @file    bl0942.h
  * @author  Juraj Lonc (juraj.lonc@gmail.com)
  * @brief   Header file of BL0942 driver.
  ******************************************************************************
  */

#ifndef INC_BL0942_H_
#define INC_BL0942_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "bl094x_utils.h"

// read-only registers
#define BL0942_REG_I_WAVE 			0x01
#define BL0942_REG_V_WAVE 			0x02
#define BL0942_REG_I_RMS			0x03
#define BL0942_REG_V_RMS 			0x04
#define BL0942_REG_I_FAST_RMS		0x05
#define BL0942_REG_WATT 			0x06
#define BL0942_REG_CF_CNT 			0x07
#define BL0942_REG_FREQ 			0x08
#define BL0942_REG_STATUS 			0x09

#define BL0942_REG_READ_ALL			0xaa

// read-write registers
#define BL0942_REG_I_RMSOS 			0x12
#define BL0942_REG_WA_CREEP 		0x14
#define BL0942_REG_I_FAST_RMS_TH 	0x15
#define BL0942_REG_I_FAST_RMS_CYC 	0x16
#define BL0942_REG_FREQ_CYC 		0x17
#define BL0942_REG_OT_FUNX 			0x18
#define BL0942_REG_MODE 			0x19
#define BL0942_REG_GAIN_CR 			0x1a
#define BL0942_REG_SOFT_RESET 		0x1c
#define BL0942_REG_USR_WRPROT 		0x1d

#define BL0942_ADDR_READ		 	0x58
#define BL0942_ADDR_WRITE		 	0xA8

#define BL0942_UNLOCK_CODE		 	0x55
#define BL0942_SOFT_RESET_CODE	 	0x5A


#define BL0942_MODE_RMS_UPDATE_SEL_400MS		0
#define BL0942_MODE_RMS_UPDATE_SEL_800MS		1
#define BL0942_MODE_AC_FREQ_SEL_50HZ			0
#define BL0942_MODE_AC_FREQ_SEL_60HZ			1
#define BL0942_MODE_CF_CNT_ADD_SEL_SIGNED		0
#define BL0942_MODE_CF_CNT_ADD_SEL_ABSOLUTE		1
#define BL0942_MODE_UART_RATE_SEL_EXTERNAL_PIN	0b00	// same as 0b01
#define BL0942_MODE_UART_RATE_SEL_19200BPS		0b10
#define BL0942_MODE_UART_RATE_SEL_38400BPS		0b11

#define BL0942_FUNX_SEL_ACTIVE_ENERGY_CF		0b00
#define BL0942_FUNX_SEL_OVER_CURRENT_EVENT		0b01
#define BL0942_FUNX_SEL_ZERO_CROSSING_VOLTAGE	0b10
#define BL0942_FUNX_SEL_ZERO_CROSSING_CURRENT	0b11



//typedef enum {
//	BL0942_COM_UART=0,
//	BL0942_COM_SPI=1
//} BL0942_Com;

typedef struct {
	bl094x_write_ptr write_reg;
	bl094x_read_ptr read_reg;
	bl094x_cs_ptr chip_select;
	uint8_t addr;
//	BL0942_Com com_type;
} BL0942_Handle;


typedef union {
	struct {
		uint8_t  :2;
		uint8_t CF_EN :1;
		uint8_t RMS_UPDATE_SEL :1;
		uint8_t  :1;
		uint8_t AC_FREQ_SEL :1;
		uint8_t CF_CNT_CLR_SEL :1;
		uint8_t CF_CNT_ADD_SEL :1;
		uint8_t UART_RATE_SEL :2;

	};
	struct {
		uint8_t DATA[2];
	};
} BL0942_Mode;


typedef union {
	struct {
		uint8_t CF1_FUNX_SEL :2;
		uint8_t CF2_FUNX_SEL :2;
		uint8_t ZX_FUNX_SEL :2;
	};
	struct {
		uint8_t DATA;
	};
} BL0942_OtFunx;

typedef union {
	struct {
		uint8_t CF_REVP_F :1;
		uint8_t CREEP_F :1;
		uint8_t  :6;
		uint8_t I_ZX_LTH_F :1;
		uint8_t V_ZX_LTH_F :1;
	};
	struct {
		uint8_t DATA[2];
	};
} BL0942_Status;

typedef union {
	struct {
		uint8_t HEAD;
		uint8_t I_RMS[3];
		uint8_t V_RMS[3];
		uint8_t I_FAST_RMS[3];
		uint8_t WATT[3];
		uint8_t CF_CNT[3];
		uint8_t FREQ[3];
		uint8_t STATUS[3];
		uint8_t CHECKSUM;
	};
	struct {
		uint8_t DATA[23];
	};
} BL0942_AllData;

typedef enum {
	BL0942_FAST_RMS_05CYC=0b000,
	BL0942_FAST_RMS_1CYC=0b001,
	BL0942_FAST_RMS_2CYC=0b010,
	BL0942_FAST_RMS_4CYC=0b011,
	BL0942_FAST_RMS_8CYC=0b100
} BL0942_FastRMSCycles;

typedef enum {
	BL0942_FREQ_2CYC=0b00,
	BL0942_FREQ_4CYC=0b01,
	BL0942_FREQ_8CYC=0b10,
	BL0942_FREQ_16CYC=0b11
} BL0942_FrequencyUpdateCycles;

typedef enum {
	BL0942_GAIN_1=0b00,
	BL0942_GAIN_4=0b01,
	BL0942_GAIN_16=0b10,
	BL0942_GAIN_24=0b11
} BL0942_Gain;

void BL0942_init_SPI(BL0942_Handle *handle, bl094x_write_ptr write_reg, bl094x_read_ptr read_reg, bl094x_cs_ptr chip_select);
void BL0942_init_UART(BL0942_Handle *handle, uint8_t addr1, uint8_t addr2, bl094x_write_ptr write_reg, bl094x_read_ptr read_reg);

uint8_t BL0942_read_i_wave(BL0942_Handle *handle, int32_t *val);
uint8_t BL0942_read_v_wave(BL0942_Handle *handle, int32_t *val);
uint8_t BL0942_read_i_rms(BL0942_Handle *handle, uint32_t *val);
uint8_t BL0942_read_v_rms(BL0942_Handle *handle, uint32_t *val);
uint8_t BL0942_read_i_fast_rms(BL0942_Handle *handle, uint32_t *i_fast_rms);
uint8_t BL0942_read_watt(BL0942_Handle *handle, int32_t *val);
uint8_t BL0942_read_cf_count(BL0942_Handle *handle, uint32_t *val);
uint8_t BL0942_read_frequency_reg(BL0942_Handle *handle, uint16_t *val);
uint8_t BL0942_read_frequency(BL0942_Handle *handle, uint16_t *frequency);
uint8_t BL0942_read_status(BL0942_Handle *handle, BL0942_Status *status);



uint8_t BL0942_read_all(BL0942_Handle *handle, BL0942_AllData *data);



uint8_t BL0942_read_i_rms_offset(BL0942_Handle *handle, uint8_t *offset);
uint8_t BL0942_set_i_rms_offset(BL0942_Handle *handle, uint8_t offset);
uint8_t BL0942_read_anticreep_threshold(BL0942_Handle *handle, uint8_t *wa_creep);
uint8_t BL0942_set_anticreep_threshold(BL0942_Handle *handle, uint8_t wa_creep);
uint8_t BL0942_read_i_fast_rms_threshold(BL0942_Handle *handle, uint16_t *i_rms_threshold);
uint8_t BL0942_set_i_fast_rms_threshold(BL0942_Handle *handle, uint16_t *i_rms_threshold);
uint8_t BL0942_read_i_fast_rms_cycles(BL0942_Handle *handle, BL0942_FastRMSCycles *cycles);
uint8_t BL0942_set_i_fast_rms_cycles(BL0942_Handle *handle, BL0942_FastRMSCycles cycles);
uint8_t BL0942_read_frequency_update_cycles(BL0942_Handle *handle, BL0942_FrequencyUpdateCycles *cycles);
uint8_t BL0942_set_frequency_update_cycles(BL0942_Handle *handle, BL0942_FrequencyUpdateCycles cycles);
uint8_t BL0942_read_ot_funx(BL0942_Handle *handle, BL0942_OtFunx *status);
uint8_t BL0942_set_ot_funx(BL0942_Handle *handle, BL0942_OtFunx status);
uint8_t BL0942_read_mode(BL0942_Handle *handle, BL0942_Mode *mode);
uint8_t BL0942_set_mode(BL0942_Handle *handle, BL0942_Mode *mode);
uint8_t BL0942_read_i_gain(BL0942_Handle *handle, uint8_t *gain);
uint8_t BL0942_set_i_gain(BL0942_Handle *handle, uint8_t gain);

uint8_t BL0942_soft_reset(BL0942_Handle *handle);

uint8_t BL0942_disable_write_protection(BL0942_Handle *handle);


#ifdef __cplusplus
}
#endif

#endif /* INC_BL0942_H_ */
