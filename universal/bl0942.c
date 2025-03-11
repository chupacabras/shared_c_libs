/**
  ******************************************************************************
  * @file    bl0942.c
  * @author  Juraj Lonc (juraj.lonc@gmail.com)
  * @brief   BL0942 driver.
  ******************************************************************************
  */

#include "bl0942.h"

static uint8_t tmp_data[3];

/**
 * @brief  BL0942 initialization.
 * @param	handle*		pointer to BL0942_Handle
 * @param	write_reg	The function that writes command to the module. Hardware dependent.
 * @param	read_reg	The function that reads data from the module. Hardware dependent.
 * @param	delay_ms	The function that makes delay in milliseconds. Hardware dependent.
 * @retval	status		0=device found
 */
// SPI max speed 900kHz
void BL0942_init_SPI(BL0942_Handle *handle, bl094x_write_ptr write_reg, bl094x_read_ptr read_reg, bl094x_cs_ptr chip_select) {
	handle->write_reg=write_reg;
	handle->read_reg=read_reg;
	handle->chip_select=chip_select;
//	handle->com_type=BL0942_COM_SPI;
	handle->addr=0;
}

void BL0942_init_UART(BL0942_Handle *handle, uint8_t addr1, uint8_t addr2, bl094x_write_ptr write_reg, bl094x_read_ptr read_reg) {
	handle->write_reg=write_reg;
	handle->read_reg=read_reg;
	handle->chip_select=NULL;
//	handle->com_type=BL0942_COM_UART;
	handle->addr=0;
	if (addr1) handle->addr|=0b01;
	if (addr2) handle->addr|=0b10;
}

static uint8_t BL0942_read_reg(BL0942_Handle *handle, uint8_t reg_addr, uint8_t *data, uint8_t count) {
	uint8_t tmp[3];
	tmp[0]=BL0942_ADDR_READ | handle->addr;
	tmp[1]=reg_addr;

	if (handle->chip_select!=NULL) handle->chip_select(true);
	handle->write_reg(tmp, 2);
	uint8_t stat=handle->read_reg(data, count);
	stat=handle->read_reg(&tmp[2], 1);
	if (handle->chip_select!=NULL) handle->chip_select(false);

	uint8_t chksum=BL094X_calculate_checksum(tmp, (reg_addr==BL0942_REG_READ_ALL)?1:2, data, count);

	if (tmp[2]!=chksum) return 10;

	return stat;
}

static uint8_t BL0942_write_reg(BL0942_Handle *handle, uint8_t reg_addr, uint8_t *data, uint8_t count) {
	uint8_t tmp[3];
	tmp[0]=BL0942_ADDR_WRITE | handle->addr;
	tmp[1]=reg_addr;

	uint8_t chksum=BL094X_calculate_checksum(tmp, 2, data, count);

	if (handle->chip_select!=NULL) handle->chip_select(true);
	handle->write_reg(tmp, 2);
	handle->write_reg(data, count);
	uint8_t stat=handle->write_reg(&chksum, 1);
	if (handle->chip_select!=NULL) handle->chip_select(false);

	return stat;
}

uint8_t BL0942_read_i_wave(BL0942_Handle *handle, int32_t *val) {
	uint8_t ret=BL0942_read_reg(handle, BL0942_REG_I_WAVE, tmp_data, 3);
	if (ret!=0) return ret;

	BL094X_convert_signed20_value(tmp_data, val);

	return 0;
}

uint8_t BL0942_read_v_wave(BL0942_Handle *handle, int32_t *val) {
	uint8_t ret=BL0942_read_reg(handle, BL0942_REG_V_WAVE, tmp_data, 3);
	if (ret!=0) return ret;

	BL094X_convert_signed20_value(tmp_data, val);

	return 0;
}


// Current RMS register equation:
// I = IP - IN, [mV]
// I_RMS_REG = 305978 * I / Vref
// I = I_RMS_REG * Vref / 305978
//
// Vref = 1.218 V
// I(max) = ± 42mV = 30mV rms
uint8_t BL0942_read_i_rms(BL0942_Handle *handle, uint32_t *val) {
	uint8_t ret=BL0942_read_reg(handle, BL0942_REG_I_RMS, tmp_data, 3);
	if (ret!=0) return ret;

	BL094X_convert_unsigned24_value(tmp_data, val);

	return 0;
}


// Voltage RMS register equation:
// V = VP - GND, [mV]
// V_RMS_REG = 73989 * V / Vref
// V = V_RMS_REG * Vref / 73989 [mV]
//
// Vref = 1.218 V
// VP(max) = ± 100mV = 70mV rms
uint8_t BL0942_read_v_rms(BL0942_Handle *handle, uint32_t *val) {
	uint8_t ret=BL0942_read_reg(handle, BL0942_REG_V_RMS, tmp_data, 3);
	if (ret!=0) return ret;

	BL094X_convert_unsigned24_value(tmp_data, val);

	return 0;
}


// I_FAST_RMS_REG ~ I_RMS_REG * 0.363
uint8_t BL0942_read_i_fast_rms(BL0942_Handle *handle, uint32_t *val) {
	uint8_t ret=BL0942_read_reg(handle, BL0942_REG_I_FAST_RMS, tmp_data, 3);
	if (ret!=0) return ret;

	BL094X_convert_unsigned24_value(tmp_data, val);

	return 0;
}


// V = VP - GND, [mV]
// I = IP - IN, [mV]
// WATT_REG = 3537 * I * V * cos(fi) / Vref^2
// Vref = 1.218V
// W_VAL = WATT_REG * Vref^2 / 3537
uint8_t BL0942_read_watt(BL0942_Handle *handle, int32_t *val) {
	uint8_t ret=BL0942_read_reg(handle, BL0942_REG_WATT, tmp_data, 3);
	if (ret!=0) return ret;

	BL094X_convert_signed24_value(tmp_data, val);

	return 0;
}

// Cumulative time of each CF pulse:
// tCF = 1638.4 * 256 / WATT_REG
uint8_t BL0942_read_cf_count(BL0942_Handle *handle, uint32_t *val) {
	uint8_t ret=BL0942_read_reg(handle, BL0942_REG_CF_CNT, tmp_data, 3);
	if (ret!=0) return ret;

	BL094X_convert_unsigned24_value(tmp_data, val);

	return 0;
}

uint8_t BL0942_read_frequency_reg(BL0942_Handle *handle, uint16_t *val) {
	uint8_t ret=BL0942_read_reg(handle, BL0942_REG_FREQ, tmp_data, 3);
	if (ret!=0) return ret;

	BL094X_convert_unsigned16_value(tmp_data, val);

	return 0;
}

// f = 1000000 / FREQ_REG, [Hz]
// returns hundreths of Hz; value 5000 = 50.00 Hz
uint8_t BL0942_read_frequency(BL0942_Handle *handle, uint16_t *frequency) {
	uint8_t ret=BL0942_read_reg(handle, BL0942_REG_FREQ, tmp_data, 3);
	if (ret!=0) return ret;

	uint32_t val;
	BL094X_convert_unsigned24_value(tmp_data, &val);
	val=1000000000/val;
	uint8_t m=val%10;
	val/=10;
	if (m>=5) {
		val++;
	}
	*frequency=val;

	return 0;
}


uint8_t BL0942_read_status(BL0942_Handle *handle, BL0942_Status *status) {
	uint8_t ret=BL0942_read_reg(handle, BL0942_REG_STATUS, tmp_data, 3);
	if (ret!=0) return ret;

	status->DATA[0]=tmp_data[0];
	status->DATA[1]=tmp_data[1];

	return 0;
}

uint8_t BL0942_read_all(BL0942_Handle *handle, BL0942_AllData *data) {
	uint8_t ret=BL0942_read_reg(handle, BL0942_REG_READ_ALL, data->DATA, 22);
	if (ret!=0) return ret;

//	BL094X_convert_unsigned16_value(data, val);

	return 0;
}

uint8_t BL0942_read_i_rms_offset(BL0942_Handle *handle, uint8_t *offset) {
	uint8_t ret=BL0942_read_reg(handle, BL0942_REG_I_RMSOS, tmp_data, 3);
	if (ret!=0) return ret;

	*offset=tmp_data[0];

	return 0;
}

uint8_t BL0942_set_i_rms_offset(BL0942_Handle *handle, uint8_t offset) {
	tmp_data[0]=offset;
	tmp_data[1]=0;
	tmp_data[2]=0;

	uint8_t ret=BL0942_write_reg(handle, BL0942_REG_I_RMSOS, tmp_data, 3);

	return ret;
}

uint8_t BL0942_read_anticreep_threshold(BL0942_Handle *handle, uint8_t *wa_creep) {
	uint8_t ret=BL0942_read_reg(handle, BL0942_REG_WA_CREEP, tmp_data, 3);
	if (ret!=0) return ret;

	*wa_creep=tmp_data[0];

	return 0;
}

// WA_CREEP = WATT_REG * 256 / 3125
// WATT_REG = WA_CREEP * 3125 / 256
uint8_t BL0942_set_anticreep_threshold(BL0942_Handle *handle, uint8_t wa_creep) {
	tmp_data[0]=wa_creep;
	tmp_data[1]=0;
	tmp_data[2]=0;

	uint8_t ret=BL0942_write_reg(handle, BL0942_REG_WA_CREEP, tmp_data, 3);

	return ret;
}

uint8_t BL0942_read_i_fast_rms_threshold(BL0942_Handle *handle, uint16_t *i_rms_threshold) {
	uint8_t ret=BL0942_read_reg(handle, BL0942_REG_I_FAST_RMS_TH, tmp_data, 3);
	if (ret!=0) return ret;

	BL094X_convert_unsigned16_value(tmp_data, i_rms_threshold);

	return 0;
}

uint8_t BL0942_set_i_fast_rms_threshold(BL0942_Handle *handle, uint16_t *i_rms_threshold) {
	tmp_data[0]=*i_rms_threshold & 0xff;
	tmp_data[1]=*i_rms_threshold>>8;
	tmp_data[2]=0;

	uint8_t ret=BL0942_write_reg(handle, BL0942_REG_I_FAST_RMS_TH, tmp_data, 3);

	return ret;
}

uint8_t BL0942_read_i_fast_rms_cycles(BL0942_Handle *handle, BL0942_FastRMSCycles *cycles) {
	uint8_t ret=BL0942_read_reg(handle, BL0942_REG_I_FAST_RMS_CYC, tmp_data, 3);
	if (ret!=0) return ret;

	*cycles=tmp_data[0];

	return 0;
}

uint8_t BL0942_set_i_fast_rms_cycles(BL0942_Handle *handle, BL0942_FastRMSCycles cycles) {
	tmp_data[0]=cycles;
	tmp_data[1]=0;
	tmp_data[2]=0;

	uint8_t ret=BL0942_write_reg(handle, BL0942_REG_I_FAST_RMS_CYC, tmp_data, 3);

	return ret;
}

uint8_t BL0942_read_frequency_update_cycles(BL0942_Handle *handle, BL0942_FrequencyUpdateCycles *cycles) {
	uint8_t ret=BL0942_read_reg(handle, BL0942_REG_FREQ_CYC, tmp_data, 3);
	if (ret!=0) return ret;

	*cycles=tmp_data[0];

	return 0;
}

uint8_t BL0942_set_frequency_update_cycles(BL0942_Handle *handle, BL0942_FrequencyUpdateCycles cycles) {
	tmp_data[0]=cycles;
	tmp_data[1]=0;
	tmp_data[2]=0;

	uint8_t ret=BL0942_write_reg(handle, BL0942_REG_FREQ_CYC, tmp_data, 3);

	return ret;
}

uint8_t BL0942_read_ot_funx(BL0942_Handle *handle, BL0942_OtFunx *status) {
	uint8_t ret=BL0942_read_reg(handle, BL0942_REG_OT_FUNX, tmp_data, 3);
	if (ret!=0) return ret;

	status->DATA=tmp_data[0];

	return 0;
}

uint8_t BL0942_set_ot_funx(BL0942_Handle *handle, BL0942_OtFunx status) {
	tmp_data[0]=status.DATA;
	tmp_data[1]=0;
	tmp_data[2]=0;

	uint8_t ret=BL0942_write_reg(handle, BL0942_REG_OT_FUNX, tmp_data, 3);

	return ret;
}

uint8_t BL0942_read_mode(BL0942_Handle *handle, BL0942_Mode *mode) {
	uint8_t ret=BL0942_read_reg(handle, BL0942_REG_MODE, tmp_data, 3);
	if (ret!=0) return ret;

	mode->DATA[0]=tmp_data[0];
	mode->DATA[1]=tmp_data[1];

	return 0;
}

uint8_t BL0942_set_mode(BL0942_Handle *handle, BL0942_Mode *mode) {
	tmp_data[0]=mode->DATA[0];
	tmp_data[1]=mode->DATA[1];
	tmp_data[2]=0;

	uint8_t ret=BL0942_write_reg(handle, BL0942_REG_MODE, tmp_data, 3);

	return ret;
}

uint8_t BL0942_read_i_gain(BL0942_Handle *handle, uint8_t *gain) {
	uint8_t ret=BL0942_read_reg(handle, BL0942_REG_GAIN_CR, tmp_data, 3);
	if (ret!=0) return ret;

	*gain=tmp_data[0];

	return 0;
}

uint8_t BL0942_set_i_gain(BL0942_Handle *handle, uint8_t gain) {
	tmp_data[0]=gain;
	tmp_data[1]=0;
	tmp_data[2]=0;

	uint8_t ret=BL0942_write_reg(handle, BL0942_REG_GAIN_CR, tmp_data, 3);

	return ret;
}

uint8_t BL0942_soft_reset(BL0942_Handle *handle) {
	tmp_data[0]=BL0942_SOFT_RESET_CODE;
	tmp_data[1]=BL0942_SOFT_RESET_CODE;
	tmp_data[2]=BL0942_SOFT_RESET_CODE;

	uint8_t ret=BL0942_write_reg(handle, BL0942_REG_SOFT_RESET, tmp_data, 3);

	return ret;
}

uint8_t BL0942_disable_write_protection(BL0942_Handle *handle) {
	tmp_data[0]=BL0942_UNLOCK_CODE;
	tmp_data[1]=0;
	tmp_data[2]=0;

	uint8_t ret=BL0942_write_reg(handle, BL0942_REG_USR_WRPROT, tmp_data, 3);

	return ret;
}
