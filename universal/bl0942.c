/**
  ******************************************************************************
  * @file    bl0942.c
  * @author  Juraj Lonc (juraj.lonc@gmail.com)
  * @brief   BL0942 driver.
  ******************************************************************************
  */

#include "bl0942.h"

uint8_t tmp_data[3];

/**
 * @brief  BL0942 initialization.
 * @param	handle*		pointer to BL0942_Handle
 * @param	write_reg	The function that writes command to the module. Hardware dependent.
 * @param	read_reg	The function that reads data from the module. Hardware dependent.
 * @param	delay_ms	The function that makes delay in milliseconds. Hardware dependent.
 * @retval	status		0=device found
 */
void BL0942_init_SPI(BL0942_Handle *handle, bl0942_write_ptr write_reg, bl0942_read_ptr read_reg) {
	handle->write_reg=write_reg;
	handle->read_reg=read_reg;
	handle->com_type=BL0942_COM_SPI;

//	obj->addr=addrpin==0?BL0942_I2C_ADDRESS_LOW:BL0942_I2C_ADDRESS_HIGH;
//
//	// after power-up: it takes 20 milliseconds at most to enter idle state
//	obj->delay_ms(20);
//	uint8_t ret=obj->write_reg(obj, BL0942_CMD_INIT, 0, 0);
//	if (ret!=0) return ret;
//
//	obj->delay_ms(20);
//	BL0942_soft_reset(obj);
//	// Soft reset takes no more than 20 milliseconds
//	obj->delay_ms(20);
//	return 0;
}

void BL0942_init_UART(BL0942_Handle *handle, uint8_t addr1, uint8_t addr2, bl0942_write_ptr write_reg, bl0942_read_ptr read_reg) {
	handle->write_reg=write_reg;
	handle->read_reg=read_reg;
	handle->com_type=BL0942_COM_UART;
	handle->addr=0;
	if (addr1) handle->addr|=0b01;
	if (addr2) handle->addr|=0b10;

//	obj->addr=addrpin==0?BL0942_I2C_ADDRESS_LOW:BL0942_I2C_ADDRESS_HIGH;
//
//	// after power-up: it takes 20 milliseconds at most to enter idle state
//	obj->delay_ms(20);
//	uint8_t ret=obj->write_reg(obj, BL0942_CMD_INIT, 0, 0);
//	if (ret!=0) return ret;
//
//	obj->delay_ms(20);
//	BL0942_soft_reset(obj);
//	// Soft reset takes no more than 20 milliseconds
//	obj->delay_ms(20);
//	return 0;
}

static uint8_t BL0942_read_reg(BL0942_Handle *handle, uint8_t reg_addr, uint8_t *data, uint8_t count) {
	uint8_t tmp[3];
	tmp[0]=BL0942_ADDR_READ | handle->addr;
	tmp[1]=reg_addr;

	handle->write_reg(tmp, 2);
	uint8_t stat=handle->read_reg(data, count);
	stat=handle->read_reg(&tmp[2], 1);

	uint8_t chksum=BL0942_calculate_checksum(tmp, 2, data, count);
	if (tmp[2]!=chksum) return 10;

	return stat;
}

static uint8_t BL0942_write_reg(BL0942_Handle *handle, uint8_t reg_addr, uint8_t *data, uint8_t count) {
	uint8_t tmp[3];
	tmp[0]=BL0942_ADDR_WRITE | handle->addr;
	tmp[1]=reg_addr;

	uint8_t chksum=BL0942_calculate_checksum(tmp, 2, data, count);

	handle->write_reg(tmp, 2);
	handle->write_reg(data, count);
	uint8_t stat=handle->write_reg(&chksum, 1);

	return stat;
}

uint8_t BL0942_calculate_checksum(uint8_t *data1, uint8_t len1, uint8_t *data2, uint8_t len2) {
	uint8_t tmp=0;
	uint8_t q=0;

	for (q=0; q<len1; q++) {
		tmp+=data1[q];
	}

	for (q=0; q<len2; q++) {
		tmp+=data2[q];
	}

	tmp^=0xff;

	return tmp;
}

static void convert_signed24_value(uint8_t *data, int32_t *val) {
	bool neg=false;
	if (data[0] & 0x80) {
		neg=true;
		data[0]&=0x7f;
		data[0]^=0xff;
		data[1]^=0xff;
		data[2]^=0xff;
	}

	*val=data[0];
	*val<<=8;
	*val|=data[1];
	*val<<=8;
	*val|=data[2];

	if (neg) {
		*val=-*val;
	}
}

static void convert_signed20_value(uint8_t *data, int32_t *val) {
	bool neg=false;
	if (data[0] & 0x08) {
		neg=true;
		data[0]&=0x07;
		data[0]^=0xff;
		data[1]^=0xff;
		data[2]^=0xff;
	}

	*val=data[0];
	*val<<=8;
	*val|=data[1];
	*val<<=8;
	*val|=data[2];

	if (neg) {
		*val=-*val;
	}
}

static void convert_unsigned24_value(uint8_t *data, uint32_t *val) {
	*val=data[0];
	*val<<=8;
	*val|=data[1];
	*val<<=8;
	*val|=data[2];
}

static void convert_unsigned16_value(uint8_t *data, uint16_t *val) {
	*val=data[0];
	*val<<=8;
	*val|=data[1];
}

uint8_t BL0942_read_i_wave(BL0942_Handle *handle, int32_t *val) {
	uint8_t ret=BL0942_read_reg(handle, BL0942_REG_I_WAVE, tmp_data, 3);
	if (ret!=0) return ret;

	convert_signed20_value(tmp_data, val);

	return 0;
}

uint8_t BL0942_read_v_wave(BL0942_Handle *handle, int32_t *val) {
	uint8_t ret=BL0942_read_reg(handle, BL0942_REG_V_WAVE, tmp_data, 3);
	if (ret!=0) return ret;

	convert_signed20_value(tmp_data, val);

	return 0;
}


// Current RMS register equation:
// I = IP - IN, [mV]
// I_RMS_REG = 305978 * I / Vref
// I = I_RMS_REG * Vref / 305978
uint8_t BL0942_read_i_rms(BL0942_Handle *handle, uint32_t *val) {
	uint8_t ret=BL0942_read_reg(handle, BL0942_REG_I_RMS, tmp_data, 3);
	if (ret!=0) return ret;

	convert_unsigned24_value(tmp_data, val);

	return 0;
}


// Voltage RMS register equation:
// V = VP - GND, [mV]
// V_RMS_REG = 73989 * V / Vref
// V = V_RMS_REG * Vref / 73989 [mV]
uint8_t BL0942_read_v_rms(BL0942_Handle *handle, uint32_t *val) {
	uint8_t ret=BL0942_read_reg(handle, BL0942_REG_V_RMS, tmp_data, 3);
	if (ret!=0) return ret;

	convert_unsigned24_value(tmp_data, val);

	return 0;
}


// I_FAST_RMS_REG ~ I_RMS_REG * 0.363
uint8_t BL0942_read_i_fast_rms(BL0942_Handle *handle, uint32_t *val) {
	uint8_t ret=BL0942_read_reg(handle, BL0942_REG_I_FAST_RMS, tmp_data, 3);
	if (ret!=0) return ret;

	convert_unsigned24_value(tmp_data, val);

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

	convert_signed24_value(tmp_data, val);

	return 0;
}

// Cumulative time of each CF pulse:
// tCF = 1638.4 * 256 / WATT_REG
uint8_t BL0942_read_cf_count(BL0942_Handle *handle, uint32_t *val) {
	uint8_t ret=BL0942_read_reg(handle, BL0942_REG_CF_CNT, tmp_data, 3);
	if (ret!=0) return ret;

	convert_unsigned24_value(tmp_data, val);

	return 0;
}

uint8_t BL0942_read_frequency_reg(BL0942_Handle *handle, uint16_t *val) {
	uint8_t ret=BL0942_read_reg(handle, BL0942_REG_FREQ, tmp_data, 3);
	if (ret!=0) return ret;

	convert_unsigned16_value(tmp_data, val);

	return 0;
}

// f = 1000000 / FREQ_REG, [Hz]
// returns hundreths of Hz; value 5000 = 50.00 Hz
uint8_t BL0942_read_frequency(BL0942_Handle *handle, uint16_t *frequency) {
	uint8_t ret=BL0942_read_reg(handle, BL0942_REG_FREQ, tmp_data, 3);
	if (ret!=0) return ret;

	uint32_t val;
	convert_unsigned24_value(tmp_data, &val);
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

	status->DATA[0]=tmp_data[1];
	status->DATA[1]=tmp_data[2];

	return 0;
}

uint8_t BL0942_read_all(BL0942_Handle *handle, BL0942_AllData *data) {
	uint8_t ret=BL0942_read_reg(handle, BL0942_REG_READ_ALL, data->DATA, 23);
	if (ret!=0) return ret;

//	convert_unsigned16_value(data, val);

	return 0;
}

uint8_t BL0942_read_i_rms_offset(BL0942_Handle *handle, uint8_t *offset) {
	uint8_t ret=BL0942_read_reg(handle, BL0942_REG_I_RMSOS, tmp_data, 3);
	if (ret!=0) return ret;

	*offset=tmp_data[2];

	return 0;
}

uint8_t BL0942_set_i_rms_offset(BL0942_Handle *handle, uint8_t offset) {
	tmp_data[0]=0;
	tmp_data[1]=0;
	tmp_data[2]=offset;

	uint8_t ret=BL0942_write_reg(handle, BL0942_REG_I_RMSOS, tmp_data, 3);

	return ret;
}

uint8_t BL0942_read_anticreep_threshold(BL0942_Handle *handle, uint8_t *wa_creep) {
	uint8_t ret=BL0942_read_reg(handle, BL0942_REG_WA_CREEP, tmp_data, 3);
	if (ret!=0) return ret;

	*wa_creep=tmp_data[2];

	return 0;
}

// WA_CREEP = WATT_REG * 256 / 3125
// WATT_REG = WA_CREEP * 3125 / 256
uint8_t BL0942_set_anticreep_threshold(BL0942_Handle *handle, uint8_t wa_creep) {
	tmp_data[0]=0;
	tmp_data[1]=0;
	tmp_data[2]=wa_creep;

	uint8_t ret=BL0942_write_reg(handle, BL0942_REG_WA_CREEP, tmp_data, 3);

	return ret;
}

uint8_t BL0942_read_i_fast_rms_threshold(BL0942_Handle *handle, uint16_t *i_rms_threshold) {
	uint8_t ret=BL0942_read_reg(handle, BL0942_REG_I_FAST_RMS_TH, tmp_data, 3);
	if (ret!=0) return ret;

	convert_unsigned16_value(tmp_data, i_rms_threshold);

	return 0;
}

uint8_t BL0942_set_i_fast_rms_threshold(BL0942_Handle *handle, uint16_t *i_rms_threshold) {
	tmp_data[0]=0;
	tmp_data[1]=*i_rms_threshold>>8;
	tmp_data[2]=*i_rms_threshold & 0xff;

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
	tmp_data[0]=0;
	tmp_data[1]=0;
	tmp_data[2]=cycles;

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
	tmp_data[0]=0;
	tmp_data[1]=0;
	tmp_data[2]=cycles;

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
	tmp_data[0]=0;
	tmp_data[1]=0;
	tmp_data[2]=status.DATA;

	uint8_t ret=BL0942_write_reg(handle, BL0942_REG_OT_FUNX, tmp_data, 3);

	return ret;
}

uint8_t BL0942_read_mode(BL0942_Handle *handle, BL0942_Mode *mode) {
	uint8_t ret=BL0942_read_reg(handle, BL0942_REG_MODE, tmp_data, 3);
	if (ret!=0) return ret;

	mode->DATA[0]=tmp_data[2];
	mode->DATA[1]=tmp_data[1];

	return 0;
}

uint8_t BL0942_set_mode(BL0942_Handle *handle, BL0942_Mode *mode) {
	tmp_data[0]=0;
	tmp_data[1]=mode->DATA[1];
	tmp_data[2]=mode->DATA[0];

	uint8_t ret=BL0942_write_reg(handle, BL0942_REG_MODE, tmp_data, 3);

	return ret;
}

uint8_t BL0942_read_i_gain(BL0942_Handle *handle, uint8_t *gain) {
	uint8_t ret=BL0942_read_reg(handle, BL0942_REG_GAIN_CR, tmp_data, 3);
	if (ret!=0) return ret;

	*gain=tmp_data[2];

	return 0;
}

uint8_t BL0942_set_i_gain(BL0942_Handle *handle, uint8_t gain) {
	tmp_data[0]=0;
	tmp_data[1]=0;
	tmp_data[2]=gain;

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
	tmp_data[0]=0;
	tmp_data[1]=0;
	tmp_data[2]=BL0942_UNLOCK_CODE;

	uint8_t ret=BL0942_write_reg(handle, BL0942_REG_USR_WRPROT, tmp_data, 3);

	return ret;
}
