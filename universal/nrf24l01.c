/* Based on:
 */
/* Copyright (c) 2009 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is confidential property of Nordic
 * Semiconductor ASA.Terms and conditions of usage are described in detail
 * in NORDIC SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRENTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 */
/*
   Copyright (c) 2015,2016,2020 Piotr Stolarz for the Ardiono port

   This software is distributed WITHOUT ANY WARRANTY; without even the
   implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
   See the License for more information.
 */


#include "nrf24l01.h"


/* send and receive multiple bytes over SPI */
static void nrf24_spi_transceive(NRF24_t *nrf24, uint8_t* dataout, uint8_t* datain, uint8_t len);
/* send multiple bytes over SPI */
static void nrf24_spi_transmit(NRF24_t *nrf24, const uint8_t* dataout, uint8_t len);
/* Write to a single register of nrf24 */
static void nrf24_write_register_byte(NRF24_t *nrf24, uint8_t reg, uint8_t value);
/* Read single register from nrf24 */
static uint8_t nrf24_read_register_byte(NRF24_t *nrf24, uint8_t reg);
/* Read single register from nrf24 */
static void nrf24_read_register_multibyte(NRF24_t *nrf24, uint8_t reg, uint8_t* value, uint8_t len);
/* Write to a single register of nrf24 */
static void nrf24_write_register_multibyte(NRF24_t *nrf24, uint8_t reg, const uint8_t* value, uint8_t len);



/* send and receive multiple bytes over SPI */
static void nrf24_spi_transceive(NRF24_t *nrf24, uint8_t* dataout, uint8_t* datain, uint8_t len) {
	uint8_t i;

	for (i = 0; i < len; i++) {
		datain[i] = nrf24->spi_transfer(dataout[i]);
	}
}

/* send multiple bytes over SPI */
static void nrf24_spi_transmit(NRF24_t *nrf24, const uint8_t* dataout, uint8_t len) {
	uint8_t i;

	for (i = 0; i < len; i++) {
		nrf24->spi_transfer(dataout[i]);
	}
}

/* Write to a single register of nrf24 */
static void nrf24_write_register_byte(NRF24_t *nrf24, uint8_t reg, uint8_t value) {
	nrf24->set_csn(0);
	nrf24->spi_transfer(NRF24_CMD_W_REGISTER | (NRF24_REGISTER_MASK & reg));
	nrf24->spi_transfer(value);
	nrf24->set_csn(1);
}

/* Read single register from nrf24 */
static uint8_t nrf24_read_register_byte(NRF24_t *nrf24, uint8_t reg) {
	nrf24->set_csn(0);
	nrf24->spi_transfer(NRF24_CMD_R_REGISTER | (NRF24_REGISTER_MASK & reg));
	uint8_t val=nrf24->spi_transfer(0);
	nrf24->set_csn(1);
	return val;
}

/* Read single register from nrf24 */
static void nrf24_read_register_multibyte(NRF24_t *nrf24, uint8_t reg, uint8_t* value, uint8_t len) {
	nrf24->set_csn(0);
	nrf24->spi_transfer(reg);
	nrf24_spi_transceive(nrf24, value, value, len);
	nrf24->set_csn(1);
}

/* Write to a single register of nrf24 */
static void nrf24_write_register_multibyte(NRF24_t *nrf24, uint8_t reg, const uint8_t* value, uint8_t len) {
	nrf24->set_csn(0);
	nrf24->spi_transfer(reg);
	nrf24_spi_transmit(nrf24, value, len);
	nrf24->set_csn(1);
}




uint8_t nrf24_read_rx_payload(NRF24_t *nrf24, uint8_t *buf) {
	uint8_t src=nrf24_get_rx_data_source(nrf24);
	uint8_t len=0;

	if (src < 7)
		len = nrf24_read_rx_payload_width(nrf24);
	else
		len = 0;

	if (len>0) {
		nrf24_read_register_multibyte(nrf24, NRF24_CMD_R_RX_PAYLOAD, buf, len);
	}

	return len;
}

void nrf24_write_tx_payload(NRF24_t *nrf24, const void* buf, uint8_t len) {
//	uint8_t data_len = len < nrf24->payload_len ? len : nrf24->payload_len; // min(len, payload_len);
//	uint8_t blank_len = nrf24->dynamic_payloads_enabled ? 0 : nrf24->payload_len - data_len;
//
//
//	nrf24->set_csn(0);
//
//	/* Write cmd to write payload */
//	nrf24->spi_transfer(NRF24_CMD_W_TX_PAYLOAD);
//
//	/* Write payload */
//	nrf24_spi_transmit(nrf24, (uint8_t *)buf, data_len);
//
//	while (blank_len--) nrf24->spi_transfer(0);
//
//	nrf24->set_csn(1);

		nrf24->set_csn(0);

		/* Write cmd to write payload */
		nrf24->spi_transfer(NRF24_CMD_W_TX_PAYLOAD);

		/* Write payload */
		nrf24_spi_transmit(nrf24, (uint8_t *)buf, len);

		nrf24->set_csn(1);

}

void nrf24_flush_tx(NRF24_t *nrf24) {
	nrf24->set_csn(0);
	nrf24->spi_transfer(NRF24_CMD_FLUSH_TX);
	nrf24->set_csn(1);
}

void nrf24_flush_rx(NRF24_t *nrf24) {
	nrf24->set_csn(0);
	nrf24->spi_transfer(NRF24_CMD_FLUSH_RX);
	nrf24->set_csn(1);
}

void nrf24_reuse_tx_payload(NRF24_t *nrf24) {
	nrf24->set_csn(0);
	nrf24->spi_transfer(NRF24_CMD_REUSE_TX_PL);
	nrf24->set_csn(1);
}

uint8_t nrf24_read_rx_payload_width(NRF24_t *nrf24) {
	uint8_t val;
	nrf24->set_csn(0);
	nrf24->spi_transfer(NRF24_CMD_R_RX_PL_WID);
	val = nrf24->spi_transfer(0);
	nrf24->set_csn(1);

	return val;
}

void nrf24_write_ack_payload(NRF24_t *nrf24, uint8_t pipe_num, const void* tx_pload, uint8_t len) {
//	uint8_t data_len = len < nrf24->payload_len ? len : nrf24->payload_len; // min(len, payload_len);
//	uint8_t blank_len = nrf24->dynamic_payloads_enabled ? 0 : nrf24->payload_len - data_len;
//
//
//	nrf24->set_csn(0);
//
//	/* Write cmd to write payload */
//	nrf24->spi_transfer(NRF24_CMD_W_ACK_PAYLOAD | pipe_num);
//
//	/* Write payload */
//	nrf24_spi_transmit(nrf24, (uint8_t *)tx_pload, data_len);
//
//	while (blank_len--) nrf24->spi_transfer(0);
//
//	nrf24->set_csn(1);

	nrf24->set_csn(0);

		/* Write cmd to write payload */
		nrf24->spi_transfer(NRF24_CMD_W_ACK_PAYLOAD | pipe_num);

		/* Write payload */
		nrf24_spi_transmit(nrf24, (uint8_t *)tx_pload, len);

		nrf24->set_csn(1);

}

void nrf24_write_tx_payload_noack(NRF24_t *nrf24, const void* buf, uint8_t len) {
//	uint8_t data_len = len < nrf24->payload_len ? len : nrf24->payload_len; // min(len, payload_len);
//	uint8_t blank_len = nrf24->dynamic_payloads_enabled ? 0 : nrf24->payload_len - data_len;
//
//
//	nrf24->set_csn(0);
//
//	/* Write cmd to write payload */
//	nrf24->spi_transfer(NRF24_CMD_W_TX_PAYLOAD_NO_ACK);
//
//	/* Write payload */
//	nrf24_spi_transmit(nrf24, (uint8_t *)buf, data_len);
//
//	while (blank_len--) nrf24->spi_transfer(0);
//
//	nrf24->set_csn(1);

	nrf24->set_csn(0);

	/* Write cmd to write payload */
	nrf24->spi_transfer(NRF24_CMD_W_TX_PAYLOAD_NO_ACK);

	/* Write payload */
	nrf24_spi_transmit(nrf24, (uint8_t *)buf, len);

	nrf24->set_csn(1);

}

NRF24_Register_Status nrf24_get_status(NRF24_t *nrf24) {
	NRF24_Register_Status status;
	nrf24->set_csn(0);
	status.DATA = nrf24->spi_transfer(NRF24_CMD_NOP);
	nrf24->set_csn(1);

	return status;
}

NRF24_Register_Config nrf24_get_config(NRF24_t *nrf24) {
	NRF24_Register_Config c;
	c.DATA=nrf24_read_register_byte(nrf24, NRF24_REG_CONFIG);

	return c;
}

void nrf24_set_crc_mode(NRF24_t *nrf24, NRF24_CRC_Mode crc) {
	NRF24_Register_Config c;
//	nrf24_read_register(nrf24, NRF24_REG_CONFIG, &(c.DATA), 1);
	c.DATA=nrf24_read_register_byte(nrf24, NRF24_REG_CONFIG);

	if (crc == NRF24_CRC_DISABLED) {
		c.EN_CRC = false;
		c.CRCO = false;
	} else {
		c.EN_CRC = true;
		if (crc == NRF24_CRC_16) {
			c.CRCO = true;
		} else {
			c.CRCO = false;
		}
	}

	nrf24_write_register_byte(nrf24, NRF24_REG_CONFIG, c.DATA);
}

NRF24_CRC_Mode nrf24_get_crc_mode(NRF24_t *nrf24) {
	NRF24_Register_Config c;
//	nrf24_read_register(nrf24, NRF24_REG_CONFIG, &(c.DATA), 1);
	c.DATA=nrf24_read_register_byte(nrf24, NRF24_REG_CONFIG);

	if (!c.EN_CRC) {
		return NRF24_CRC_DISABLED;
	} else {
		if (c.CRCO) {
			return NRF24_CRC_16;
		} else {
			return NRF24_CRC_8;
		}
	}
}

void nrf24_set_operation_mode(NRF24_t *nrf24, NRF24_Operation_Mode op_mode) {
	NRF24_Register_Config c;
	c.DATA=nrf24_read_register_byte(nrf24, NRF24_REG_CONFIG);
	c.PRIM_RX=op_mode;
	nrf24_write_register_byte(nrf24, NRF24_REG_CONFIG, c.DATA);
}

NRF24_Operation_Mode nrf24_get_operation_mode(NRF24_t *nrf24) {
	NRF24_Register_Config c;
	c.DATA=nrf24_read_register_byte(nrf24, NRF24_REG_CONFIG);
	return c.PRIM_RX;
}

void nrf24_set_irq_mode(NRF24_t *nrf24, NRF24_IRQ_Source int_source, bool irq_enabled) {
	NRF24_Register_Config c;
	c.DATA=nrf24_read_register_byte(nrf24, NRF24_REG_CONFIG);

	switch (int_source) {
	case NRF24_MAX_RT:
		c.MASK_MAX_RT=!irq_enabled;
		break;
	case NRF24_TX_DS:
		c.MASK_TX_DS=!irq_enabled;
		break;
	case NRF24_RX_DR:
		c.MASK_RX_DR=!irq_enabled;
		break;
	}
	nrf24_write_register_byte(nrf24, NRF24_REG_CONFIG, c.DATA);
}

bool nrf24_get_irq_mode(NRF24_t *nrf24, NRF24_IRQ_Source int_source) {
	NRF24_Register_Config c;
//	nrf24_read_register(nrf24, NRF24_REG_CONFIG, &(c.DATA), 1);
	c.DATA=nrf24_read_register_byte(nrf24, NRF24_REG_CONFIG);

	switch (int_source) {
	case NRF24_MAX_RT:
		return !c.MASK_MAX_RT;
		break;
	case NRF24_TX_DS:
		return !c.MASK_TX_DS;
		break;
	case NRF24_RX_DR:
		return !c.MASK_RX_DR;
		break;
	}
	return false;
}

void nrf24_set_power_mode(NRF24_t *nrf24, NRF24_Power_Mode pwr_mode) {
	NRF24_Register_Config c;
//	nrf24_read_register(nrf24, NRF24_REG_CONFIG, &(c.DATA), 1);
	c.DATA=nrf24_read_register_byte(nrf24, NRF24_REG_CONFIG);

	c.PWR_UP=pwr_mode;

	nrf24_write_register_byte(nrf24, NRF24_REG_CONFIG, c.DATA);
}

NRF24_Power_Mode hal_nrf_get_power_mode(NRF24_t *nrf24) {
	NRF24_Register_Config c;
	c.DATA=nrf24_read_register_byte(nrf24, NRF24_REG_CONFIG);
	return (NRF24_Power_Mode)c.PWR_UP;
}

void nrf24_set_address_width(NRF24_t *nrf24, NRF24_Address_Width address_width) {
    uint8_t setup_aw = ((uint8_t)address_width-2) & 0b00000011;
    nrf24_write_register_byte(nrf24, NRF24_REG_SETUP_AW, setup_aw);
}

uint8_t nrf24_get_address_width(NRF24_t *nrf24) {
    return nrf24_read_register_byte(nrf24, NRF24_REG_SETUP_AW)+2;
}

bool nrf24_received_power_detected(NRF24_t *nrf24) {
	NRF24_Register_ReceivedPowerDetector c;
	c.DATA=nrf24_read_register_byte(nrf24, NRF24_REG_RPD);
	return c.RPD;
}

void nrf24_set_rf_channel(NRF24_t *nrf24, uint8_t channel) {
	NRF24_Register_RFChannel ch;
	ch.RF_CH = channel;
	nrf24_write_register_byte(nrf24, NRF24_REG_RF_CH, ch.DATA);
}

uint8_t nrf24_get_rf_channel(NRF24_t *nrf24) {
	NRF24_Register_RFChannel c;
	c.DATA=nrf24_read_register_byte(nrf24, NRF24_REG_RF_CH);
	return c.RF_CH;
}

void nrf24_set_auto_retry(NRF24_t *nrf24, uint16_t retry_delay, uint8_t retry_count) {
	NRF24_Register_SetupRetransmission s;
	s.ARC = retry_count;
	s.ARD = (retry_delay >> 8) & 0x0f;

	nrf24_write_register_byte(nrf24, NRF24_REG_SETUP_RETR, s.DATA);
}

uint8_t nrf24_get_auto_retry_count(NRF24_t *nrf24) {
	NRF24_Register_SetupRetransmission c;
	c.DATA=nrf24_read_register_byte(nrf24, NRF24_REG_SETUP_RETR);
	return c.ARC;
}

uint16_t hal_nrf_get_auto_retry_delay(NRF24_t *nrf24) {
	NRF24_Register_SetupRetransmission s;
	s.DATA=nrf24_read_register_byte(nrf24, NRF24_REG_SETUP_RETR);

	return (uint16_t)(s.ARD+1)*250;
}

NRF24_Register_FIFOStatus nrf24_get_fifo_status(NRF24_t *nrf24) {
	NRF24_Register_FIFOStatus s;
	s.DATA=nrf24_read_register_byte(nrf24, NRF24_REG_FIFO_STATUS);
	return s;
}

NRF24_PowerOutputLevel nrf24_get_power_output_level(NRF24_t *nrf24) {
	NRF24_Register_RFSetup s;
	s.DATA=nrf24_read_register_byte(nrf24, NRF24_REG_RF_SETUP);
	return s.RF_PWR;
}

void nrf24_set_power_output_level(NRF24_t *nrf24, NRF24_PowerOutputLevel power) {
	NRF24_Register_RFSetup s;
	s.DATA=nrf24_read_register_byte(nrf24, NRF24_REG_RF_SETUP);

	s.RF_PWR = power;

	nrf24_write_register_byte(nrf24, NRF24_REG_RF_SETUP, s.DATA);
}

void nrf24_set_air_datarate(NRF24_t *nrf24, NRF24_AirDataRate air_data_rate) {
	NRF24_Register_RFSetup s;
	s.DATA=nrf24_read_register_byte(nrf24, NRF24_REG_RF_SETUP);

	if (air_data_rate == NRF24_AIR_DATA_RATE_250KBPS) {
		s.RF_DR_LOW = 1;
		s.RF_DR_HIGH = 0;
	} else {
		s.RF_DR_LOW = 0;
		if (air_data_rate == NRF24_AIR_DATA_RATE_1MBPS) {
			s.RF_DR_HIGH = 0;
		} else {
			// 2Mbps
			s.RF_DR_HIGH = 1;
		}
	}

	nrf24_write_register_byte(nrf24, NRF24_REG_RF_SETUP, s.DATA);
}

NRF24_AirDataRate nrf24_get_air_datarate(NRF24_t *nrf24) {
	NRF24_Register_RFSetup s;
	s.DATA=nrf24_read_register_byte(nrf24, NRF24_REG_RF_SETUP);

	if (s.RF_DR_LOW == 1) {
		return NRF24_AIR_DATA_RATE_250KBPS;
	} else {
		if (s.RF_DR_HIGH == 0) {
			return NRF24_AIR_DATA_RATE_1MBPS;
		} else {
			return NRF24_AIR_DATA_RATE_2MBPS;
		}
	}
}

void nrf24_enable_continious_wave(NRF24_t *nrf24, bool enable) {
	NRF24_Register_RFSetup s;
	s.DATA=nrf24_read_register_byte(nrf24, NRF24_REG_RF_SETUP);

	s.CONT_WAVE=enable;

	nrf24_write_register_byte(nrf24, NRF24_REG_RF_SETUP, s.DATA);
}

bool nrf24_is_continious_wave_enabled(NRF24_t *nrf24) {
	NRF24_Register_RFSetup s;
	s.DATA=nrf24_read_register_byte(nrf24, NRF24_REG_RF_SETUP);

	return s.CONT_WAVE;
}

void nrf24_set_pll_mode(NRF24_t *nrf24, bool pll_lock) {
	NRF24_Register_RFSetup s;
	s.DATA=nrf24_read_register_byte(nrf24, NRF24_REG_RF_SETUP);

	s.PLL_LOCK=pll_lock;

    nrf24_write_register_byte(nrf24, NRF24_REG_RF_SETUP, s.DATA);
}

bool nrf24_get_pll_mode(NRF24_t *nrf24) {
	NRF24_Register_RFSetup s;
	s.DATA=nrf24_read_register_byte(nrf24, NRF24_REG_RF_SETUP);
	return s.PLL_LOCK;
}

NRF24_Register_TransmitObserve nrf24_get_auto_retry_status(NRF24_t *nrf24) {
	NRF24_Register_TransmitObserve s;
	s.DATA=nrf24_read_register_byte(nrf24, NRF24_REG_OBSERVE_TX);
	return s;
}

uint8_t nrf24_get_retransmit_attempts(NRF24_t *nrf24) {
	NRF24_Register_TransmitObserve s;
	s.DATA=nrf24_read_register_byte(nrf24, NRF24_REG_OBSERVE_TX);
	return s.ARC_CNT;
}

uint8_t nrf24_get_packet_lost_count(NRF24_t *nrf24) {
	NRF24_Register_TransmitObserve s;
	s.DATA=nrf24_read_register_byte(nrf24, NRF24_REG_OBSERVE_TX);
	return s.PLOS_CNT;
}

void nrf24_clear_irq_flags(NRF24_t *nrf24) {
	nrf24_write_register_byte(nrf24, NRF24_REG_STATUS, NRF24_STATUS_IRQ_FLAGS);
}

void nrf24_clear_irq_flag(NRF24_t *nrf24, NRF24_IRQ_Source int_source) {
    nrf24_write_register_byte(nrf24, NRF24_REG_STATUS, int_source);
}

uint8_t nrf24_get_irq_flags(NRF24_t *nrf24) {
	return nrf24_read_register_byte(nrf24, NRF24_REG_STATUS) & NRF24_STATUS_IRQ_FLAGS;
}

bool nrf24_rx_fifo_empty(NRF24_t *nrf24) {
	NRF24_Register_FIFOStatus s;
	s.DATA=nrf24_read_register_byte(nrf24, NRF24_REG_FIFO_STATUS);
	return s.RX_EMPTY;
}

bool nrf24_rx_fifo_full(NRF24_t *nrf24) {
	NRF24_Register_FIFOStatus s;
	s.DATA=nrf24_read_register_byte(nrf24, NRF24_REG_FIFO_STATUS);
	return s.RX_FULL;
}

bool nrf24_tx_fifo_empty(NRF24_t *nrf24) {
	NRF24_Register_FIFOStatus s;
	s.DATA=nrf24_read_register_byte(nrf24, NRF24_REG_FIFO_STATUS);
	return s.TX_EMPTY;
}

bool nrf24_tx_fifo_full(NRF24_t *nrf24) {
	NRF24_Register_FIFOStatus s;
	s.DATA=nrf24_read_register_byte(nrf24, NRF24_REG_FIFO_STATUS);
	return s.TX_FULL;
}

bool nrf24_get_reuse_tx_status(NRF24_t *nrf24) {
	NRF24_Register_FIFOStatus s;
	s.DATA=nrf24_read_register_byte(nrf24, NRF24_REG_FIFO_STATUS);
	return s.TX_REUSE;
}

void nrf24_enable_dynamic_payload(NRF24_t *nrf24, bool enable) {
	NRF24_Register_Feature feature;
	feature.DATA=nrf24_read_register_byte(nrf24, NRF24_REG_FEATURE);
	feature.EN_DPL=enable;

	nrf24_write_register_byte(nrf24, NRF24_REG_FEATURE, feature.DATA);
}

bool nrf24_is_dynamic_payload_enabled(NRF24_t *nrf24) {
	NRF24_Register_Feature s;
	s.DATA=nrf24_read_register_byte(nrf24, NRF24_REG_FEATURE);
	return s.EN_DPL;
}

void nrf24_enable_ack_payload(NRF24_t *nrf24, bool enable) {
	NRF24_Register_Feature feature;
	feature.DATA=nrf24_read_register_byte(nrf24, NRF24_REG_FEATURE);
	feature.EN_ACK_PAY=enable;

	nrf24_write_register_byte(nrf24, NRF24_REG_FEATURE, feature.DATA);
}

bool nrf24_is_ack_payload_enabled(NRF24_t *nrf24) {
	NRF24_Register_Feature s;
	s.DATA=nrf24_read_register_byte(nrf24, NRF24_REG_FEATURE);
	return s.EN_ACK_PAY;
}

void nrf24_enable_dynamic_ack(NRF24_t *nrf24, bool enable) {
	NRF24_Register_Feature feature;
	feature.DATA=nrf24_read_register_byte(nrf24, NRF24_REG_FEATURE);
	feature.EN_DYN_ACK=enable;

	nrf24_write_register_byte(nrf24, NRF24_REG_FEATURE, feature.DATA);
}

bool nrf24_is_dynamic_ack_enabled(NRF24_t *nrf24) {
	NRF24_Register_Feature s;
	s.DATA=nrf24_read_register_byte(nrf24, NRF24_REG_FEATURE);
	return s.EN_DYN_ACK;
}

void nrf24_setup_dynamic_payload(NRF24_t *nrf24, NRF24_Register_EnableDynamicPaylodLength setup) {
	nrf24_write_register_byte(nrf24, NRF24_REG_DYNPD, setup.DATA);
}

void nrf24_set_rx_payload_width(NRF24_t *nrf24, uint8_t pipe_num, uint8_t pload_width) {
	nrf24_write_register_byte(nrf24, NRF24_REG_RX_PW_P0+pipe_num, pload_width);
}

uint8_t nrf24_get_rx_payload_width(NRF24_t *nrf24, uint8_t pipe_num) {
    return nrf24_read_register_byte(nrf24, NRF24_REG_RX_PW_P0+pipe_num);
}

uint8_t nrf24_get_rx_data_source(NRF24_t *nrf24) {
	NRF24_Register_Status s;
	s.DATA=nrf24_read_register_byte(nrf24, NRF24_REG_STATUS);
    return s.RX_P_NO;
}

void nrf24_set_address(NRF24_t *nrf24, const NRF24_Address address, const uint8_t *addr) {
	switch(address) {
	case NRF24_TX:
	case NRF24_PIPE0:
	case NRF24_PIPE1:
		nrf24_write_register_multibyte(nrf24, NRF24_CMD_W_REGISTER+NRF24_REG_RX_ADDR_P0+(uint8_t)address, addr, nrf24_get_address_width(nrf24));
		break;
	case NRF24_PIPE2:
	case NRF24_PIPE3:
	case NRF24_PIPE4:
	case NRF24_PIPE5:
		nrf24_write_register_byte(nrf24, NRF24_REG_RX_ADDR_P0 + (uint8_t)address, *addr);
		break;

	case NRF24_ALL:
	default:
		break;
	}
}

uint8_t nrf24_get_address(NRF24_t *nrf24, NRF24_Address address, uint8_t *addr) {
	uint8_t len;
	switch (address)
	{
	case NRF24_PIPE0:
	case NRF24_PIPE1:
	case NRF24_TX:
		len=nrf24_get_address_width(nrf24);
		nrf24_read_register_multibyte(nrf24, NRF24_REG_RX_ADDR_P0 + (uint8_t)address, addr, len);
		return len;
	default:
		*addr = nrf24_read_register_byte(nrf24, NRF24_REG_RX_ADDR_P0 + (uint8_t)address);
		return 1;
	}
}

void nrf24_open_pipe(NRF24_t *nrf24, NRF24_Address pipe, bool auto_ack) {
	NRF24_Register_EnabledRXAddr en_rxaddr;
	NRF24_Register_EnableAA en_aa;

	en_rxaddr.DATA=nrf24_read_register_byte(nrf24, NRF24_REG_EN_RXADDR);
	en_aa.DATA=nrf24_read_register_byte(nrf24, NRF24_REG_EN_AA);

//	switch(pipe) {
//	case NRF24_PIPE0:
//		en_rxaddr.ERX_P0=true;
//		en_aa.ENAA_P0=auto_ack;
//		break;
//	case NRF24_PIPE1:
//		en_rxaddr.ERX_P1=true;
//		en_aa.ENAA_P1=auto_ack;
//		break;
//	case NRF24_PIPE2:
//		en_rxaddr.ERX_P2=true;
//		en_aa.ENAA_P2=auto_ack;
//		break;
//	case NRF24_PIPE3:
//		en_rxaddr.ERX_P3=true;
//		en_aa.ENAA_P3=auto_ack;
//		break;
//	case NRF24_PIPE4:
//		en_rxaddr.ERX_P4=true;
//		en_aa.ENAA_P4=auto_ack;
//		break;
//	case NRF24_PIPE5:
//		en_rxaddr.ERX_P5=true;
//		en_aa.ENAA_P5=auto_ack;
//		break;
//
//	case NRF24_ALL:
//		en_rxaddr = (NRF24_Register_EnabledRXAddr)NRF24_ALL_PIPES;
//
//		if(auto_ack) {
//			en_aa = (NRF24_Register_EnableAA)NRF24_ALL_PIPES;
//		} else {
//			en_aa = (NRF24_Register_EnableAA)0;
//		}
//		break;
//
//	case NRF24_TX:
//	default:
//		return;
//	}

	switch(pipe) {
	case NRF24_PIPE0:
	case NRF24_PIPE1:
	case NRF24_PIPE2:
	case NRF24_PIPE3:
	case NRF24_PIPE4:
	case NRF24_PIPE5:
		en_rxaddr.DATA |= (1 << pipe);
		if (auto_ack) {
			en_aa.DATA |= (1 << pipe);
		} else {
			en_aa.DATA &= ~(1 << pipe);
		}
		break;

	case NRF24_ALL:
		en_rxaddr.DATA = NRF24_ALL_PIPES;

		if(auto_ack) {
			en_aa.DATA = NRF24_ALL_PIPES;
		} else {
			en_aa.DATA = 0;
		}
		break;

	case NRF24_TX:
	default:
		return;
	}

	nrf24_write_register_byte(nrf24, NRF24_REG_EN_RXADDR, en_rxaddr.DATA);
	nrf24_write_register_byte(nrf24, NRF24_REG_EN_AA, en_aa.DATA);
}

void nrf24_close_pipe(NRF24_t *nrf24, NRF24_Address pipe) {
	NRF24_Register_EnabledRXAddr en_rxaddr;
	NRF24_Register_EnableAA en_aa;

	en_rxaddr.DATA=nrf24_read_register_byte(nrf24, NRF24_REG_EN_RXADDR);
	en_aa.DATA=nrf24_read_register_byte(nrf24, NRF24_REG_EN_AA);

//	switch(pipe) {
//	case NRF24_PIPE0:
//		en_rxaddr.ERX_P0=false;
//		en_aa.ENAA_P0=false;
//		break;
//	case NRF24_PIPE1:
//		en_rxaddr.ERX_P1=false;
//		en_aa.ENAA_P1=false;
//		break;
//	case NRF24_PIPE2:
//		en_rxaddr.ERX_P2=false;
//		en_aa.ENAA_P2=false;
//		break;
//	case NRF24_PIPE3:
//		en_rxaddr.ERX_P3=false;
//		en_aa.ENAA_P3=false;
//		break;
//	case NRF24_PIPE4:
//		en_rxaddr.ERX_P4=false;
//		en_aa.ENAA_P4=false;
//		break;
//	case NRF24_PIPE5:
//		en_rxaddr.ERX_P5=false;
//		en_aa.ENAA_P5=false;
//		break;
//	case NRF24_ALL:
//		en_rxaddr = (NRF24_Register_EnabledRXAddr)0;
//		en_aa = (NRF24_Register_EnableAA)0;
//		break;
//
//	case NRF24_TX:
//	default:
//		return;
//	}

	switch(pipe) {
		case NRF24_PIPE0:
		case NRF24_PIPE1:
		case NRF24_PIPE2:
		case NRF24_PIPE3:
		case NRF24_PIPE4:
		case NRF24_PIPE5:
			en_rxaddr.DATA &= ~(1 << pipe);
			en_aa.DATA &= ~(1 << pipe);
			break;

		case NRF24_ALL:
			en_rxaddr.DATA=0;
			en_aa.DATA=0;
			break;

		case NRF24_TX:
		default:
			return;
		}

	nrf24_write_register_byte(nrf24, NRF24_REG_EN_RXADDR, en_rxaddr.DATA);
	nrf24_write_register_byte(nrf24, NRF24_REG_EN_AA, en_aa.DATA);

}

NRF24_PipeStatus nrf24_get_pipe_status(NRF24_t *nrf24, NRF24_Address pipe) {
	NRF24_PipeStatus status;
	NRF24_Register_EnabledRXAddr en_rxaddr;
	NRF24_Register_EnableAA en_aa;

	en_rxaddr.DATA=nrf24_read_register_byte(nrf24, NRF24_REG_EN_RXADDR);
	en_aa.DATA=nrf24_read_register_byte(nrf24, NRF24_REG_EN_AA);

	status.RX_ADDR_ENABLED=(en_rxaddr.DATA & (1<<pipe))!=0;
	status.AUTO_ACK_ENABLED=(en_aa.DATA & (1<<pipe))!=0;

    return status;
}



void nrf24_debug(NRF24_t *nrf24, nrf24_debug_print_fn print_fn) {
	uint8_t q;
	char buf1[20];

	for (q = 0; q <= 0x1d; q++) {

		if ((q <= 0x17) || (q >= 0x1c)) {
			print_fn("REG ");
			char2hex((uint8_t*)buf1, q);
			print_fn(buf1);
			print_fn(": ");

			nrf24->set_csn(0);
			nrf24->spi_transfer(q);

			uint8_t val = nrf24->spi_transfer(0);

			char2hex((uint8_t*)buf1, val);
			print_fn(buf1);
			print_fn(" - ");
			char2binary((uint8_t*)buf1, val, true);
			print_fn(buf1);
			print_fn("\n");
			nrf24->set_csn(1);
            
#ifdef NRF24_DEBUG_DETAILS
			uint8_t addr[5];
			uint8_t w;

			switch (q) {
			case 0x00:
				print_fn("CONFIG - Configuration Register\n");
				print_fn("MASK_RX_DR=");
				print_fn((val & 0b01000000) ? "1 - hide int" : "0 - show int");
				print_fn("\n");
				print_fn("MASK_TX_DS=");
				print_fn((val & 0b00100000) ? "1 - hide int" : "0 - show int");
				print_fn("\n");
				print_fn("MASK_MAX_RT=");
				print_fn((val & 0b00010000) ? "1 - hide int" : "0 - show int");
				print_fn("\n");
				print_fn("EN_CRC=");
				print_fn((val & 0b00001000) ? "1" : "0");
				print_fn("\n");
				print_fn("CRCO=");
				print_fn((val & 0b0000100) ? "1 - 2 bytes" : "0 - 1 byte");
				print_fn("\n");
				print_fn("PWR_UP=");
				print_fn((val & 0b0000010) ? "1 - power up" : "0 - power down");
				print_fn("\n");
				print_fn("PRIM_RX=");
				print_fn((val & 0b0000001) ? "1 - PRX" : "0 - PTX");
				print_fn("\n");
				break;
			case 0x01:
				print_fn("EN_AA - Enable Auto Acknowledgment\n");
				print_fn("ENAA_P5=");
				print_fn((val & 0b00100000) ? "1" : "0");
				print_fn("\n");
				print_fn("ENAA_P4=");
				print_fn((val & 0b00010000) ? "1" : "0");
				print_fn("\n");
				print_fn("ENAA_P3=");
				print_fn((val & 0b00001000) ? "1" : "0");
				print_fn("\n");
				print_fn("ENAA_P2=");
				print_fn((val & 0b00000100) ? "1" : "0");
				print_fn("\n");
				print_fn("ENAA_P1=");
				print_fn((val & 0b00000010) ? "1" : "0");
				print_fn("\n");
				print_fn("ENAA_P0=");
				print_fn((val & 0b00000001) ? "1" : "0");
				print_fn("\n");
				break;
			case 0x02:
				print_fn("EN_RXADDR - Enabled RX Addresses\n");
				print_fn("ERX_P5=");
				print_fn((val & 0b00100000) ? "1" : "0");
				print_fn("\n");
				print_fn("ERX_P4=");
				print_fn((val & 0b00010000) ? "1" : "0");
				print_fn("\n");
				print_fn("ERX_P3=");
				print_fn((val & 0b00001000) ? "1" : "0");
				print_fn("\n");
				print_fn("ERX_P2=");
				print_fn((val & 0b00000100) ? "1" : "0");
				print_fn("\n");
				print_fn("ERX_P1=");
				print_fn((val & 0b00000010) ? "1" : "0");
				print_fn("\n");
				print_fn("ERX_P0=");
				print_fn((val & 0b00000001) ? "1" : "0");
				print_fn("\n");
				break;
			case 0x03:
				print_fn("SETUP_AW - Setup of Address Widths\n");
				print_fn("AW=");
				print_fn((val == 0) ? "00 - illegal" : (val == 1) ? "01 - 3 bytes" : (val == 2) ? "10 - 4 bytes" : "11 - 5 bytes");
				print_fn("\n");
				break;
			case 0x04:
				print_fn("SETUP_RETR - Setup of Automatic Retransmission\n");
				print_fn("ARD (Auto Retransmit Delay)=");
				long2string((uint8_t*)buf1, (val >> 4));
				print_fn(buf1);
				print_fn(", =");
				long2string((uint8_t*)buf1, ((val >> 4) + 1)*250L);
				print_fn(buf1);
				print_fn("us");
				print_fn("\n");
				print_fn("ARC (Auto Retransmit Count)=");
				long2string((uint8_t*)buf1, (val & 0x0f));
				print_fn(buf1);
				print_fn(", =");
				if (val == 0) {
					print_fn("disabled");
				} else {
					long2string((uint8_t*)buf1, (val & 0x0f));
					print_fn(buf1);
				}
				print_fn("\n");
				break;
			case 0x05:
				print_fn("RF_CH - RF Channel\n");
				print_fn("RF_CH=");
				long2string((uint8_t*)buf1, (val & 0b01111111));
				print_fn(buf1);
				print_fn("\n");
				break;
			case 0x06:
				print_fn("RF_SETUP - RF Setup Register\n");
				print_fn("CONT_WAVE (Enables continuous carrier transmit when high)=");
				print_fn(val & 0b10000000 ? "1" : "0");
				print_fn("\n");
				print_fn("RF_DR_LOW (Set RF Data Rate to 250kbps)=");
				print_fn(val & 0b00100000 ? "1" : "0");
				print_fn("\n");
				print_fn("PLL_LOCK (Force PLL lock signal. Only used in test)=");
				print_fn(val & 0b00010000 ? "1" : "0");
				print_fn("\n");
				print_fn("RF_DR_HIGH (Select between the high speed data rates)=");
				print_fn(val & 0b00001000 ? "1" : "0");
				print_fn("\n");
				uint8_t s = val & 0b00101000;
				print_fn("Speed=");
				print_fn(s == 0 ? "1Mbps" : s == 0b00001000 ? "2Mbps" : s == 0b00100000 ? "250kbps" : "Unknown");
				print_fn("\n");
				s = (val >> 1)&0b11;
				print_fn("Power=");
				print_fn(s == 0 ? "-18dBm" : s == 0b01 ? "-12dBm" : s == 0b10 ? "-6dBm" : "0dBm");
				print_fn("\n");

				break;
			case 0x07:
				print_fn("STATUS - Status Register\n");
				print_fn("RX_DR (Data Ready RX FIFO interrupt)=");
				print_fn((val & 0b01000000) ? "1" : "0");
				print_fn("\n");
				print_fn("TX_DS (Data Sent TX FIFO interrupt)=");
				print_fn((val & 0b00100000) ? "1" : "0");
				print_fn("\n");
				print_fn("MAX_RT (Maximum number of TX retransmits interrupt)=");
				print_fn((val & 0b00010000) ? "1" : "0");
				print_fn("\n");
				print_fn("RX_P_NO (Data pipe number)=");
				long2string((uint8_t*)buf1, (val >> 1) & 0b111);
				print_fn(buf1);
				print_fn("\n");
				print_fn("TX_FULL=");
				print_fn((val & 0b0000001) ? "1 - TX FIFO full" : "0 - Available locations in TX FIFO");
				print_fn("\n");
				break;
			case 0x08:
				print_fn("OBSERVE_TX - Transmit observe register\n");
				print_fn("PLOS_CNT (Count lost packets)=");
				long2string((uint8_t*)buf1, ((val >> 4) & 0x0f));
				print_fn(buf1);
				print_fn("\n");
				print_fn("ARC_CNT (Count retransmitted packets)=");
				long2string((uint8_t*)buf1, ((val) & 0x0f));
				print_fn(buf1);
				print_fn("\n");
				break;
			case 0x09:
				print_fn("RPD - Received Power Detector\n");
				print_fn("RPD=");
				print_fn((val & 0b00000001) ? "1" : "0");
				print_fn("\n");
				break;
			case 0x0a:
				print_fn("RX_ADDR_P0 - Receive address data pipe 0\n");

				nrf24_read_register_multibyte(nrf24, NRF24_REG_RX_ADDR_P0, addr, 5);

				for (w=0; w<5; w++) {
					char2hex((uint8_t*)buf1, addr[w]);
					print_fn((char*)buf1);
					print_fn(" ");
				}
				print_fn("\n");
				break;
			case 0x0b:
				print_fn("RX_ADDR_P1 - Receive address data pipe 1\n");
				nrf24_read_register_multibyte(nrf24, NRF24_REG_RX_ADDR_P1, addr, 5);
				for (w=0; w<5; w++) {
					char2hex((uint8_t*)buf1, addr[w]);
					print_fn((char*)buf1);
					print_fn(" ");
				}
				print_fn("\n");
				break;
			case 0x0c:
				print_fn("RX_ADDR_P2 - Receive address data pipe 2\n");
				nrf24_read_register_multibyte(nrf24, NRF24_REG_RX_ADDR_P2, addr, 1);
				char2hex((uint8_t*)buf1, addr[0]);
				print_fn((char*)buf1);
				print_fn("\n");
				break;
			case 0x0d:
				print_fn("RX_ADDR_P3 - Receive address data pipe 3\n");
				nrf24_read_register_multibyte(nrf24, NRF24_REG_RX_ADDR_P3, addr, 1);
				char2hex((uint8_t*)buf1, addr[0]);
				print_fn((char*)buf1);
				print_fn("\n");
				break;
			case 0x0e:
				print_fn("RX_ADDR_P4 - Receive address data pipe 4\n");
				nrf24_read_register_multibyte(nrf24, NRF24_REG_RX_ADDR_P4, addr, 1);
				char2hex((uint8_t*)buf1, addr[0]);
				print_fn((char*)buf1);
				print_fn("\n");
				break;
			case 0x0f:
				print_fn("RX_ADDR_P5 - Receive address data pipe 5\n");
				nrf24_read_register_multibyte(nrf24, NRF24_REG_RX_ADDR_P5, addr, 1);
				char2hex((uint8_t*)buf1, addr[0]);
				print_fn((char*)buf1);
				print_fn("\n");
				break;
			case 0x10:
				print_fn("TX_ADDR - Transmit address\n");

				nrf24_read_register_multibyte(nrf24, NRF24_REG_TX_ADDR, addr, 5);

				for (w=0; w<5; w++) {
					char2hex((uint8_t*)buf1, addr[w]);
					print_fn((char*)buf1);
					print_fn(" ");
				}
				print_fn("\n");
				break;
			case 0x11:
				print_fn("RX_PW_P0 - Number of bytes in RX payload in data pipe 0\n");
				print_fn("length=");
				long2string((uint8_t*)buf1, (val));
				print_fn(buf1);

				print_fn("\n");

				break;
			case 0x12:
				print_fn("RX_PW_P1 - Number of bytes in RX payload in data pipe 1\n");
				print_fn("length=");
				long2string((uint8_t*)buf1, (val));
				print_fn(buf1);

				print_fn("\n");

				break;
			case 0x13:
				print_fn("RX_PW_P2 - Number of bytes in RX payload in data pipe 2\n");
				print_fn("length=");
				long2string((uint8_t*)buf1, (val));
				print_fn(buf1);

				print_fn("\n");

				break;
			case 0x14:
				print_fn("RX_PW_P3 - Number of bytes in RX payload in data pipe 3\n");
				print_fn("length=");
				long2string((uint8_t*)buf1, (val));
				print_fn(buf1);

				print_fn("\n");

				break;
			case 0x15:
				print_fn("RX_PW_P4 - Number of bytes in RX payload in data pipe 4\n");
				print_fn("length=");
				long2string((uint8_t*)buf1, (val));
				print_fn(buf1);

				print_fn("\n");

				break;
			case 0x16:
				print_fn("RX_PW_P5 - Number of bytes in RX payload in data pipe 5\n");
				print_fn("length=");
				long2string((uint8_t*)buf1, (val));
				print_fn(buf1);

				print_fn("\n");

				break;
			case 0x17:
				print_fn("FIFO_STATUS - FIFO Status Register\n");
				print_fn("TX_REUSE=");
				print_fn(val & 0b01000000 ? "1" : "0");
				print_fn("\n");
				print_fn("TX_FULL (TX FIFO full flag)=");
				print_fn(val & 0b00100000 ? "1" : "0");
				print_fn("\n");
				print_fn("TX_EMPTY (TX FIFO empty flag)=");
				print_fn(val & 0b00010000 ? "1" : "0");
				print_fn("\n");
				print_fn("RX_FULL (RX FIFO full flag)=");
				print_fn(val & 0b00000010 ? "1" : "0");
				print_fn("\n");
				print_fn("RX_EMPTY (RX FIFO empty flag)=");
				print_fn(val & 0b00000001 ? "1" : "0");
				print_fn("\n");

				break;
			case 0x1c:
				print_fn("DYNPD - Enable dynamic payload length\n");
				print_fn("DPL_P5 (Enable dynamic payload length data pipe 5)=");
				print_fn(val & 0b00100000 ? "1" : "0");
				print_fn("\n");
				print_fn("DPL_P4 (Enable dynamic payload length data pipe 4)=");
				print_fn(val & 0b00010000 ? "1" : "0");
				print_fn("\n");
				print_fn("DPL_P3 (Enable dynamic payload length data pipe 3)=");
				print_fn(val & 0b00001000 ? "1" : "0");
				print_fn("\n");
				print_fn("DPL_P2 (Enable dynamic payload length data pipe 2)=");
				print_fn(val & 0b00000100 ? "1" : "0");
				print_fn("\n");
				print_fn("DPL_P1 (Enable dynamic payload length data pipe 1)=");
				print_fn(val & 0b00000010 ? "1" : "0");
				print_fn("\n");
				print_fn("DPL_P0 (Enable dynamic payload length data pipe 0)=");
				print_fn(val & 0b00000001 ? "1" : "0");
				print_fn("\n");

				break;
			case 0x1d:
				print_fn("FEATURE - Feature Register\n");
				print_fn("EN_DPL (Enables Dynamic Payload Length)=");
				print_fn(val & 0b00000100 ? "1" : "0");
				print_fn("\n");
				print_fn("EN_ACK_PAY (Enables Payload with ACK)=");
				print_fn(val & 0b00000010 ? "1" : "0");
				print_fn("\n");
				print_fn("EN_DYN_ACK (Enables the W_TX_PAYLOAD_NOACK command)=");
				print_fn(val & 0b00000001 ? "1" : "0");
				print_fn("\n");

				break;
			default:
				break;
			}
#endif
			print_fn("\n");
		}

	}
}

/* init the hardware pins */
// wait 100ms to properly power up, before initialization !
void nrf24_init(NRF24_t *nrf24, nrf24_setup_pins_ptr setup_pins, nrf24_spi_transfer_ptr spi_transfer, nrf24_csn_ptr set_csn, nrf24_ce_ptr set_ce, nrf24_hw_delay_us_ptr delay_us) {
//void nrf24_init(NRF24_t *nrf24, void (*setup_pins)(void), uint8_t (*spi_transfer)(const uint8_t), void (*set_csn)(uint8_t), void (*set_ce)(uint8_t), void (*delay_us)(uint16_t)) {
    nrf24->setup_pins=setup_pins;
    nrf24->spi_transfer=spi_transfer;
    nrf24->set_csn=set_csn;
    nrf24->set_ce=set_ce;
    nrf24->delay_us=delay_us;

    nrf24->setup_pins();
    nrf24->set_ce(0);
    nrf24->set_csn(1);

    nrf24_config(nrf24);
}

void nrf24_config(NRF24_t *nrf24) {
	// Set 1500us (500us is minimum for 32B payload at 250kbps) timeouts, to make testing a little easier
	nrf24_set_auto_retry(nrf24, 1500, 15);
    
    // transmitting sometimes stuck when bit 6 is set
    nrf24_write_register_byte(nrf24, NRF24_REG_RF_SETUP, 0);

	// Set PA level to maximum power
	nrf24_set_power_output_level(nrf24, NRF24_POWER_OUTPUT_0DBM);

	// Set the data rate to the slowest (and most reliable) speed
	nrf24_set_air_datarate(nrf24, NRF24_AIR_DATA_RATE_250KBPS);

	// Enable CRC and request 2-byte (16bit) CRC
	nrf24_set_crc_mode(nrf24, NRF24_CRC_16);

	// Enable dynamic payloads on all pipes
	nrf24_enable_ack_payload(nrf24, true);
	nrf24_enable_dynamic_payload(nrf24, true);
	nrf24_enable_dynamic_ack(nrf24, true);
	NRF24_Register_EnableDynamicPaylodLength dyn;
	dyn.DATA=NRF24_ALL_PIPES;
	nrf24_setup_dynamic_payload(nrf24, dyn);

	// enable all interrupt sources
	nrf24_set_irq_mode(nrf24, NRF24_TX_DS, true);
	nrf24_set_irq_mode(nrf24, NRF24_RX_DR, true);
	nrf24_set_irq_mode(nrf24, NRF24_MAX_RT, true);

	nrf24_flush_tx(nrf24);
	nrf24_flush_rx(nrf24);

	nrf24_clear_irq_flags(nrf24);

}

void nrf24_config_tx(NRF24_t *nrf24, const uint8_t *addr) {
    
	if (addr) {
		nrf24_set_address(nrf24, NRF24_TX, addr);
		nrf24_set_address(nrf24, NRF24_PIPE0, addr);
	}

	/* open RX pipe 0 for receiving ack */
	nrf24_open_pipe(nrf24, NRF24_PIPE0, true);
}

void nrf24_config_rx_pipe(NRF24_t *nrf24, NRF24_Address pipe, const uint8_t *addr, bool auto_ack, uint8_t pload_width) {
	nrf24_open_pipe(nrf24, pipe, auto_ack);

	if (addr) {
		nrf24_set_address(nrf24, pipe, addr);
	}
	nrf24_set_rx_payload_width(nrf24, (uint8_t)pipe, pload_width);
}

void nrf24_pulse_ce(NRF24_t *nrf24) {
	do {
		uint8_t count;
		count = 20U;
		nrf24->set_ce(1);
		while(count--) {}
		nrf24->set_ce(0);
	} while(false);
}
