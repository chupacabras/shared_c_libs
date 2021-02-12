#include "nrf24l01.h"

static uint8_t payload_len;
static bool dynamic_payloads_enabled;

/* init the hardware pins */
void nrf24_init(void) {
	nrf24_setupPins();
	nrf24_ce_digitalWrite(0);
	nrf24_csn_digitalWrite(1);
	payload_len=32;
}

NRF24_Register_Status nrf24_getStatus(void) {
	NRF24_Register_Status status;
	nrf24_csn_digitalWrite(0);
	status.DATA = nrf24_spi_transfer(NRF24_CMD_NOP);
	nrf24_csn_digitalWrite(1);
	return status;
}

/* Set status register */
void nrf24_setStatus(NRF24_Register_Status status) {
	nrf24_writeRegisterByte(NRF24_REG_STATUS, status.DATA);
}

void nrf24_powerUpRx(void) {
	NRF24_Register_Status status = nrf24_flushRx();
	status.RX_DR = 1;
	status.TX_DS = 1;
	status.MAX_RT = 1;

	nrf24_writeRegisterByte(NRF24_REG_STATUS, status.DATA);

	NRF24_Register_Config config;
	nrf24_readRegister(NRF24_REG_CONFIG, &(config.DATA), 1);
//	config.EN_CRC = 1;
	config.PWR_UP = 1;
	config.PRIM_RX = 1;

	nrf24_ce_digitalWrite(0);
	nrf24_writeRegisterByte(NRF24_REG_CONFIG, config.DATA);
	nrf24_ce_digitalWrite(1);

	// wait for the radio to come up (130us actually only needed)
	nrf24_hw_delay_us(130);
}

void nrf24_powerUpTx(void) {

	NRF24_Register_Status status;
	status.RX_DR = 1;
	status.TX_DS = 1;
	status.MAX_RT = 1;

	nrf24_writeRegisterByte(NRF24_REG_STATUS, status.DATA);

	NRF24_Register_Config config;
	nrf24_readRegister(NRF24_REG_CONFIG, &(config.DATA), 1);
//	config.EN_CRC = 1;
	config.PWR_UP = 1;
	config.PRIM_RX = 0;
	
	nrf24_writeRegisterByte(NRF24_REG_CONFIG, config.DATA);
	
	nrf24_ce_digitalWrite(1);
}

void nrf24_powerDown(void) {
	nrf24_ce_digitalWrite(0);
	nrf24_writeRegisterByte(NRF24_REG_CONFIG, NRF24_DEFAULT_CONFIG);
}

/* send and receive multiple bytes over SPI */
void nrf24_transferSync(uint8_t* dataout, uint8_t* datain, uint8_t len) {
	uint8_t i;

	for (i = 0; i < len; i++) {
		datain[i] = nrf24_spi_transfer(dataout[i]);
	}
}

/* send multiple bytes over SPI */
void nrf24_transmitSync(uint8_t* dataout, uint8_t len) {
	uint8_t i;

	for (i = 0; i < len; i++) {
		nrf24_spi_transfer(dataout[i]);
	}
}

/* Read single register from nrf24 */
void nrf24_readRegister(uint8_t reg, uint8_t* value, uint8_t len) {
	nrf24_csn_digitalWrite(0);
	nrf24_spi_transfer(NRF24_CMD_R_REGISTER | (NRF24_REGISTER_MASK & reg));
	nrf24_transferSync(value, value, len);
	nrf24_csn_digitalWrite(1);
}

/* Write to a single register of nrf24 */
void nrf24_writeRegister(uint8_t reg, uint8_t* value, uint8_t len) {
	nrf24_csn_digitalWrite(0);
	nrf24_spi_transfer(NRF24_CMD_W_REGISTER | (NRF24_REGISTER_MASK & reg));
	nrf24_transmitSync(value, len);
	nrf24_csn_digitalWrite(1);
}

/* Clocks only one byte into the given nrf24 register */
void nrf24_writeRegisterByte(uint8_t reg, uint8_t value) {
	nrf24_csn_digitalWrite(0);
	nrf24_spi_transfer(NRF24_CMD_W_REGISTER | (NRF24_REGISTER_MASK & reg));
	nrf24_spi_transfer(value);
	nrf24_csn_digitalWrite(1);
}

/* Returns the number of retransmissions occurred for the last message */
uint8_t nrf24_retransmissionCount(void) {
	uint8_t rv;
	nrf24_readRegister(NRF24_REG_OBSERVE_TX, &rv, 1);
	rv = rv & 0x0F;
	return rv;
}

uint8_t nrf24_lastMessageStatus(void) {
	NRF24_Register_Status rv;

	rv = (NRF24_Register_Status) nrf24_getStatus();

	/* Transmission went OK */
	if (rv.TX_DS) {
		return NRF24_TRANSMISSON_OK;
	}		/* Maximum retransmission count is reached */
		/* Last message probably went missing ... */
	else if (rv.MAX_RT) {
		return NRF24_MESSAGE_LOST;
	}
		/* Probably still sending ... */
	else {
		return 0xFF;
	}
}

bool nrf24_isSending(void) {
	NRF24_Register_Status status;

	/* read the current status */
	status = (NRF24_Register_Status) nrf24_getStatus();

	/* if sending successful (TX_DS) or max retries exceded (MAX_RT). */
	if (status.TX_DS || status.MAX_RT) {
		return false;
	}

	return true;
}

/* Checks if receive FIFO is empty or not */
bool nrf24_rxFifoEmpty(void) {
	NRF24_Register_FIFOStatus fifoStatus;

	nrf24_readRegister(NRF24_REG_FIFO_STATUS, &(fifoStatus.DATA), 1);

	return fifoStatus.RX_EMPTY;
}

NRF24_Register_FIFOStatus nrf24_getFifoStatus(void) {
	NRF24_Register_FIFOStatus fifoStatus;

	nrf24_readRegister(NRF24_REG_FIFO_STATUS, &(fifoStatus.DATA), 1);

	return fifoStatus;
}

/* Set the TX address */
void nrf24_setTxAddress(uint8_t* adr) {
	/* RX_ADDR_P0 must be set to the sending addr for auto ack to work. */
	nrf24_writeRegister(NRF24_REG_RX_ADDR_P0, adr, NRF24_ADDR_LEN);
	nrf24_writeRegister(NRF24_REG_TX_ADDR, adr, NRF24_ADDR_LEN);
}

/* Set the RX address */
void nrf24_setRxAddress(uint8_t * adr) {
	nrf24_writeRegister(NRF24_REG_RX_ADDR_P1, adr, NRF24_ADDR_LEN);
}


/* enable rx address for specific pipe */
void nrf24_enableRxAddress(uint8_t pipe, bool enable) {
	NRF24_Register_EnabledRXAddr s;
	nrf24_readRegister(NRF24_REG_EN_RXADDR, &(s.DATA), 1);

	if (pipe < 6) {
		if (enable) {
			s.DATA |= (1 << pipe);
		} else {
			s.DATA &= ~(1 << pipe);
		}

		nrf24_writeRegisterByte(NRF24_REG_EN_RXADDR, s.DATA);
	}
}


/* Set channel */
// frequency = 2400MHz + channel * 1MHz;
void nrf24_setChannel(uint8_t channel) {
	NRF24_Register_RFChannel ch;
	ch.RF_CH = channel;
	nrf24_writeRegisterByte(NRF24_REG_RF_CH, ch.DATA);
}

/* CRC - disable */
void nrf24_CRC_disable(void) {
	NRF24_Register_Config c;
	nrf24_readRegister(NRF24_REG_CONFIG, &(c.DATA), 1);

	c.EN_CRC = 0;

	nrf24_writeRegisterByte(NRF24_REG_CONFIG, c.DATA);
}

/* CRC - get length */
NRF24_CRC_Length nrf24_CRC_getLength(void) {
	NRF24_Register_Config c;
	nrf24_readRegister(NRF24_REG_CONFIG, &(c.DATA), 1);

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

/* CRC - set length */
void nrf24_CRC_setLength(NRF24_CRC_Length crc) {
	NRF24_Register_Config c;
	nrf24_readRegister(NRF24_REG_CONFIG, &(c.DATA), 1);

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

	nrf24_writeRegisterByte(NRF24_REG_CONFIG, c.DATA);
}

/* set retries */
void nrf24_setRetries(NRF24_Retry_Delay delay, uint8_t count) {
	NRF24_Register_SetupRetransmission s;
	s.ARC = count;
	s.ARD = delay;

	nrf24_writeRegisterByte(NRF24_REG_SETUP_RETR, s.DATA);
}

/* set air data rate */
void nrf24_setAirDataRate(NRF24_AirDataRate air_data_rate) {
	NRF24_Register_RFSetup s;
	nrf24_readRegister(NRF24_REG_RF_SETUP, &(s.DATA), 1);

	if (air_data_rate == NRF24_AIR_DATA_RATE_250KBPS) {
		s.RF_DR_LOW = 1;
	} else {
		s.RF_DR_LOW = 0;
		if (air_data_rate == NRF24_AIR_DATA_RATE_1MBPS) {
			s.RF_DR_HIGH = 0;
		} else {
			// 2Mbps
			s.RF_DR_HIGH = 1;
		}
	}

	nrf24_writeRegisterByte(NRF24_REG_RF_SETUP, s.DATA);
}

/* get air data rate */
NRF24_AirDataRate nrf24_getAirDataRate(void) {
	NRF24_Register_RFSetup s;
	nrf24_readRegister(NRF24_REG_RF_SETUP, &(s.DATA), 1);

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

/* set auto acknowledgment */
void nrf24_setAutoAcknowledgment(bool enable) {
	if (enable) {
		nrf24_writeRegisterByte(NRF24_REG_EN_AA, 0b111111);
	} else {
		nrf24_writeRegisterByte(NRF24_REG_EN_AA, 0b000000);
	}
}

/* set auto acknowledgment for specific pipe */
void nrf24_setPipeAutoAcknowledgment(uint8_t pipe, bool enable) {
	NRF24_Register_EnableAA s;
	nrf24_readRegister(NRF24_REG_EN_AA, &(s.DATA), 1);

	if (pipe < 6) {
		if (enable) {
			s.DATA |= (1 << pipe);
		} else {
			s.DATA &= ~(1 << pipe);
		}

		nrf24_writeRegisterByte(NRF24_REG_EN_AA, s.DATA);
	}
}

/* detect carrier, received power detector */
bool nrf24_receivedPowerDetector(void) {
	NRF24_Register_ReceivedPowerDetector s;
	nrf24_readRegister(NRF24_REG_RPD, &(s.DATA), 1);

	return s.RPD;
}

/* get power output level */
NRF24_PowerOutputLevel nrf24_getPowerOutputLevel(void) {
	NRF24_Register_RFSetup s;
	nrf24_readRegister(NRF24_REG_RF_SETUP, &(s.DATA), 1);

	return s.RF_PWR;
}

/* set power output level */
void nrf24_setPowerOutputLevel(NRF24_PowerOutputLevel power) {
	NRF24_Register_RFSetup s;
	nrf24_readRegister(NRF24_REG_RF_SETUP, &(s.DATA), 1);

	s.RF_PWR = power;

	nrf24_writeRegisterByte(NRF24_REG_RF_SETUP, s.DATA);
}

/* Returns the length of data waiting in the RX fifo */
uint8_t nrf24_getReceivedPayloadLength(void) {
	uint8_t val;
	nrf24_csn_digitalWrite(0);
	nrf24_spi_transfer(NRF24_CMD_R_RX_PL_WID);
	val = nrf24_spi_transfer(0x00);
	nrf24_csn_digitalWrite(1);

	return val;
}

NRF24_Register_Status nrf24_flushRx(void) {
	NRF24_Register_Status s;
	nrf24_csn_digitalWrite(0);
	s.DATA = nrf24_spi_transfer(NRF24_CMD_FLUSH_RX);
	nrf24_csn_digitalWrite(1);

	return s;
}

NRF24_Register_Status nrf24_flushTx(void) {
	NRF24_Register_Status s;
	nrf24_csn_digitalWrite(0);
	s.DATA = nrf24_spi_transfer(NRF24_CMD_FLUSH_TX);
	nrf24_csn_digitalWrite(1);

	return s;
}

/* configure the module */
void nrf24_config(uint8_t channel, uint8_t pay_length) {
	/* Use static payload length ... */
	payload_len = pay_length;

	nrf24_ce_digitalWrite(0);
	nrf24_csn_digitalWrite(1);

	// Must allow the radio time to settle else configuration bits will not necessarily stick.
	// This is actually only required following power up but some settling time also appears to
	// be required after resets too. For full coverage, we'll always assume the worst.
	// Enabling 16b CRC is by far the most obvious case if the wrong timing is used - or skipped.
	// Technically we require 4.5ms + 14us as a worst case. We'll just call it 5ms for good measure.
	// WARNING: Delay is based on P-variant whereby non-P *may* require different timing.
	nrf24_hw_delay_us(5000);

	uint8_t d;

	// Set 1500uS (minimum for 32B payload in ESB@250KBPS) timeouts, to make testing a little easier
	// WARNING: If this is ever lowered, either 250KBS mode with AA is broken or maximum packet
	// sizes must never be used. See documentation for a more complete explanation.
	NRF24_Register_SetupRetransmission *se = (NRF24_Register_SetupRetransmission*) & d;
	se->ARD = NRF24_RETRY_DELAY_1500uS;
	se->ARC = 0b1111;
	nrf24_writeRegisterByte(NRF24_REG_SETUP_RETR, se->DATA);

	// Restore our default PA level
	nrf24_setPowerOutputLevel(NRF24_POWER_OUTPUT_0DBM);

	// Then set the data rate to the slowest (and most reliable) speed supported by all
	// hardware.
	nrf24_setAirDataRate(NRF24_AIR_DATA_RATE_1MBPS);

	// Initialize CRC and request 2-byte (16bit) CRC
	nrf24_CRC_setLength(NRF24_CRC_16);

	// Disable dynamic payloads, to match dynamic_payloads_enabled setting
	nrf24_writeRegisterByte(NRF24_REG_DYNPD, 0);

	// Reset current status
	// Notice reset and flush is the last thing we do
	d = 0;
	NRF24_Register_Status *stat = (NRF24_Register_Status*) & d;
	stat->RX_DR = 1;
	stat->TX_DS = 1;
	stat->MAX_RT = 1;
	nrf24_setStatus(*stat);

	// Auto Acknowledgment
	nrf24_setAutoAcknowledgment(true);


	// Set up default configuration.  Callers can always change it later.
	// This channel should be universally safe and not bleed over into adjacent
	// spectrum.
	nrf24_setChannel(channel);

	// Flush buffers
	nrf24_flushRx();
	nrf24_flushTx();

}

NRF24_Register_Status nrf24_write_payload(const void* buf, uint8_t len) {
	uint8_t data_len = len < payload_len ? len : payload_len; // min(len, payload_len);
	uint8_t blank_len = dynamic_payloads_enabled ? 0 : payload_len - data_len;


	/* Pull down chip select */
	nrf24_csn_digitalWrite(0);

	/* Write cmd to write payload */
	NRF24_Register_Status status;
	status.DATA = nrf24_spi_transfer(NRF24_CMD_W_TX_PAYLOAD);

	/* Write payload */
	nrf24_transmitSync((uint8_t *)buf, data_len);

	while (blank_len--) nrf24_spi_transfer(0);

	/* Pull up chip select */
	nrf24_csn_digitalWrite(1);



	return status;
}

NRF24_Register_Status nrf24_write_ack_payload(const void* buf, uint8_t len) {
	uint8_t data_len = len < payload_len ? len : payload_len; // min(len, payload_len);
	uint8_t blank_len = dynamic_payloads_enabled ? 0 : payload_len - data_len;


	/* Pull down chip select */
	nrf24_csn_digitalWrite(0);

	/* Write cmd to write payload */
	NRF24_Register_Status status;
	status.DATA = nrf24_spi_transfer(NRF24_CMD_W_ACK_PAYLOAD);

	/* Write payload */
	nrf24_transmitSync((uint8_t *)buf, data_len);

	while (blank_len--) nrf24_spi_transfer(0);

	/* Pull up chip select */
	nrf24_csn_digitalWrite(1);



	return status;
}

NRF24_Register_Status nrf24_read_payload(void* buf, uint8_t len) {
	uint8_t data_len = len < payload_len ? len : payload_len; // min(len, payload_len);
	uint8_t blank_len = dynamic_payloads_enabled ? 0 : payload_len - data_len;

	/* Pull down chip select */
	nrf24_csn_digitalWrite(0);

	/* Write cmd to write payload */
	NRF24_Register_Status status;
	status.DATA = nrf24_spi_transfer(NRF24_CMD_R_RX_PAYLOAD);

	/* Write payload */
	nrf24_transferSync((uint8_t *)buf, (uint8_t *)buf, data_len);

	while (blank_len--) nrf24_spi_transfer(0);

	/* Pull up chip select */
	nrf24_csn_digitalWrite(1);



	return status;
}

void nrf24_startRx(uint8_t channel, uint8_t* rx_address) {
	nrf24_setChannel(channel);
	nrf24_setAutoAcknowledgment(true);
	nrf24_setRetries(NRF24_RETRY_DELAY_1000uS, 0b1111);
	nrf24_setAirDataRate(NRF24_AIR_DATA_RATE_250KBPS);
	nrf24_setPowerOutputLevel(NRF24_POWER_OUTPUT_0DBM);
	nrf24_CRC_setLength(NRF24_CRC_16);
//	nrf24_disableDynamicPayloads();
	
	nrf24_writeRegisterByte(NRF24_REG_RX_PW_P0, 0);
	nrf24_writeRegisterByte(NRF24_REG_RX_PW_P1, 32);
	
	NRF24_Register_Config config;
	nrf24_readRegister(NRF24_REG_CONFIG, &(config.DATA), 1);
	config.MASK_MAX_RT = 0;
	config.MASK_TX_DS = 0;
	config.MASK_RX_DR = 0;
	config.EN_CRC=1;
	config.CRCO=1;
	nrf24_writeRegisterByte(NRF24_REG_CONFIG, config.DATA);
	
	nrf24_setRxAddress(rx_address);
//	nrf24_enableRxAddress(0, true);
//	nrf24_enableRxAddress(1, true);
	nrf24_writeRegisterByte(NRF24_REG_EN_RXADDR, 0xff);
	nrf24_powerUpRx();
}

void nrf24_startTx(uint8_t channel, uint8_t* tx_address) {
	nrf24_setChannel(channel);
	nrf24_setAutoAcknowledgment(true);
	nrf24_setRetries(NRF24_RETRY_DELAY_1000uS, 0b1111);
	nrf24_setAirDataRate(NRF24_AIR_DATA_RATE_250KBPS);
	nrf24_setPowerOutputLevel(NRF24_POWER_OUTPUT_0DBM);
	nrf24_CRC_setLength(NRF24_CRC_16);
	
	nrf24_writeRegisterByte(NRF24_REG_RX_PW_P0, 0);
	nrf24_writeRegisterByte(NRF24_REG_RX_PW_P1, 32);
	
	NRF24_Register_Config config;
	nrf24_readRegister(NRF24_REG_CONFIG, &(config.DATA), 1);
	config.MASK_MAX_RT = 0;
	config.MASK_TX_DS = 0;
	config.MASK_RX_DR = 0;
	config.EN_CRC=1;
	config.CRCO=1;
	nrf24_writeRegisterByte(NRF24_REG_CONFIG, config.DATA);
	
	nrf24_setTxAddress(tx_address);
	nrf24_enableRxAddress(0, true);
	nrf24_powerUpTx();
}
