#include "sx127x.h"

const uint32_t SX127X_BW_FREQ[] = {7800, 10400, 15600, 20800, 31200, 41700, 62500, 125000, 250000, 500000};

/**
 * \brief Read byte from LoRa module
 *
 * Reads data from LoRa module from given address.
 *
 * \param[in]  module	Pointer to LoRa structure
 * \param[in]  addr		Address from which data will be read
 *
 * \return			  Read data
 */
static uint8_t SX127X_SPIRead(SX127X_Handle *module, uint8_t addr);

/**
 * \brief Write byte to LoRa module
 *
 * Writes data to LoRa module under given address.
 *
 * \param[in]  module	Pointer to LoRa structure
 * \param[in]  addr		Address under which data will be written
 * \param[in]  cmd 		Data to write
 */
static void SX127X_SPIWrite(SX127X_Handle *module, uint8_t addr, uint8_t cmd);

/**
 * \brief Read data from LoRa module
 *
 * Reads data from LoRa module from given address.
 *
 * \param[in]  module	Pointer to LoRa structure
 * \param[in]  addr		Address from which data will be read
 * \param[out] rxBuf	Pointer to store read data
 * \param[in]  length   Number of bytes to read
 */
static void SX127X_SPIBurstRead(SX127X_Handle *module, uint8_t addr, uint8_t *rxBuf, uint8_t length);

/**
 * \brief Write data to LoRa module
 *
 * Writes data to LoRa module under given address.
 *
 * \param[in]  module	Pointer to LoRa structure
 * \param[in]  addr		Address under which data will be written
 * \param[in]  txBuf	Pointer to data
 * \param[in]  length   Number of bytes to write
 */
static void SX127X_SPIBurstWrite(SX127X_Handle *module, uint8_t addr, uint8_t *txBuf, uint8_t length);


static uint8_t SX127X_SPIRead(SX127X_Handle *module, uint8_t addr) {
	uint8_t tmp;

	module->set_nss(0);
	module->read_reg(module, addr, &tmp, 1);
	module->set_nss(1);

	return tmp;
}

static void SX127X_SPIWrite(SX127X_Handle *module, uint8_t addr, uint8_t cmd) {
	module->set_nss(0);
	module->write_reg(module, addr | 0x80, &cmd, 1);
	module->set_nss(1);
}

static void SX127X_SPIBurstRead(SX127X_Handle *module, uint8_t addr, uint8_t *rxBuf, uint8_t length) {
	if (length ==0) {
		return;
	} else {
		module->set_nss(0);
		module->read_reg(module, addr, rxBuf, length);
		module->set_nss(1);
	}
}

static void SX127X_SPIBurstWrite(SX127X_Handle *module, uint8_t addr, uint8_t *txBuf, uint8_t length) {
	if (length ==0) {
		return;
	} else {
		module->set_nss(0);
		module->write_reg(module, addr | 0x80, txBuf, length);
		module->set_nss(1);
	}
}


void SX127X_reset(SX127X_Handle *module) {
	module->set_nss(1);
	module->set_nreset(0);
	module->delay_ms(1);
	module->set_nreset(1);
	module->delay_ms(100);
}

void SX127X_clearLoRaIrq(SX127X_Handle *module) {
	SX127X_SPIWrite(module, SX127X_LORA_REG_IRQ_FLAGS, 0xFF);
}

uint8_t SX127X_RSSI_LoRa(SX127X_Handle *module) {
	uint16_t temp;
	temp = SX127X_SPIRead(module, SX127X_LORA_REG_RSSI_VALUE); //Read RegRssiValue, Rssi value
	temp = temp + 127 - 137; //127:Max RSSI, 137:RSSI offset

	return (uint8_t) temp;
}

uint8_t SX127X_RSSI_FSK(SX127X_Handle *module) {
	uint8_t temp;
	temp = SX127X_SPIRead(module, SX127X_FSK_REG_RSSI_VALUE);
	temp = 127 - (temp >> 1); //127:Max RSSI

	return temp;
}







void SX127X_setLoRaIrqMode(SX127X_Handle *module, SX127X_IRQ_Source irq_source, bool irq_enabled) {
	uint8_t d;
	d = SX127X_SPIRead(module, SX127X_LORA_REG_IRQ_FLAGS_MASK);

	if (irq_enabled) {
		d=d & (~irq_source);
	} else {
		d=d | (irq_source);
	}

	SX127X_SPIWrite(module, SX127X_LORA_REG_IRQ_FLAGS_MASK, d);
}
SX127X_Register_LoRa_RegIrqFlags SX127X_getLoRaIrq(SX127X_Handle *module) {
	SX127X_Register_LoRa_RegIrqFlags d;
	d.DATA = SX127X_SPIRead(module, SX127X_LORA_REG_IRQ_FLAGS);
	return d;
}
uint8_t SX127X_getVersion(SX127X_Handle *module) {
	return SX127X_SPIRead(module, SX127X_REG_VERSION);
}
// returns dBm value
int16_t SX127X_getPacketRssiDbm(SX127X_Handle *module) {
	int16_t val=SX127X_SPIRead(module, SX127X_LORA_REG_PKT_RSSI_VALUE);
	if (module->highFrequency) {
		return val-157;
	} else {
		return val-164;
	}
}
// returns dB value; 1bit=0.25dB
int8_t SX127X_getPacketSnrDb(SX127X_Handle *module) {
	uint8_t val=SX127X_SPIRead(module, SX127X_LORA_REG_PKT_SNR_VALUE);
	return ((int8_t)val);
}

void SX127X_setFrequency(SX127X_Handle *module, uint32_t frequency) {
	// Fstep=Fxosc/2^19
	// Frf=Fstep * frf(23:0)
	// frf(23:0) = Frf / (Fxosc/2^19)) = Frf * 2^19 / Fxosc
	uint64_t freq = module->frequency;
	freq=(freq << 19) / 32000000;
	uint8_t freq_reg[3];
	freq_reg[0] = (uint8_t) (freq >> 16);
	freq_reg[1] = (uint8_t) (freq >> 8);
	freq_reg[2] = (uint8_t) (freq >> 0);
	//setting  frequency parameter
	SX127X_SPIBurstWrite(module, SX127X_REG_FR_MSB, (uint8_t*) freq_reg, 3);


	SX127X_Operation_Mode op=SX127X_getOperationMode(module);
	op.LowFrequencyModeOn=frequency < 779000000;
	SX127X_SPIWrite(module, SX127X_REG_OP_MODE, op.DATA);
}




void SX127X_init(SX127X_Handle *module, device_write_ptr write_reg, device_read_ptr read_reg, device_delay_ms_ptr delay_ms, device_set_nss_ptr set_nss, device_set_nreset_ptr set_nreset) {
	module->set_nreset = set_nreset;
	module->set_nss = set_nss;
	module->delay_ms = delay_ms;
	module->write_reg = write_reg;
	module->read_reg = read_reg;

	SX127X_reset(module);
}

void SX127X_config_LoRa(SX127X_Handle *module, uint32_t frequency, SX127X_SpreadFactor sf, SX127X_CRC crc, SX127X_CodingRate codingRate, SX127X_LoRaBandwidth bw, uint8_t syncWord) {
	module->frequency = frequency;
	module->LoRa_SF = sf;
	module->LoRa_BW = bw;
	module->LoRa_CR = codingRate;

	SX127X_setLongRangeMode(module, SX127X_LORA_MODE);

	// Preamble length = 8+4=12byte
	SX127X_setPreambleLength(module, 8);

	SX127X_setSyncWord(module, syncWord);

	SX127X_setFrequency(module, frequency);
	SX127X_setCrcMode(module, crc);
	SX127X_setSpreadingFactor(module, sf);
	SX127X_setCodingRate(module, codingRate);
	SX127X_setSignalBandwidth(module, bw);

	//RegHopPeriod NO FHSS
	SX127X_SPIWrite(module, SX127X_LORA_REG_HOP_PERIOD, 0x00);

	// set highest LNA gain
	// not needed, LNA gain is set by AGC
	//SX127X_setLnaGain(module, SX127X_LNA_GAIN_HIGHEST);

	SX127X_enableAgc(module, true);

	// enable LNA boost for HF mode
	SX127X_enableLnaBoostHf(module, true);


	// set maximum RX symbol timeout
	SX127X_setRxSymbolTimeout(module, 0x3ff);

	// optimize for low datarate
	// LowDataRateOptimize=1, mandated for when the symbol length exceeds 16ms -> bitrate < 62.5 Bps (500 bps)
	// Rs = BW / 2^SF, Ts = 2^SF / BW
	uint16_t rs=1<<sf;
	rs=rs/SX127X_BW_FREQ[bw];
	SX127X_setLowDatarateOptimize(module, rs<62);

	// disable over-current protection
	SX127X_disableOcp(module);

	// set TX and RX base address to 0x00
	SX127X_SPIWrite(module, SX127X_LORA_REG_FIFO_TX_BASE_ADDR, 0x00);
	SX127X_SPIWrite(module, SX127X_LORA_REG_FIFO_RX_BASE_ADDR, 0x00);


	// enable interrupts
	SX127X_setLoRaIrqMode(module, SX127X_RX_DONE, true);
	SX127X_setLoRaIrqMode(module, SX127X_VALID_HEADER, true);
	SX127X_setLoRaIrqMode(module, SX127X_PAYLOAD_CRC_ERROR, true);
	SX127X_setLoRaIrqMode(module, SX127X_RX_TIMEOUT, true);
	SX127X_setLoRaIrqMode(module, SX127X_TX_DONE, true);
	SX127X_setLoRaIrqMode(module, SX127X_CAD_DONE, true);

	//Enter standby mode
	SX127X_setMode(module, SX127X_MODE_STANDBY);
}

void SX127X_config_LoRa_Tx(SX127X_Handle *module, uint8_t dbm) {
	SX127X_setOutputPowerDbm(module, dbm);

	// TxDone on DIO0
	SX127X_SPIWrite(module, SX127X_REG_DIO_MAPPING1, SX127X_DIOMAPPING1_DIO0_01);

	SX127X_clearLoRaIrq(module);
}

uint8_t SX127X_readReceivedPacket(SX127X_Handle *module, uint8_t *buf) {
	// last packet addr
	uint8_t addr = SX127X_SPIRead(module, SX127X_LORA_REG_FIFO_RX_CURRENT_ADDR);
//	uint8_t addr = SX127X_SPIRead(module, SX127X_LORA_REG_FIFO_RX_BASE_ADDR);
	SX127X_SPIWrite(module, SX127X_LORA_REG_FIFO_ADDR_PTR, addr);

	uint8_t packetSize;
	SX127X_Modem_Config1 cfg1;
	cfg1.DATA=SX127X_SPIRead(module, SX127X_LORA_REG_MODEM_CONFIG1);
	if (cfg1.ImplicitHeaderModeOn) {
		packetSize = module->packetLength;
	} else {
		packetSize = SX127X_SPIRead(module, SX127X_LORA_REG_RX_NB_BYTES);
	}

//	memset(buf, 0x00, packetSize);

	SX127X_SPIBurstRead(module, SX127X_REG_FIFO, buf, packetSize);
	SX127X_clearLoRaIrq(module);

	return packetSize;

}
void SX127X_config_LoRa_Rx(SX127X_Handle *module, uint8_t defaultLength) {
	SX127X_setMode(module, SX127X_MODE_STANDBY);

	// to set defaults
	SX127X_setOutputPowerDbm(module, 17);

	module->packetLength = defaultLength;

	// must be disabled in RX
	SX127X_setHighPowerOperation(module, false);

	// RxDone on DIO0
	SX127X_SPIWrite(module, SX127X_REG_DIO_MAPPING1, SX127X_DIOMAPPING1_DIO0_00);

	// set address in FIFO
	uint8_t addr = SX127X_SPIRead(module, SX127X_LORA_REG_FIFO_RX_BASE_ADDR);
	SX127X_SPIWrite(module, SX127X_LORA_REG_FIFO_ADDR_PTR, addr);

	SX127X_clearLoRaIrq(module);

	SX127X_setMode(module, SX127X_MODE_RXCONTINUOUS);
}

void SX127X_sendPacket(SX127X_Handle *module, uint8_t *data, uint8_t packet_size) {
	// FIFO can only be filled in Standby mode
	SX127X_setMode(module, SX127X_MODE_STANDBY);

	// set address in FIFO
	uint8_t addr = SX127X_SPIRead(module, SX127X_LORA_REG_FIFO_TX_BASE_ADDR);
	SX127X_SPIWrite(module, SX127X_LORA_REG_FIFO_ADDR_PTR, addr);

	// set payload length
	SX127X_SPIWrite(module, SX127X_LORA_REG_PAYLOAD_LENGTH, packet_size);

	// fill data info FIFO
	SX127X_SPIBurstWrite(module, SX127X_REG_FIFO, data, packet_size);

	// start sending
	SX127X_setMode(module, SX127X_MODE_TX);
}

SX127X_Operation_Mode SX127X_getOperationMode(SX127X_Handle *module) {
	SX127X_Operation_Mode c;
	c.DATA=SX127X_SPIRead(module, SX127X_REG_OP_MODE);
	return c;
}

void SX127X_setMode(SX127X_Handle *module, SX127X_Mode mode) {
	uint8_t d=SX127X_SPIRead(module, SX127X_REG_OP_MODE);

	SX127X_SPIWrite(module, SX127X_REG_OP_MODE, (d & 0b11111000) | mode);
}

// high power operation must be disabled for RX
void SX127X_setHighPowerOperation(SX127X_Handle *module, bool enable_high_power) {
	if (enable_high_power) {
		// enable high power operation
		// Set Pmax to +20dBm for PA_HP
		// default value=0x84 = disabled
		SX127X_SPIWrite(module, SX127X_REG_PA_DAC, 0x87);

	} else {
		// set normal output power
		// disable high power operation for RX
		// default value=0x84 = disabled
		SX127X_SPIWrite(module, SX127X_REG_PA_DAC, 0x84);
	}
}


void SX127X_setLongRangeMode(SX127X_Handle *module, SX127X_LongRangeMode long_range_mode) {
	SX127X_Operation_Mode op=SX127X_getOperationMode(module);

	// This bit can be modified only in Sleep mode. A write operation on other device modes is ignored.
	if (op.Mode!=SX127X_MODE_SLEEP) {
		op.Mode=SX127X_MODE_SLEEP;
		SX127X_SPIWrite(module, SX127X_REG_OP_MODE, op.DATA);
	}

	op.LongRangeMode=(long_range_mode==SX127X_LORA_MODE);
	SX127X_SPIWrite(module, SX127X_REG_OP_MODE, op.DATA);

//	SX127X_setMode(module, SX127X_MODE_STANDBY);

}

void SX127X_setCrcMode(SX127X_Handle *module, SX127X_CRC crc) {
	SX127X_Modem_Config2 cfg;
	cfg.DATA=SX127X_SPIRead(module, SX127X_LORA_REG_MODEM_CONFIG2);
	cfg.RxPayloadCrcOn=crc;
	SX127X_SPIWrite(module, SX127X_LORA_REG_MODEM_CONFIG2, cfg.DATA);
}

void SX127X_setSpreadingFactor(SX127X_Handle *module, SX127X_SpreadFactor sf) {
	SX127X_Modem_Config2 cfg;
	cfg.DATA=SX127X_SPIRead(module, SX127X_LORA_REG_MODEM_CONFIG2);
	cfg.SpreadingFactor=sf;
	SX127X_SPIWrite(module, SX127X_LORA_REG_MODEM_CONFIG2, cfg.DATA);

	if (sf==SX127X_LORA_SF_6) {
		// The header must be set to Implicit mode
		SX127X_Modem_Config1 cfg1;
		cfg1.DATA=SX127X_SPIRead(module, SX127X_LORA_REG_MODEM_CONFIG1);
		cfg1.ImplicitHeaderModeOn=1;
		SX127X_SPIWrite(module, SX127X_LORA_REG_MODEM_CONFIG1, cfg1.DATA);
	}

	// optimize detection
	uint8_t det=SX127X_SPIRead(module, SX127X_LORA_REG_DETECT_OPTIMIZE);
	det &= 0b11111000;

	if (sf==SX127X_LORA_SF_6) {
		det |= 0b00000101;
	} else {
		det |= 0b00000011;
	}
	SX127X_SPIWrite(module, SX127X_LORA_REG_DETECT_OPTIMIZE, det);
	SX127X_SPIWrite(module, SX127X_LORA_REG_DETECTION_THRESHOLD, (sf==SX127X_LORA_SF_6)?0x0C:0x0A);

}

void SX127X_setCodingRate(SX127X_Handle *module, SX127X_CodingRate codingRate) {
	SX127X_Modem_Config1 cfg;
	cfg.DATA=SX127X_SPIRead(module, SX127X_LORA_REG_MODEM_CONFIG1);
	cfg.CodingRate=codingRate;
	SX127X_SPIWrite(module, SX127X_LORA_REG_MODEM_CONFIG1, cfg.DATA);
}

void SX127X_setSignalBandwidth(SX127X_Handle *module, SX127X_LoRaBandwidth bw) {
	SX127X_Modem_Config1 cfg;
	cfg.DATA=SX127X_SPIRead(module, SX127X_LORA_REG_MODEM_CONFIG1);
	cfg.Bw=bw;
	SX127X_SPIWrite(module, SX127X_LORA_REG_MODEM_CONFIG1, cfg.DATA);

	uint8_t reg36=0x03;
	uint8_t reg3a=0x65;

	if (bw==SX127X_LORA_BW_500KHZ) {
		// Sensitivity Optimization for a 500 kHz Bandwidth
		// See errata notes
		if (module->frequency>=862000000) {
			reg36=0x02;
			reg3a=0x64;
		} else if (module->frequency>=410000000 && module->frequency>=525000000) {
			reg36=0x02;
			reg3a=0x7f;
		}
	}

	SX127X_SPIWrite(module, SX127X_LORA_REG_HIGH_BW_OPTIMIZE1, reg36);
	SX127X_SPIWrite(module, SX127X_LORA_REG_HIGH_BW_OPTIMIZE2, reg3a);
}

// value between 0x000 - 0x3ff
void SX127X_setRxSymbolTimeout(SX127X_Handle *module, uint16_t timeout) {
	if (timeout>0x3ff) timeout=0x3ff;

	// write MSB
	SX127X_Modem_Config2 cfg;
	cfg.DATA=SX127X_SPIRead(module, SX127X_LORA_REG_MODEM_CONFIG2);
	cfg.SymbTimeoutMSB=(timeout>>8) & 0b11;
	SX127X_SPIWrite(module, SX127X_LORA_REG_MODEM_CONFIG2, cfg.DATA);

	// write LSB
	SX127X_SPIWrite(module, SX127X_LORA_REG_SYMB_TIMEOUT_LSB, timeout & 0xff);
}

// AGC = automatic gain control
void SX127X_enableAgc(SX127X_Handle *module, bool enable_agc) {
	SX127X_Modem_Config3 cfg;
	cfg.DATA=SX127X_SPIRead(module, SX127X_LORA_REG_MODEM_CONFIG3);
	cfg.AgcAutoOn=enable_agc;
	SX127X_SPIWrite(module, SX127X_LORA_REG_MODEM_CONFIG3, cfg.DATA);
}

// optimize for low datarate
// LowDataRateOptimize=1, mandated for when the symbol length exceeds 16ms -> bitrate < 62.5 Bps (500 bps)
// Rs = BW / 2^SF, Ts = 2^SF / BW
void SX127X_setLowDatarateOptimize(SX127X_Handle *module, bool enable_optimize) {
	SX127X_Modem_Config3 cfg;
	cfg.DATA=SX127X_SPIRead(module, SX127X_LORA_REG_MODEM_CONFIG3);
	cfg.LowDataRateOptimize=enable_optimize;
	SX127X_SPIWrite(module, SX127X_LORA_REG_MODEM_CONFIG3, cfg.DATA);
}

void SX127X_enableLnaBoostHf(SX127X_Handle *module, bool enable) {
	SX127X_Lna cfg;
	cfg.DATA=SX127X_SPIRead(module, SX127X_REG_LNA);
	cfg.LnaBoostHf=enable?0b11:0;
	SX127X_SPIWrite(module, SX127X_REG_LNA, cfg.DATA);
}

void SX127X_setLnaGain(SX127X_Handle *module, SX127X_LnaGain lnaGain) {
	SX127X_Lna cfg;
	cfg.DATA=SX127X_SPIRead(module, SX127X_REG_LNA);
	cfg.LnaGain=lnaGain;
	SX127X_SPIWrite(module, SX127X_REG_LNA, cfg.DATA);
}

void SX127X_disableOcp(SX127X_Handle *module) {
	SX127X_Ocp ocp;
	ocp.DATA=SX127X_SPIRead(module, SX127X_REG_OCP);
	ocp.OcpOn=0;
	SX127X_SPIWrite(module, SX127X_REG_OCP, ocp.DATA);
}

// Imax current in mA; from 45mA to 240mA
void SX127X_enableOcp(SX127X_Handle *module, uint8_t imax) {
	SX127X_Ocp ocp;
	ocp.DATA=SX127X_SPIRead(module, SX127X_REG_OCP);
	ocp.OcpOn=1;
	if (imax<=45) {
		ocp.OcpTrim=0;
	} else if (imax<=120) {
		ocp.OcpTrim=(imax/5)-9;
	} else if (imax<=240) {
		ocp.OcpTrim=(imax/10)+3;
	} else {
		ocp.OcpTrim=0b11111;
	}
	SX127X_SPIWrite(module, SX127X_REG_OCP, ocp.DATA);
}

// +20dBm max. 1% duty cycle
void SX127X_setOutputPowerDbm(SX127X_Handle *module, int8_t dbm) {
	/* Output power of module is from -4 dBm to +15 dBm in "low" power devices,
	 +2 dBm to +17 dBm in high power devices or +20 dBm in high power mode */
	// only high power mode implemented (PA_BOOST)

	if (dbm > 20) {
		dbm=20;
	}
	if (dbm < 2) {
		dbm=2;
	}
	if (dbm < 20 && dbm>17) {
		dbm=17;
	}

	SX127X_PaConfig pa;
	// PA_BOOST is selected
	pa.PaSelect=1;

	// if PaSelect = 1 (PA_BOOST pin):
	// OutputPower = Pout - 2
	// if PaSelect = 0 (RFO pin):
	// OutputPower = Pout - Pmax + 15

	pa.OutputPower=(dbm>17)?0b1111:(dbm-2);
	pa.MaxPower=0b111;

	module->high_power_settings=(dbm==20);

	SX127X_setHighPowerOperation(module, module->high_power_settings);
	SX127X_SPIWrite(module, SX127X_REG_PA_CONFIG, pa.DATA);

}

// PreambleLength from 6 to 65535
// yielding total preamble lengths of 6+4 to 65535+4 symbols
void SX127X_setPreambleLength(SX127X_Handle *module, uint16_t len) {
	if (len<6) len=6;

	SX127X_SPIWrite(module, SX127X_LORA_REG_PREAMBLE_MSB, (len >> 8) & 0xff);
	SX127X_SPIWrite(module, SX127X_LORA_REG_PREAMBLE_LSB, len & 0xff);
}

// default value = 0x12
// Value 0x34 is reserved for LoRaWAN networks
void SX127X_setSyncWord(SX127X_Handle *module, uint8_t syncWord) {
	SX127X_SPIWrite(module, SX127X_LORA_REG_SYNC_WORD, syncWord);
}

// CAD = carrier activity detection
// timeout = loop count.
// it takes 100 loops on 72MHz
bool SX127X_cad(SX127X_Handle *module, uint16_t timeout) {
	SX127X_setMode(module, SX127X_MODE_STANDBY);

	SX127X_clearLoRaIrq(module);

	SX127X_setMode(module, SX127X_MODE_CAD);

	while (true) {
		if (timeout==0) return false;

		SX127X_Register_LoRa_RegIrqFlags irq=SX127X_getLoRaIrq(module);
		if (irq.CadDone) {
			return irq.CadDetected;
		}
		timeout--;
	}

}
