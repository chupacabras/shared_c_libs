#include "sx127x.h"

uint8_t SX127X_SPIRead(SX127X_t *module, uint8_t addr) {
	uint8_t tmp;

	module->set_nss(0);
	module->read_reg(module, addr, &tmp, 1);
	module->set_nss(1);

	return tmp;
}

void SX127X_SPIWrite(SX127X_t *module, uint8_t addr, uint8_t cmd) {
	module->set_nss(0);
	module->write_reg(module, addr | 0x80, &cmd, 1);
	module->set_nss(1);
}

void SX127X_SPIBurstRead(SX127X_t *module, uint8_t addr, uint8_t *rxBuf, uint8_t length) {
	if (length <= 1) {
		return;
	} else {
		module->set_nss(0);
		module->read_reg(module, addr, rxBuf, length);
		module->set_nss(1);
	}
}

void SX127X_SPIBurstWrite(SX127X_t *module, uint8_t addr, uint8_t *txBuf, uint8_t length) {
	if (length <= 1) {
		return;
	} else {
		module->set_nss(0);
		module->write_reg(module, addr | 0x80, txBuf, length);
		module->set_nss(1);
	}
}

void SX127X_init(SX127X_t *module, uint64_t frequency, SX127X_Power_t power, SX127X_SpreadFactor_t LoRa_SF, SX127X_LoRaBandwidth_t LoRa_BW, SX127X_CodingRate_t LoRa_CR,
		SX127X_CRC_Sum_t LoRa_CRC_sum, uint8_t packetLength, device_write_ptr write_reg, device_read_ptr read_reg, device_delay_ms_ptr delay_ms,
		device_get_dio0_ptr get_dio0, device_set_nss_ptr set_nss, device_set_nreset_ptr set_nreset) {

	module->set_nreset = set_nreset;
	module->set_nss = set_nss;
	module->get_dio0 = get_dio0;
	module->delay_ms = delay_ms;
	module->write_reg = write_reg;
	module->read_reg = read_reg;

	module->set_nss(1);
	module->set_nreset(1);

	module->frequency = frequency;
	module->highFrequency = frequency > 525000000;
	module->power = power;
	module->LoRa_SF = LoRa_SF;
	module->LoRa_BW = LoRa_BW;
	module->LoRa_CR = LoRa_CR;
	module->LoRa_CRC_sum = LoRa_CRC_sum;
	module->packetLength = packetLength;

	SX127X_config(module);
}

void SX127X_reset(SX127X_t *module) {
	module->set_nss(1);
	module->set_nreset(0);
	module->delay_ms(1);
	module->set_nreset(1);
	module->delay_ms(100);
}

void SX127X_config(SX127X_t *module) {
	SX127X_sleep(module); //Change modem mode Must in Sleep mode
	module->delay_ms(15);

	SX127X_entryLoRa(module);
	//SX127X_SPIWrite(module, 0x5904); //?? Change digital regulator form 1.6V to 1.47V: see errata note

	// Fstep=Fxosc/2^19
	// Frf=Fstep * frf(23:0)
	// frf(23:0) = Frf / (Fxosc/2^19)) = Frf * 2^19 / Fxosc
	uint64_t freq = ((uint64_t) module->frequency << 19) / 32000000;
	uint8_t freq_reg[3];
	freq_reg[0] = (uint8_t) (freq >> 16);
	freq_reg[1] = (uint8_t) (freq >> 8);
	freq_reg[2] = (uint8_t) (freq >> 0);
	SX127X_SPIBurstWrite(module, SX127X_RegFrMsb, (uint8_t*) freq_reg, 3); //setting  frequency parameter

	// LoRaWAN
	SX127X_SPIWrite(module, SX127X_FSK_RegSyncWord, 0x34);

	// output power
	SX127X_SPIWrite(module, SX127X_RegPaConfig, SX127X_Power[module->power]); //Setting output power parameter

	// disable over-current protection
	SX127X_SPIWrite(module, SX127X_RegOcp, 0x0B); //RegOcp,Close Ocp
	// maximum gain, LNA boost
	SX127X_SPIWrite(module, SX127X_RegLna, 0x23); //RegLNA,High & LNA Enable

	if (SX127X_SpreadFactor[module->LoRa_SF] == 6) { //SFactor=6
		uint8_t tmp;
		SX127X_SPIWrite(module, SX127X_LoRa_RegModemConfig1, ((SX127X_LoRaBandwidth[module->LoRa_BW] << 4) + (SX127X_CodingRate[module->LoRa_CR] << 1) + 0x01)); //Implicit Enable CRC Enable(0x02) & Error Coding rate 4/5(0x01), 4/6(0x02), 4/7(0x03), 4/8(0x04)

		SX127X_SPIWrite(module, SX127X_LoRa_RegModemConfig2, ((SX127X_SpreadFactor[module->LoRa_SF] << 4) + (SX127X_CRC_Sum[module->LoRa_CRC_sum] << 2) + 0x03));

		tmp = SX127X_SPIRead(module, SX127X_LoRa_RegDetectOptimize);
		tmp &= 0xF8;
		tmp |= 0x05;
		SX127X_SPIWrite(module, SX127X_LoRa_RegDetectOptimize, tmp);
		SX127X_SPIWrite(module, SX127X_LoRa_RegDetectionThreshold, 0x0C);
	} else {
		SX127X_SPIWrite(module, SX127X_LoRa_RegModemConfig1, ((SX127X_LoRaBandwidth[module->LoRa_BW] << 4) + (SX127X_CodingRate[module->LoRa_CR] << 1) + 0x00)); //Explicit Enable CRC Enable(0x02) & Error Coding rate 4/5(0x01), 4/6(0x02), 4/7(0x03), 4/8(0x04)

		SX127X_SPIWrite(module, SX127X_LoRa_RegModemConfig2, ((SX127X_SpreadFactor[module->LoRa_SF] << 4) + (SX127X_CRC_Sum[module->LoRa_CRC_sum] << 2) + 0x00)); //SFactor &  LNA gain set by the internal AGC loop
	}

	// enable AGC (automatic gain control)
	SX127X_SPIWrite(module, SX127X_LoRa_RegModemConfig3, 0x04);

	SX127X_SPIWrite(module, SX127X_LoRa_RegSymbTimeoutLsb, 0x08); //RegSymbTimeoutLsb Timeout = 0x3FF(Max)
	SX127X_SPIWrite(module, SX127X_LoRa_RegPreambleMsb, 0x00); //RegPreambleMsb
	SX127X_SPIWrite(module, SX127X_LoRa_RegPreambleLsb, 8); //RegPreambleLsb 8+4=12byte Preamble
	SX127X_SPIWrite(module, SX127X_RegDioMapping2, 0x01); //RegDioMapping2 0b01: DIO5 (ClkOut), DIO4 (PllLock), DIO0 (TxDone)
	module->readBytes = 0;
	SX127X_standby(module); //Entry standby mode
}

int8_t SX127X_LoRaEntryRx(SX127X_t *module, uint8_t length, uint32_t timeout) {
	uint8_t addr;

	module->packetLength = length;

	SX127X_config(module); //Setting base parameter
	// set normal output power
	SX127X_SPIWrite(module, SX127X_RegPaDac, 0x84); //Normal and RX

	// FHSS enabled
	SX127X_SPIWrite(module, SX127X_LoRa_RegHopPeriod, 0xFF);

	SX127X_SPIWrite(module, SX127X_RegDioMapping1, 0x01); //DIO0=00 (RxDone), DIO1=00 (RxTimeout), DIO2=00 (FhssChangeChannel), DIO3=01 (ValidHeader))

	SX127X_SPIWrite(module, SX127X_LoRa_RegIrqFlagsMask, 0x3F); //Open RxDone interrupt & Timeout
	SX127X_clearLoRaIrq(module);
	SX127X_SPIWrite(module, SX127X_LoRa_RegPayloadLength, length); //Payload Length 21byte(this register must difine when the data long of one byte in SF is 6)
	addr = SX127X_SPIRead(module, SX127X_LoRa_RegFifoRxBaseAddr); //Read RxBaseAddr
	SX127X_SPIWrite(module, SX127X_LoRa_RegFifoAddrPtr, addr); //RxBaseAddr->FiFoAddrPtr
	// LoRa mode, RX continuous
	SX127X_SPIWrite(module, SX127X_RegOpMode,
	SX127X_OPMODE_LORA | SX127X_OPMODE_RXCONTINUOUS | (module->highFrequency ?
	SX127X_OPMODE_HIGH_FREQ :
																				SX127X_OPMODE_LOW_FREQ));
	//SX127X_SPIWrite(module, SX127X_LoRa_RegOpMode,0x05);	//Continuous Rx Mode //High Frequency Mode
	module->readBytes = 0;

	while (1) {
		if ((SX127X_SPIRead(module, SX127X_LoRa_RegModemStat) & 0x04) == 0x04) { //Rx-on going RegModemStat
			module->status = RX;
			return 1;
		}
		if (--timeout == 0) {
			SX127X_reset(module);
			SX127X_config(module);
			return 0;
		}
		module->delay_ms(1);
	}
}

uint8_t SX127X_LoRaRxPacket(SX127X_t *module) {
	unsigned char addr;
	unsigned char packet_size;

	if (module->get_dio0()) {
		memset(module->rxBuffer, 0x00, SX127X_MAX_PACKET);

		addr = SX127X_SPIRead(module, SX127X_LoRa_RegFifoRxCurrentAddr); //last packet addr
		SX127X_SPIWrite(module, SX127X_LoRa_RegFifoAddrPtr, addr); //RxBaseAddr -> FiFoAddrPtr

		if (module->LoRa_SF == SX127X_LORA_SF_6) { //When SpreadFactor is six,will used Implicit Header mode(Excluding internal packet length)
			packet_size = module->packetLength;
		} else {
			packet_size = SX127X_SPIRead(module, SX127X_LoRa_RegRxNbBytes); //Number for received bytes
		}

		SX127X_SPIBurstRead(module, SX127X_RegFifo, module->rxBuffer, packet_size);
		module->readBytes = packet_size;
		SX127X_clearLoRaIrq(module);
	}
	return module->readBytes;
}

void SX127X_sleep(SX127X_t *module) {
	SX127X_SPIWrite(module, SX127X_RegOpMode,
	SX127X_OPMODE_SLEEP | (module->highFrequency ?
	SX127X_OPMODE_HIGH_FREQ :
													SX127X_OPMODE_LOW_FREQ));
	module->status = SLEEP;
}

void SX127X_standby(SX127X_t *module) {
	SX127X_SPIWrite(module, SX127X_RegOpMode,
	SX127X_OPMODE_STANDBY | (module->highFrequency ?
	SX127X_OPMODE_HIGH_FREQ :
														SX127X_OPMODE_LOW_FREQ));
	module->status = STANDBY;
}

void SX127X_entryLoRa(SX127X_t *module) {
	SX127X_SPIWrite(module, SX127X_RegOpMode,
	SX127X_OPMODE_SLEEP | SX127X_OPMODE_LORA | (module->highFrequency ?
	SX127X_OPMODE_HIGH_FREQ :
																		SX127X_OPMODE_LOW_FREQ));
}

void SX127X_clearLoRaIrq(SX127X_t *module) {
	SX127X_SPIWrite(module, SX127X_LoRa_RegIrqFlags, 0xFF);
}

int8_t SX127X_LoRaEntryTx(SX127X_t *module, uint8_t length, uint32_t timeout) {
	uint8_t addr;
	uint8_t temp;

	module->packetLength = length;

	SX127X_config(module); //setting base parameter
	SX127X_SPIWrite(module, SX127X_RegPaDac, 0x87); //Tx for 20dBm
	SX127X_SPIWrite(module, SX127X_LoRa_RegHopPeriod, 0x00); //RegHopPeriod NO FHSS
	SX127X_SPIWrite(module, SX127X_RegDioMapping1, 0x41); //DIO0=01, DIO1=00,DIO2=00, DIO3=01
	SX127X_clearLoRaIrq(module);
	SX127X_SPIWrite(module, SX127X_LoRa_RegIrqFlagsMask, 0xF7); //Open TxDone interrupt
	SX127X_SPIWrite(module, SX127X_LoRa_RegPayloadLength, length); //RegPayloadLength 21byte
	addr = SX127X_SPIRead(module, SX127X_LoRa_RegFifoTxBaseAddr); //RegFiFoTxBaseAddr
	SX127X_SPIWrite(module, SX127X_LoRa_RegFifoAddrPtr, addr); //RegFifoAddrPtr

	while (1) {
		temp = SX127X_SPIRead(module, SX127X_LoRa_RegPayloadLength);
		if (temp == length) {
			module->status = TX;
			return 1;
		}

		if (--timeout == 0) {
			SX127X_reset(module);
			SX127X_config(module);
			return 0;
		}
	}
}

int8_t SX127X_LoRaTxPacket(SX127X_t *module, uint8_t *txBuffer, uint8_t length, uint32_t timeout) {
	SX127X_SPIBurstWrite(module, SX127X_RegFifo, txBuffer, length);
	SX127X_SPIWrite(module, SX127X_RegOpMode,
	SX127X_OPMODE_LORA | SX127X_OPMODE_TX | (module->highFrequency ?
	SX127X_OPMODE_HIGH_FREQ :
																		SX127X_OPMODE_LOW_FREQ)); //Tx Mode

	while (1) {
		if (module->get_dio0()) { //if(Get_NIRQ()) //Packet send over
			SX127X_SPIRead(module, SX127X_LoRa_RegIrqFlags);
			SX127X_clearLoRaIrq(module); //Clear irq
			SX127X_standby(module); //Entry Standby mode
			return 0;
		}

		if (--timeout == 0) {
			SX127X_reset(module);
			SX127X_config(module);
			return 1;
		}
		module->delay_ms(1);
	}
}

int8_t SX127X_transmit(SX127X_t *module, uint8_t *txBuf, uint8_t length, uint32_t timeout) {
	if (SX127X_LoRaEntryTx(module, length, timeout)) {
		return SX127X_LoRaTxPacket(module, txBuf, length, timeout);
	}

	return 1;
}

int8_t SX127X_receive(SX127X_t *module, uint8_t length, uint32_t timeout) {
	return SX127X_LoRaEntryRx(module, length, timeout);
}

uint8_t SX127X_available(SX127X_t *module) {
	return SX127X_LoRaRxPacket(module);
}

uint8_t SX127X_read(SX127X_t *module, uint8_t *rxBuf, uint8_t length) {
	if (length != module->readBytes)
		length = module->readBytes;
	memcpy(rxBuf, module->rxBuffer, length);
	rxBuf[length] = '\0';
	module->readBytes = 0;

	return length;
}

uint8_t SX127X_RSSI_LoRa(SX127X_t *module) {
	uint16_t temp = 10;
	temp = SX127X_SPIRead(module, SX127X_LoRa_RegRssiValue); //Read RegRssiValue, Rssi value
	temp = temp + 127 - 137; //127:Max RSSI, 137:RSSI offset

	return (uint8_t) temp;
}

uint8_t SX127X_RSSI_FSK(SX127X_t *module) {
	uint8_t temp = 0xff;
	temp = SX127X_SPIRead(module, SX127X_FSK_RegRssiValue);
	temp = 127 - (temp >> 1); //127:Max RSSI

	return temp;
}
