/**
  ******************************************************************************
  * @file    rfm69.c
  * @author  Juraj Lonc (juraj.lonc@gmail.com)
  * @brief   RFM69 (RFM69H) radio driver.
  ******************************************************************************
  */

#include "rfm69.h"

void RFM69_init(RFM69_t *module, RFM69_Model_t model, uint64_t frequency, uint8_t node_addr, rfm69_device_spi_write_ptr write_spi, rfm69_device_spi_read_ptr read_spi, device_delay_ms_ptr delay_ms,
		rfm69_device_get_dio0_ptr get_dio0, rfm69_device_set_nss_ptr set_nss, rfm69_device_set_reset_ptr set_reset, uint8_t *receive_buf) {

	module->status=RFM69_STATUS_WAITING;
	module->set_reset = set_reset;
	module->set_nss = set_nss;
	module->get_dio0 = get_dio0;
	module->delay_ms = delay_ms;
	module->write_spi = write_spi;
	module->read_spi = read_spi;
	module->receive_buf=receive_buf;

	RFM69_reset(module);
	module->model=model;
	module->high_power_device=(model==RFM69HW);
    
	// RSSI threshold, recommended value; 0xE4 = 228 -> -114dB
	RFM69_register_write(module, RFM69_RegRssiThresh, 0xE4);

	// set higher threshold for RSSI, to be less sensitive
	// RSSI trigger level for Rssi interrupt: - RssiThreshold / 2 [dBm]; default: -114 dB (0xE4), new value: -110 dB (0xDC)
	RFM69_register_write(module, RFM69_RegRssiThresh, 220);

	// Fifo threshold, recommended value 0x8F
	RFM69_register_write(module, RFM69_RegFifoThresh, 0x8F);

	// ContinuousDagc, Fading Margin Improvement, recommended value 0x30
	RFM69_register_write(module, RFM69_RegTestDagc, 0x30);
    
//    RFM69_register_write(module, RFM69_RegPreambleLsb, 0x80);
//    RFM69_register_write(module, RFM69_RegPreambleLsb, 0x04);
//    RFM69_register_write(module, RFM69_RegPreambleLsb, 0x10);
    RFM69_register_write(module, RFM69_RegPreambleLsb, 0x80);

	RFM69_set_frequency(module, frequency);

	// choose 57600 bps as bitrate
	RFM69_set_bitrate(module, 57600);
    
	// set rx bandwidth to 120 kHz
	RFM69_set_rx_bandwidth_filter(module, 120000, RFM69_FSK);

	// set deviation to 50kHz; default 5kHz
	RFM69_set_FSK_deviation_frequency(module, 50000);

	// set DIO0 to signal PayloadReady in Rx (0b01), and TxReady in Tx (value 0b00)
	RFM69_register_write(module, RFM69_RegDioMapping1, RFM69_DIOMAPPING1_DIO0_01);

	// just made up network ID
	uint8_t network[]={0xBE, 0xEF};
	RFM69_set_network_address(module, network, 2);

	// variable packet
	// dcfree none
	// crc on
	// crc auto clear on
	// address filtering by node address and broadcast
	// hardware address filtering is on; in promiscuous mode is address filtering made in software
	RFM69_register_write(module, RFM69_RegPacketConfig1, RFM69_PACKET_VARIABLE_LENGTH | RFM69_DCFREE_NONE | RFM69_CRC_ON | RFM69_CRC_AUTOCLEAR_ON | RFM69_ADDRESSFILTERING_MATCH_NODE_AND_BROADCAST);
//	RFM69_register_write(module, RFM69_RegPacketConfig1, RFM69_PACKET_VARIABLE_LENGTH | RFM69_DCFREE_NONE | RFM69_CRC_ON | RFM69_CRC_AUTOCLEAR_ON | RFM69_ADDRESSFILTERING_NONE);

	// maximum payload length
	// If PacketFormat = 0 (fixed), payload length.
	// If PacketFormat = 1 (variable), max length in Rx, not used in Tx.
	RFM69_register_write(module, RFM69_RegPayloadLength, 0xff);

	// set node address and broadcast address
	RFM69_set_node_address(module, node_addr);
	RFM69_set_broadcast_address(module, RFM69_BROADCAST_ADDRESS);


//	RFM69_set_mode(module, RFM69_OPMODE_SLEEP); // Modem mode must be changed in Sleep mode
//	module->delay_ms(15);

	// disable over-current protection for RFM69HW
	// set output power level
	if (module->high_power_device) {
		RFM69_register_write(module, RFM69_RegOcp, RFM69_OCP_OFF | 0b1010);
//		RFM69_register_write(module, RFM69_RegPaLevel, (RFM69_register_read(module, RFM69_RegPaLevel) & 0x1F) | RFM69_PALEVEL_PA1_ON | RFM69_PALEVEL_PA2_ON | 0b01111); // enable P1 & P2 amplifier stages; max power
//        RFM69_register_write(module, RFM69_RegPaLevel, RFM69_PALEVEL_PA0_OFF | RFM69_PALEVEL_PA1_ON | RFM69_PALEVEL_PA2_OFF | 0b01111); // enable P1 amplifier stages; max power
        RFM69_register_write(module, RFM69_RegPaLevel, RFM69_PALEVEL_PA0_OFF | RFM69_PALEVEL_PA1_ON | RFM69_PALEVEL_PA2_ON | 0b01111); // enable P1 amplifier stages; max power
	} else {
		RFM69_register_write(module, RFM69_RegOcp, RFM69_OCP_ON | 0b1010); // default value 0b1010 -> 95 mA
		RFM69_register_write(module, RFM69_RegPaLevel, RFM69_PALEVEL_PA0_ON | RFM69_PALEVEL_PA1_OFF | RFM69_PALEVEL_PA2_OFF | 0b11111); // enable P0 only; max power
	}
    
//    RFM69_register_write(module, RFM69_RegPaLevel, (RFM69_register_read(module, 0x11) & 0x1F) | 0x40);

	module->promiscuous=false;

	RFM69_set_mode(module, RFM69_OPMODE_STANDBY); //Entry standby mode
}

void RFM69_reset(RFM69_t *module) {
	module->set_nss(1);
	module->set_reset(1);
	module->delay_ms(1);
	module->set_reset(0);
	module->delay_ms(20);
    
    module->mode=RFM69_OPMODE_STANDBY;
}

void RFM69_set_promiscuous(RFM69_t *module, bool promiscuous) {
	RFM69_register_write(module, RFM69_RegPacketConfig1, (RFM69_register_read(module, RFM69_RegPacketConfig1) & 0b11111011) | (promiscuous?RFM69_ADDRESSFILTERING_NONE:RFM69_ADDRESSFILTERING_MATCH_NODE_AND_BROADCAST));

	module->promiscuous=promiscuous;
}

uint8_t RFM69_register_read(RFM69_t *module, uint8_t addr) {
	uint8_t tmp;

	module->set_nss(0);
	module->write_spi(module, &addr, 1);
	module->read_spi(module, &tmp, 1);
	module->set_nss(1);
	
	return tmp;
}

void RFM69_register_write(RFM69_t *module, uint8_t addr, uint8_t cmd) {
	uint8_t tmp;

	module->set_nss(0);
	tmp=addr | 0x80;
	module->write_spi(module, &tmp, 1);
	module->write_spi(module, &cmd, 1);
	module->set_nss(1);
	
}

void RFM69_burst_read(RFM69_t *module, uint8_t addr, uint8_t *rxBuf, uint8_t length) {
	if (length <= 1) {
		return;
	} else {
		module->set_nss(0);
		module->write_spi(module, &addr, 1);
		module->read_spi(module, rxBuf, length);
		module->set_nss(1);
		
	}
}

void RFM69_burst_write(RFM69_t *module, uint8_t addr, uint8_t *txBuf, uint8_t length) {
	uint8_t tmp;
	if (length <= 1) {
		return;
	} else {
		module->set_nss(0);
		tmp=addr | 0x80;
		module->write_spi(module, &tmp, 1);
		module->write_spi(module, txBuf, length);
		module->set_nss(1);
		
	}
}

/*
4.3.5. RC Timer Accuracy
All timings of the Listen Mode rely on the accuracy of the internal low-power RC oscillator. This oscillator is automatically
calibrated at the device power-up, and it is a user-transparent process.
For applications enduring large temperature variations, and for which the power supply is never removed, RC calibration
can be performed upon user request. RcCalStart in RegOsc1 can be used to trigger this calibration, and the flag
RcCalDone will be set automatically when the calibration is over.
 */
void RFM69_rc_calibration(RFM69_t *module) {
	RFM69_set_mode(module, RFM69_OPMODE_STANDBY);

	RFM69_register_write(module, RFM69_RegOsc1, RFM69_OSC1_RCCAL_START);
	while ((RFM69_register_read(module, RFM69_RegOsc1) & RFM69_OSC1_RCCAL_DONE) == 0x00);
}

void RFM69_set_mode(RFM69_t *module, uint8_t new_mode) {
//	if (new_mode == module->mode)
//		return;

	RFM69_register_write(module, RFM69_RegOpMode, (RFM69_register_read(module, RFM69_RegOpMode) & 0b11100011) | new_mode);

	if (module->high_power_device && module->high_power_settings) {
		switch (new_mode) {
		case RFM69_OPMODE_TX:
			RFM69_set_high_power_settings(module, true);
			break;
		case RFM69_OPMODE_RX:
			RFM69_set_high_power_settings(module, false);
			break;
		default:
			return;
		}
	}
    
	// packet mode is used, so this check is not really needed
	// but waiting for mode ready is necessary when going from sleep because the FIFO may not be immediately available from previous mode
//	while ((module->mode == RFM69_OPMODE_SLEEP) && ((RFM69_register_read(module, RFM69_RegIrqFlags1) & RFM69_IRQ_MODE_READY) == 0x00)); // wait for ModeReady

	module->mode = new_mode;
}

void RFM69_temperature_measurement_start(RFM69_t *module) {
	RFM69_set_mode(module, RFM69_OPMODE_STANDBY);
//	module->delay_ms(10);
	RFM69_register_write(module, RFM69_RegTemp1, RFM69_START_TEMP);
}

bool RFM69_temperature_measurement_running(RFM69_t *module) {
	return (RFM69_register_read(module, RFM69_RegTemp1) & 0b00000100)!=0;
}

// get CMOS temperature (8bit)
int8_t RFM69_temperature_read(RFM69_t *module) {
	return ~RFM69_register_read(module, RFM69_RegTemp2)-(int8_t)91;
//	return 164-RFM69_register_read(module, RFM69_RegTemp2);
}

uint8_t RFM69_get_version(RFM69_t *module) {
	return RFM69_register_read(module, RFM69_RegVersion);
}

void RFM69_set_high_power_settings(RFM69_t *module, bool enable) {
	// enabling only works if this is a high power device
	if (enable && !module->high_power_device) enable = false;

	RFM69_register_write(module, RFM69_RegTestPa1, enable ? 0x5D : 0x55);
	RFM69_register_write(module, RFM69_RegTestPa2, enable ? 0x7C : 0x70);
}

// set *transmit/TX* output power: 0=min, 31=max
// this results in a "weaker" transmitted signal, and directly results in a lower RSSI at the receiver
// the power configurations are explained in the SX1231H datasheet (Table 10 on p21; RegPaLevel p66): http://www.semtech.com/images/datasheet/sx1231h.pdf
// valid powerLevel parameter values are 0-31 and result in a directly proportional effect on the output/transmission power
// this function implements 2 modes as follows:
//       - for RFM69W the range is from 0-31 [-18dBm to 13dBm] (PA0 only on RFIO pin)
//       - for RFM69HW the range is from 0-31 [5dBm to 20dBm]  (PA1 & PA2 on PA_BOOST pin & high Power PA settings - see section 3.3.7 in datasheet, p22)
void RFM69_set_power_level(RFM69_t *module, uint8_t power_level) {
	power_level = (power_level > 31 ? 31 : power_level);

	RFM69_register_write(module, RFM69_RegPaLevel, (RFM69_register_read(module, RFM69_RegPaLevel) & 0xE0) | power_level);
}

int RFM69_set_power_dbm(RFM69_t *module, int8_t dbm) {
	/* Output power of module is from -18 dBm to +13 dBm
	 * in "low" power devices, -2 dBm to +20 dBm in high power devices */
	int ret=0;
	if (dbm < -18) {
		dbm=-18;
		ret=-1;
	}
	if (dbm > 20) {
		dbm=20;
		ret=-1;
	}

	if (!(module->high_power_device) && (dbm > 13)) {
		dbm=13;
		ret=-1;
	}

	if ((module->high_power_device) && (dbm < -2)) {
		dbm=-2;
		ret=-1;
	}

	uint8_t power_level = 0;

	if (!module->high_power_device) {
		// only PA0 can be used
		power_level = dbm + 18;

		// enable PA0 only
		RFM69_register_write(module, RFM69_RegPaLevel, RFM69_PALEVEL_PA0_ON | RFM69_PALEVEL_PA1_OFF | RFM69_PALEVEL_PA2_OFF | power_level);
	} else {
		if ((dbm >= -2) && (dbm <= 13)) {
			// use PA1 on pin PA_BOOST
			power_level = dbm + 18;

			// enable PA1 only
			RFM69_register_write(module, RFM69_RegPaLevel, RFM69_PALEVEL_PA0_OFF | RFM69_PALEVEL_PA1_ON | RFM69_PALEVEL_PA2_OFF | power_level);

			// disable high power settings
			module->high_power_settings=false;
			RFM69_set_high_power_settings(module, false);

		} else if ((dbm > 13) && (dbm <= 17)) {
			// use PA1 and PA2 combined on pin PA_BOOST
			power_level = dbm + 14;

			// enable PA1+PA2
			RFM69_register_write(module, RFM69_RegPaLevel, RFM69_PALEVEL_PA0_OFF | RFM69_PALEVEL_PA1_ON | RFM69_PALEVEL_PA2_ON | power_level);

			// disable high power settings
			module->high_power_settings=false;
			RFM69_set_high_power_settings(module, false);

		} else {
			// output power from 18 dBm to 20 dBm, use PA1+PA2 with high power settings
			power_level = dbm + 11;

			// enable PA1+PA2
			RFM69_register_write(module, RFM69_RegPaLevel, RFM69_PALEVEL_PA0_OFF | RFM69_PALEVEL_PA1_ON | RFM69_PALEVEL_PA2_ON | power_level);

			// enable high power settings
			module->high_power_settings=true;
			RFM69_set_high_power_settings(module, true);
		}
	}

	return ret;
}

void RFM69_set_frequency(RFM69_t *module, uint64_t frequency) {
	module->frequency = frequency;

//	// Fstep=Fxosc/2^19
//	// Frf=Fstep * frf(23:0)
//	// frf(23:0) = Frf / (Fxosc/2^19)) = Frf * 2^19 / Fxosc
//	uint64_t freq = (((uint64_t) frequency) << 19) / RFM69_XTAL;
////	uint64_t freq = frequency / RFM69_FSTEP;
//	uint8_t freq_reg[3];
//	freq_reg[0] = (uint8_t) (freq >> 16);
//	freq_reg[1] = (uint8_t) (freq >> 8);
//	freq_reg[2] = (uint8_t) (freq >> 0);
//	RFM69_burst_write(module, RFM69_RegFrMsb, (uint8_t*) freq_reg, 3); //setting  frequency parameter
//    
    
    frequency /= RFM69_FSTEP;

  // set new frequency
  RFM69_register_write(module, RFM69_RegFrMsb, frequency >> 16);
  RFM69_register_write(module, RFM69_RegFrMid, frequency >> 8);
  RFM69_register_write(module, RFM69_RegFrLsb, frequency);
}

// default 4800 bps
void RFM69_set_bitrate(RFM69_t *module, uint32_t bitrate) {
	uint32_t reg=RFM69_XTAL/bitrate;
	RFM69_register_write(module, RFM69_RegBitRateMsb, (reg >> 8) & 0xff);
	RFM69_register_write(module, RFM69_RegBitRateLsb, reg & 0xff);
}

void RFM69_set_data_modulation(RFM69_t *module, RFM69_ModulationType_t modulation_type, RFM69_DataMode_t data_mode, RFM69_ModulationShaping_t shaping) {
	RFM69_register_write(module, RFM69_RegDataModul, data_mode | modulation_type | shaping);
}

// default 10,400 Hz for FSK, 5,200 Hz for OOK
// BitRate < 2 x RxBw
void RFM69_set_rx_bandwidth_filter(RFM69_t *module, uint32_t bw, RFM69_ModulationType_t modulation_type) {
	uint16_t bw_mant=RFM69_XTAL/bw;
	uint8_t bw_exp=0;
	while (bw_mant>=32) {
		bw_mant/=2;
		bw_exp++;
	}
	if (bw_mant>=24) {
		bw_mant=0b10; // val=24
	} else if (bw_mant>=20) {
		bw_mant=0b01; // val=20
	} else {
		bw_mant=0b00; // val=16
	}
	if (modulation_type==RFM69_FSK) {
		bw_exp-=2;
	} else {
		// OOK
		bw_exp-=3;
	}

	// Cut-off frequency of the DC offset canceller (DCC)
	// ~4% of the RxBw by default
	// DccFreq = 0b010

	RFM69_register_write(module, RFM69_RegRxBw, 0b01000000 | ((bw_mant << 2) & 0b00011000) | (bw_exp & 0b00000111));

	// set AFC bandwidth to 4x, and cut-off frequency to 1%
	bw_mant=(RFM69_XTAL/4)/bw;
	bw_exp=0;
	while (bw_mant>=32) {
		bw_mant/=2;
		bw_exp++;
	}
	if (bw_mant>=24) {
		bw_mant=0b10; // val=24
	} else if (bw_mant>=20) {
		bw_mant=0b01; // val=20
	} else {
		bw_mant=0b00; // val=16
	}
	if (modulation_type==RFM69_FSK) {
		bw_exp-=2;
	} else {
		// OOK
		bw_exp-=3;
	}

	RFM69_register_write(module, RFM69_RegAfcBw, 0b10000000 | ((bw_mant << 2) & 0b00011000) | (bw_exp & 0b00000111));
}

// get the received signal strength indicator (RSSI)
int16_t RFM69_read_RSSI(RFM69_t *module) {
	int16_t rssi = 0;

	if (RFM69_register_read(module, RFM69_RegTestDagc)==0) {
		// DAGC is disabled (not in continuous mode), so trigger measuring
		RFM69_register_write(module, RFM69_RegRssiConfig, RFM69_RSSI_START);
		while ((RFM69_register_read(module, RFM69_RegRssiConfig) & RFM69_RSSI_DONE) == 0x00); // wait for RSSI_Ready
	}

	rssi = -RFM69_register_read(module, RFM69_RegRssiValue);
	rssi >>= 1;
	return rssi;
}

// To enable encryption: RFM69_encrypt(&module, "1234567890ABCDEF");
// To disable encryption: RFM69_encrypt(&module, 0)
// KEY has to be 16 bytes
void RFM69_encrypt(RFM69_t *module, const uint8_t* aes_key) {
	RFM69_set_mode(module, RFM69_OPMODE_STANDBY);
	if (aes_key != 0) {
		RFM69_burst_write(module, RFM69_RegAesKey1, (uint8_t*) aes_key, 16);
	}
	RFM69_register_write(module, RFM69_RegPacketConfig2, (RFM69_register_read(module, RFM69_RegPacketConfig2) & 0xFE) | (aes_key ? RFM69_AES_ON : RFM69_AES_OFF));
}

// Fdev > 600 Hz
// Fdev + Bitrate/2 <= 500 kHz
void RFM69_set_FSK_deviation_frequency(RFM69_t *module, uint32_t fdev_hz) {
	// Fdev[Hz] = Fstep * Fdev[bits] = Fdev[bits] * 32000000 / 2^19
	uint64_t f=fdev_hz;
	f=f << 19;
	f/=RFM69_XTAL;
	RFM69_register_write(module, RFM69_RegFdevMsb, (f>>8) & 0xff);
	RFM69_register_write(module, RFM69_RegFdevLsb, f & 0xff);
}

// cannot contain 0x00 bytes
// max length: 8 (write "size-1" value into register)
void RFM69_set_network_address(RFM69_t *module, uint8_t *network, uint8_t size) {
	RFM69_burst_write(module, RFM69_RegSyncValue1, network, size);

	RFM69_register_write(module, RFM69_RegSyncConfig, (RFM69_register_read(module, RFM69_RegSyncConfig) & 0b01000000) | RFM69_SYNC_ON | ((size-1)<<3));
}

void RFM69_set_node_address(RFM69_t *module, uint8_t node_addr) {
	RFM69_register_write(module, RFM69_RegNodeAdrs, node_addr);
	module->node_address=node_addr;
}

void RFM69_set_broadcast_address(RFM69_t *module, uint8_t broadcast_addr) {
	RFM69_register_write(module, RFM69_RegBroadcastAdrs, broadcast_addr);
	module->broadcast_address=broadcast_addr;
}

void RFM69_send_packet(RFM69_t *module, uint8_t target_address, void* buf, uint8_t size, bool request_ack, bool send_ack, bool first_packet, bool last_packet) {
	// turn off receiver to prevent reception while filling fifo
	RFM69_set_mode(module, RFM69_OPMODE_STANDBY);
    
	// wait for ModeReady
	while ((RFM69_register_read(module, RFM69_RegIrqFlags1) & RFM69_IRQ_MODE_READY) == 0);
	
	// set DIO0 to signal "PacketSent"
	RFM69_register_write(module, RFM69_RegDioMapping1, RFM69_DIOMAPPING1_DIO0_00);

	// control byte
	uint8_t ctl=0x00;
	if (send_ack)
		ctl = RFM69_CTL_SENDACK;
	else if (request_ack)
		ctl = RFM69_CTL_REQACK;

	if (first_packet)
		ctl |= RFM69_CTL_FIRST_PACKET;
	if (last_packet)
		ctl |= RFM69_CTL_LAST_PACKET;

	module->request_ack=request_ack;

	uint8_t tmp;
	// write to FIFO
	module->set_nss(0);
	tmp=RFM69_RegFifo | 0x80;
	module->write_spi(module, &tmp, 1);
//	tmp=size+2+(module->promiscuous?1:0);
//	tmp=size+2+(addr_filtering?0:1);
    tmp=size+3;
	module->write_spi(module, &tmp, 1);
	module->write_spi(module, &target_address, 1);
	module->write_spi(module, &(module->node_address), 1);
	module->write_spi(module, &ctl, 1);
	if (size>0) module->write_spi(module, buf, size);
	module->set_nss(1);
	
    
	RFM69_set_mode(module, RFM69_OPMODE_TX);
    
	module->timer=0;
	module->status=RFM69_STATUS_TX;

}

static void RFM69_process_received_packet(RFM69_t *module) {
	RFM69_set_mode(module, RFM69_OPMODE_STANDBY);

//	bool addr_filtering=RFM69_is_address_filtering(module);

	uint8_t tmp;
	// write to FIFO
	module->set_nss(0);
	tmp=RFM69_RegFifo;
	uint8_t payload_lenth, target_addr;
	module->write_spi(module, &tmp, 1);
	module->read_spi(module, &payload_lenth, 1);
	module->read_spi(module, &target_addr, 1);

	if (!module->promiscuous && (target_addr!=module->node_address) && (target_addr!=module->broadcast_address)) {
		module->set_nss(1);
		return;
	}

//	module->payload_length=payload_lenth-2-(addr_filtering?0:1);
    module->payload_length=payload_lenth-3;
	module->target_addr=target_addr;

	module->read_spi(module, &module->sender_addr, 1);
	uint8_t control;
	module->read_spi(module, &control, 1);
	module->read_spi(module, module->receive_buf, module->payload_length);
	module->set_nss(1);

	module->ack_received=(control & RFM69_CTL_SENDACK)>0;
	module->request_ack=(control & RFM69_CTL_REQACK)>0;
	module->first_packet=(control & RFM69_CTL_FIRST_PACKET)>0;
	module->last_packet=(control & RFM69_CTL_LAST_PACKET)>0;

	module->received_rssi=RFM69_read_RSSI(module);

	module->status=RFM69_STATUS_RECEIVED;
}

static void RFM69_prepare_receiving(RFM69_t *module) {
	if (RFM69_register_read(module, RFM69_RegIrqFlags2) & RFM69_IRQFLAGS2_PAYLOADREADY)
		RFM69_register_write(module, RFM69_RegPacketConfig2, (RFM69_register_read(module, RFM69_RegPacketConfig2) & 0xFB) | RFM69_PACKET2_RXRESTART); // avoid RX deadlocks
	// set DIO0 to signal "PayloadReady"
	RFM69_register_write(module, RFM69_RegDioMapping1, RFM69_DIOMAPPING1_DIO0_01);

	RFM69_set_mode(module, RFM69_OPMODE_RX);
}

void RFM69_start_listening(RFM69_t *module) {
	RFM69_prepare_receiving(module);
	module->status=RFM69_STATUS_RX;
}

void RFM69_interrupt_handler_io(RFM69_t *module) {
//	printStats("");
	if ((module->status==RFM69_STATUS_TX) || (module->status==RFM69_STATUS_SEND_ACK)) {
		if (RFM69_register_read(module, RFM69_RegIrqFlags2) & RFM69_IRQFLAGS2_PACKETSENT) {
			// packet sent
			if (module->request_ack) {
				RFM69_prepare_receiving(module);
				module->timer=0;
				module->status=RFM69_STATUS_RECEIVE_ACK;
			} else {
				RFM69_set_mode(module, RFM69_OPMODE_STANDBY);
				if (module->status==RFM69_STATUS_TX) {
					module->status=RFM69_STATUS_SENT;
				} else if (module->status==RFM69_STATUS_SEND_ACK) {
					module->status=RFM69_STATUS_RECEIVED_AND_ACK;
				}
			}
		}
	} else if (module->status==RFM69_STATUS_RECEIVE_ACK) {
		if (RFM69_register_read(module, RFM69_RegIrqFlags2) & RFM69_IRQFLAGS2_PAYLOADREADY) {
			RFM69_process_received_packet(module);
			if (module->status==RFM69_STATUS_RECEIVED) {
				module->status=RFM69_STATUS_ACK_RECEIVED;
			} else {
				RFM69_prepare_receiving(module);
			}
		}
	} else if (module->status==RFM69_STATUS_RX) {
		if (RFM69_register_read(module, RFM69_RegIrqFlags2) & RFM69_IRQFLAGS2_PAYLOADREADY) {
			RFM69_process_received_packet(module);
			if (module->status==RFM69_STATUS_RECEIVED) {
				if ((module->request_ack) && (module->target_addr!=module->broadcast_address)) {
					RFM69_send_packet(module, module->sender_addr, NULL, 0, false, true, true, true);
					module->timer=0;
					module->status=RFM69_STATUS_SEND_ACK;

				}
			} else {
				RFM69_prepare_receiving(module);
			}

		}
	}

}

// call inside timer with 1ms period
void RFM69_interrupt_handler_timer(RFM69_t *module) {
	if (module->timer<0xffff) module->timer++;

}

void RFM69_loop(RFM69_t *module) {
	// in case interrupt is not handled properly
//	RFM69_interrupt_handler_io(module);

	if (module->status==RFM69_STATUS_TX || module->status==RFM69_STATUS_SEND_ACK) {
		if (module->timer>1000) {
			RFM69_set_mode(module, RFM69_OPMODE_STANDBY);
			module->status=RFM69_STATUS_TX_TIMEDOUT;
		}
	} else if (module->status==RFM69_STATUS_RECEIVE_ACK) {
		if (module->timer>1000) {
			module->status=RFM69_STATUS_ACK_FAILED;
		}
	}
}
