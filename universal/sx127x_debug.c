#include "sx127x.h"

//#define SX127X_DEBUG_REGISTER_DETAILS	1

void SX127X_printDebug(SX127X_Handle *module, sx127x_debug_print_fn print_fn) {
	uint8_t reg;
	uint8_t val;
	uint8_t buf[20];
	bool loraMode=false;

#if SX127X_DEBUG_REGISTER_DETAILS==1
	uint8_t valPrev;
	uint8_t valPrev2;
	uint8_t tmp;
	bool isFSK=false;
	bool packetFormat;
	bool hfMode;
	uint32_t bw;
#endif

	for (reg = 1; reg <= 0x70; reg++) {
		if (loraMode) {
			if (reg>=0x02 && reg<=0x05) continue;
			if (reg==0x27) continue;
			if (reg==0x2b) continue;
			if (reg>=0x2d && reg<=0x30) continue;
			if (reg==0x32) continue;
			if (reg>=0x34 && reg<=0x36) continue;
			if (reg==0x38) continue;
			if (reg>=0x3a && reg<=0x3f) continue;
			if (reg==0x44) continue;
			if (reg==0x5d) continue;
		} else {
			if (reg>=0x17 && reg<=0x19) continue;
		}
		if (reg==0x43) continue;
		if (reg>=0x45 && reg<=0x4a) continue;
		if (reg==0x4c) continue;
		if (reg>=0x4e && reg<=0x5a) continue;
		if (reg>=0x5e && reg<=0x60) continue;
		if (reg>=0x65 && reg<=0x6f) continue;

#if SX127X_DEBUG_REGISTER_DETAILS==1
		valPrev2=valPrev;
		valPrev=val;
#endif

//		val=SX127X_SPIRead(module, reg);
		module->set_nss(0);
		module->read_reg(module, reg, &val, 1);
		module->set_nss(1);

		char2hex(buf, reg);
		print_fn((char*)buf);
		print_fn(", ");
		char2hex(buf, val);
		print_fn((char*)buf);
		print_fn("\n");
#if SX127X_DEBUG_REGISTER_DETAILS==1
		switch (reg) {
		case 0x01:
			loraMode=(val & 0b10000000)>0;
			print_fn("RegOpMode\n");
			print_fn("-LongRangeMode: ");
			print_fn(loraMode ? "1, LoRa Mode" : "0, FSK/OOK Mode");
			print_fn("\n");

			if (loraMode) {
				print_fn("-AccessSharedReg: ");
				print_fn((val & 0b01000000) ? "1, FSK registers" : "0, LoRa registers");
				print_fn("\n");

				hfMode=(val & 0b00001000)==0;
				print_fn("-LowFrequencyModeOn: ");
				print_fn(!hfMode ? "1, LF mode" : "0, HF mode");
				print_fn("\n");

				print_fn("-Mode: ");
				tmp=(val)&0b111;
				switch (tmp) {
				case 0b000:
					print_fn("000, Sleep");
					break;
				case 0b001:
					print_fn("001, Stdby");
					break;
				case 0b010:
					print_fn("010, FS mode TX");
					break;
				case 0b011:
					print_fn("011, TX");
					break;
				case 0b100:
					print_fn("100, FS mode RX");
					break;
				case 0b101:
					print_fn("101, RX continuous");
					break;
				case 0b110:
					print_fn("110, RX single");
					break;
				case 0b111:
					print_fn("111, Channel activity detection");
					break;
				default:
					break;
				}
				print_fn("\n");


			} else {

				tmp=(val>>5)&0b11;
				print_fn("-ModulationType: ");
				switch (tmp) {
				case 0b00:
					print_fn("00, FSK");
					isFSK=true;
					break;
				case 0b01:
					print_fn("01, OOK");
					break;
				case 0b10:
					print_fn("10, reserved");
					break;
				case 0b11:
					print_fn("11, reserved");
					break;
				default:
					break;
				}
				print_fn("\n");
				print_fn("-LowFrequencyModeOn: ");
				hfMode=(val & 0b00001000)==0;
				print_fn(!hfMode ? "1, LF Mode" : "0, HF Mode");
				print_fn("\n");
				print_fn("-Mode: ");
				tmp=(val)&0b111;
				switch (tmp) {
				case 0b000:
					print_fn("000, Sleep");
					break;
				case 0b001:
					print_fn("001, Stdby");
					break;
				case 0b010:
					print_fn("010, FS mode TX");
					break;
				case 0b011:
					print_fn("011, TX");
					break;
				case 0b100:
					print_fn("100, FS mode RX");
					break;
				case 0b101:
					print_fn("101, RX");
					break;
				case 0b110:
					print_fn("110, reserved");
					break;
				case 0b111:
					print_fn("111, reserved");
					break;
				default:
					break;
				}
				print_fn("\n");
			}


			break;
		case 0x02:
			print_fn("RegBitrateMsb\n");
			break;
		case 0x03:
			print_fn("RegBitrateLsb\n");

			//				uint8_t frac=SX127X_SPIRead(module, SX127X_FSK_REG_BITRATE_FRAC);
			uint8_t frac;
			module->set_nss(0);
			module->read_reg(module, SX127X_FSK_REG_BITRATE_FRAC, &frac, 1);
			module->set_nss(1);

			uint32_t br;
			br=16*SX127X_FXOSC;
			br=br/(16*((((uint16_t)valPrev)<<8) + val)+frac);

			print_fn("BitRate= ");
			long2string(buf, br);
			print_fn((char*)buf);
			print_fn(" bps\n");

			break;
		case 0x04:
			print_fn("RegFdevMsb, frequency deviation\n");
			break;
		case 0x05:
			print_fn("RegFdevLsb\n");

			uint64_t fdev=SX127X_FXOSC * ((((uint16_t)valPrev)<<8) + val);
			fdev=fdev>>19;

			print_fn("Fdev= ");
			long2string(buf, fdev);
			print_fn((char*)buf);
			print_fn(" Hz\n");

			break;
		case 0x06:
			print_fn("RegFrfMsb, RF carrier frequency\n");
			break;
		case 0x07:
			print_fn("RegFrfMid\n");
			break;
		case 0x08:
			print_fn("RegFrfLsb\n");

			uint64_t frf=SX127X_FXOSC;
			frf=frf *((((uint32_t)valPrev2)<<16) +(((uint16_t)valPrev)<<8) + val);
//			uint64_t frf=SX127X_FXOSC *((((uint32_t)valPrev2)<<16) +(((uint16_t)valPrev)<<8) + val);
			frf=frf>>19;

			print_fn("Frf= ");
			long2string(buf, frf);
			print_fn((char*)buf);
			print_fn(" Hz\n");

			break;
		case 0x09:
			print_fn("RegPaConfig\n");
			print_fn("-PaSelect: ");
			print_fn((val & 0b10000000) ? "1, PA_BOOST pin. Maximum power of +20 dBm" : "0, RFO pin. Maximum power of +14 dBm");
			bool pa_select=(val & 0b10000000)>0;
			print_fn("\n");
			print_fn("-MaxPower: Pmax= ");
			tmp=(val>>4)&0b111;
			float pmax=10.8f+0.6f*tmp;
			double2string(buf, pmax, 1, 5);
			print_fn((char*)buf);
			print_fn(" dBm\n");

			print_fn("-OutputPower: Pout= ");
			tmp=(val)&0b1111;
			float pout;
			if (pa_select) {
				pout=2+tmp;
			} else {
				pout=pmax-15+tmp;
			}
			double2string(buf, pout, 1, 5);
			print_fn((char*)buf);
			print_fn(" dBm\n");

			break;
		case 0x0a:
			print_fn("RegPaRamp\n");
			print_fn("-ModulationShaping: ");
			tmp=(val>>5)&0b11;
			switch (tmp) {
			case 0b00:
				print_fn("00, no shaping");
				break;
			case 0b01:
				print_fn(isFSK?"01, Gaussian filter BT = 1.0":"01, filtering with fcutoff = bit_rate");
				break;
			case 0b10:
				print_fn(isFSK?"10, Gaussian filter BT = 0.5":"10, filtering with fcutoff = 2*bit_rate (for bit_rate < 125 kb/s)");
				break;
			case 0b11:
				print_fn(isFSK?"11, Gaussian filter BT = 0.3":"11, reserved");
				break;
			default:
				break;
			}
			print_fn("\n");
			print_fn("-PaRamp: ");
			tmp=(val)&0b1111;
			switch (tmp) {
			case 0b0000:
				print_fn("0000, 3.4 ms");
				break;
			case 0b0001:
				print_fn("0001, 2 ms");
				break;
			case 0b0010:
				print_fn("0010, 1 ms");
				break;
			case 0b0011:
				print_fn("0011, 500 us");
				break;
			case 0b0100:
				print_fn("0100, 250 us");
				break;
			case 0b0101:
				print_fn("0101, 125 us");
				break;
			case 0b0110:
				print_fn("0110, 100 us");
				break;
			case 0b0111:
				print_fn("0111, 62 us");
				break;
			case 0b1000:
				print_fn("1000, 50 us");
				break;
			case 0b1001:
				print_fn("1001, 40 us (d)");
				break;
			case 0b1010:
				print_fn("1010, 31 us");
				break;
			case 0b1011:
				print_fn("1011, 25 us");
				break;
			case 0b1100:
				print_fn("1100, 20 us");
				break;
			case 0b1101:
				print_fn("1101, 15 us");
				break;
			case 0b1110:
				print_fn("1110, 12 us");
				break;
			case 0b1111:
				print_fn("1111, 10 us");
				break;
			default:
				break;
			}

			print_fn("\n");
			break;

		case 0x0b:
			print_fn("RegOcp, Over Current Protection control\n");
			print_fn("-OcpOn: ");
			print_fn((val & 0b10000000) ? "1, OCP enabled" : "0, OCP disabled");
			print_fn("\n");
			tmp=(val)&0b11111;
			print_fn("-OcpTrim (Trimming of OCP current): ");
			uint16_t imax;
			if (tmp<=15) {
				imax=45+tmp*5;
			} else if (tmp<=27) {
				imax=tmp*10-30;
			} else {
				imax=240;
			}
			long2string(buf, imax);
			print_fn((char*)buf);
			print_fn(" mA\n");

			break;
		case 0x0c:
			print_fn("RegLna, LNA settings\n");
			print_fn("-LnaGain: ");
			tmp=(val>>5)&0b111;
			switch (tmp) {
			case 0b000:
				print_fn("000, reserved");
				break;
			case 0b001:
				print_fn("001, G1 = highest gain");
				break;
			case 0b010:
				print_fn("010, G2 = highest gain, 6 dB");
				break;
			case 0b011:
				print_fn("011, G3 = highest gain, 12 dB");
				break;
			case 0b100:
				print_fn("100, G4 = highest gain, 24 dB");
				break;
			case 0b101:
				print_fn("101, G5 = highest gain, 36 dB");
				break;
			case 0b110:
				print_fn("110, G6 = highest gain, 48 dB");
				break;
			case 0b111:
				print_fn("111, reserved");
				break;
			default:
				break;
			}
			print_fn("\n");

			print_fn("-LnaBoostLf: ");
			tmp=(val>>3)&0b11;
			switch (tmp) {
			case 0b00:
				print_fn("00, Default LNA current");
				break;
			case 0b01:
				print_fn("01, reserved");
				break;
			case 0b10:
				print_fn("10, reserved");
				break;
			case 0b11:
				print_fn("11, reserved");
				break;
			default:
				break;
			}
			print_fn("\n");

			print_fn("-LnaBoostHf: ");
			tmp=(val)&0b11;
			switch (tmp) {
			case 0b00:
				print_fn("00, Default LNA current");
				break;
			case 0b01:
				print_fn("01, reserved");
				break;
			case 0b10:
				print_fn("10, reserved");
				break;
			case 0b11:
				print_fn("11, Boost on, 150% LNA current");
				break;
			default:
				break;
			}
			print_fn("\n");


			break;
		case 0x0d:
			if (loraMode) {
				print_fn("RegFifoAddrPtr\n");
			} else {
				print_fn("RegRxConfig\n");
				print_fn("-RestartRxOnCollision: ");
				print_fn((val & 0b10000000)>0 ? "1, Automatic restart On" : "0, No automatic Restart");
				print_fn("\n");

				print_fn("-RestartRxWithoutPllLock: ");
				print_fn((val & 0b01000000)>0 ? "1" : "0");
				print_fn("\n");

				print_fn("-RestartRxWithPllLock: ");
				print_fn((val & 0b00100000)>0 ? "1" : "0");
				print_fn("\n");

				bool afcAutoOn=(val & 0b00010000)>0;
				print_fn("-AfcAutoOn: ");
				print_fn(afcAutoOn ? "1" : "0");
				print_fn("\n");

				bool agcAutoOn=(val & 0b00001000)>0;
				print_fn("-AgcAutoOn: ");
				print_fn(agcAutoOn ? "1" : "0");
				print_fn("\n");

				print_fn("-RxTrigger: ");
				tmp=(val)&0b111;
				switch (tmp) {
				case 0b000:
					print_fn("000, None");
					break;
				case 0b001:
					print_fn("001, Rssi Interrupt");
					break;
				case 0b010:
					print_fn("010, unknown");
					break;
				case 0b011:
					print_fn("011, unknown");
					break;
				case 0b100:
					print_fn("100, unknown");
					break;
				case 0b101:
					print_fn("101, unknown");
					break;
				case 0b110:
					print_fn("110, PreambleDetect");
					break;
				case 0b111:
					print_fn("111, Rssi Interrupt & PreambleDetect");
					break;
				default:
					break;
				}
				print_fn("\n");

			}
			break;
		case 0x0e:
			if (loraMode) {
				print_fn("RegFifoTxBaseAddr\n");
			} else {
				print_fn("RegRssiConfig\n");
				int8_t sig=(int8_t)val;
				sig=sig/8;
				print_fn("-RssiOffset: ");
				long2string(buf, sig);
				print_fn((char*)buf);
				print_fn(" dB\n");

				print_fn("-RssiSmoothing: ");
				tmp=(val)&0b111;
				switch (tmp) {
				case 0b000:
					print_fn("000, 2");
					break;
				case 0b001:
					print_fn("001, 4");
					break;
				case 0b010:
					print_fn("010, 8");
					break;
				case 0b011:
					print_fn("011, 16");
					break;
				case 0b100:
					print_fn("100, 32");
					break;
				case 0b101:
					print_fn("101, 64");
					break;
				case 0b110:
					print_fn("110, 128");
					break;
				case 0b111:
					print_fn("111, 256");
					break;
				default:
					break;
				}
				print_fn(" samples used\n");

			}
			break;
		case 0x0f:
			if (loraMode) {
				print_fn("RegFifoRxBaseAddr\n");
			} else {
				print_fn("RegRssiCollision\n");
				print_fn("-RssiCollisionThreshold: ");
				long2string(buf, val);
				print_fn((char*)buf);
				print_fn(" dB\n");

			}
			break;
		case 0x10:
			if (loraMode) {
				print_fn("RegFifoRxCurrentAddr\n");
			} else {
				print_fn("RegRssiThresh\n");
				print_fn("-RssiThreshold: ");
				float f=0.5*val;
				double2string(buf, f, 1, 5);
				print_fn((char*)buf);
				print_fn(" dBm\n");
			}
			break;
		case 0x11:
			if (loraMode) {
				print_fn("RegIrqFlagsMask\n");

				print_fn("-RxTimeoutMask: ");
				print_fn((val & 0b10000000)>0 ? "1" : "0");
				print_fn("\n");

				print_fn("-RxDoneMask: ");
				print_fn((val & 0b01000000)>0 ? "1" : "0");
				print_fn("\n");

				print_fn("-PayloadCrcErrorMask: ");
				print_fn((val & 0b00100000)>0 ? "1" : "0");
				print_fn("\n");

				print_fn("-ValidHeaderMask: ");
				print_fn((val & 0b00010000)>0 ? "1" : "0");
				print_fn("\n");

				print_fn("-TxDoneMask: ");
				print_fn((val & 0b00001000)>0 ? "1" : "0");
				print_fn("\n");

				print_fn("-CadDoneMask: ");
				print_fn((val & 0b00000100)>0 ? "1" : "0");
				print_fn("\n");

				print_fn("-FhssChangeChannelMask: ");
				print_fn((val & 0b00000010)>0 ? "1" : "0");
				print_fn("\n");

				print_fn("-CadDetectedMask: ");
				print_fn((val & 0b00000001)>0 ? "1" : "0");
				print_fn("\n");
			} else {
				print_fn("RegRssiValue\n");
				print_fn("-RssiValue: ");
				float f=-0.5*val;
				double2string(buf, f, 1, 5);
				print_fn((char*)buf);
				print_fn(" dBm\n");
			}
			break;
		case 0x12:
			if (loraMode) {
				print_fn("RegIrqFlags\n");

				print_fn("-RxTimeout: ");
				print_fn((val & 0b10000000)>0 ? "1" : "0");
				print_fn("\n");

				print_fn("-RxDone: ");
				print_fn((val & 0b01000000)>0 ? "1" : "0");
				print_fn("\n");

				print_fn("-PayloadCrcError: ");
				print_fn((val & 0b00100000)>0 ? "1" : "0");
				print_fn("\n");

				print_fn("-ValidHeader: ");
				print_fn((val & 0b00010000)>0 ? "1" : "0");
				print_fn("\n");

				print_fn("-TxDone: ");
				print_fn((val & 0b00001000)>0 ? "1" : "0");
				print_fn("\n");

				print_fn("-CadDone: ");
				print_fn((val & 0b00000100)>0 ? "1" : "0");
				print_fn("\n");

				print_fn("-FhssChangeChannel: ");
				print_fn((val & 0b00000010)>0 ? "1" : "0");
				print_fn("\n");

				print_fn("-CadDetected: ");
				print_fn((val & 0b00000001)>0 ? "1" : "0");
				print_fn("\n");
			} else {
				print_fn("RegRxBw\n");
				print_fn("-RxBwMant: ");
				tmp=(val>>3)&0b11;
				switch (tmp) {
				case 0b00:
					print_fn("00, RxBwMant = 16");
					break;
				case 0b01:
					print_fn("01, RxBwMant = 20");
					break;
				case 0b10:
					print_fn("10, RxBwMant = 24");
					break;
				case 0b11:
					print_fn("11, reserved");
					break;
				default:
					break;
				}
				print_fn("\n");

				print_fn("-RxBwExp: ");
				tmp=(val)&0b111;
				print_fn((char*)buf);
				print_fn("\n");

			}
			break;

		case 0x13:
			if (loraMode) {

			} else {
				print_fn("RegAfcBw\n");
				print_fn("-RxBwMantAfc: ");
				tmp=(val>>3)&0b11;
				switch (tmp) {
				case 0b00:
					print_fn("00, RxBwMant = 16");
					break;
				case 0b01:
					print_fn("01, RxBwMant = 20");
					break;
				case 0b10:
					print_fn("10, RxBwMant = 24");
					break;
				case 0b11:
					print_fn("11, reserved");
					break;
				default:
					break;
				}
				print_fn("\n");

				print_fn("-RxBwExpAfc: ");
				tmp=(val)&0b111;
				print_fn((char*)buf);
				print_fn("\n");

			}
			break;
		case 0x14:
			if (loraMode) {

			} else {
				print_fn("RegOokPeak\n");
				print_fn("-BitSyncOn: ");
				print_fn((val & 0b00100000)>0 ? "1" : "0");
				print_fn("\n");

				print_fn("-OokThreshType: ");
				tmp=(val>>3)&0b11;
				switch (tmp) {
				case 0b00:
					print_fn("00, fixed threshold");
					break;
				case 0b01:
					print_fn("01, peak mode (default)");
					break;
				case 0b10:
					print_fn("10, average mode");
					break;
				case 0b11:
					print_fn("11, reserved");
					break;
				default:
					break;
				}
				print_fn("\n");

				print_fn("-OokPeakTheshStep: ");
				tmp=(val)&0b111;
				switch (tmp) {
				case 0b000:
					print_fn("000, 0.5 dB");
					break;
				case 0b001:
					print_fn("001, 1.0 dB");
					break;
				case 0b010:
					print_fn("010, 1.5 dB");
					break;
				case 0b011:
					print_fn("011, 2.0 dB");
					break;
				case 0b100:
					print_fn("100, 3.0 dB");
					break;
				case 0b101:
					print_fn("101, 4.0 dB");
					break;
				case 0b110:
					print_fn("110, 5.0 dB");
					break;
				case 0b111:
					print_fn("111, 6.0 dB");
					break;
				default:
					break;
				}
				print_fn("\n");

			}
			break;
		case 0x15:
			if (loraMode) {

			} else {
				print_fn("RegOokFix\n");
				print_fn("-OokFixedThreshold: ");
				long2string(buf, val);
				print_fn((char*)buf);
				print_fn("\n");

			}
			break;
		case 0x16:
			if (loraMode) {

			} else {
				print_fn("RegOokAvg\n");
				print_fn("-OokPeakThreshDec: ");
				tmp=(val>>5)&0b111;
				switch (tmp) {
				case 0b000:
					print_fn("000, once per chip");
					break;
				case 0b001:
					print_fn("001, once every 2 chips");
					break;
				case 0b010:
					print_fn("010, once every 4 chips");
					break;
				case 0b011:
					print_fn("011, once every 8 chips");
					break;
				case 0b100:
					print_fn("100, twice in each chip");
					break;
				case 0b101:
					print_fn("101, 4 times in each chip");
					break;
				case 0b110:
					print_fn("110, 8 times in each chip");
					break;
				case 0b111:
					print_fn("111, 16 times in each chip");
					break;
				default:
					break;
				}
				print_fn("\n");


				print_fn("-OokAverageOffset: ");
				tmp=(val>>2)&0b11;
				switch (tmp) {
				case 0b00:
					print_fn("00");
					break;
				case 0b01:
					print_fn("01");
					break;
				case 0b10:
					print_fn("10");
					break;
				case 0b11:
					print_fn("11");
					break;
				default:
					break;
				}
				long2string(buf, tmp*2);
				print_fn((char*)buf);
				print_fn(".0 dB\n");

				print_fn("-OokAverageThreshFilt: ");
				tmp=(val)&0b11;
				switch (tmp) {
				case 0b00:
					print_fn("00");
					break;
				case 0b01:
					print_fn("01");
					break;
				case 0b10:
					print_fn("10");
					break;
				case 0b11:
					print_fn("11");
					break;
				default:
					break;
				}
				print_fn("\n");


			}
			break;
		case 0x19:
			if (loraMode) {
				print_fn("RegPktSnrValue\n");
				float f=0.25*((int8_t)val);
				double2string(buf, f, 2, 5);
				print_fn((char*)buf);
				print_fn("\n");
			} else {
			}
			break;
		case 0x1a:
			if (loraMode) {
				print_fn("RegPktRssiValue\n");
				int16_t rssi=val-(hfMode?157:164);
				long2string(buf, rssi);
				print_fn((char*)buf);
				print_fn(" dBm\n");
			} else {
			}
			break;
		case 0x1b:
			if (loraMode) {
				print_fn("RegRssiValue\n");
				int16_t rssi=val-(hfMode?157:164);
				long2string(buf, rssi);
				print_fn((char*)buf);
				print_fn(" dBm\n");
			} else {
			}
			break;
		case 0x1d:
			if (loraMode) {
				print_fn("RegModemConfig1\n");
				print_fn("-Bw: ");
				tmp=(val>>4)&0b1111;
				switch (tmp) {
				case 0b0000:
					print_fn("0000, 7.8 kHz");
					bw=7800;
					break;
				case 0b0001:
					print_fn("0001, 10.4 kHz");
					bw=10400;
					break;
				case 0b0010:
					print_fn("0010, 15.6 kHz");
					bw=15600;
					break;
				case 0b0011:
					print_fn("0011, 20.8 kHz");
					bw=20800;
					break;
				case 0b0100:
					print_fn("0100, 31.25 kHz");
					bw=31250;
					break;
				case 0b0101:
					print_fn("0101, 41.7 kHz");
					bw=41700;
					break;
				case 0b0110:
					print_fn("0110, 62.5 kHz");
					bw=62500;
					break;
				case 0b0111:
					print_fn("0111, 125 kHz");
					bw=125000;
					break;
				case 0b1000:
					print_fn("1000, 250 kHz");
					bw=250000;
					break;
				case 0b1001:
					print_fn("1001, 500 kHz");
					bw=500000;
					break;
				case 0b1010:
					print_fn("1010, reserved");
					break;
				case 0b1011:
					print_fn("1011, reserved");
					break;
				case 0b1100:
					print_fn("1100, reserved");
					break;
				case 0b1101:
					print_fn("1101, reserved");
					break;
				case 0b1110:
					print_fn("1110, reserved");
					break;
				case 0b1111:
					print_fn("1111, reserved");
					break;
				default:
					break;
				}
				print_fn("\n");

				print_fn("-CodingRate: ");
				tmp=(val>>1)&0b111;
				switch (tmp) {
				case 0b000:
					print_fn("000, reserved");
					break;
				case 0b001:
					print_fn("001, 4/5");
					break;
				case 0b010:
					print_fn("010, 4/6");
					break;
				case 0b011:
					print_fn("011, 4/7");
					break;
				case 0b100:
					print_fn("100, 4/8");
					break;
				case 0b101:
					print_fn("101, reserved");
					break;
				case 0b110:
					print_fn("110, reserved");
					break;
				case 0b111:
					print_fn("111, reserved");
					break;
				default:
					break;
				}
				print_fn("\n");

				print_fn("-ImplicitHeaderModeOn: ");
				print_fn((val & 0b00000001)>0 ? "1" : "0");
				print_fn("\n");

			} else {
			}
			break;
		case 0x1e:
			if (loraMode) {
				print_fn("RegModemConfig2\n");
				print_fn("-SpreadingFactor: ");
				tmp=(val>>4)&0b1111;
				long2string(buf, tmp);
				print_fn((char*)buf);
				print_fn(" - ");
				uint16_t sf=1<<tmp;
				long2string(buf, sf);
				print_fn((char*)buf);
				print_fn(" chips / symbol\n");

				print_fn("-TxContinuousMode: ");
				print_fn((val & 0b00001000)>0 ? "1" : "0");
				print_fn("\n");

				print_fn("-RxPayloadCrcOn: ");
				print_fn((val & 0b00000100)>0 ? "1" : "0");
				print_fn("\n");

				print_fn("-SymbTimeout (MSB): ");
				tmp=(val)&0b11;
				long2string(buf, tmp);
				print_fn((char*)buf);
				print_fn("\n");

			} else {
			}
			break;
		case 0x1f:
			if (loraMode) {
				print_fn("-SymbTimeout: ");
				uint16_t sy=(valPrev)&0b11;
				sy=(sy<<8) + val;
				long2string(buf, sy);
				print_fn((char*)buf);
				print_fn("\n");
			} else {
				print_fn("RegPreambleDetect\n");
				print_fn("-PreambleDetectorOn: ");
				print_fn((val & 0b10000000)>0 ? "1" : "0");
				print_fn("\n");

				print_fn("-PreambleDetectorSize: ");
				tmp=(val>>5)&0b11;
				switch (tmp) {
				case 0b00:
					print_fn("00, 1 byte");
					break;
				case 0b01:
					print_fn("01, 2 bytes");
					break;
				case 0b10:
					print_fn("10, 3 bytes");
					break;
				case 0b11:
					print_fn("11, reserved");
					break;
				default:
					break;
				}
				print_fn("\n");

				print_fn("-PreambleDetectorTol: ");
				tmp=(val)&0b11111;
				long2string(buf, val);
				print_fn((char*)buf);
				print_fn("\n");

			}
			break;
		case 0x20:
			if (loraMode) {
				print_fn("RegPreambleMsb\n");
			} else {
			}
			break;
		case 0x21:
			if (loraMode) {
				print_fn("RegPreambleLsb\n");
				print_fn("-PreambleLength: ");
				uint16_t len=((uint16_t)valPrev<<8)+val;

				long2string(buf, len);
				print_fn((char*)buf);
				print_fn("\n");
			} else {
			}
			break;
		case 0x22:
			if (loraMode) {
				print_fn("RegPayloadLength\n");
			} else {
			}
			break;
		case 0x23:
			if (loraMode) {
				print_fn("RegMaxPayloadLength\n");
			} else {
			}
			break;
		case 0x24:
			if (loraMode) {

			} else {
				print_fn("RegOsc\n");
				print_fn("-RcCalStart: ");
				print_fn((val & 0b10000000)>0 ? "1" : "0");
				print_fn("\n");

				print_fn("-ClkOut: ");
				tmp=(val>>5)&0b111;
				switch (tmp) {
				case 0b000:
					print_fn("000, FXOSC");
					break;
				case 0b001:
					print_fn("001, FXOSC / 2");
					break;
				case 0b010:
					print_fn("010, FXOSC / 4");
					break;
				case 0b011:
					print_fn("011, FXOSC / 8");
					break;
				case 0b100:
					print_fn("100, FXOSC / 16");
					break;
				case 0b101:
					print_fn("101, FXOSC / 32");
					break;
				case 0b110:
					print_fn("110, RC (default)");
					break;
				case 0b111:
					print_fn("111, OFF");
					break;
				default:
					break;
				}
				print_fn("\n");


			}
			break;
		case 0x25:
			if (loraMode) {
				print_fn("RegFifoRxByteAddr\n");
			} else {
				print_fn("RegPreambleMsb\n");
			}
			break;
		case 0x26:
			if (loraMode) {

			} else {
				print_fn("RegPreambleLsb\n");

				uint16_t pre= ((((uint16_t)valPrev)<<8) + val);

				print_fn("Size= ");
				long2string(buf, pre);
				print_fn((char*)buf);
				print_fn("\n");
			}

			break;
		case 0x27:
			if (loraMode) {

			} else {
				print_fn("RegSyncConfig\n");

				print_fn("-AutoRestartRxMode: ");
				tmp=(val>>6)&0b11;
				switch (tmp) {
				case 0b00:
					print_fn("00, Off");
					break;
				case 0b01:
					print_fn("01, On");
					break;
				case 0b10:
					print_fn("10, On, wait PLL");
					break;
				case 0b11:
					print_fn("11, reserved");
					break;
				default:
					break;
				}
				print_fn("\n");

				print_fn("-PreamblePolarity: ");
				print_fn((val & 0b00100000)>0 ? "1, 0x55" : "0, 0xAA (default)");
				print_fn("\n");

				print_fn("-SyncOn: ");
				print_fn((val & 0b00100000)>0 ? "1" : "0");
				print_fn("\n");

				uint8_t cfg;
				module->set_nss(0);
				module->read_reg(module, SX127X_FSK_REG_PACKET_CONFIG2, &cfg, 1);
				module->set_nss(1);

				print_fn("-SyncSize: ");
				tmp=(val)&0b111;
				if ((cfg & 0b00100000)==0) {
					// IoHomeOn==0
					tmp++;
				}
				long2string(buf, tmp);
				print_fn((char*)buf);
				print_fn("\n");


			}
			break;
		case 0x28:
			if (loraMode) {

			} else {
				print_fn("RegSyncValue1, Sync word (MSB)\n");
			}
			break;
		case 0x29:
			if (loraMode) {

			} else {
				print_fn("RegSyncValue2\n");
			}
			break;
		case 0x2a:
			if (loraMode) {
				print_fn("-RegFei: ");
				uint32_t freq=((uint32_t)(valPrev2 & 0x0f)<<16)+((uint32_t)valPrev<<8) +val;
				uint64_t fe=freq<<24;
				fe=(fe*bw)/((uint32_t)500*SX127X_FXOSC);
				long2string(buf, fe);
				print_fn((char*)buf);
				print_fn("\n");

			} else {
				print_fn("RegSyncValue3\n");
			}
			break;
		case 0x2b:
			if (loraMode) {

			} else {
				print_fn("RegSyncValue4\n");
			}
			break;
		case 0x2c:
			if (loraMode) {

			} else {
				print_fn("RegSyncValue5\n");
			}
			break;
		case 0x2d:
			if (loraMode) {

			} else {
				print_fn("RegSyncValue6\n");
			}
			break;
		case 0x2e:
			if (loraMode) {

			} else {
				print_fn("RegSyncValue7\n");
			}
			break;
		case 0x2f:
			if (loraMode) {

			} else {
				print_fn("RegSyncValue8\n");
			}
			break;
		case 0x30:
			if (loraMode) {

			} else {
				print_fn("RegPacketConfig1\n");

				packetFormat=(val & 0b10000000)>0;
				print_fn("-PacketFormat: ");
				print_fn(packetFormat ? "1, variable length" : "0, fixed length");
				print_fn("\n");

				print_fn("-DcFree: ");
				tmp=(val>>5)&0b11;
				switch (tmp) {
				case 0b00:
					print_fn("00, None");
					break;
				case 0b01:
					print_fn("01, Manchester");
					break;
				case 0b10:
					print_fn("10, Whitening");
					break;
				case 0b11:
					print_fn("11, reserved");
					break;
				default:
					break;
				}
				print_fn("\n");

				print_fn("-CrcOn: ");
				print_fn((val & 0b00010000)>0 ? "1" : "0");
				print_fn("\n");

				print_fn("-CrcAutoClearOff: ");
				print_fn((val & 0b00001000)>0 ? "1" : "0");
				print_fn("\n");

				print_fn("-AddressFiltering: ");
				tmp=(val>>1)&0b11;
				switch (tmp) {
				case 0b00:
					print_fn("00, None");
					break;
				case 0b01:
					print_fn("01, match NodeAddress");
					break;
				case 0b10:
					print_fn("10, match NodeAddress or BroadcastAddress");
					break;
				case 0b11:
					print_fn("11, reserved");
					break;
				default:
					break;
				}
				print_fn("\n");

				print_fn("-CrcWhiteningType: ");
				print_fn((val & 0b00000001)>0 ? "1, IBM CRC" : "0, CCITT CRC");
				print_fn("\n");

			}
			break;
		case 0x31:
			if (loraMode) {

			} else {
				print_fn("RegPacketConfig2\n");

				print_fn("-DataMode: ");
				print_fn((val & 0b01000000)>0 ? "1, Packet mode" : "0, Continuous mode");
				print_fn("\n");

				print_fn("-IoHomeOn: ");
				print_fn((val & 0b00100000)>0 ? "1" : "0");
				print_fn("\n");

				print_fn("-BeaconOn: ");
				print_fn((val & 0b00001000)>0 ? "1" : "0");
				print_fn("\n");

				print_fn("-PayloadLength (MSB): ");
				tmp=(val)&0b111;
				long2string(buf, tmp);
				print_fn((char*)buf);
				print_fn("\n");

			}
			break;
		case 0x32:
			if (loraMode) {

			} else {
				print_fn("RegPayloadLength (LSB)\n");
				print_fn("-PayloadLength ");
				if (packetFormat) {
					print_fn("(max length in Rx)");
				} else {
					print_fn("(payload length)");
				}
				print_fn(": ");
				uint16_t len=valPrev&0b111;
				len=len<<8;
				len+=val;
				long2string(buf, len);
				print_fn((char*)buf);
				print_fn("\n");
			}
			break;
		case 0x33:
			print_fn("RegNodeAdrs\n");
			break;
		case 0x34:
			print_fn("RegBroadcastAdrs\n");
			break;
		case 0x35:
			if (loraMode) {

			} else {
				print_fn("RegFifoThresh\n");
				print_fn("-TxStartCondition: ");
				print_fn((val & 0b10000000)>0 ? "1, FifoEmpty goes low" : "0, FifoLevel");
				print_fn("\n");

				print_fn("-FifoThreshold: ");
				tmp=(val)&0b111111;
				long2string(buf, tmp);
				print_fn((char*)buf);
				print_fn("\n");


			}
			break;
		case 0x36:
			if (loraMode) {

			} else {
				print_fn("RegSeqConfig1\n");
			}
			break;
		case 0x37:
			if (loraMode) {
				print_fn("RegDetectionThreshold\n");
			} else {
				print_fn("RegSeqConfig2\n");

				print_fn("-FromReceive: ");
				tmp=(val>>5)&0b111;
				switch (tmp) {
				case 0b000:
					print_fn("000");
					break;
				case 0b001:
					print_fn("001");
					break;
				case 0b010:
					print_fn("010");
					break;
				case 0b011:
					print_fn("011");
					break;
				case 0b100:
					print_fn("100");
					break;
				case 0b101:
					print_fn("101");
					break;
				case 0b110:
					print_fn("110");
					break;
				case 0b111:
					print_fn("111");
					break;
				default:
					break;
				}
				print_fn("\n");

				print_fn("-FromRxTimeout: ");
				tmp=(val>>3)&0b11;
				switch (tmp) {
				case 0b00:
					print_fn("00");
					break;
				case 0b01:
					print_fn("01");
					break;
				case 0b10:
					print_fn("10");
					break;
				case 0b11:
					print_fn("11");
					break;
				default:
					break;
				}
				print_fn("\n");

				print_fn("-FromPacketReceived: ");
				tmp=(val)&0b111;
				switch (tmp) {
				case 0b000:
					print_fn("000");
					break;
				case 0b001:
					print_fn("001");
					break;
				case 0b010:
					print_fn("010");
					break;
				case 0b011:
					print_fn("011");
					break;
				case 0b100:
					print_fn("100");
					break;
				case 0b101:
					print_fn("101");
					break;
				case 0b110:
					print_fn("110");
					break;
				case 0b111:
					print_fn("111");
					break;
				default:
					break;
				}
				print_fn("\n");

			}
			break;
		case 0x39:
			if (loraMode) {
				print_fn("RegSyncWord\n");
			} else {

			}
			break;
		case 0x3c:
			print_fn("RegTemp\n");
			int8_t t= ~val-(int8_t)91;
			long2string(buf, t);
			print_fn((char*)buf);
			print_fn(" C\n");

			break;
		case 0x3e:
			if (loraMode) {

			} else {
				print_fn("RegIrqFlags1\n");

				print_fn("-ModeReady: ");
				print_fn((val & 0b10000000)>0 ? "1" : "0");
				print_fn("\n");

				print_fn("-RxReady: ");
				print_fn((val & 0b01000000)>0 ? "1" : "0");
				print_fn("\n");

				print_fn("-TxReady: ");
				print_fn((val & 0b00100000)>0 ? "1" : "0");
				print_fn("\n");

				print_fn("-PllLock: ");
				print_fn((val & 0b00010000)>0 ? "1" : "0");
				print_fn("\n");

				print_fn("-Rssi: ");
				print_fn((val & 0b00001000)>0 ? "1" : "0");
				print_fn("\n");

				print_fn("-Timeout: ");
				print_fn((val & 0b00000100)>0 ? "1" : "0");
				print_fn("\n");

				print_fn("-PreambleDetect: ");
				print_fn((val & 0b00000010)>0 ? "1" : "0");
				print_fn("\n");

				print_fn("-SyncAddressMatch: ");
				print_fn((val & 0b00000001)>0 ? "1" : "0");
				print_fn("\n");


			}
			break;
		case 0x3f:
			if (loraMode) {

			} else {
				print_fn("RegIrqFlags2\n");

				print_fn("-FifoFull: ");
				print_fn((val & 0b10000000)>0 ? "1" : "0");
				print_fn("\n");

				print_fn("-FifoEmpty: ");
				print_fn((val & 0b01000000)>0 ? "1" : "0");
				print_fn("\n");

				print_fn("-FifoLevel: ");
				print_fn((val & 0b00100000)>0 ? "1" : "0");
				print_fn("\n");

				print_fn("-FifoOverrun: ");
				print_fn((val & 0b00010000)>0 ? "1" : "0");
				print_fn("\n");

				print_fn("-PacketSent: ");
				print_fn((val & 0b00001000)>0 ? "1" : "0");
				print_fn("\n");

				print_fn("-PayloadReady: ");
				print_fn((val & 0b00000100)>0 ? "1" : "0");
				print_fn("\n");

				print_fn("-CrcOk: ");
				print_fn((val & 0b00000010)>0 ? "1" : "0");
				print_fn("\n");

				print_fn("-LowBat: ");
				print_fn((val & 0b00000001)>0 ? "1" : "0");
				print_fn("\n");


			}
			break;

		case 0x40:
			print_fn("RegDioMapping1\n");
			tmp=(val>>6)&0b11;
			print_fn("-Dio0Mapping: ");
			switch (tmp) {
			case 0b00:
				print_fn("00");
				break;
			case 0b01:
				print_fn("01");
				break;
			case 0b10:
				print_fn("10");
				break;
			case 0b11:
				print_fn("11");
				break;
			default:
				break;
			}
			print_fn("\n");

			tmp=(val>>4)&0b11;
			print_fn("-Dio1Mapping: ");
			switch (tmp) {
			case 0b00:
				print_fn("00");
				break;
			case 0b01:
				print_fn("01");
				break;
			case 0b10:
				print_fn("10");
				break;
			case 0b11:
				print_fn("11");
				break;
			default:
				break;
			}
			print_fn("\n");

			tmp=(val>>2)&0b11;
			print_fn("-Dio2Mapping: ");
			switch (tmp) {
			case 0b00:
				print_fn("00");
				break;
			case 0b01:
				print_fn("01");
				break;
			case 0b10:
				print_fn("10");
				break;
			case 0b11:
				print_fn("11");
				break;
			default:
				break;
			}
			print_fn("\n");

			tmp=(val)&0b11;
			print_fn("-Dio3Mapping: ");
			switch (tmp) {
			case 0b00:
				print_fn("00");
				break;
			case 0b01:
				print_fn("01");
				break;
			case 0b10:
				print_fn("10");
				break;
			case 0b11:
				print_fn("11");
				break;
			default:
				break;
			}
			print_fn("\n");

			break;
		case 0x41:
			print_fn("RegDioMapping2\n");
			tmp=(val>>6)&0b11;
			print_fn("-Dio4Mapping: ");
			switch (tmp) {
			case 0b00:
				print_fn("00");
				break;
			case 0b01:
				print_fn("01");
				break;
			case 0b10:
				print_fn("10");
				break;
			case 0b11:
				print_fn("11");
				break;
			default:
				break;
			}
			print_fn("\n");

			tmp=(val>>4)&0b11;
			print_fn("-Dio5Mapping: ");
			switch (tmp) {
			case 0b00:
				print_fn("00");
				break;
			case 0b01:
				print_fn("01");
				break;
			case 0b10:
				print_fn("10");
				break;
			case 0b11:
				print_fn("11");
				break;
			default:
				break;
			}
			print_fn("\n");


			print_fn("-MapPreambleDetect: ");
			print_fn((val & 0b1) ? "1, PreambleDetect interrupt" : "0, Rssi interrupt");
			print_fn("\n");

			break;
		case 0x42:
			print_fn("RegVersion\n");
			print_fn("-Version: 0x");
			char2hex(buf, val);
			print_fn((char*)buf);
			print_fn("\n");

			break;
		case 0x5d:
			if (loraMode) {

			} else {
				print_fn("RegBitrateFrac\n");
				long2string(buf, val & 0x0f);
				print_fn((char*)buf);
				print_fn("\n");


			}
			break;
		case 0x70:
			print_fn("RegPllHf\n");
			tmp=(val>>6)&0b11;
			print_fn("-PllBandwidth: ");
			switch (tmp) {
			case 0b00:
				print_fn("00, 75 kHz");
				break;
			case 0b01:
				print_fn("01, 150 kHz");
				break;
			case 0b10:
				print_fn("10, 225 kHz");
				break;
			case 0b11:
				print_fn("11, 300 kHz");
				break;
			default:
				break;
			}
			print_fn("\n");

			tmp=(val>>4)&0b11;
			print_fn("-Dio5Mapping: ");
			switch (tmp) {
			case 0b00:
				print_fn("00");
				break;
			case 0b01:
				print_fn("01");
				break;
			case 0b10:
				print_fn("10");
				break;
			case 0b11:
				print_fn("11");
				break;
			default:
				break;
			}
			print_fn("\n");


			print_fn("-MapPreambleDetect: ");
			print_fn((val & 0b1) ? "1, PreambleDetect interrupt" : "0, Rssi interrupt");
			print_fn("\n");

			break;
		default:
			break;
		}
#endif
		print_fn("\n");
	}

}

void SX127X_printFifo(SX127X_Handle *module, sx127x_debug_print_fn print_fn, uint8_t *buf) {
	uint8_t d=0;
//	SX127X_SPIWrite(module, SX127X_LORA_REG_FIFO_ADDR_PTR, 0);
	module->set_nss(0);
	module->write_reg(module, SX127X_LORA_REG_FIFO_ADDR_PTR | 0x80, &d, 1);
	module->set_nss(1);

	uint16_t q=0;
	print_fn("FIFO:\n");
	for (q=0; q<=0xff; q++) {
//		uint8_t d = SX127X_SPIRead(module, SX127X_REG_FIFO);

		module->set_nss(0);
		module->read_reg(module, SX127X_REG_FIFO, &d, 1);
		module->set_nss(1);
		char2hex(buf, d);
		print_fn((char*)buf);
		print_fn(" ");
		if (q>0 && q%16==15) {
			print_fn("\n");
		}
	}
	print_fn("\n");
}
