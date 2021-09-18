/**
 * based on: Wojciech Domski <Wojciech.Domski@gmail.com>
 * www: www.Domski.pl
 *
 * work based on DORJI.COM sample code and
 * https://github.com/realspinner/SX1278_LoRa
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "device.h"

#ifndef SX127X_H
#define	SX127X_H

// SX1276   137 - 1020 MHz
// SX1277   137 - 1020 MHz
// SX1278   137 - 525 MHz
// SX1279   137 - 960MHz

#define SX127X_MAX_PACKET		   256
#define SX127X_DEFAULT_TIMEOUT		3000

//SX127x Internal registers Address

/********************Common registers***************************/
// Common settings
#define SX127X_RegFifo						  0x00
#define SX127X_RegOpMode						0x01
#define SX127X_RegFrMsb						 0x06
#define SX127X_RegFrMid						 0x07
#define SX127X_RegFrLsb						 0x08
// Tx settings
#define SX127X_RegPaConfig					  0x09
#define SX127X_RegPaRamp						0x0A
#define SX127X_RegOcp						   0x0B
// Rx settings
#define SX127X_RegLna						   0x0C
// I/O settings
#define SX127X_RegDioMapping1				   0x40
#define SX127X_RegDioMapping2				   0x41
// Version
#define SX127X_RegVersion					   0x42
// Additional settings
#define SX127X_RegTcxo						  0x4B
#define SX127X_RegPaDac						 0x4D
#define SX127X_RegFormerTemp					0x5B
#define SX127X_RegAgcRef						0x61
#define SX127X_RegAgcThresh1					0x62
#define SX127X_RegAgcThresh2					0x63
#define SX127X_RegAgcThresh3					0x64

/********************FSK/OOK mode***************************/
#define SX127X_FSK_RegBitRateMsb				0x02
#define SX127X_FSK_RegBitRateLsb				0x03
#define SX127X_FSK_RegFdevMsb				   0x04
#define SX127X_FSK_RegFdevLsb				   0x05
#define SX127X_FSK_RegRxConfig				  0x0d
#define SX127X_FSK_RegRssiConfig				0x0e
#define SX127X_FSK_RegRssiCollision			 0x0f
#define SX127X_FSK_RegRssiThresh				0x10
#define SX127X_FSK_RegRssiValue				 0x11
#define SX127X_FSK_RegRxBw					  0x12
#define SX127X_FSK_RegAfcBw					 0x13
#define SX127X_FSK_RegOokPeak				   0x14
#define SX127X_FSK_RegOokFix					0x15
#define SX127X_FSK_RegOokAvg					0x16
#define SX127X_FSK_RegAfcFei					0x1a
#define SX127X_FSK_RegAfcMsb					0x1b
#define SX127X_FSK_RegAfcLsb					0x1c
#define SX127X_FSK_RegFeiMsb					0x1d
#define SX127X_FSK_RegFeiLsb					0x1e
#define SX127X_FSK_RegPreambleDetect			0x1f
#define SX127X_FSK_RegRxTimeout1				0x20
#define SX127X_FSK_RegRxTimeout2				0x21
#define SX127X_FSK_RegRxTimeout3				0x22
#define SX127X_FSK_RegRxDelay				   0x23
#define SX127X_FSK_RegOsc					   0x24
#define SX127X_FSK_RegPreambleMsb			   0x25
#define SX127X_FSK_RegPreambleLsb			   0x26
#define SX127X_FSK_RegSyncConfig				0x27
#define SX127X_FSK_RegSyncValue1				0x28
#define SX127X_FSK_RegSyncValue2				0x29
#define SX127X_FSK_RegSyncValue3				0x2a
#define SX127X_FSK_RegSyncValue4				0x2b
#define SX127X_FSK_RegSyncValue5				0x2c
#define SX127X_FSK_RegSyncValue6				0x2d
#define SX127X_FSK_RegSyncValue7				0x2e
#define SX127X_FSK_RegSyncValue8				0x2f
#define SX127X_FSK_RegPacketConfig1			 0x30
#define SX127X_FSK_RegPacketConfig2			 0x31
#define SX127X_FSK_RegPayloadLength			 0x32
#define SX127X_FSK_RegNodeAdrs				  0x33
#define SX127X_FSK_RegBroadcastAdrs			 0x34
#define SX127X_FSK_RegFifoThresh				0x35
#define SX127X_FSK_RegSeqConfig1				0x36
#define SX127X_FSK_RegSeqConfig2				0x37
#define SX127X_FSK_RegTimerResol				0x38
#define SX127X_FSK_RegTimer1Coef				0x39
#define SX127X_FSK_RegSyncWord				  0x39
#define SX127X_FSK_RegTimer2Coef				0x3a
#define SX127X_FSK_RegImageCal				  0x3b
#define SX127X_FSK_RegTemp					  0x3c
#define SX127X_FSK_RegLowBat					0x3d
#define SX127X_FSK_RegIrqFlags1				 0x3e
#define SX127X_FSK_RegIrqFlags2				 0x3f
#define SX127X_FSK_RegPllHop					0x44
#define SX127X_FSK_RegBitRateFrac			   0x5d

/********************LoRa mode***************************/
#define SX127X_LoRa_RegFifoAddrPtr			  0x0D
#define SX127X_LoRa_RegFifoTxBaseAddr		   0x0E
#define SX127X_LoRa_RegFifoRxBaseAddr		   0x0F
#define SX127X_LoRa_RegFifoRxCurrentAddr		0x10
#define SX127X_LoRa_RegIrqFlagsMask			 0x11
#define SX127X_LoRa_RegIrqFlags				 0x12
#define SX127X_LoRa_RegRxNbBytes				0x13
#define SX127X_LoRa_RegRxHeaderCntValueMsb	  0x14
#define SX127X_LoRa_RegRxHeaderCntValueLsb	  0x15
#define SX127X_LoRa_RegRxPacketCntValueMsb	  0x16
#define SX127X_LoRa_RegRxPacketCntValueLsb	  0x17
#define SX127X_LoRa_RegModemStat				0x18
#define SX127X_LoRa_RegPktSnrValue			  0x19
#define SX127X_LoRa_RegPktRssiValue			 0x1A
#define SX127X_LoRa_RegRssiValue				0x1B
#define SX127X_LoRa_RegHopChannel			   0x1C
#define SX127X_LoRa_RegModemConfig1			 0x1D
#define SX127X_LoRa_RegModemConfig2			 0x1E
#define SX127X_LoRa_RegSymbTimeoutLsb		   0x1F
#define SX127X_LoRa_RegPreambleMsb			  0x20
#define SX127X_LoRa_RegPreambleLsb			  0x21
#define SX127X_LoRa_RegPayloadLength			0x22
#define SX127X_LoRa_RegMaxPayloadLength		 0x23
#define SX127X_LoRa_RegHopPeriod				0x24
#define SX127X_LoRa_RegFifoRxByteAddr		   0x25
#define SX127X_LoRa_RegModemConfig3			 0x26
#define SX127X_LoRa_RegDetectOptimize		   0x31
#define SX127X_LoRa_RegInvertIQ				 0x33
#define SX127X_LoRa_RegDetectionThreshold	   0x37
#define SX127X_LoRa_RegSyncWord				 0x39

#define SX127X_OPMODE_LORA		0b10000000
#define SX127X_OPMODE_FSK		0b00000000
#define SX127X_OPMODE_LOW_FREQ		0b00001000
#define SX127X_OPMODE_HIGH_FREQ		0b00000000

#define SX127X_OPMODE_SLEEP		0b00000000
#define SX127X_OPMODE_STANDBY		0b00000001
#define SX127X_OPMODE_FSTX		0b00000010
#define SX127X_OPMODE_TX		0b00000011
#define SX127X_OPMODE_FSRX		0b00000100
#define SX127X_OPMODE_RXCONTINUOUS	0b00000101
#define SX127X_OPMODE_RXSINGLE		0b00000110
#define SX127X_OPMODE_CAD		0b00000111

// transmit power

typedef enum {
	SX127X_POWER_20DBM = 0,
	SX127X_POWER_17DBM = 1,
	SX127X_POWER_14DBM = 2,
	SX127X_POWER_11DBM = 3
} SX127X_Power_t;

static const uint8_t SX127X_Power[4] = { 0xFF, //20dbm
		0xFC, //17dbm
		0xF9, //14dbm
		0xF6, //11dbm
		};

// spread factor

typedef enum {
	SX127X_LORA_SF_6 = 0,
	SX127X_LORA_SF_7 = 1,
	SX127X_LORA_SF_8 = 2,
	SX127X_LORA_SF_9 = 3,
	SX127X_LORA_SF_10 = 4,
	SX127X_LORA_SF_11 = 5,
	SX127X_LORA_SF_12 = 6,
} SX127X_SpreadFactor_t;

static const uint8_t SX127X_SpreadFactor[7] = { 6, 7, 8, 9, 10, 11, 12 };

//LoRa bandwidth

typedef enum {
	SX127X_LORA_BW_7_8KHZ = 0,
	SX127X_LORA_BW_10_4KHZ = 1,
	SX127X_LORA_BW_15_6KHZ = 2,
	SX127X_LORA_BW_20_8KHZ = 3,
	SX127X_LORA_BW_31_2KHZ = 4,
	SX127X_LORA_BW_41_7KHZ = 5,
	SX127X_LORA_BW_62_5KHZ = 6,
	SX127X_LORA_BW_125KHZ = 7,
	SX127X_LORA_BW_250KHZ = 8,
	SX127X_LORA_BW_500KHZ = 9,
} SX127X_LoRaBandwidth_t;

static const uint8_t SX127X_LoRaBandwidth[10] = { 0, //   7.8KHz,
		1, //  10.4KHz,
		2, //  15.6KHz,
		3, //  20.8KHz,
		4, //  31.2KHz,
		5, //  41.7KHz,
		6, //  62.5KHz,
		7, // 125.0KHz,
		8, // 250.0KHz,
		9 // 500.0KHz
		};

// coding rate

typedef enum {
	SX127X_LORA_CR_4_5 = 0,
	SX127X_LORA_CR_4_6 = 1,
	SX127X_LORA_CR_4_7 = 2,
	SX127X_LORA_CR_4_8 = 3,
} SX127X_CodingRate_t;

static const uint8_t SX127X_CodingRate[4] = { 0x01, 0x02, 0x03, 0x04 };

// crc enable

typedef enum {
	SX127X_LORA_CRC_DIS = 0, SX127X_LORA_CRC_EN = 1,
} SX127X_CRC_Sum_t;

static const uint8_t SX127X_CRC_Sum[2] = { 0x00, 0x01 };

typedef enum {
	SLEEP, STANDBY, TX, RX
} SX127X_Status_t;


typedef uint8_t (*device_get_dio0_ptr)(void);
typedef void (*device_set_nss_ptr)(uint8_t);
typedef void (*device_set_nreset_ptr)(uint8_t);

typedef struct {
	uint8_t Dio3Mapping :2;
	uint8_t Dio2Mapping :2;
	uint8_t Dio1Mapping :2;
	uint8_t Dio0Mapping :2;
} SX127X_RegDioMapping1_t;

typedef struct {
	uint8_t MapPreambleDetect :1;
	uint8_t :3;
	uint8_t Dio5Mapping :2;
	uint8_t Dio4Mapping :2;
} SX127X_RegDioMapping2_t;

typedef struct {
	device_write_ptr write_reg;
	device_read_ptr read_reg;
	device_delay_ms_ptr delay_ms;
	device_get_dio0_ptr get_dio0;
	device_set_nss_ptr set_nss;
	device_set_nreset_ptr set_nreset;

	uint64_t frequency;
	bool highFrequency;
	SX127X_Power_t power;
	SX127X_SpreadFactor_t LoRa_SF;
	SX127X_LoRaBandwidth_t LoRa_BW;
	SX127X_CodingRate_t LoRa_CR;
	SX127X_CRC_Sum_t LoRa_CRC_sum;
	uint8_t packetLength;

	SX127X_Status_t status;

	uint8_t rxBuffer[SX127X_MAX_PACKET];
	uint8_t readBytes;
} SX127X_t;

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
uint8_t SX127X_SPIRead(SX127X_t *module, uint8_t addr);

/**
 * \brief Write byte to LoRa module
 *
 * Writes data to LoRa module under given address.
 *
 * \param[in]  module	Pointer to LoRa structure
 * \param[in]  addr		Address under which data will be written
 * \param[in]  cmd 		Data to write
 */
void SX127X_SPIWrite(SX127X_t *module, uint8_t addr, uint8_t cmd);

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
void SX127X_SPIBurstRead(SX127X_t *module, uint8_t addr, uint8_t *rxBuf,
		uint8_t length);

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
void SX127X_SPIBurstWrite(SX127X_t *module, uint8_t addr, uint8_t *txBuf,
		uint8_t length);

/**
 * \brief Initialize LoRa module
 *
 * Initialize LoRa module and initialize LoRa structure.
 *
 * \param[in]  module		Pointer to LoRa structure
 * \param[in]  frequency	Frequency in [Hz]
 * \param[in]  power		Power level, accepts SX127X_POWER_*
 * \param[in]  LoRa_SF	  LoRa spread rate, accepts SX127X_LORA_SF_*
 * \param[in]  LoRa_BW	  LoRa bandwidth, accepts SX127X_LORA_BW_*
 * \param[in]  LoRa_CR	  LoRa coding rate, accepts SX127X_LORA_CR_*
 * \param[in]  LoRa_CRC_sum Hardware CRC check, SX127X_LORA_CRC_EN or
 *						  SX127X_LORA_CRC_DIS
 * \param[in]  packetLength Package length, no more than 256 bytes
 * \param[in]  write_reg 	Pointer to function that writes to SPI register. Hardware specific.
 * \param[in]  read_reg		Pointer to function that reads from SPI register. Hardware specific.
 * \param[in]  get_dio0		Pointer to function that gets status of DIO0 pin. Hardware specific.
 * \param[in]  set_nss		Pointer to function that sets SPI slave select. Hardware specific.
 * \param[in]  set_nreset	Pointer to function that sets reset pin. Hardware specific.
 */
void SX127X_init(SX127X_t *module, uint64_t frequency, SX127X_Power_t power, SX127X_SpreadFactor_t LoRa_SF, SX127X_LoRaBandwidth_t LoRa_BW, SX127X_CodingRate_t LoRa_CR,
		SX127X_CRC_Sum_t LoRa_CRC_sum, uint8_t packetLength, device_write_ptr write_reg, device_read_ptr read_reg, device_delay_ms_ptr delay_ms,
		device_get_dio0_ptr get_dio0, device_set_nss_ptr set_nss, device_set_nreset_ptr set_nreset);

/**
 * \brief Configure LoRa module
 *
 * Configure LoRa module according to parameters stored in
 * module structure.
 *
 * \param[in]  module	Pointer to LoRa structure
 */
void SX127X_config(SX127X_t *module);

/**
 * \brief Resets LoRa module
 *
 * Resets LoRa module.
 *
 * \param[in]  module	Pointer to LoRa structure
 */
void SX127X_reset(SX127X_t *module);

/**
 * \brief Enter standby mode
 *
 * Enters standby mode.
 *
 * \param[in]  module	Pointer to LoRa structure
 */
void SX127X_standby(SX127X_t *module);

/**
 * \brief Enter sleep mode
 *
 * Enters sleep mode.
 *
 * \param[in]  module	Pointer to LoRa structure
 */
void SX127X_sleep(SX127X_t *module);

/**
 * \brief Entry LoRa mode
 *
 * Module supports different operation mode.
 * To use LoRa operation mode one has to enter this
 * particular mode to transmit and receive data
 * using LoRa.
 *
 * \param[in]  module	Pointer to LoRa structure
 */
void SX127X_entryLoRa(SX127X_t *module);

/**
 * \brief Clear IRQ
 *
 * Clears interrupt flags.
 *
 * \param[in]  module	Pointer to LoRa structure
 */
void SX127X_clearLoRaIrq(SX127X_t *module);

/**
 * \brief Entry reception mode
 *
 * Entry reception mode
 *
 * \param[in]  module	Pointer to LoRa structure
 * \param[in]  length   Length of message to be received
 * \param[in]  timeout  Timeout in [ms]
 *
 * \return	 1 if entering reception mode
 *			 0 if timeout was exceeded
 */
int8_t SX127X_LoRaEntryRx(SX127X_t *module, uint8_t length, uint32_t timeout);

/**
 * \brief Read data
 *
 * Read data and store it in module's RX buffer
 *
 * \param[in]  module	Pointer to LoRa structure
 *
 * \return	 returns number of read bytes
 */
uint8_t SX127X_LoRaRxPacket(SX127X_t *module);

/**
 * \brief Entry reception mode
 *
 * Entry reception mode
 *
 * \param[in]  module	Pointer to LoRa structure
 * \param[in]  length   Length of message to be received
 * \param[in]  timeout  Timeout in [ms]
 *
 * \return	 1 if entering reception mode
 *			 0 if timeout was exceeded
 */
int8_t SX127X_LoRaEntryRx(SX127X_t *module, uint8_t length, uint32_t timeout);

/**
 * \brief Read data
 *
 * Read data and store it in module's RX buffer
 *
 * \param[in]  module	Pointer to LoRa structure
 *
 * \return	 returns number of read bytes
 */
uint8_t SX127X_LoRaRxPacket(SX127X_t *module);

/**
 * \brief Entry transmitter mode
 *
 * Entry transmitter mode
 *
 * \param[in]  module	Pointer to LoRa structure
 * \param[in]  length   Length of message to be sent
 * \param[in]  timeout  Timeout in [ms]
 *
 * \return	 1 if entering reception mode
 *			 0 if timeout was exceeded
 */
int8_t SX127X_LoRaEntryTx(SX127X_t *module, uint8_t length, uint32_t timeout);

/**
 * \brief Send data
 *
 * Transmit data
 *
 * \param[in]  module	Pointer to LoRa structure
 * \param[in]  txBuf	Data buffer with data to be sent
 * \param[in]  length   Length of message to be sent
 * \param[in]  timeout  Timeout in [ms]
 *
 * \return	 1 if entering reception mode
 *			 0 if timeout was exceeded
 */
int8_t SX127X_LoRaTxPacket(SX127X_t *module, uint8_t *txBuf, uint8_t length,
		uint32_t timeout);

/**
 * \brief Entry transmitter mode and send data
 *
 * Entry transmitter mode and send data.
 * Combination of SX127X_LoRaEntryTx() and SX127X_LoRaTxPacket().
 *
 * \param[in]  module	Pointer to LoRa structure
 * \param[in]  txBuf	Data buffer with data to be sent
 * \param[in]  length   Length of message to be sent
 * \param[in]  timeout  Timeout in [ms]
 *
 * \return	 1 if entered TX mode and sent data
 *			 0 if timeout was exceeded
 */
int8_t SX127X_transmit(SX127X_t *module, uint8_t *txBuf, uint8_t length,
		uint32_t timeout);

/**
 * \brief Entry reception mode
 *
 * Same as SX127X_LoRaEntryRx()
 *
 * \param[in]  module	Pointer to LoRa structure
 * \param[in]  length   Length of message to be received
 * \param[in]  timeout  Timeout in [ms]
 *
 * \return	 1 if entering reception mode
 *			 0 if timeout was exceeded
 */
int8_t SX127X_receive(SX127X_t *module, uint8_t length, uint32_t timeout);

/**
 * \brief Returns number of received data
 *
 * Returns the number of received data which are
 * held in internal buffer.
 * Same as SX127X_LoRaRxPacket().
 *
 * \param[in]  module	Pointer to LoRa structure
 *
 * \return	 returns number of read bytes
 */
uint8_t SX127X_available(SX127X_t *module);

/**
 * \brief Read received data to buffer
 *
 * Reads data from internal buffer to external
 * buffer. Reads exactly number of bytes which are stored in
 * internal buffer.
 *
 * \param[in]  module	Pointer to LoRa structure
 * \param[out] rxBuf	External buffer to store data.
 *					  External buffer is terminated with '\0'
 *					  character
 * \param[in]  length   Length of message to be received
 *
 * \return	 returns number of read bytes
 */
uint8_t SX127X_read(SX127X_t *module, uint8_t *rxBuf, uint8_t length);

/**
 * \brief Returns RSSI (LoRa)
 *
 * Returns RSSI in LoRa mode.
 *
 * \param[in]  module	Pointer to LoRa structure
 *
 * \return	 RSSI value
 */
uint8_t SX127X_RSSI_LoRa(SX127X_t *module);

/**
 * \brief Returns RSSI
 *
 * Returns RSSI (general mode).
 *
 * \param[in]  module	Pointer to LoRa structure
 *
 * \return	 RSSI value
 */
uint8_t SX127X_RSSI_FSK(SX127X_t *module);

#endif	/* SX127X_H */

