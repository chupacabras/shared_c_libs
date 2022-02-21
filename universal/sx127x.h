/* based on: 
*/
/**
 * Wojciech Domski <Wojciech.Domski@gmail.com>
 * www: www.Domski.pl
 *
 * work based on DORJI.COM sample code and
 * https://github.com/realspinner/SX1278_LoRa
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "device.h"
#include "utils.h"

#ifndef SX127X_H
#define	SX127X_H

// SX1276   137 - 1020 MHz
// SX1277   137 - 1020 MHz
// SX1278   137 - 525 MHz
// SX1279   137 - 960MHz

#define SX127X_MAX_PACKET		   256
//#define SX127X_DEFAULT_TIMEOUT		3000



#define SX127X_DEFAULT_VERSION_VALUE		0x12
#define SX127X_FXOSC		32000000	// 32MHz
#define SX127X_FSTEP			   61.03515625

#define SX127X_LORAWAN_SYNC_WORD			   0x34
#define SX127X_OCP_IMAX						   240


//SX127x Internal registers Address

/********************Common registers***************************/
// Common settings
#define SX127X_REG_FIFO						  0x00
#define SX127X_REG_OP_MODE						0x01
#define SX127X_REG_FR_MSB						 0x06
#define SX127X_REG_FR_MID						 0x07
#define SX127X_REG_FR_LSB						 0x08
// Tx settings
#define SX127X_REG_PA_CONFIG					  0x09
#define SX127X_REG_PA_RAMP						0x0A
#define SX127X_REG_OCP						   0x0B
// Rx settings
#define SX127X_REG_LNA						   0x0C
// I/O settings
#define SX127X_REG_DIO_MAPPING1				   0x40
#define SX127X_REG_DIO_MAPPING2				   0x41
// Version
#define SX127X_REG_VERSION					   0x42
// Additional settings
#define SX127X_REG_TCXO						  0x4B
#define SX127X_REG_PA_DAC						 0x4D
#define SX127X_REG_FORMER_TEMP					0x5B
#define SX127X_REG_AGC_REF						0x61
#define SX127X_REG_AGC_THRESH1					0x62
#define SX127X_REG_AGC_THRESH2					0x63
#define SX127X_REG_AGC_THRESH3					0x64

/********************FSK/OOK mode***************************/
#define SX127X_FSK_REG_BITRATE_MSB				0x02
#define SX127X_FSK_REG_BITRATE_LSB				0x03
#define SX127X_FSK_REG_FDEV_MSB				   0x04
#define SX127X_FSK_REG_FDEV_LSB				   0x05
#define SX127X_FSK_REG_RX_CONFIG				  0x0d
#define SX127X_FSK_REG_RSSI_CONFIG				0x0e
#define SX127X_FSK_REG_RSSI_COLLISION			 0x0f
#define SX127X_FSK_REG_RSSI_THRESH				0x10
#define SX127X_FSK_REG_RSSI_VALUE				 0x11
#define SX127X_FSK_REG_RX_BB					  0x12
#define SX127X_FSK_REG_AFC_BW					 0x13
#define SX127X_FSK_REG_OOK_PEAK				   0x14
#define SX127X_FSK_REG_OOK_FIX					0x15
#define SX127X_FSK_REG_OOK_AAVG					0x16
#define SX127X_FSK_REG_AFC_FEI					0x1a
#define SX127X_FSK_REG_AFC_MSB					0x1b
#define SX127X_FSK_REG_AFC_LSB					0x1c
#define SX127X_FSK_REG_FEI_MSB					0x1d
#define SX127X_FSK_REG_FEI_LSB					0x1e
#define SX127X_FSK_REG_PREAMBLE_DETECT			0x1f
#define SX127X_FSK_REG_RX_TIMEOUT1				0x20
#define SX127X_FSK_REG_RX_TIMEOUT2				0x21
#define SX127X_FSK_REG_RX_TIMEOUT3				0x22
#define SX127X_FSK_REG_RX_DELAY				   0x23
#define SX127X_FSK_REG_OSC					   0x24
#define SX127X_FSK_REG_PREAMBLE_MSB			   0x25
#define SX127X_FSK_REG_PREAMBLE_LSB			   0x26
#define SX127X_FSK_REG_SYNC_CONFIG				0x27
#define SX127X_FSK_REG_SYNC_VALUE1				0x28
#define SX127X_FSK_REG_SYNC_VALUE2				0x29
#define SX127X_FSK_REG_SYNC_VALUE3				0x2a
#define SX127X_FSK_REG_SYNC_VALUE4				0x2b
#define SX127X_FSK_REG_SYNC_VALUE5				0x2c
#define SX127X_FSK_REG_SYNC_VALUE6				0x2d
#define SX127X_FSK_REG_SYNC_VALUE7				0x2e
#define SX127X_FSK_REG_SYNC_VALUE8				0x2f
#define SX127X_FSK_REG_PACKET_CONFIG1			 0x30
#define SX127X_FSK_REG_PACKET_CONFIG2			 0x31
#define SX127X_FSK_REG_PAYLOAD_LENGTH			 0x32
#define SX127X_FSK_REG_NODE_ADDRESS				  0x33
#define SX127X_FSK_REG_BROADCAST_ADDRESS			 0x34
#define SX127X_FSK_REG_FIFO_THRESH				0x35
#define SX127X_FSK_REG_SEQ_CONFIG1				0x36
#define SX127X_FSK_REG_SEQ_CONFIG2				0x37
#define SX127X_FSK_REG_TIMER_RESOL				0x38
#define SX127X_FSK_REG_TIMER1_COEF				0x39
#define SX127X_FSK_REG_TIMER2_COEF				0x3a
#define SX127X_FSK_REG_IMAGE_CAL				  0x3b
#define SX127X_FSK_REG_TEMP					  0x3c
#define SX127X_FSK_REG_LOW_BAT					0x3d
#define SX127X_FSK_REG_IRQ_FLAGS1				 0x3e
#define SX127X_FSK_REG_IRQ_FLAGS2				 0x3f
#define SX127X_FSK_REG_PLL_HOP					0x44
#define SX127X_FSK_REG_BITRATE_FRAC			   0x5d

/********************LoRa mode***************************/
#define SX127X_LORA_REG_FIFO_ADDR_PTR			  0x0D
#define SX127X_LORA_REG_FIFO_TX_BASE_ADDR		   0x0E
#define SX127X_LORA_REG_FIFO_RX_BASE_ADDR		   0x0F
#define SX127X_LORA_REG_FIFO_RX_CURRENT_ADDR		0x10
#define SX127X_LORA_REG_IRQ_FLAGS_MASK			 0x11
#define SX127X_LORA_REG_IRQ_FLAGS				 0x12
#define SX127X_LORA_REG_RX_NB_BYTES				0x13
#define SX127X_LORA_REG_RX_HEADER_CNT_VALUE_MSB	  0x14
#define SX127X_LORA_REG_RX_HEADER_CNT_VALUE_LSB	  0x15
#define SX127X_LORA_REG_RX_PACKET_CNT_VALUE_MSB	  0x16
#define SX127X_LORA_REG_RX_PACKET_CNT_VALUE_LSB	  0x17
#define SX127X_LORA_REG_MODEM_STAT				0x18
#define SX127X_LORA_REG_PKT_SNR_VALUE			  0x19
#define SX127X_LORA_REG_PKT_RSSI_VALUE			 0x1A
#define SX127X_LORA_REG_RSSI_VALUE				0x1B
#define SX127X_LORA_REG_HOP_CHANNEL			   0x1C
#define SX127X_LORA_REG_MODEM_CONFIG1			 0x1D
#define SX127X_LORA_REG_MODEM_CONFIG2			 0x1E
#define SX127X_LORA_REG_SYMB_TIMEOUT_LSB		   0x1F
#define SX127X_LORA_REG_PREAMBLE_MSB			  0x20
#define SX127X_LORA_REG_PREAMBLE_LSB			  0x21
#define SX127X_LORA_REG_PAYLOAD_LENGTH			0x22
#define SX127X_LORA_REG_MAX_PAYLOAD_LENGTH		 0x23
#define SX127X_LORA_REG_HOP_PERIOD				0x24
#define SX127X_LORA_REG_FIFO_RX_BYTE_ADDR		   0x25
#define SX127X_LORA_REG_MODEM_CONFIG3			 0x26
#define SX127X_LORA_REG_DETECT_OPTIMIZE		   0x31
#define SX127X_LORA_REG_INVERT_IQ				 0x33
#define SX127X_LORA_REG_HIGH_BW_OPTIMIZE1				 0x36
#define SX127X_LORA_REG_DETECTION_THRESHOLD   0x37
#define SX127X_LORA_REG_SYNC_WORD				 0x39
#define SX127X_LORA_REG_HIGH_BW_OPTIMIZE2				 0x3a


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


// RegDioMapping1
#define SX127X_DIOMAPPING1_DIO0_00            0b00000000  // Default
#define SX127X_DIOMAPPING1_DIO0_01            0b01000000
#define SX127X_DIOMAPPING1_DIO0_10            0b10000000
#define SX127X_DIOMAPPING1_DIO0_11            0b11000000

#define SX127X_DIOMAPPING1_DIO1_00            0b00000000  // Default
#define SX127X_DIOMAPPING1_DIO1_01            0b00010000
#define SX127X_DIOMAPPING1_DIO1_10            0b00100000
#define SX127X_DIOMAPPING1_DIO1_11            0b00110000

#define SX127X_DIOMAPPING1_DIO2_00            0b00000000  // Default
#define SX127X_DIOMAPPING1_DIO2_01            0b00000100
#define SX127X_DIOMAPPING1_DIO2_10            0b00001000
#define SX127X_DIOMAPPING1_DIO2_11            0b00001100

#define SX127X_DIOMAPPING1_DIO3_00            0b00000000  // Default
#define SX127X_DIOMAPPING1_DIO3_01            0b00000001
#define SX127X_DIOMAPPING1_DIO3_10            0b00000010
#define SX127X_DIOMAPPING1_DIO3_11            0b00000011


// RegDioMapping2
#define SX127X_DIOMAPPING2_DIO4_00            0b00000000  // Default
#define SX127X_DIOMAPPING2_DIO4_01            0b01000000
#define SX127X_DIOMAPPING2_DIO4_10            0b10000000
#define SX127X_DIOMAPPING2_DIO4_11            0b11000000

#define SX127X_DIOMAPPING2_DIO5_00            0b00000000  // Default
#define SX127X_DIOMAPPING2_DIO5_01            0b00010000
#define SX127X_DIOMAPPING2_DIO5_10            0b00100000
#define SX127X_DIOMAPPING2_DIO5_11            0b00110000


typedef enum {
	SX127X_FSK_OOK_MODE = 0,
	SX127X_LORA_MODE = 1
} SX127X_LongRangeMode;

typedef enum {
	SX127X_MODE_SLEEP = 0b000,
	SX127X_MODE_STANDBY = 0b001,
	SX127X_MODE_FSTX = 0b010,
	SX127X_MODE_TX = 0b011,
	SX127X_MODE_FSRX = 0b100,
	SX127X_MODE_RXCONTINUOUS = 0b101,
	SX127X_MODE_RXSINGLE = 0b110,
	SX127X_MODE_CAD = 0b111
} SX127X_Mode;


typedef enum {
	SX127X_LNA_GAIN_HIGHEST = 0b001,
	SX127X_LNA_GAIN_6DB = 0b010,
	SX127X_LNA_GAIN_12DB = 0b011,
	SX127X_LNA_GAIN_24DB = 0b100,
	SX127X_LNA_GAIN_36DB = 0b101,
	SX127X_LNA_GAIN_48DB = 0b110
} SX127X_LnaGain;


// transmit power
typedef enum {
	SX127X_POWER_20DBM = 0xFF,	// +20dBm max. 1% duty cycle
	SX127X_POWER_17DBM = 0xFC,
	SX127X_POWER_14DBM = 0xF9,
	SX127X_POWER_11DBM = 0xF6
} SX127X_Power;

// spread factor
typedef enum {
	SX127X_LORA_SF_6 = 6,
	SX127X_LORA_SF_7 = 7,
	SX127X_LORA_SF_8 = 8,
	SX127X_LORA_SF_9 = 9,
	SX127X_LORA_SF_10 = 10,
	SX127X_LORA_SF_11 = 11,
	SX127X_LORA_SF_12 = 12,
} SX127X_SpreadFactor;

//LoRa bandwidth
typedef enum {
	SX127X_LORA_BW_7_8KHZ = 0b0000,
	SX127X_LORA_BW_10_4KHZ = 0b0001,
	SX127X_LORA_BW_15_6KHZ = 0b0010,
	SX127X_LORA_BW_20_8KHZ = 0b0011,
	SX127X_LORA_BW_31_2KHZ = 0b0100,
	SX127X_LORA_BW_41_7KHZ = 0b0101,
	SX127X_LORA_BW_62_5KHZ = 0b0110,
	SX127X_LORA_BW_125KHZ = 0b0111,
	SX127X_LORA_BW_250KHZ = 0b1000,
	SX127X_LORA_BW_500KHZ = 0b1001,
} SX127X_LoRaBandwidth;

// coding rate
typedef enum {
	SX127X_LORA_CR_4_5 = 0b001,
	SX127X_LORA_CR_4_6 = 0b010,
	SX127X_LORA_CR_4_7 = 0b011,
	SX127X_LORA_CR_4_8 = 0b100,
} SX127X_CodingRate;

// crc enable
typedef enum {
	SX127X_LORA_CRC_DISABLED = 0,
	SX127X_LORA_CRC_ENABLED = 1,
} SX127X_CRC;


/**
 * An enum describing the radio's irq sources.
 */
typedef enum {
	SX127X_CAD_DETECTED = 0b00000001,     /* Valid Lora signal detected during CAD operation */
	SX127X_FHSS_CHANGE_CHANNEL = 0b00000010,     /* FHSS change channel interrupt */
	SX127X_CAD_DONE = 0b00000100,     /* CAD complete: write to clear */
	SX127X_TX_DONE = 0b00001000,     /* FIFO Payload transmission complete interrupt */
	SX127X_VALID_HEADER = 0b00010000,     /* Valid header received in Rx */
	SX127X_PAYLOAD_CRC_ERROR = 0b00100000,          /* Payload CRC error interrupt */
	SX127X_RX_DONE = 0b01000000,           /* Packet reception complete interrupt */
	SX127X_RX_TIMEOUT = 0b10000000,           /* Timeout interrupt */
	SX127X_ALL_IRQ = 0b11111111           /* All interrupts */
} SX127X_IRQ_Source;

//typedef uint8_t (*device_get_dio0_ptr)(void);
typedef void (*device_set_nss_ptr)(uint8_t);
typedef void (*device_set_nreset_ptr)(uint8_t);


// 0x11; LoRa RegIrqFlagsMask
typedef struct tag_SX127X_Register_LoRa_RegIrqFlagsMask {
	union {
		struct {
			uint8_t DATA;
		};
		struct {
			uint8_t CadDetectedMask :1;			// bit 0; Cad Detected Interrupt Mask: setting this bit masks the corresponding IRQ in RegIrqFlags
			uint8_t FhssChangeChannelMask :1;	// bit 1; FHSS change channel interrupt mask: setting this bit masks the corresponding IRQ in RegIrqFlag
			uint8_t CadDoneMask :1;				// bit 2; CAD complete interrupt mask: setting this bit masks the corresponding IRQ in RegIrqFlags
			uint8_t TxDoneMask :1;				// bit 3; FIFO Payload transmission complete interrupt mask: setting this bit masks the corresponding IRQ in RegIrqFlags
			uint8_t ValidHeaderMask :1;			// bit 4; Valid header received in Rx mask: setting this bit masks the corresponding IRQ in RegIrqFlags
			uint8_t PayloadCrcErrorMask :1;		// bit 5; Payload CRC error interrupt mask: setting this bit masks the corresponding IRQ in RegIrqFlags
			uint8_t RxDoneMask :1;				// bit 6; Packet reception complete interrupt mask: setting this bit masks the corresponding IRQ in RegIrqFlags
			uint8_t RxTimeoutMask :1;			// bit 7; Timeout interrupt mask: setting this bit masks the corresponding IRQ in RegIrqFlags
		};
	};
} SX127X_Register_LoRa_RegIrqFlagsMask;

// 0x12; LoRa RegIrqFlags
typedef struct tag_SX127X_Register_LoRa_RegIrqFlags {
	union {
		struct {
			uint8_t DATA;
		};
		struct {
			uint8_t CadDetected :1;			// bit 0; Valid Lora signal detected during CAD operation: writing a 1 clears the IRQ
			uint8_t FhssChangeChannel :1;	// bit 1; FHSS change channel interrupt: writing a 1 clears the IRQ
			uint8_t CadDone :1;				// bit 2; CAD complete: write to clear: writing a 1 clears the IRQ
			uint8_t TxDone :1;				// bit 3; FIFO Payload transmission complete interrupt: writing a 1 clears the IRQ
			uint8_t ValidHeader :1;			// bit 4; Valid header received in Rx: writing a 1 clears the IRQ
			uint8_t PayloadCrcError :1;		// bit 5; Payload CRC error interrupt: writing a 1 clears the IRQ
			uint8_t RxDone :1;				// bit 6; Packet reception complete interrupt: writing a 1 clears the IRQ
			uint8_t RxTimeout :1;			// bit 7; Timeout interrupt: writing a 1 clears the IRQ
		};
	};
} SX127X_Register_LoRa_RegIrqFlags;

// 0x40; RegDioMapping1
typedef struct tag_SX127X_Register_DioMapping1 {
	union {
		struct {
			uint8_t DATA;
		};
		struct {
			uint8_t Dio3Mapping :2;	// bits 0-1;
			uint8_t Dio2Mapping :2;	// bits 2-3;
			uint8_t Dio1Mapping :2;	// bits 4-5;
			uint8_t Dio0Mapping :2;	// bits 6-7;
		};
	};
} SX127X_RegDioMapping1;

// 0x41; RegDioMapping2
typedef struct tag_SX127X_RegDioMapping2 {
	union {
		struct {
			uint8_t DATA;
		};
		struct {
			uint8_t MapPreambleDetect :1;	// bit 0; Allows the mapping of either Rssi Or PreambleDetect to the DIO pins
			uint8_t :3;						// bits 1-3;
			uint8_t Dio5Mapping :2;			// bits 4-5;
			uint8_t Dio4Mapping :2;			// bits 6-7;
		};
	};
} SX127X_RegDioMapping2;

typedef struct {
	device_write_ptr write_reg;
	device_read_ptr read_reg;
	device_delay_ms_ptr delay_ms;
	//device_get_dio0_ptr get_dio0;
	device_set_nss_ptr set_nss;
	device_set_nreset_ptr set_nreset;

	uint32_t frequency;
	bool high_power_settings;
	SX127X_SpreadFactor LoRa_SF;
	SX127X_LoRaBandwidth LoRa_BW;
	SX127X_CodingRate LoRa_CR;
	SX127X_CRC LoRa_CRC;

	bool highFrequency;
	SX127X_Power power;
	uint8_t packetLength;

} SX127X_Handle;

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
void SX127X_init(SX127X_Handle *module, device_write_ptr write_reg, device_read_ptr read_reg, device_delay_ms_ptr delay_ms, device_set_nss_ptr set_nss, device_set_nreset_ptr set_nreset);

/**
 * \brief Configure LoRa module
 *
 * Configure LoRa module according to parameters stored in
 * module structure.
 *
 * \param[in]  module	Pointer to LoRa structure
 */
void SX127X_config_LoRa(SX127X_Handle *module, uint32_t frequency, SX127X_SpreadFactor sf, SX127X_CRC crc, SX127X_CodingRate codingRate, SX127X_LoRaBandwidth bw, uint8_t syncWord);


/**
 * \brief Resets LoRa module
 *
 * Resets LoRa module.
 *
 * \param[in]  module	Pointer to LoRa structure
 */
void SX127X_reset(SX127X_Handle *module);


/**
 * \brief Clear IRQ
 *
 * Clears interrupt flags.
 *
 * \param[in]  module	Pointer to LoRa structure
 */
void SX127X_clearLoRaIrq(SX127X_Handle *module);

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
void SX127X_config_LoRa_Rx(SX127X_Handle *module, uint8_t defaultLength);

/**
 * \brief Enter transmission mode
 *
 * Enter transmission mode
 *
 * \param[in]  module	Pointer to LoRa structure
 * \param[in]  dbm  	Transmitting power
 *
 */
void SX127X_config_LoRa_Tx(SX127X_Handle *module, uint8_t dbm);

/**
 * \brief Returns RSSI (LoRa)
 *
 * Returns RSSI in LoRa mode.
 *
 * \param[in]  module	Pointer to LoRa structure
 *
 * \return	 RSSI value
 */
uint8_t SX127X_RSSI_LoRa(SX127X_Handle *module);

/**
 * \brief Returns RSSI
 *
 * Returns RSSI (general mode).
 *
 * \param[in]  module	Pointer to LoRa structure
 *
 * \return	 RSSI value
 */
uint8_t SX127X_RSSI_FSK(SX127X_Handle *module);





void SX127X_setLoRaIrqMode(SX127X_Handle *module, SX127X_IRQ_Source irq_source, bool irq_enabled);
SX127X_Register_LoRa_RegIrqFlags SX127X_getLoRaIrq(SX127X_Handle *module);
int16_t SX127X_getPacketRssiDbm(SX127X_Handle *module);
int8_t SX127X_getPacketSnrDb(SX127X_Handle *module);


typedef void (*sx127x_debug_print_fn)(char*);
void SX127X_printDebug(SX127X_Handle *module, sx127x_debug_print_fn print_fn);
void SX127X_printFifo(SX127X_Handle *module, sx127x_debug_print_fn print_fn, uint8_t *buf);


// 0x01; LoRa RegOpMode
typedef struct tag_SX127X_Operation_Mode {
	union {
		struct {
			uint8_t DATA;
		};
		struct {
			uint8_t Mode : 3;				// bits 0-2; Device mode
			uint8_t LowFrequencyModeOn : 1;	// bit 3; Access Low Frequency Mode registers; 0 = High Frequency Mode (access to HF test registers), 1 = Low Frequency Mode (access to LF test registers)
			uint8_t : 2;					// bits 4-5; reserved
			uint8_t AccessSharedReg : 1;	// bit 6; Enable CRC, default = 1
			uint8_t LongRangeMode : 1;		// bit 7; mask interrupt caused by MAX_RT: 0: reflect on IRQ, 1: interrupt not reflected
		};
	};
} SX127X_Operation_Mode;

// 0x1d; LoRa RegModemConfig1
typedef struct tag_SX127X_Modem_Config1 {
	union {
		struct {
			uint8_t DATA;
		};
		struct {
			uint8_t ImplicitHeaderModeOn : 1;	// bit 0; 0 = Explicit Header mode; 1 = Implicit Header mode
			uint8_t CodingRate : 3;				// bits 1-3; Error coding rate; 001 = 4/5, 010 = 4/6, 011 = 4/7, 100 = 4/8
			uint8_t Bw : 4;						// bits 4-7; Signal bandwidth: 0000 = 7.8 kHz, 0001 = 10.4 kHz, 0010 = 15.6 kHz, 0011 = 20.8kHz, 0100 = 31.25 kHz, 0101 = 41.7 kHz, 0110 = 62.5 kHz, 0111 = 125 kHz, 1000 = 250 kHz, 1001 = 500 kHz
		};
	};
} SX127X_Modem_Config1;

// 0x1e; LoRa RegModemConfig2
typedef struct tag_SX127X_Modem_Config2 {
	union {
		struct {
			uint8_t DATA;
		};
		struct {
			uint8_t SymbTimeoutMSB : 2;		// bits 0-1; RX Time-Out MSB
			uint8_t RxPayloadCrcOn : 1;		// bit 2; Enable CRC generation and check on payload: 0 = CRC disable, 1 = CRC enable
			uint8_t TxContinuousMode : 1;	// bit 3; 0 = normal mode, a single packet is sent, 1 = continuous mode, send multiple packets across the FIFO (used for spectral analysis)
			uint8_t SpreadingFactor : 4;	// bits 4-7; SF rate (expressed as a base-2 logarithm); 6 = 64 chips / symbol, 7 = 128 chips / symbol, 8 = 256 chips / symbol, 9 = 512 chips / symbol, 10 = 1024 chips / symbol, 11 = 2048 chips / symbol, 12 = 4096 chips / symbol
		};
	};
} SX127X_Modem_Config2;

// 0x26; LoRa RegModemConfig3
typedef struct tag_SX127X_Modem_Config3 {
	union {
		struct {
			uint8_t DATA;
		};
		struct {
			uint8_t : 2;						// bits 0-1; reserved
			uint8_t AgcAutoOn : 1;				// bit 2; 0 = LNA gain set by register LnaGain, 1 = LNA gain set by the internal AGC loop
			uint8_t LowDataRateOptimize : 1;	// bit 3; 0 = Disabled, 1 = Enabled; mandated for when the symbol length exceeds 16ms
			uint8_t : 4;						// bits 4-7; unused
		};
	};
} SX127X_Modem_Config3;

// 0x0c; RegLna
typedef struct tag_SX127X_Lna {
	union {
		struct {
			uint8_t DATA;
		};
		struct {
			uint8_t LnaBoostHf : 2;	// bits 0-1; High Frequency (RFI_HF) LNA current adjustment, 00 - Default LNA current, 11 - Boost on, 150% LNA current
			uint8_t : 1;			// bit 2; reserved
			uint8_t LnaBoostLf : 2;	// bits 3-4; Low Frequency (RFI_LF) LNA current adjustment, 00 - Default LNA current, Other - Reserved
			uint8_t LnaGain : 3;	// bits 5-7; LNA gain setting: 000 - reserved, 001 - G1 = highest gain, 010 - G2 = highest gain -6 dB, 011 - G3 = highest gain -12 dB, 100 - G4 = highest gain -24 dB, 101 - G5 = highest gain -36 dB, 110 - G6 = highest gain -48 dB, 111 - reserved
									// Note: Reading this address always returns the current LNA gain (which may be different from what had been previously selected if AGC is enabled.
		};
	};
} SX127X_Lna;

// 0x0b; RegOcp
typedef struct tag_SX127X_Ocp {
	union {
		struct {
			uint8_t DATA;
		};
		struct {
			uint8_t OcpTrim : 5;	// bits 0-4; Trimming of OCP current
			uint8_t OcpOn : 1;		// bit 5; Enables overload current protection (OCP) for the PA: 0 - OCP disabled, 1 - OCP enabled
			uint8_t : 2;			// bits 6-7; unused
		};
	};
} SX127X_Ocp;

// 0x09; RegPaConfig
typedef struct tag_SX127X_PaConfig {
	union {
		struct {
			uint8_t DATA;
		};
		struct {
			uint8_t OutputPower : 4;	// bits 0-3; Pout=Pmax-(15-OutputPower) if PaSelect = 0 (RFO pin)
										//			 Pout=17-(15-OutputPower) if PaSelect = 1 (PA_BOOST pin)
			uint8_t MaxPower : 3;		// bits 4-6; Select max output power: Pmax=10.8+0.6*MaxPower [dBm]
			uint8_t PaSelect : 1;		// bit 7; Selects PA output pin, 0 - RFO pin. Output power is limited to +14 dBm., 1 - PA_BOOST pin. Output power is limited to +20 dBm
		};
	};
} SX127X_PaConfig;

//// EU863-870 Frequency Plan
//typedef enum {
//	SX127X_LORA_CRC_DISABLED = 0,
//	SX127X_LORA_CRC_ENABLED = 1,
//} SX127X_Frequency;

void SX127X_sendPacket(SX127X_Handle *module, uint8_t *data, uint8_t packet_size);

uint8_t SX127X_readReceivedPacket(SX127X_Handle *module, uint8_t *buf);


SX127X_Operation_Mode SX127X_getOperationMode(SX127X_Handle *module);
void SX127X_setMode(SX127X_Handle *module, SX127X_Mode mode);
void SX127X_setHighPowerOperation(SX127X_Handle *module, bool enable_high_power);
void SX127X_setLongRangeMode(SX127X_Handle *module, SX127X_LongRangeMode long_range_mode);
void SX127X_setCrcMode(SX127X_Handle *module, SX127X_CRC crc);
void SX127X_setFrequency(SX127X_Handle *module, uint32_t frequency);
uint8_t SX127X_getVersion(SX127X_Handle *module);
void SX127X_setCodingRate(SX127X_Handle *module, SX127X_CodingRate codingRate);
void SX127X_setSpreadingFactor(SX127X_Handle *module, SX127X_SpreadFactor sf);
void SX127X_setSignalBandwidth(SX127X_Handle *module, SX127X_LoRaBandwidth bw);
void SX127X_enableAgc(SX127X_Handle *module, bool enable_agc);
void SX127X_setLowDatarateOptimize(SX127X_Handle *module, bool enable_optimize);
void SX127X_setLnaGain(SX127X_Handle *module, SX127X_LnaGain lnaGain);
void SX127X_enableLnaBoostHf(SX127X_Handle *module, bool enable);
void SX127X_setRxSymbolTimeout(SX127X_Handle *module, uint16_t timeout);
void SX127X_disableOcp(SX127X_Handle *module);
void SX127X_enableOcp(SX127X_Handle *module, uint8_t imax);
void SX127X_setOutputPowerDbm(SX127X_Handle *module, int8_t dbm);
void SX127X_setPreambleLength(SX127X_Handle *module, uint16_t len);
void SX127X_setSyncWord(SX127X_Handle *module, uint8_t syncWord);
bool SX127X_cad(SX127X_Handle *module, uint16_t timeout);

/*

	CH00_868_100,	1% or LBT+AFA	25 mW
	CH01_868_300,	1% or LBT+AFA	25 mW
	CH02_868_500,	1% or LBT+AFA	25 mW

	CH03_867_100,	1% or LBT+AFA	25 mW
	CH04_867_300,	1% or LBT+AFA	25 mW
	CH05_867_500,	1% or LBT+AFA	25 mW
	CH06_867_700,	1% or LBT+AFA	25 mW
	CH07_867_900,	1% or LBT+AFA	25 mW

	ch08 868.8	fsk
	869.525 - SF9BW125 (RX2) - downlink only

CH_10_868 	865.20 MHz	1% or LBT+AFA	25 mW
CH_11_868 	865.50 MHz	1% or LBT+AFA	25 mW
CH_12_868 	865.80 MHz	1% or LBT+AFA	25 mW
CH_13_868 	866.10 MHz	1% or LBT+AFA	25 mW
CH_14_868 	866.40 MHz	1% or LBT+AFA	25 mW
CH_15_868 	866.70 MHz	1% or LBT+AFA	25 mW
CH_16_868 	867 MHz		1% or LBT+AFA	25 mW
CH_17_868 	868 MHz		1% or LBT+AFA	25 mW
*/

#endif	/* SX127X_H */

