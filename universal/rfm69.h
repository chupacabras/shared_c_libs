/**
  ******************************************************************************
  * @file    rfm69.h
  * @author  Juraj Lonc (juraj.lonc@gmail.com)
  * @brief   Header file of RFM69 (RFM69H, SX1231) driver.
  ******************************************************************************
  */

#ifndef RFM69_H_
#define RFM69_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "device.h"

#define RFM69_XTAL			   32000000
//#define RFM69_FSTEP			   61.03515625	// = 15625 / 256

#define RFM69_FIFO_LENGTH		   66
#define RFM69_MAX_PACKET		   61	 // to take advantage of the built in AES/CRC we want to limit the frame size to the internal FIFO size (66 bytes - 3 bytes overhead - 2 bytes crc)
#define RFM69_DEFAULT_VERSION		   0x24

//RFM69 Internal registers Address

/********************Common registers***************************/
// Common settings
#define RFM69_RegFifo						  0x00
#define RFM69_RegOpMode						0x01
#define RFM69_RegDataModul						 0x02
#define RFM69_RegBitRateMsb				0x03
#define RFM69_RegBitRateLsb				0x04
#define RFM69_RegFdevMsb				   0x05
#define RFM69_RegFdevLsb				   0x06
#define RFM69_RegFrMsb						 0x07
#define RFM69_RegFrMid						 0x08
#define RFM69_RegFrLsb						 0x09
#define RFM69_RegOsc1					   0x0A
#define RFM69_RegAfcCtrl					 0x0B
#define RFM69_RegListen1					 0x0D
#define RFM69_RegListen2					 0x0E
#define RFM69_RegListen3					 0x0F
// Version
#define RFM69_RegVersion					   0x10

// Tx settings
#define RFM69_RegPaLevel					  0x11
#define RFM69_RegPaRamp						0x12
#define RFM69_RegOcp						   0x13

// Rx settings
#define RFM69_RegLna						   0x18
#define RFM69_RegRxBw					  0x19
#define RFM69_RegAfcBw					 0x1A
#define RFM69_RegOokPeak				   0x1B
#define RFM69_RegOokAvg					0x1C
#define RFM69_RegOokFix					0x1D
#define RFM69_RegAfcFei					0x1E
#define RFM69_RegAfcMsb					0x1F
#define RFM69_RegAfcLsb					0x20
#define RFM69_RegFeiMsb					0x21
#define RFM69_RegFeiLsb					0x22
#define RFM69_RegRssiConfig				0x23
#define RFM69_RegRssiValue				 0x24

// IRQ I/O settings
#define RFM69_RegDioMapping1				   0x25
#define RFM69_RegDioMapping2				   0x26
#define RFM69_RegIrqFlags1				 0x27
#define RFM69_RegIrqFlags2				 0x28
#define RFM69_RegRssiThresh				0x29
#define RFM69_RegRxTimeout1				0x2A
#define RFM69_RegRxTimeout2				0x2B

// Packet engine registers
#define RFM69_RegPreambleMsb			   0x2C
#define RFM69_RegPreambleLsb			   0x2D
#define RFM69_RegSyncConfig				0x2E
#define RFM69_RegSyncValue1				0x2F
#define RFM69_RegSyncValue2				0x30
#define RFM69_RegSyncValue3				0x31
#define RFM69_RegSyncValue4				0x32
#define RFM69_RegSyncValue5				0x33
#define RFM69_RegSyncValue6				0x34
#define RFM69_RegSyncValue7				0x35
#define RFM69_RegSyncValue8				0x36
#define RFM69_RegPacketConfig1			 0x37
#define RFM69_RegPayloadLength			 0x38
#define RFM69_RegNodeAdrs				  0x39
#define RFM69_RegBroadcastAdrs			 0x3A
#define RFM69_RegAutoModes			 0x3B
#define RFM69_RegFifoThresh				0x3C
#define RFM69_RegPacketConfig2			 0x3D
#define RFM69_RegAesKey1				0x3E
#define RFM69_RegAesKey2				0x3F
#define RFM69_RegAesKey3				0x40
#define RFM69_RegAesKey4				0x41
#define RFM69_RegAesKey5				0x42
#define RFM69_RegAesKey6				0x43
#define RFM69_RegAesKey7				0x44
#define RFM69_RegAesKey8				0x45
#define RFM69_RegAesKey9				0x46
#define RFM69_RegAesKey10				0x47
#define RFM69_RegAesKey11				0x48
#define RFM69_RegAesKey12				0x49
#define RFM69_RegAesKey13				0x4A
#define RFM69_RegAesKey14				0x4B
#define RFM69_RegAesKey15				0x4C
#define RFM69_RegAesKey16				0x4D

// Temperature sensor
#define RFM69_RegTemp1					  0x4E
#define RFM69_RegTemp2					  0x4F

// Test registers
#define RFM69_RegTestLna					  0x58
#define RFM69_RegTestTcxo					  0x59
#define RFM69_RegTestPa1					  0x5A
#define RFM69_RegTestPa2					  0x5C
#define RFM69_RegTestDagc					  0x6F
#define RFM69_RegTestAfc					  0x71


#define RFM69_OPMODE_SEQUENCER_OFF		0b10000000
#define RFM69_OPMODE_SEQUENCER_ON		0b00000000
#define RFM69_OPMODE_LISTEN_ON		0b01000000
#define RFM69_OPMODE_LISTEN_OFF		0b00000000
#define RFM69_OPMODE_LISTEN_ABORT		0b00100000

#define RFM69_OPMODE_SLEEP		0b00000000			// XTAL OFF
#define RFM69_OPMODE_STANDBY		0b00000100		// XTAL ON
#define RFM69_OPMODE_FS		0b00001000			// PLL ON
#define RFM69_OPMODE_TX		0b00001100			// TX MODE
#define RFM69_OPMODE_RX		0b00010000			// RX MODE

#define RFM69_START_TEMP		0b00001000
#define RFM69_TEMP_MEAS_RUNNING		0b00000100

#define RFM69_IRQ_MODE_READY		0b10000000
#define RFM69_IRQ_RX_READY		0b01000000
#define RFM69_IRQ_TX_READY		0b00100000
#define RFM69_IRQ_PLL_LOCK		0b00010000
#define RFM69_IRQ_RSSI		0b00001000
#define RFM69_IRQ_TIMEOUT		0b00000100
#define RFM69_IRQ_AUTO_MODE		0b00000010
#define RFM69_IRQ_SYNC_ADDR_MATCH		0b00000001


// RegOsc1
#define RFM69_OSC1_RCCAL_START       0x80
#define RFM69_OSC1_RCCAL_DONE        0x40

// RegOcp
#define RFM69_OCP_OFF       0
#define RFM69_OCP_ON        0b10000

// RegPaLevel
#define RFM69_PALEVEL_PA0_ON     0b10000000  // Default
#define RFM69_PALEVEL_PA0_OFF    0x00
#define RFM69_PALEVEL_PA1_ON     0b01000000
#define RFM69_PALEVEL_PA1_OFF    0x00  // Default
#define RFM69_PALEVEL_PA2_ON     0b00100000
#define RFM69_PALEVEL_PA2_OFF    0x00  // Default

// RSSI
#define RFM69_RSSI_START       0b00000001
#define RFM69_RSSI_DONE       0b00000010

// AES
#define RFM69_AES_ON                     0b00000001
#define RFM69_AES_OFF                    0b00000000  // Default

// Sync (network)
#define RFM69_SYNC_ON                     0b10000000
#define RFM69_SYNC_OFF                     0b00000000

// RegPacketConfig1
#define RFM69_PACKET_FIXED_LENGTH       0
#define RFM69_PACKET_VARIABLE_LENGTH       0b10000000
#define RFM69_DCFREE_NONE        0
#define RFM69_DCFREE_MANCHESTER        0b00100000
#define RFM69_DCFREE_WHITENING        0b01000000
#define RFM69_CRC_ON        0b00010000
#define RFM69_CRC_OFF        0b00000000
#define RFM69_CRC_AUTOCLEAR_ON        0b00000000
#define RFM69_CRC_AUTOCLEAR_OFF        0b00001000
#define RFM69_ADDRESSFILTERING_NONE        			0b00000000
#define RFM69_ADDRESSFILTERING_MATCH_NODE      		0b00000010
#define RFM69_ADDRESSFILTERING_MATCH_NODE_AND_BROADCAST 0b00000100

// RegPacketConfig2
#define RFM69_PACKET2_RXRESTARTDELAY_1BIT        0x00  // Default
#define RFM69_PACKET2_RXRESTARTDELAY_2BITS       0x10
#define RFM69_PACKET2_RXRESTARTDELAY_4BITS       0x20
#define RFM69_PACKET2_RXRESTARTDELAY_8BITS       0x30
#define RFM69_PACKET2_RXRESTARTDELAY_16BITS      0x40
#define RFM69_PACKET2_RXRESTARTDELAY_32BITS      0x50
#define RFM69_PACKET2_RXRESTARTDELAY_64BITS      0x60
#define RFM69_PACKET2_RXRESTARTDELAY_128BITS     0x70
#define RFM69_PACKET2_RXRESTARTDELAY_256BITS     0x80
#define RFM69_PACKET2_RXRESTARTDELAY_512BITS     0x90
#define RFM69_PACKET2_RXRESTARTDELAY_1024BITS    0xA0
#define RFM69_PACKET2_RXRESTARTDELAY_2048BITS    0xB0
#define RFM69_PACKET2_RXRESTARTDELAY_NONE        0xC0

#define RFM69_PACKET2_RXRESTART                  0x04
#define RFM69_PACKET2_AUTORXRESTART_ON           0x02  // Default
#define RFM69_PACKET2_AUTORXRESTART_OFF          0x00
#define RFM69_PACKET2_AES_ON                     0x01
#define RFM69_PACKET2_AES_OFF                    0x00  // Default

// RegDioMapping1
#define RFM69_DIOMAPPING1_DIO0_00            0b00000000  // Default
#define RFM69_DIOMAPPING1_DIO0_01            0b01000000
#define RFM69_DIOMAPPING1_DIO0_10            0b10000000
#define RFM69_DIOMAPPING1_DIO0_11            0b11000000

#define RFM69_DIOMAPPING1_DIO1_00            0b00000000  // Default
#define RFM69_DIOMAPPING1_DIO1_01            0b00010000
#define RFM69_DIOMAPPING1_DIO1_10            0b00100000
#define RFM69_DIOMAPPING1_DIO1_11            0b00110000

#define RFM69_DIOMAPPING1_DIO2_00            0b00000000  // Default
#define RFM69_DIOMAPPING1_DIO2_01            0b00000100
#define RFM69_DIOMAPPING1_DIO2_10            0b00001000
#define RFM69_DIOMAPPING1_DIO2_11            0b00001100

#define RFM69_DIOMAPPING1_DIO3_00            0b00000000  // Default
#define RFM69_DIOMAPPING1_DIO3_01            0b00000001
#define RFM69_DIOMAPPING1_DIO3_10            0b00000010
#define RFM69_DIOMAPPING1_DIO3_11            0b00000011


// RegDioMapping2
#define RFM69_DIOMAPPING2_DIO4_00            0b00000000  // Default
#define RFM69_DIOMAPPING2_DIO4_01            0b01000000
#define RFM69_DIOMAPPING2_DIO4_10            0b10000000
#define RFM69_DIOMAPPING2_DIO4_11            0b11000000

#define RFM69_DIOMAPPING2_DIO5_00            0b00000000  // Default
#define RFM69_DIOMAPPING2_DIO5_01            0b00010000
#define RFM69_DIOMAPPING2_DIO5_10            0b00100000
#define RFM69_DIOMAPPING2_DIO5_11            0b00110000

// RegIrqFlags2
#define RFM69_IRQFLAGS2_FIFOFULL             0b10000000
#define RFM69_IRQFLAGS2_FIFONOTEMPTY         0b01000000
#define RFM69_IRQFLAGS2_FIFOLEVEL            0b00100000
#define RFM69_IRQFLAGS2_FIFOOVERRUN          0b00010000
#define RFM69_IRQFLAGS2_PACKETSENT           0b00001000
#define RFM69_IRQFLAGS2_PAYLOADREADY         0b00000100
#define RFM69_IRQFLAGS2_CRCOK                0b00000010
#define RFM69_IRQFLAGS2_LOWBAT               0b00000001


#define RFM69_BROADCAST_ADDRESS			255


// control bits
#define RFM69_CTL_SENDACK   	0x80
#define RFM69_CTL_REQACK    	0x40
#define RFM69_CTL_FIRST_PACKET    0x20
#define RFM69_CTL_LAST_PACKET    0x10


typedef enum {
	RFM69H,
	RFM69HW
} RFM69_Model_t;

typedef enum {
	RFM69_FSK = 0b00000,
	RFM69_OOK = 0b01000
} RFM69_ModulationType_t;

typedef enum {
	RFM69_PACKET_MODE = 0b0000000,
	RFM69_CONTINUOUS_W_SYNC = 0b1000000,
	RFM69_CONTINUOUS_WO_SYNC = 0b1100000
} RFM69_DataMode_t;

typedef enum {
	RFM69_FSK_NO_SHAPING = 0b00,
	RFM69_FSK_GAUSS_BT10 = 0b01,
	RFM69_FSK_GAUSS_BT05 = 0b10,
	RFM69_FSK_GAUSS_BT03 = 0b11,
	RFM69_OOK_NO_SHAPING = 0b00,
	RFM69_OOK_FILTER_BR = 0b01,
	RFM69_OOK_FILTER_2BR = 0b10
} RFM69_ModulationShaping_t;

typedef enum {
	RFM69_STATUS_WAITING,
	RFM69_STATUS_TX,
	RFM69_STATUS_RECEIVE_ACK,
	RFM69_STATUS_SENT,
	RFM69_STATUS_TX_TIMEDOUT,
	RFM69_STATUS_RECEIVED,
	RFM69_STATUS_RECEIVED_AND_ACK,
	RFM69_STATUS_ACK_FAILED,
	RFM69_STATUS_ACK_RECEIVED,
	RFM69_STATUS_RX,
	RFM69_STATUS_DELAY_SEND_ACK,
	RFM69_STATUS_SEND_ACK,
} RFM69_Status_t;


typedef uint8_t (*rfm69_device_get_dio0_ptr)(void);
typedef void (*rfm69_device_set_nss_ptr)(uint8_t);
typedef void (*rfm69_device_set_reset_ptr)(uint8_t);

typedef int8_t(*rfm69_device_spi_write_ptr)(void *, uint8_t *, uint16_t);
typedef int8_t(*rfm69_device_spi_read_ptr) (void *, uint8_t *, uint16_t);


typedef struct {
	rfm69_device_spi_write_ptr write_spi;
	rfm69_device_spi_read_ptr read_spi;
	device_delay_ms_ptr delay_ms;
	rfm69_device_get_dio0_ptr get_dio0;
	rfm69_device_set_nss_ptr set_nss;
	rfm69_device_set_reset_ptr set_reset;

	uint64_t frequency;
	uint8_t mode;
	RFM69_Model_t model;
	bool high_power_device;
	bool high_power_settings;
	uint8_t node_address;
	uint8_t broadcast_address;
	bool promiscuous;
	uint8_t *receive_buf;

	RFM69_Status_t status;
	bool request_ack;
	bool ack_received;
	bool first_packet;
	bool last_packet;
	int16_t received_rssi;
	uint8_t sender_addr;
	uint8_t target_addr;
	uint8_t payload_length;
	uint16_t timer;

} RFM69_t;

void RFM69_init(RFM69_t *module, RFM69_Model_t model, uint64_t frequency, uint8_t node_addr, rfm69_device_spi_write_ptr write_spi, rfm69_device_spi_read_ptr read_spi, device_delay_ms_ptr delay_ms,
		rfm69_device_get_dio0_ptr get_dio0, rfm69_device_set_nss_ptr set_nss, rfm69_device_set_reset_ptr set_reset, uint8_t *receive_buf);

void RFM69_reset(RFM69_t *module);

uint8_t RFM69_register_read(RFM69_t *module, uint8_t addr);
void RFM69_register_write(RFM69_t *module, uint8_t addr, uint8_t cmd);
void RFM69_burst_read(RFM69_t *module, uint8_t addr, uint8_t *rxBuf, uint8_t length);
void RFM69_burst_write(RFM69_t *module, uint8_t addr, uint8_t *txBuf, uint8_t length);

void RFM69_rc_calibration(RFM69_t *module);
void RFM69_set_mode(RFM69_t *module, uint8_t new_mode);
void RFM69_temperature_measurement_start(RFM69_t *module);
bool RFM69_temperature_measurement_running(RFM69_t *module);
int8_t RFM69_temperature_read(RFM69_t *module);
uint8_t RFM69_get_version(RFM69_t *module);
void RFM69_set_high_power(RFM69_t *module, bool enable);
void RFM69_set_power_level(RFM69_t *module, uint8_t power_level);
void RFM69_set_frequency(RFM69_t *module, uint64_t frequency);
void RFM69_set_bitrate(RFM69_t *module, uint32_t bitrate);
void RFM69_set_data_modulation(RFM69_t *module, RFM69_ModulationType_t modulation_type, RFM69_DataMode_t data_mode, RFM69_ModulationShaping_t shaping);
void RFM69_set_rx_bandwidth_filter(RFM69_t *module, uint32_t bw, RFM69_ModulationType_t modulation_type);
int16_t RFM69_read_RSSI(RFM69_t *module);
void RFM69_encrypt(RFM69_t *module, const uint8_t* aes_key);
void RFM69_set_FSK_deviation_frequency(RFM69_t *module, uint32_t fdev_hz);
void RFM69_set_network_address(RFM69_t *module, uint8_t *network, uint8_t size);
void RFM69_set_node_address(RFM69_t *module, uint8_t node_addr);
void RFM69_set_broadcast_address(RFM69_t *module, uint8_t broadcast_addr);
void RFM69_set_promiscuous(RFM69_t *module, bool promiscuous);

int RFM69_set_power_dbm(RFM69_t *module, int8_t dbm);
void RFM69_set_high_power_settings(RFM69_t *module, bool enable);

void RFM69_send_packet(RFM69_t *module, uint8_t target_address, void* buf, uint8_t size, bool request_ack, bool send_ack, bool first_packet, bool last_packet);
void RFM69_interrupt_handler_timer(RFM69_t *module);
void RFM69_interrupt_handler_io(RFM69_t *module);

void RFM69_loop(RFM69_t *module);
void RFM69_start_listening(RFM69_t *module);

#ifdef __cplusplus
}
#endif

#endif /* RFM69_H_ */
