/*
* based on this library:
* ----------------------------------------------------------------------------
* ?THE COFFEEWARE LICENSE? (Revision 1):
* <ihsan@kehribar.me> wrote this file. As long as you retain this notice you
* can do whatever you want with this stuff. If we meet some day, and you think
* this stuff is worth it, you can buy me a coffee in return.
* -----------------------------------------------------------------------------
* This library is based on this library: 
*   https://github.com/aaronds/arduino-nrf24l01
* Which is based on this library: 
*   http://www.tinkerer.eu/AVRLib/nRF24L01
* -----------------------------------------------------------------------------
*/

#ifndef NRF24L01_H
#define	NRF24L01_H

#include <stdint.h>
#include <stdbool.h>

#ifndef NRF24_ADDR_LEN
#define NRF24_ADDR_LEN 5
#endif

#define NRF24_CMD_R_REGISTER	0b00000000 // last 5 bits = register address
#define NRF24_CMD_W_REGISTER	0b00100000 // last 5 bits = register address
#define NRF24_REGISTER_MASK	0b00011111
#define NRF24_CMD_R_RX_PAYLOAD  0b01100001
#define NRF24_CMD_W_TX_PAYLOAD  0b10100000
#define NRF24_CMD_FLUSH_TX	  0b11100001
#define NRF24_CMD_FLUSH_RX	  0b11100010
#define NRF24_CMD_REUSE_TX_PL   0b11100011
#define NRF24_CMD_R_RX_PL_WID   0b01100000
#define NRF24_CMD_W_ACK_PAYLOAD 0b10101000 // last 3 bits = pipe number, 0b000 to 0b101
#define NRF24_CMD_W_TX_PAYLOAD_NO_ACK 0b10110000
#define NRF24_CMD_NOP		   0b11111111

#define NRF24_REG_CONFIG	  0x00
#define NRF24_REG_EN_AA	   0x01
#define NRF24_REG_EN_RXADDR   0x02
#define NRF24_REG_SETUP_AW	0x03
#define NRF24_REG_SETUP_RETR  0x04
#define NRF24_REG_RF_CH	   0x05
#define NRF24_REG_RF_SETUP	0x06
#define NRF24_REG_STATUS	  0x07
#define NRF24_REG_OBSERVE_TX  0x08
#define NRF24_REG_RPD		 0x09
#define NRF24_REG_RX_ADDR_P0  0x0A
#define NRF24_REG_RX_ADDR_P1  0x0B
#define NRF24_REG_RX_ADDR_P2  0x0C
#define NRF24_REG_RX_ADDR_P3  0x0D
#define NRF24_REG_RX_ADDR_P4  0x0E
#define NRF24_REG_RX_ADDR_P5  0x0F
#define NRF24_REG_TX_ADDR	 0x10
#define NRF24_REG_RX_PW_P0	0x11
#define NRF24_REG_RX_PW_P1	0x12
#define NRF24_REG_RX_PW_P2	0x13
#define NRF24_REG_RX_PW_P3	0x14
#define NRF24_REG_RX_PW_P4	0x15
#define NRF24_REG_RX_PW_P5	0x16
#define NRF24_REG_FIFO_STATUS 0x17
#define NRF24_REG_DYNPD	   0x1C
#define NRF24_REG_FEATURE	 0x1D

#define NRF24_DEFAULT_CONFIG	 0b00001000

#define NRF24_TRANSMISSON_OK 0
#define NRF24_MESSAGE_LOST   1

typedef enum {
	NRF24_CRC_DISABLED = 0,
	NRF24_CRC_8 = 1,
	NRF24_CRC_16 = 2
} NRF24_CRC_Length;

typedef enum {
	NRF24_RETRY_DELAY_250uS = 0b0000,
	NRF24_RETRY_DELAY_500uS = 0b0001,
	NRF24_RETRY_DELAY_750uS = 0b0010,
	NRF24_RETRY_DELAY_1000uS = 0b0011,
	NRF24_RETRY_DELAY_1250uS = 0b0100,
	NRF24_RETRY_DELAY_1500uS = 0b0101,
	NRF24_RETRY_DELAY_1750uS = 0b0110,
	NRF24_RETRY_DELAY_2000uS = 0b0111,
	NRF24_RETRY_DELAY_2250uS = 0b1000,
	NRF24_RETRY_DELAY_2500uS = 0b1001,
	NRF24_RETRY_DELAY_2750uS = 0b1010,
	NRF24_RETRY_DELAY_3000uS = 0b1011,
	NRF24_RETRY_DELAY_3250uS = 0b1100,
	NRF24_RETRY_DELAY_3500uS = 0b1101,
	NRF24_RETRY_DELAY_3750uS = 0b1110,
	NRF24_RETRY_DELAY_4000uS = 0b1111
		
} NRF24_Retry_Delay;

typedef enum {
	NRF24_AIR_DATA_RATE_250KBPS = 0,
	NRF24_AIR_DATA_RATE_1MBPS = 1,
	NRF24_AIR_DATA_RATE_2MBPS = 2
} NRF24_AirDataRate;

typedef enum {
	NRF24_POWER_OUTPUT_0DBM = 0b11,		// max power
	NRF24_POWER_OUTPUT_MINUS_6DBM = 0b10,
	NRF24_POWER_OUTPUT_MINUS_12DBM = 0b01,
	NRF24_POWER_OUTPUT_MINUS_18DBM = 0b00   // min power
} NRF24_PowerOutputLevel;

// 0x00; configuration register
typedef struct tag_NRF24_Register_Config {
	union {
	struct {
		uint8_t DATA;
	};
	struct {
		uint8_t PRIM_RX : 1;	// bit 0; RX/TX control: 0: PTX, 1: PRX
		uint8_t PWR_UP : 1;		// bit 1; 0: power down, 1: power up
		uint8_t CRCO : 1;		// bit 2; CRC encoding scheme: 0: 1-byte, 1: 2-bytes
		uint8_t EN_CRC : 1;		// bit 3; Enable CRC, default = 1
		uint8_t MASK_MAX_RT : 1;	// bit 4; mask interrupt caused by MAX_RT: 0: reflect on IRQ, 1: interrupt not reflected
		uint8_t MASK_TX_DS : 1;	// bit 5; mask interrupt caused by TX_DS: 0: reflect on IRQ, 1: interrupt not reflected
		uint8_t MASK_RX_DR : 1;	// bit 6; mask interrupt caused by RX_DR: 0: reflect on IRQ, 1: interrupt not reflected
	};
	};
} NRF24_Register_Config;

// 0x01; enable auto acknowledgment register
typedef struct tag_NRF24_Register_EnableAA {
	union {
	struct {
		uint8_t DATA;
	};
	struct {
		uint8_t ENAA_P0 : 1;	// bit 0; enable auto acknowledgment data pipe 0
		uint8_t ENAA_P1 : 1;	// bit 1; enable auto acknowledgment data pipe 1
		uint8_t ENAA_P2 : 1;	// bit 2; enable auto acknowledgment data pipe 2
		uint8_t ENAA_P3 : 1;	// bit 3; enable auto acknowledgment data pipe 3
		uint8_t ENAA_P4 : 1;	// bit 4; enable auto acknowledgment data pipe 4
		uint8_t ENAA_P5 : 1;	// bit 5; enable auto acknowledgment data pipe 5
	};
	};
} NRF24_Register_EnableAA;

// 0x02; enabled RX addresses register
typedef struct tag_NRF24_Register_EnabledRXAddr {
	union {
	struct {
		uint8_t DATA;
	};
	struct {
		uint8_t ERX_P0 : 1;	// bit 0; enable data pipe 0
		uint8_t ERX_P1 : 1;	// bit 1; enable data pipe 1
		uint8_t ERX_P2 : 1;	// bit 2; enable data pipe 2
		uint8_t ERX_P3 : 1;	// bit 3; enable data pipe 3
		uint8_t ERX_P4 : 1;	// bit 4; enable data pipe 4
		uint8_t ERX_P5 : 1;	// bit 5; enable data pipe 5
	};
	};
} NRF24_Register_EnabledRXAddr;

// 0x03; setup of address widths register
typedef struct tag_NRF24_Register_SetupAW {
	union {
	struct {
		uint8_t DATA;
	};
	struct {
		uint8_t AW : 2;	// bits 0-1; RX/TX address field width: 0b00 - invalid, 0b01 - 3 bytes, 0b10 - 4 bytes, 0b11 - 5 bytes (default))
	};
	};
} NRF24_Register_SetupAW;

// 0x04; setup of automatic retransmission register
typedef struct tag_NRF24_Register_SetupRetransmission {
	union {
	struct {
		uint8_t DATA;
	};
	struct {
		uint8_t ARC : 4;	// bits 0-3; auto retransmit count: 0b0000 = disabled; default = 0b0011
		uint8_t ARD : 4;	// bits 4-7; auto retransmit delay: 0b0000 = wait 250us, 0b1111 = wait 4000us
	};
	};
} NRF24_Register_SetupRetransmission;

// 0x05; RF channel register
typedef struct tag_NRF24_Register_RFChannel {
	union {
	struct {
		uint8_t DATA;
	};
	struct {
		uint8_t RF_CH : 7;	// bits 0-6; frequency channel; frequency = 2400MHz + channel * 1MHz; default = 0b0000010
	};
	};
} NRF24_Register_RFChannel;

// 0x06; RF setup register
typedef struct tag_NRF24_Register_RFSetup {
	union {
	struct {
		uint8_t DATA;
	};
	struct {
		uint8_t  : 1;	// bit 0; obsolete
		uint8_t RF_PWR : 2;	// bits 1-2; output power in TX mode: 0b00 = -18dBm, 0b01 = -12dBm, 0b10 = -6dBm, 0b11 = 0dBm (default))
		uint8_t RF_DR_HIGH : 1;	// bit 3; select between high speed modes, ignored if RF_DR_LOW=1; 0 = 1Mbps, 1 = 2Mbps
		uint8_t PLL_LOCK : 1;	// bit 4; force PLL lock signal, for test only
		uint8_t RF_DR_LOW : 1;	// bit 5; set RF data rate to 250kbps
		uint8_t : 1;		// bit 6; 
		uint8_t CONT_WAVE : 1;	// bit 7; enables continuous carrier transmit when high
	};
	};
} NRF24_Register_RFSetup;

// 0x07; status register
typedef struct tag_NRF24_Register_Status {
	union {
	struct {
		uint8_t DATA;
	};
	struct {
		uint8_t TX_FULL : 1;	// bit 0; TX FIFO full flag, 0 = available locations if TX FIFO, 1 = TX FIFO full
		uint8_t RX_P_NO : 3;	// bits 1-3; data pipe number for the payload available for reading from RX_FIFO; 000-101 = data pipe, 111 = RX FIFO empty
		uint8_t MAX_RT : 1;		// bit 4; maximum number of retransmits interrupt; must be cleared to enable further communication; write 1 to clear bit
		uint8_t TX_DS : 1;		// bit 5; data sent TX FIFO interrupt; if AUTO_ACK=1, this bit is set high only if ACK is received; write 1 to clear bit
		uint8_t RX_DR : 1;		// bit 6; data ready RX FIFO interrupt; write 1 to clear bit
	};
	};
} NRF24_Register_Status;

// 0x08; transmit observe register
typedef struct tag_NRF24_Register_TransmitObserve {
	union {
	struct {
		uint8_t DATA;
	};
	struct {
		uint8_t ARC_CNT : 4;	// bits 0-3; retransmitted packets count
		uint8_t PLOS_CNT : 4;	// bits 4-7; lost packets count; counter is reset by writing to RF_CH
	};
	};
} NRF24_Register_TransmitObserve;

// 0x09; received power detector register
typedef struct tag_NRF24_Register_ReceivedPowerDetector {
	union {
	struct {
		uint8_t DATA;
	};
	struct {
		uint8_t RPD : 1;	// bit 0; received power detector
	};
	};
} NRF24_Register_ReceivedPowerDetector;

// 0x0A; receive address data pipe 0 register
typedef struct tag_NRF24_Register_ReceiveAddrPipe0 {
	uint8_t RX_ADDR_P0[5]; // 5 bytes maximum, LSB written first; default = 0xE7E7E7E7E7
} NRF24_Register_ReceiveAddrPipe0;

// 0x0B; receive address data pipe 1 register
typedef struct tag_NRF24_Register_ReceiveAddrPipe1 {
	uint8_t RX_ADDR_P1[5]; // 5 bytes maximum, LSB written first; default = 0xC2C2C2C2C2
} NRF24_Register_ReceiveAddrPipe1;

// 0x0C; receive address data pipe 2 register
typedef struct tag_NRF24_Register_ReceiveAddrPipe2 {
	uint8_t RX_ADDR_P2; // only LSB; MSB equal to RX_ADDR_P1; default = 0xC3
} NRF24_Register_ReceiveAddrPipe2;

// 0x0D; receive address data pipe 3 register
typedef struct tag_NRF24_Register_ReceiveAddrPipe3 {
	uint8_t RX_ADDR_P3; // only LSB; MSB equal to RX_ADDR_P1; default = 0xC4
} NRF24_Register_ReceiveAddrPipe3;

// 0x0E; receive address data pipe 4 register
typedef struct tag_NRF24_Register_ReceiveAddrPipe4 {
	uint8_t RX_ADDR_P4; // only LSB; MSB equal to RX_ADDR_P1; default = 0xC5
} NRF24_Register_ReceiveAddrPipe4;

// 0x0F; receive address data pipe 5 register
typedef struct tag_NRF24_Register_ReceiveAddrPipe5 {
	uint8_t RX_ADDR_P5; // only LSB; MSB equal to RX_ADDR_P1; default = 0xC6
} NRF24_Register_ReceiveAddrPipe5;

// 0x10; transmit address register
typedef struct tag_NRF24_Register_TransmitAddress {
	uint8_t TX_ADDR[5]; // transmit address, used for PTX device only; LSB first; default = 0xE7E7E7E7E7
} NRF24_Register_TransmitAddress;

// 0x11; number of bytes in RX payload in data pipe 0 register
typedef struct tag_NRF24_Register_RXPayloadWidthPipe0 {
	union {
	struct {
		uint8_t DATA;
	};
	struct {
		uint8_t RX_PW_P0 : 6;	// bits 0-5; number of bytes in RX payload in data pipe 0 (1 to 32 bytes), 0 = payload not used
	};
	};
} NRF24_Register_RXPayloadWidthPipe0;

// 0x12; number of bytes in RX payload in data pipe 1 register
typedef struct tag_NRF24_Register_RXPayloadWidthPipe1 {
	union {
	struct {
		uint8_t DATA;
	};
	struct {
		uint8_t RX_PW_P1 : 6;	// bits 0-5; number of bytes in RX payload in data pipe 1 (1 to 32 bytes), 0 = payload not used
	};
	};
} NRF24_Register_RXPayloadWidthPipe1;

// 0x13; number of bytes in RX payload in data pipe 2 register
typedef struct tag_NRF24_Register_RXPayloadWidthPipe2 {
	union {
	struct {
		uint8_t DATA;
	};
	struct {
		uint8_t RX_PW_P2 : 6;	// bits 0-5; number of bytes in RX payload in data pipe 2 (1 to 32 bytes), 0 = payload not used
	};
	};
} NRF24_Register_RXPayloadWidthPipe2;

// 0x14; number of bytes in RX payload in data pipe 3 register
typedef struct tag_NRF24_Register_RXPayloadWidthPipe3 {
	union {
	struct {
		uint8_t DATA;
	};
	struct {
		uint8_t RX_PW_P3 : 6;	// bits 0-5; number of bytes in RX payload in data pipe 3 (1 to 32 bytes), 0 = payload not used
	};
	};
} NRF24_Register_RXPayloadWidthPipe3;

// 0x15; number of bytes in RX payload in data pipe 4 register
typedef struct tag_NRF24_Register_RXPayloadWidthPipe4 {
	union {
	struct {
		uint8_t DATA;
	};
	struct {
		uint8_t RX_PW_P4 : 6;	// bits 0-5; number of bytes in RX payload in data pipe 4 (1 to 32 bytes), 0 = payload not used
	};
	};
} NRF24_Register_RXPayloadWidthPipe4;

// 0x16; number of bytes in RX payload in data pipe 5 register
typedef struct tag_NRF24_Register_RXPayloadWidthPipe5 {
	union {
	struct {
		uint8_t DATA;
	};
	struct {
		uint8_t RX_PW_P5 : 6;	// bits 0-5; number of bytes in RX payload in data pipe 5 (1 to 32 bytes), 0 = payload not used
	};
	};
} NRF24_Register_RXPayloadWidthPipe5;

// 0x17; FIFO status register
typedef struct tag_NRF24_Register_FIFOStatus {
	union {
	struct {
		uint8_t DATA;
	};
	struct {
		uint8_t RX_EMPTY : 1;	// bit 0; RX FIFO empty flag; 0 = data in RX FIFO, 1 = RX FIFO empty
		uint8_t RX_FULL : 1;	// bit 1; RX FIFO full flag; 0 = available locations in RX FIFO, 1 = RX FIFO full
		uint8_t  : 2;		// bits 2-3; reserved
		uint8_t TX_EMPTY : 1;	// bit 4; TX FIFO empty flag; 0 = data in TX FIFO, 1 = TX FIFO empty
		uint8_t TX_FULL : 1;	// bit 5; TX FIFO full flag; 0 = available locations in TX FIFO, 1 = TX FIFO full
		uint8_t TX_REUSE : 1;	// bit 6; pulse CE high for at least for 10us to reuse last transmitted payload
	};
	};
} NRF24_Register_FIFOStatus;

// 0x1C; enable dynamic payload length register
typedef struct tag_NRF24_Register_EnableDynamicPaylodLength {
	union {
	struct {
		uint8_t DATA;
	};
	struct {
		uint8_t DPL_P0 : 1;	// bit 0; enable dynamic payload length data pipe 0 (requires EN_DPL and ENAA_P0)
		uint8_t DPL_P1 : 1;	// bit 1; 
		uint8_t DPL_P2 : 1;	// bit 2; 
		uint8_t DPL_P3 : 1;	// bit 3; 
		uint8_t DPL_P4 : 1;	// bit 4; 
		uint8_t DPL_P5 : 1;	// bit 5; 
	};
	};
} NRF24_Register_EnableDynamicPaylodLength;

// 0x1D; feature register
typedef struct tag_NRF24_Register_Feature {
	union {
	struct {
		uint8_t DATA;
	};
	struct {
		uint8_t EN_DYN_ACK : 1;	// bit 0; enables the W_TX_PAYLOAD_NOACK command
		uint8_t EN_ACK_PAY : 1;	// bit 1; enables payload with ACK
		uint8_t EN_DPL : 1;		// bit 2; enables dynamic payload length
	};
	};
} NRF24_Register_Feature;



/* adjustment functions */
void nrf24_init(void);
void nrf24_config(uint8_t channel, uint8_t pay_length);
void nrf24_setTxAddress(uint8_t* adr);
void nrf24_setRxAddress(uint8_t * adr);
void nrf24_setChannel(uint8_t channel);
void nrf24_enableDynamicPayloads(void);

/* state check functions */
bool nrf24_isSending(void);
NRF24_Register_Status nrf24_getStatus(void);
void nrf24_setStatus(NRF24_Register_Status status);
bool nrf24_rxFifoEmpty(void);

/* post transmission analysis */
uint8_t nrf24_lastMessageStatus(void);
uint8_t nrf24_retransmissionCount(void);
NRF24_Register_Status nrf24_write_payload(const void* buf, uint8_t len);
NRF24_Register_Status nrf24_read_payload(void* buf, uint8_t len);
NRF24_Register_Status nrf24_write_ack_payload(const void* buf, uint8_t len);

/* use in dynamic length mode */
uint8_t nrf24_getReceivedPayloadLength(void);

/* power management */
void nrf24_powerUpRx(void);
void nrf24_powerUpTx(void);
void nrf24_powerDown(void);

/* low level interface ... */
void nrf24_transmitSync(uint8_t* dataout, uint8_t len);
void nrf24_transferSync(uint8_t* dataout, uint8_t* datain, uint8_t len);
void nrf24_writeRegisterByte(uint8_t reg, uint8_t value);
void nrf24_readRegister(uint8_t reg, uint8_t* value, uint8_t len);
void nrf24_writeRegister(uint8_t reg, uint8_t* value, uint8_t len);

NRF24_CRC_Length nrf24_CRC_getLength(void);
void nrf24_CRC_disable(void);
void nrf24_CRC_setLength(NRF24_CRC_Length crc);
void nrf24_setRetries(NRF24_Retry_Delay delay, uint8_t count);
void nrf24_setAirDataRate(NRF24_AirDataRate air_data_rate);
NRF24_AirDataRate nrf24_getAirDataRate(void);
void nrf24_setAutoAcknowledgment(bool enable);
void nrf24_setPipeAutoAcknowledgment(uint8_t pipe, bool enable);
bool nrf24_receivedPowerDetector(void);
NRF24_PowerOutputLevel nrf24_getPowerOutputLevel(void);
void nrf24_setPowerOutputLevel(NRF24_PowerOutputLevel power);
NRF24_Register_Status nrf24_flushRx(void);
NRF24_Register_Status nrf24_flushTx(void);
void nrf24_enableRxAddress(uint8_t pipe, bool enable);
NRF24_Register_FIFOStatus nrf24_getFifoStatus(void);

void nrf24_startRx(uint8_t channel, uint8_t* rx_address);
void nrf24_startTx(uint8_t channel, uint8_t* tx_address);


/* -------------------------------------------------------------------------- */
/* You should implement the platform specific functions in your code */
/* -------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* In this function you should do the following things:
 *	- Set MISO pin input
 *	- Set MOSI pin output
 *	- Set SCK pin output
 *	- Set CSN pin output
 *	- Set CE pin output	 */
/* -------------------------------------------------------------------------- */
extern void nrf24_setupPins();

/* -------------------------------------------------------------------------- */
/* nrf24 CE pin control function
 *	- state:1 => Pin HIGH
 *	- state:0 => Pin LOW	 */
/* -------------------------------------------------------------------------- */
extern void nrf24_ce_digitalWrite(uint8_t state);

/* -------------------------------------------------------------------------- */
/* nrf24 CE pin control function
 *	- state:1 => Pin HIGH
 *	- state:0 => Pin LOW	 */
/* -------------------------------------------------------------------------- */
extern void nrf24_csn_digitalWrite(uint8_t state);

extern uint8_t nrf24_spi_transfer(uint8_t data);

extern void nrf24_hw_delay_us(uint16_t us);

#endif	/* NRF24L01_H */

