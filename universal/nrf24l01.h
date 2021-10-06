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
   Copyright (c) 2015,2016 Piotr Stolarz for the Ardiono port

   This software is distributed WITHOUT ANY WARRANTY; without even the
   implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
   See the License for more information.
 */

/**
 * This is basically nRFgo SDK's NRF HAL API ported to Arduino SDK.
 * Note the SPI interface must be initialized before using the NRF HAL API.
 *
 * The differences are:
 * 1. Addition of hal_nrf_set_cs_pin() function, which need to be called at
 *    first (before any other NRF HALL API call) to set the SPI CS pin number.
 * 2. Added new functions, mostly getters for already existing setters.
 */

/**
 * This is the nRF24L01+ transceiver used in several Nordic Semiconductor
 * devices. The transceiver is set up and controlled via an internal SPI
 * interface on the chip. The HAL for the radio transceiver hides this SPI
 * interface from the programmer.
 *
 * The nRF24LE1 uses the same 2.4GHz GFSK RF transceiver with embedded protocol
 * engine (Enhanced ShockBurst&tm;) that is found in the nRF24L01+ single chip
 * RF Transceiver.
 *
 * The RF Transceiver module is configured and operated through the RF
 * transceiver map. This register map is accessed by the MCU through a dedicated
 * on-chip Serial Peripheral interface (SPI) and is available in all power modes
 * of the RF Transceiver module. The register map contains all configuration
 * registers in the RF Transceiver and is accessible in all operation modes of
 * the transceiver. The radio transceiver HAL hides this register map and the
 * usage of the internal SPI.
 *
 * This HAL module contains setup functions for configurating the radio;
 * operation functions for controlling the radio when active and for sending
 * and receiving data; and test functions for setting the radio in test modes.
 */

#ifndef NRF24L01_H
#define	NRF24L01_H

#include <stdint.h>
#include <stdbool.h>
#include "utils.h"

/* max payload size */
#define NRF24_MAX_PL         32

/* nRF24L01 Instruction Definitions */
#define NRF24_CMD_R_REGISTER		0b00000000	// Register read - last 5 bits = register address
#define NRF24_CMD_W_REGISTER		0b00100000	// Register write - last 5 bits = register address
#define NRF24_REGISTER_MASK			0b00011111
#define NRF24_CMD_R_RX_PAYLOAD  	0b01100001	// Read RX payload
#define NRF24_CMD_W_TX_PAYLOAD  	0b10100000	// Write TX payload
#define NRF24_CMD_FLUSH_TX			0b11100001	// Flush TX register
#define NRF24_CMD_FLUSH_RX			0b11100010	// Flush RX register
#define NRF24_CMD_REUSE_TX_PL   	0b11100011	// Reuse TX payload
#define NRF24_CMD_R_RX_PL_WID   	0b01100000	// Read top RX FIFO payload width
#define NRF24_CMD_W_ACK_PAYLOAD 	0b10101000	// Write ACK payload - last 3 bits = pipe number, 0b000 to 0b101
#define NRF24_CMD_W_TX_PAYLOAD_NO_ACK	0b10110000	// Write TX payload (no ACK req.)
#define NRF24_CMD_NOP				0b11111111	// No Operation, used for reading status register

/* nRF24L01 Register Definitions */
#define NRF24_REG_CONFIG		0x00	// nRF24L01 config register
#define NRF24_REG_EN_AA			0x01	// nRF24L01 enable Auto-Acknowledge register
#define NRF24_REG_EN_RXADDR		0x02	// nRF24L01 enable RX addresses register
#define NRF24_REG_SETUP_AW		0x03	// nRF24L01 setup of address width register
#define NRF24_REG_SETUP_RETR	0x04	// nRF24L01 setup of automatic retransmission reg
#define NRF24_REG_RF_CH			0x05	// nRF24L01 RF channel register
#define NRF24_REG_RF_SETUP		0x06	// nRF24L01 RF setup register
#define NRF24_REG_STATUS		0x07	// nRF24L01 status register
#define NRF24_REG_OBSERVE_TX	0x08	// nRF24L01 transmit observe register
#define NRF24_REG_RPD			0x09	// nRF24L01 received power detector (carrier detect register)
#define NRF24_REG_RX_ADDR_P0	0x0A	// nRF24L01 receive address data pipe0
#define NRF24_REG_RX_ADDR_P1	0x0B	// nRF24L01 receive address data pipe1
#define NRF24_REG_RX_ADDR_P2	0x0C	// nRF24L01 receive address data pipe2
#define NRF24_REG_RX_ADDR_P3	0x0D	// nRF24L01 receive address data pipe3
#define NRF24_REG_RX_ADDR_P4	0x0E	// nRF24L01 receive address data pipe4
#define NRF24_REG_RX_ADDR_P5	0x0F	// nRF24L01 receive address data pipe5
#define NRF24_REG_TX_ADDR		0x10	// nRF24L01 transmit address
#define NRF24_REG_RX_PW_P0		0x11	// nRF24L01 # of bytes in RX payload for pipe0
#define NRF24_REG_RX_PW_P1		0x12	// nRF24L01 # of bytes in RX payload for pipe1
#define NRF24_REG_RX_PW_P2		0x13	// nRF24L01 # of bytes in RX payload for pipe2
#define NRF24_REG_RX_PW_P3		0x14	// nRF24L01 # of bytes in RX payload for pipe3
#define NRF24_REG_RX_PW_P4		0x15	// nRF24L01 # of bytes in RX payload for pipe4
#define NRF24_REG_RX_PW_P5		0x16	// nRF24L01 # of bytes in RX payload for pipe5
#define NRF24_REG_FIFO_STATUS	0x17	// nRF24L01 FIFO status register
#define NRF24_REG_DYNPD			0x1C	// nRF24L01 Dynamic payload setup
#define NRF24_REG_FEATURE		0x1D	// nRF24L01 Exclusive feature setup


#define NRF24_STATUS_IRQ_FLAGS		0b01110000
#define NRF24_ALL_PIPES				0b00111111


typedef enum {
	NRF24_CRC_DISABLED = 0,
	NRF24_CRC_8 = 1,
	NRF24_CRC_16 = 2
} NRF24_CRC_Mode;

/**
 * An enum describing the radio's power mode.
 */
typedef enum {
    NRF24_PTX = 0,            /* Primary TX operation */
    NRF24_PRX = 1             /* Primary RX operation */
} NRF24_Operation_Mode;

/**
 * An enum describing the radio's irq sources.
 */
typedef enum {
    NRF24_MAX_RT = 0b00010000,     /* Max retries interrupt */
    NRF24_TX_DS = 0b00100000,          /* TX data sent interrupt */
    NRF24_RX_DR = 0b01000000           /* RX data received interrupt */
} NRF24_IRQ_Source;

/**
 * An enum describing the radio's power mode.
 */
typedef enum {
    NRF24_PWR_DOWN = 0,       /* Device power-down */
    NRF24_PWR_UP = 1          /* Device power-up */
} NRF24_Power_Mode;

/**
 * An enum describing the radio's output power mode's.
 */
typedef enum {
	NRF24_POWER_OUTPUT_18DBM = 0b00,          /* Output power set to -18dBm */
	NRF24_POWER_OUTPUT_12DBM = 0b01,          /* Output power set to -12dBm */
	NRF24_POWER_OUTPUT_6DBM = 0b10,           /* Output power set to -6dBm  */
	NRF24_POWER_OUTPUT_0DBM = 0b11            /* Output power set to 0dBm   */
} NRF24_PowerOutputLevel;

typedef enum {
	NRF24_AIR_DATA_RATE_250KBPS = 0,
	NRF24_AIR_DATA_RATE_1MBPS = 1,
	NRF24_AIR_DATA_RATE_2MBPS = 2
} NRF24_AirDataRate;



/**
 * An enum describing the radio's address width.
 */
typedef enum {
    NRF24_AW_3BYTES = 3,      /* Set address width to 3 bytes */
    NRF24_AW_4BYTES = 4,          /* Set address width to 4 bytes */
    NRF24_AW_5BYTES = 5           /* Set address width to 5 bytes */
} NRF24_Address_Width;

/**
 * An enum describing the nRF24L01 pipe addresses and TX address.
 */
typedef enum {
    NRF24_PIPE0 = 0,          /* Select pipe0 */
    NRF24_PIPE1,              /* Select pipe1 */
    NRF24_PIPE2,              /* Select pipe2 */
    NRF24_PIPE3,              /* Select pipe3 */
    NRF24_PIPE4,              /* Select pipe4 */
    NRF24_PIPE5,              /* Select pipe5 */
    NRF24_TX,                 /* Refer to TX address*/
    NRF24_ALL = 0xFF          /* Close or open all pipes*/
} NRF24_Address;



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

typedef struct tag_NRF24_PipeStatus {
	uint8_t RX_ADDR_ENABLED : 1;	// bit 0; RX address enabled for this pipe
	uint8_t AUTO_ACK_ENABLED : 1;	// bit 1; auto ACK enabled for this pipe
} NRF24_PipeStatus;



typedef void (*nrf24_debug_print_fn)(char*);
typedef void (*nrf24_setup_pins_ptr)(void);
typedef uint8_t (*nrf24_spi_transfer_ptr)(const uint8_t);
typedef void (*nrf24_csn_ptr)(uint8_t);
typedef void (*nrf24_ce_ptr)(uint8_t);
typedef void (*nrf24_hw_delay_us_ptr)(uint16_t);

typedef struct {
	nrf24_setup_pins_ptr setup_pins;
	nrf24_spi_transfer_ptr spi_transfer;
	nrf24_csn_ptr set_csn;
	nrf24_ce_ptr set_ce;
	nrf24_hw_delay_us_ptr delay_us;

//	uint8_t payload_len;
//	bool dynamic_payloads_enabled;

} NRF24_t;

//typedef struct {
//	void (*setup_pins)(void);
//	uint8_t (*spi_transfer)(const uint8_t);
//	void (*set_csn)(uint8_t);
//	void (*set_ce)(uint8_t);
//	void (*delay_us)(uint16_t);
//
////	uint8_t payload_len;
////	bool dynamic_payloads_enabled;
//
//} NRF24_t;


/*
 * Setup functions prototypes
 */

/**
 * Set radio's operation mode.
 *
 * Use this function to enter PTX (primary TX) or PRX (primary RX).
 *
 * @param op_mode Operation mode.
 */
void nrf24_set_operation_mode(NRF24_t *nrf24, NRF24_Operation_Mode op_mode);

/**
 * Get radio's operation mode.
 */
NRF24_Operation_Mode nrf24_get_operation_mode(NRF24_t *nrf24);

/**
 * Enables the dynamic packet length.
 *
 * @param enable Whether enable or disable dynamic packet length.
 */
void nrf24_enable_dynamic_payload(NRF24_t *nrf24, bool enable);

/**
 * Check if dynamic packet length feature is enabled.
 */
bool nrf24_is_dynamic_payload_enabled(NRF24_t *nrf24);

/**
 * Enables the ACK payload feature.
 *
 * @param enable Whether to enable or disable ACK payload.
 */
void nrf24_enable_ack_payload(NRF24_t *nrf24, bool enable);

/**
 * Check if ACK payload feature is enabled.
 */
bool nrf24_is_ack_payload_enabled(NRF24_t *nrf24);

/**
 * Enables the dynamic ACK feature.
 *
 * @param enable Whether to enable or disable dynamic ACK.
 */
void nrf24_enable_dynamic_ack(NRF24_t *nrf24, bool enable);

/**
 * Check if dynamic ACK feature is enabled.
 */
bool nrf24_is_dynamic_ack_enabled(NRF24_t *nrf24);

/**
 * Function for enabling dynamic payload size.
 *
 * The input parameter is a byte where the bit values tells weather the pipe
 * should use dynamic payload size. For example if bit 0 is set then pipe 0
 * will accept dynamic payload size.
 *
 * @param setup Byte value telling for which pipes to enable dynamic payload
 * size.
 */
void nrf24_setup_dynamic_payload(NRF24_t *nrf24, NRF24_Register_EnableDynamicPaylodLength setup);

/**
 * Write ACK payload.
 *
 * Writes the payload that will be transmitted with the ACK on a given pipe.
 *
 * @param pipe Pipe that transmits the payload.
 * @param tx_pload Pointer to the payload data.
 * @param length Size of the data to transmit.
 */
void nrf24_write_ack_payload(NRF24_t *nrf24, uint8_t pipe_num, const void* tx_pload, uint8_t len);

/**
 * Set radio's RF channel.
 *
 * Use this function to select which RF channel to use.
 *
 * @param channel RF channel.
 */
/* Set channel */
// frequency = 2400MHz + channel * 1MHz;
void nrf24_set_rf_channel(NRF24_t *nrf24, uint8_t channel);

/**
 * Get radio's RF channel.
 */
// frequency = 2400MHz + channel * 1MHz;
uint8_t nrf24_get_rf_channel(NRF24_t *nrf24);

/**
 * Set radio's TX output power.
 *
 * Use this function set the radio's TX output power.
 *
 * @param power Radio's TX output power.
 */
/* set power output level */
void nrf24_set_power_output_level(NRF24_t *nrf24, NRF24_PowerOutputLevel power);

/**
 * Get radio's TX output power.
 */
/* get power output level */
NRF24_PowerOutputLevel nrf24_get_power_output_level(NRF24_t *nrf24);

/**
 * Set radio's on-air data-rate.
 *
 * Use this function to select radio's on-air data-rate.
 *
 * @param datarate On-air data-rate
 */
/* set air data rate */
void nrf24_set_air_datarate(NRF24_t *nrf24, NRF24_AirDataRate air_data_rate);

/**
 * Get radio's on-air data-rate.
 */
/* get air data rate */
NRF24_AirDataRate nrf24_get_air_datarate(NRF24_t *nrf24);

/**
 * Set CRC mode used by the radio.
 *
 * Use this function to set CRC mode: CRC disabled, 1 or 2 bytes.
 *
 * @param crc_mode CRC mode to use.
 */
/* CRC - set mode */
void nrf24_set_crc_mode(NRF24_t *nrf24, NRF24_CRC_Mode crc);

/**
 * Get CRC mode used by the radio.
 */
/* CRC - get mode */
NRF24_CRC_Mode nrf24_get_crc_mode(NRF24_t *nrf24);

/**
 * Set auto retransmission parameters.
 *
 * Use this function to set auto retransmission parameters.
 *
 * @param retr Number of retransmits, 0 means retransmit OFF.
 * @param delay Retransmit delay in usec (in range 250-4000 with step 250).
 */
/* set retries */
// retry_delay from 250us to 4000us
void nrf24_set_auto_retry(NRF24_t *nrf24, uint16_t retry_delay, uint8_t retry_count);

/**
 * Get auto retransmission number of retransmits.
 */
uint8_t nrf24_get_auto_retry_count(NRF24_t *nrf24);

/**
 * Get auto retransmission delay (usec).
 */
uint16_t hal_nrf_get_auto_retry_delay(NRF24_t *nrf24);

/**
 * Set payload width for selected pipe.
 *
 * Use this function to set the number of bytes expected on a selected pipe.
 *
 * @param pipe_num Pipe number to set payload width for.
 * @param pload_width number of bytes expected.
 */
void nrf24_set_rx_payload_width(NRF24_t *nrf24, uint8_t pipe_num, uint8_t pload_width);

/**
 * Get RX payload width for selected pipe.
 *
 * Use this function to get the expected payload width for selected pipe number.
 *
 * @param pipe_num Pipe number to get payload width for.
 * @return Payload_Width in bytes.
 */
uint8_t nrf24_get_rx_payload_width(NRF24_t *nrf24, uint8_t pipe_num);

/**
 * Open radio pipe and enable/disable auto acknowledge.
 *
 * Use this function to open one or all pipes, with or without auto acknowledge.
 *
 * @param pipe_num Radio pipe to open.
 * @param auto_ack Auto_Ack ON/OFF.
 */
void nrf24_open_pipe(NRF24_t *nrf24, NRF24_Address pipe, bool auto_ack);

/**
 * Close radio pipe.
 *
 * Use this function to close one pipe or all pipes.
 *
 * @param pipe_num Pipe# number to close.
 */
void nrf24_close_pipe(NRF24_t *nrf24, NRF24_Address pipe);

/**
 * Get pipe status.
 *
 * Use this function to check status for a selected pipe.
 *
 * @param  pipe_num Pipe number to check status for.
 * @return Pipe_Status.
 * @retval 0x00 Pipe is closed, autoack disabled,
 * @retval 0x01 Pipe is open, autoack disabled,
 * @retval 0x03 Pipe is open, autoack enabled.
 */
NRF24_PipeStatus nrf24_get_pipe_status(NRF24_t *nrf24, NRF24_Address pipe);

/**
 * Set radio's address width.
 *
 * Use this function to define the radio's address width, refers to both
 * RX and TX.
 *
 * @param address_width Address with in bytes.
 */
void nrf24_set_address_width(NRF24_t *nrf24, NRF24_Address_Width address_width);

/**
 * Gets the radio's address width.
 *
 * @return Address width
 */
uint8_t nrf24_get_address_width(NRF24_t *nrf24);

/**
 * Set radio's RX address and TX address.
 *
 * Use this function to set a RX address, or to set the TX address. Beware of
 * the difference for single and multibyte address registers.
 *
 * @param address Which address to set.
 * @param *addr Buffer from which the address is stored in.
 */
void nrf24_set_address(NRF24_t *nrf24, const NRF24_Address address, const uint8_t *addr);

/**
 * Get address for selected pipe.
 *
 * Use this function to get address for selected pipe.
 *
 * @param address Which address to get, Pipe- or TX-address.
 * @param *addr buffer in which address bytes are written. For pipes containing
 * only LSB byte of address, this byte is returned in the *addr buffer.
 * @return Numbers of bytes copied to addr
 */
uint8_t nrf24_get_address(NRF24_t *nrf24, NRF24_Address address, uint8_t *addr);

/**
 * Configures pipe for transmission by:
 * 1. Enabling a pipe,
 * 2. Setting its address,
 * 3. Setting RX payload length.
 *
 * @param pipe_num Pipe to open.
 * @param addr Pipe address, if 0: don't change the address.
 * @param auto_ack Auto_Ack ON/OFF.
 * @param pload_width RX payload length. Set to max payload length (NRF_MAX_PL)
 * in case if dynamic feature is enabled.
 */
//void hal_nrf_config_rx_pipe(hal_nrf_address_t pipe_num,
//    const uint8_t *addr, bool auto_ack, uint8_t pload_width);
//XXX;

/**
 * Configure TX transmission by:
 * 1. Setting receiver address.
 * 2. Setting TX output power.
 * 3. Open pipe 0 for receiving ACKs packets.
 * 4. Setting auto-retry parameters.
 *
 * @param addr Receiver address, if 0: don't change the address.
 * @param power TX output power.
 * @param retr Number of retransmits, 0 means retransmit OFF.
 * @param delay Retransmit delay in usec (in range 250-4000 with step 250).
 */
//void hal_nrf_config_tx(const uint8_t *addr,
//    hal_nrf_output_power_t power, uint8_t retr, uint16_t delay);
//XXX;

/**
 * Enable or disable interrupt for the radio.
 *
 * Use this function to enable or disable one of the interrupt sources for the
 * radio. This function only changes state for selected int_source, the rest of
 * the interrupt sources are left unchanged.
 *
 * @param int_source Radio interrupt Source.
 * @param irq_state Enable or Disable.
 */
void nrf24_set_irq_mode(NRF24_t *nrf24, NRF24_IRQ_Source int_source, bool irq_enabled);

/**
 * Get interrupt mode (enabled or disabled) for a given int_source.
 */
bool nrf24_get_irq_mode(NRF24_t *nrf24, NRF24_IRQ_Source int_source);

/**
 * Read all interrupt flags.
 *
 * Use this function to get the interrupt flags. This function is similar to
 * hal_nrf_get_clear_irq_flags() with the exception that it does NOT clear the
 * irq_flags.
 *
 * @return Interrupt_flags.
 * @retval 0x10 Max Retransmit interrupt,
 * @retval 0x20 TX Data sent interrupt,
 * @retval 0x40 RX Data received interrupt.
 */
uint8_t nrf24_get_irq_flags(NRF24_t *nrf24);

/**
 * Clear all interrupt flags.
 *
 * Use this function to clear the interrupt flags.
 *
 */
void nrf24_clear_irq_flags(NRF24_t *nrf24);

/**
 * Clear one selected interrupt flag.
 *
 * Use this function to clear one specific interrupt flag. Other interrupt
 * flags are left unchanged.
 *
 * @param int_source Interrupt source of which flag to clear.
 */
void nrf24_clear_irq_flag(NRF24_t *nrf24, NRF24_IRQ_Source int_source);

/**
 * Set radio's power mode.
 *
 * Use this function to power_up or power_down radio.
 *
 * @param pwr_mode POWER_UP or POWER_DOWN.
 */
void nrf24_set_power_mode(NRF24_t *nrf24, NRF24_Power_Mode pwr_mode);

/**
 * Get radio's operation mode.
 */
NRF24_Power_Mode hal_nrf_get_power_mode(NRF24_t *nrf24);

/*
 * Status functions prototypes
 */

/**
 * Check for TX FIFO empty.
 *
 * Use this function to check if TX FIFO is empty.
 *
 * @return TX FIFO empty bit,
 * @retval FALSE TX FIFO NOT empty,
 * @retval TRUE TX FIFO empty.
 *
 */
bool nrf24_tx_fifo_empty(NRF24_t *nrf24);

/**
 * Check for TX FIFO full.
 *
 * Use this function to check if TX FIFO is full.
 *
 * @return TX FIFO full bit.
 * @retval FALSE TX FIFO NOT full,
 * @retval TRUE TX FIFO full.
 *
 */
bool nrf24_tx_fifo_full(NRF24_t *nrf24);

/**
 * Check for RX FIFO empty.
 *
 * Use this function to check if RX FIFO is empty. Reads STATUS register to
 * check this, not FIFO_STATUS
 *
 * @return RX FIFO empty bit.
 * @retval FALSE RX FIFO NOT empty,
 * @retval TRUE RX FIFO empty.
 *
 */
/* Checks if receive FIFO is empty or not */
bool nrf24_rx_fifo_empty(NRF24_t *nrf24);

/**
 * Check for RX FIFO full.
 *
 * Use this function to check if RX FIFO is full.
 *
 * @return RX FIFO full bit.
 * @retval FALSE RX FIFO NOT full,
 * @retval TRUE RX FIFO full.
 */
bool nrf24_rx_fifo_full(NRF24_t *nrf24);

/**
 * Get FIFO_STATUS register.
 */
NRF24_Register_FIFOStatus nrf24_get_fifo_status(NRF24_t *nrf24);

/**
 * Get radio's transmit status register (OBSERVE_TX).
 *
 * @return Transmit status register.
 * @retval UpperNibble Packets lost counter.
 * @retval LowerNibble Retransmit attempts counter.
 */
NRF24_Register_TransmitObserve nrf24_get_auto_retry_status(NRF24_t *nrf24);

/**
 * Get radio's transmit attempts counter.
 *
 * @return Retransmit attempts counter.
 */
/* Returns the number of retransmissions occurred for the last message */
// Count retransmitted packets. The counter is reset when transmission of a new packet starts.
uint8_t nrf24_get_retransmit_attempts(NRF24_t *nrf24);

/**
 * Get packets lost counter.
 *
 * @return Packets lost counter.
 */
// Count lost packets. The counter is overflow protected to 15, and discontinues at max until reset.
// The counter is reset by writing to RF_CH.
uint8_t nrf24_get_packet_lost_count(NRF24_t *nrf24);

/**
 * Get the carrier detect flag.
 *
 * Use this function to get the carrier detect flag, used to detect stationary
 * disturbance on selected RF channel.
 *
 * @return Carrier Detect.
 * @retval FALSE Carrier NOT Detected,
 * @retval TRUE Carrier Detected.
 */
/**
 * RPD used for nRF24l01+
 */
/* detect carrier, received power detector */
bool nrf24_received_power_detected(NRF24_t *nrf24);

/*
 * Data operation functions prototypes
 */

/**
 * Get RX data source.
 *
 * Use this function to read which RX pipe data was received on for current top
 * level FIFO data packet.
 *
 * @return pipe number of current packet present.
 */
uint8_t nrf24_get_rx_data_source(NRF24_t *nrf24);

/**
 * Reads the received payload width.
 *
 * @return Received payload width.
 */
/* Returns the length of data waiting in the RX fifo */
uint8_t nrf24_read_rx_payload_width(NRF24_t *nrf24);

/**
 * Read RX payload.
 *
 * Use this function to read top level payload available in the RX FIFO.
 *
 * @param *rx_pload pointer to buffer in which RX payload are stored.
 * @return pipe number (MSB byte) and packet length (LSB byte).
 */
// returns length of received bytes
uint8_t nrf24_read_rx_payload(NRF24_t *nrf24, uint8_t *buf);

/**
 * Write TX payload to radio.
 *
 * Use this function to write a packet of TX payload into the radio.
 * length is a number of bytes, which are stored in *tx_pload.
 *
 * @param *tx_pload pointer to buffer in which TX payload are present.
 * @param length number of bytes to write.
 */
void nrf24_write_tx_payload(NRF24_t *nrf24, const void* buf, uint8_t len);

/**
 * Write TX payload which do not require ACK.
 *
 * When transmitting the ACK is not required nor sent from the receiver.
 * The payload will always be assumed as "sent". Use this function to write
 * a packet of TX payload into the radio. length is a number of bytes, which
 * are stored in *tx_pload.
 *
 * @param *tx_pload pointer to buffer in which TX payload are present.
 * @param length number of bytes to write.
 */
void nrf24_write_tx_payload_noack(NRF24_t *nrf24, const void* buf, uint8_t len);

/**
 * Get status of reuse TX function.
 *
 * Use this function to check if reuse TX payload is activated.
 *
 * @return Reuse TX payload mode.
 * @retval FALSE Not activated,
 * @retval TRUE Activated.
 */
bool nrf24_get_reuse_tx_status(NRF24_t *nrf24);

/**
 * Reuse TX payload.
 *
 * Use this function to set that the radio is using the last transmitted payload
 * for the next packet as well.
 */
void nrf24_reuse_tx_payload(NRF24_t *nrf24);

/**
 * Flush RX FIFO.
 *
 * Use this function to flush the radio's RX FIFO.
 */
void nrf24_flush_rx(NRF24_t *nrf24);

/**
 * Flush TX FIFO.
 *
 * Use this function to flush the radio's TX FIFO.
 */
void nrf24_flush_tx(NRF24_t *nrf24);

/**
 * No Operation command.
 *
 * Use this function to receive the radio's status register.
 *
 * @return Status register.
 */
NRF24_Register_Status nrf24_get_status(NRF24_t *nrf24);

/*
 * Test functions prototypes
 */

/**
 * Set radio's PLL mode.
 *
 * Use this function to either LOCK or UNLOCK the radio's PLL.
 *
 * @param pll_lock PLL locked, TRUE or FALSE.
 */
void nrf24_set_pll_mode(NRF24_t *nrf24, bool pll_lock);

/**
 * Get radio's PLL mode.
 */
bool nrf24_get_pll_mode(NRF24_t *nrf24);

/**
 * Enables continuous carrier transmit.
 *
 * Use this function to enable or disable continuous carrier transmission.
 *
 * @param enable Enable continuous carrier.
 */
void nrf24_enable_continious_wave(NRF24_t *nrf24, bool enable);

/**
 * Check if continuous carrier transmit is enabled.
 */
bool nrf24_is_continious_wave_enabled(NRF24_t *nrf24);



void nrf24_debug(NRF24_t *nrf24, nrf24_debug_print_fn print_fn);
NRF24_Register_Config nrf24_get_config(NRF24_t *nrf24);
void nrf24_init(NRF24_t *nrf24, nrf24_setup_pins_ptr setup_pins, nrf24_spi_transfer_ptr spi_transfer, nrf24_csn_ptr set_csn, nrf24_ce_ptr set_ce, nrf24_hw_delay_us_ptr delay_us);
//void nrf24_init(NRF24_t *nrf24, void (*setup_pins)(void), uint8_t (*spi_transfer)(const uint8_t), void (*set_csn)(uint8_t), void (*set_ce)(uint8_t), void (*delay_us)(uint16_t));
void nrf24_config(NRF24_t *nrf24);
void nrf24_config_rx_pipe(NRF24_t *nrf24, NRF24_Address pipe, const uint8_t *addr, bool auto_ack, uint8_t pload_width);
void nrf24_config_tx(NRF24_t *nrf24, const uint8_t *addr);
void nrf24_pulse_ce(NRF24_t *nrf24);

#endif	/* NRF24L01_H */

