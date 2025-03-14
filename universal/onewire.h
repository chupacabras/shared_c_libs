/**
  ******************************************************************************
  * @file    onewire.h
  * @author  Juraj Lonc (juraj.lonc@gmail.com)
  * @brief   Header file of One-Wire driver.
  ******************************************************************************
  */

#ifndef INC_ONEWIRE_H_
#define INC_ONEWIRE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

// you can exclude onewire_search by defining that to 0
#ifndef ONEWIRE_SEARCH
#define ONEWIRE_SEARCH 1
#endif

// You can exclude CRC checks altogether by defining this to 0
#ifndef ONEWIRE_CRC
#define ONEWIRE_CRC 1
#endif

// Select the table-lookup method of computing the 8-bit CRC
// by setting this to 1.  The lookup table enlarges code size by
// about 250 bytes.  It does NOT consume RAM (but did in very
// old versions of OneWire).  If you disable this, a slower
// but very compact algorithm is used.
#ifndef ONEWIRE_CRC8_TABLE
#define ONEWIRE_CRC8_TABLE 1
#endif

// You can allow 16-bit CRC checks by defining this to 1
// (Note that ONEWIRE_CRC must also be 1.)
#ifndef ONEWIRE_CRC16
#define ONEWIRE_CRC16 1
#endif


typedef void(*onewire_pin_output) (void);
typedef void(*onewire_pin_input) (void);
typedef void(*onewire_pin_high) (void);
typedef void(*onewire_pin_low) (void);
typedef uint8_t(*onewire_pin_read) (void);
typedef void(*onewire_delay_us) (uint16_t);
typedef void(*onewire_interrupts_on) (void);
typedef void(*onewire_interrupts_off) (void);

typedef struct {
	onewire_pin_output pin_output;
	onewire_pin_input pin_input;
	onewire_pin_low pin_low;
	onewire_pin_high pin_high;
	onewire_pin_read pin_read;
	onewire_delay_us delay_us;
	onewire_interrupts_on interrupts_on;
	onewire_interrupts_off interrupts_off;
#ifdef ONEWIRE_SEARCH
	uint8_t LastDiscrepancy;       /*!< Search private */
		uint8_t LastFamilyDiscrepancy; /*!< Search private */
		uint8_t LastDeviceFlag;        /*!< Search private */
		uint8_t ROM_NO[8];             /*!< 8-bytes address of last search device */
#endif
} OneWireObject;

/* OneWire commands */
#define ONEWIRE_CMD_RSCRATCHPAD			0xBE
#define ONEWIRE_CMD_WSCRATCHPAD			0x4E
#define ONEWIRE_CMD_CPYSCRATCHPAD		0x48
#define ONEWIRE_CMD_RECEEPROM			0xB8
#define ONEWIRE_CMD_RPWRSUPPLY			0xB4
#define ONEWIRE_CMD_SEARCHROM			0xF0
#define ONEWIRE_CMD_READROM				0x33
#define ONEWIRE_CMD_MATCHROM			0x55
#define ONEWIRE_CMD_SKIPROM				0xCC


void onewire_init(OneWireObject *ow, onewire_pin_output pin_output, onewire_pin_input pin_input, onewire_pin_low pin_low, onewire_pin_high pin_high, onewire_pin_read pin_read, onewire_delay_us delay_us, onewire_interrupts_on interrupts_on, onewire_interrupts_off interrupts_off);
void onewire_depower(OneWireObject *ow);
void onewire_write_bit(OneWireObject *ow, uint8_t bit);
void onewire_write_byte(OneWireObject *ow, uint8_t byte);
uint8_t onewire_read_bit(OneWireObject *ow);
uint8_t onewire_read_byte(OneWireObject *ow);
uint8_t onewire_reset(OneWireObject *ow);


void onewire_rom_match(OneWireObject *ow, uint8_t rom[8]);
void onewire_rom_skip(OneWireObject *ow);

#ifdef ONEWIRE_SEARCH
uint8_t onewire_find_first(OneWireObject *ow, uint8_t *new_addr);
uint8_t onewire_find_next(OneWireObject *ow, uint8_t *new_addr);
void onewire_setup_search_family(OneWireObject *ow, uint8_t family_code);
void onewire_reset_search(OneWireObject *ow);
uint8_t onewire_rom_search(OneWireObject *ow, uint8_t *new_addr);
bool onewire_read_rom(OneWireObject *ow, uint8_t rom[8]);

#endif

uint8_t onewire_crc8(uint8_t *addr, uint8_t len);
uint8_t onewire_check_crc16(uint8_t* input, uint16_t len, uint8_t* inverted_crc);
uint16_t onewire_crc16(uint8_t* input, uint16_t len);


#ifdef __cplusplus
}
#endif

#endif /* INC_ONEWIRE_H_ */
