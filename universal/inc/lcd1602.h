/**
  ******************************************************************************
  * @file    lcd1602.h
  * @author  Juraj Lonc (juraj.lonc@gmail.com)
  * @brief   Header file for LCD 16x02 driver.
  ******************************************************************************
  */

#ifndef INC_LCD1602_H_
#define INC_LCD1602_H_

#include <stdint.h>
#include <stdbool.h>
#include "pcf8574.h"

#define LCD_CMD_CLEAR_DISPLAY			0b00000001
#define LCD_CMD_CURSOR_RETURN			0b00000010
#define LCD_CMD_ENTRY_MODE_SET			0b00000100
#define LCD_CMD_DISPLAY_ON_OFF_CONTROL	0b00001000
#define LCD_CMD_CURSOR_DISPLAY_SHIFT	0b00010000
#define LCD_CMD_FUNCTION_SET			0b00100000
#define LCD_CMD_SET_CGRAM_ADDRESS		0b01000000
#define LCD_CMD_SET_DDRAM_ADDRESS		0b10000000

#define LCD_ENTRY_MODE_INCREMENT		0b00000010
#define LCD_ENTRY_MODE_DECREMENT		0b00000000
#define LCD_ENTRY_MODE_SHIFT			0b00000001

#define LCD_SHIFT_DISPLAY				0b00001000
#define LCD_SHIFT_CURSOR				0b00000000
#define LCD_SHIFT_RIGHT					0b00000100
#define LCD_SHIFT_LEFT					0b00000000

#define LCD_CONTROL_DISPLAY_ON			0b00000100
#define LCD_CONTROL_DISPLAY_OFF			0b00000000
#define LCD_CONTROL_CURSOR_ON			0b00000010
#define LCD_CONTROL_CURSOR_OFF			0b00000000
#define LCD_CONTROL_CURSOR_BLINK_ON		0b00000001
#define LCD_CONTROL_CURSOR_BLINK_OFF	0b00000000

#define LCD_FUNCTION_8BIT_INTERFACE		0b00010000
#define LCD_FUNCTION_4BIT_INTERFACE		0b00000000
#define LCD_FUNCTION_2ROWS				0b00001000
#define LCD_FUNCTION_1ROW				0b00000000
#define LCD_FUNCTION_5X10_FONT			0b00000100
#define LCD_FUNCTION_5X7_FONT			0b00000000

/*
 * I2C board wiring (PCF8574 -> LCD1602)
 *
 * P0 -> RS
 * P1 -> RW
 * P2 -> E
 * P3 -> backlight enable
 * P4 -> D4
 * P5 -> D5
 * P6 -> D6
 * P7 -> D7
 */

#define LCD_I2C_RS				0b0001
#define LCD_I2C_RW				0b0010
#define LCD_I2C_ENABLE			0b0100
#define LCD_I2C_BACKLIGHT_EN	0b1000

typedef struct {
	PCF8574_Object pcf8574_obj;
	device_delay_ms_ptr delay_ms;
} LCD_Object;

typedef enum {
	LCD_SHIFT__CURSOR,
	LCD_SHIFT__DISPLAY
} LCD_ShiftObject;

typedef enum {
	LCD_SHIFT_RIGHT_TO_LEFT,
	LCD_SHIFT_LEFT_TO_RIGHT
} LCD_ShiftDirection;

typedef union {
	struct {
		uint8_t DATA;
	};
	struct {
		uint8_t AC:7; // address counter
		uint8_t BF:1; // busy flag
	};
} LCD_BusyFlagAndAddressCounter;

void LCD_init_i2c(LCD_Object *lcd, uint8_t addr_pins, device_write_ptr write_reg, device_read_ptr read_reg, device_delay_ms_ptr delay_ms);
LCD_BusyFlagAndAddressCounter LCD_get_busy_flag_and_address_counter(LCD_Object *lcd);
void LCD_send_command(LCD_Object *lcd, uint8_t cmd);
void LCD_send_data(LCD_Object *lcd, uint8_t data);
void LCD_send_string(LCD_Object *lcd, char *str);

void LCD_set_position(LCD_Object *lcd, uint8_t row, uint8_t col);
void LCD_set_cursor(LCD_Object *lcd, bool show_cursor, bool blink);
void LCD_clear(LCD_Object *lcd);
void LCD_shift(LCD_Object *lcd, LCD_ShiftObject sc, LCD_ShiftDirection rl);
void LCD_custom_character(LCD_Object *lcd, uint8_t index, uint8_t *data);


#endif /* INC_LCD1602_H_ */
