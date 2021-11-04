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

#define LCD_I2C_DEFAULT_ADDR			0b111

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
	PCF8574_Handle pcf8574_obj;
	device_delay_ms_ptr delay_ms;
	uint8_t backlight_en;
} LCD_Handle;

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


/**
 * @brief  LCD initialization (I2C mode, via PCF8574).
 * @param	lcd*		pointer to LCD_Handle
 * @param	addr_pins	A0, A1, A2 address pins (PCF8574). 0b000 to 0b111
 * @param	write_reg	The function that writes command to the module. Hardware dependent.
 * @param	read_reg	The function that reads data from the module. Hardware dependent.
 * @param	delay_ms	The function that makes delay in milliseconds. Hardware dependent.
 */
void LCD_init_i2c(LCD_Handle *lcd, uint8_t addr_pins, device_write_ptr write_reg, device_read_ptr read_reg, device_delay_ms_ptr delay_ms);

/**
 * @brief  Read busy flag and address counter.
 * @param	lcd*			pointer to LCD_Handle
 * @retval	LCD_BusyFlagAndAddressCounter
 */
LCD_BusyFlagAndAddressCounter LCD_get_busy_flag_and_address_counter(LCD_Handle *lcd);

/**
 * @brief  Send command to LCD module.
 * @param	lcd*	pointer to LCD_Handle
 * @param	cmd		Command to send.
 */
void LCD_send_command(LCD_Handle *lcd, uint8_t cmd);

/**
 * @brief  Send data to LCD module.
 * @param	lcd*	pointer to LCD_Handle
 * @param	data	Data to send.
 */
void LCD_send_data(LCD_Handle *lcd, uint8_t data);

/**
 * @brief  Send string to LCD module.
 * @param	lcd*	pointer to LCD_Handle
 * @param	str*	array of chars to send (string).
 */
void LCD_send_string(LCD_Handle *lcd, char *str);

/**
 * @brief  Set position in LCD display, where to print character on display.
 * @param	lcd*	pointer to LCD_Handle
 * @param	row		Row (values 0 to 1)
 * @param	col		Column (values 0 to 39)
 */
void LCD_set_position(LCD_Handle *lcd, uint8_t row, uint8_t col);

/**
 * @brief  Set cursor.
 * @param	lcd*			pointer to LCD_Handle
 * @param	show_cursor		true = cursor is displayed
 * @param	blink			true = cursor/character is blinking
 */
void LCD_set_cursor(LCD_Handle *lcd, bool show_cursor, bool blink);

/**
 * @brief  Clear LCD.
 * @param	lcd*	pointer to LCD_Handle
 */
void LCD_clear(LCD_Handle *lcd);

/**
 * @brief  Shift display contents or cursor.
 * @param	lcd*	pointer to LCD_Handle
 * @param	sc		shift display data or cursor
 * @param	el		shift to left or right
 */
void LCD_shift(LCD_Handle *lcd, LCD_ShiftObject sc, LCD_ShiftDirection rl);

/**
 * @brief  Define custom character. A command must follow this, not data write.
 * @param	lcd*	pointer to LCD_Handle
 * @param	index	index of character position (values from 0 to 7)
 * @param	data*	array of 8 bytes. lowest 5 bits of each byte matter
 */
void LCD_custom_character(LCD_Handle *lcd, uint8_t index, uint8_t *data);

/**
 * @brief  Enable/disable LCD backlight.
 * @param	lcd*	pointer to LCD_Handle
 * @param	enable	boolean, enable backlight
 */
void LCD_backlight(LCD_Handle *lcd, bool enable);

#endif /* INC_LCD1602_H_ */
