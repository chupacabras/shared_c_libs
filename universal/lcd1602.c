/**
 ******************************************************************************
 * @file    lcd1602.c
 * @author  Juraj Lonc (juraj.lonc@gmail.com)
 * @brief   LCD 16x02 driver.
 ******************************************************************************
 */

#include "lcd1602.h"

static uint8_t data_arr[4];

/**
 * @brief  Send only 4-bit nibble (upper 4-bits from 8-bit data), when 8-bit mode active, through I2C.
 * 			Used for initialization while 4-bit mode is not active yet.
 * @param	lcd*		pointer to LCD_Object
 * @param	data		Data to send (upper 4-bits from 8-bit data).
 */
void LCD_send_internal_cmd8_nibble(LCD_Object *lcd, uint8_t data) {
	uint8_t up = data & 0xF0;

	data_arr[0] = up | LCD_I2C_ENABLE;
	data_arr[1] = up;

	lcd->pcf8574_obj.write_reg(lcd, 0, data_arr, 2);
}

/**
 * @brief  LCD initialization (I2C mode, via PCF8574).
 * @param	lcd*		pointer to LCD_Object
 * @param	addr_pins	A0, A1, A2 address pins (PCF8574). 0b000 to 0b111
 * @param	write_reg	The function that writes command to the module. Hardware dependent.
 * @param	read_reg	The function that reads data from the module. Hardware dependent.
 * @param	delay_ms	The function that makes delay in milliseconds. Hardware dependent.
 */
void LCD_init_i2c(LCD_Object *lcd, uint8_t addr_pins, device_write_ptr write_reg, device_read_ptr read_reg, device_delay_ms_ptr delay_ms) {
	PCF8574_init(&(lcd->pcf8574_obj), addr_pins, write_reg, read_reg);
	lcd->delay_ms = delay_ms;

	lcd->delay_ms(16);
	// initialize 4-bit communication
	LCD_send_internal_cmd8_nibble(lcd, 0b00110000);
	lcd->delay_ms(5);
	LCD_send_internal_cmd8_nibble(lcd, 0b00110000);
	lcd->delay_ms(1);
	LCD_send_internal_cmd8_nibble(lcd, 0b00110000);
	LCD_send_internal_cmd8_nibble(lcd, 0b00100000);

	// 4-bit mode, 2 lines, 5x7 format
	LCD_send_command(lcd, LCD_CMD_FUNCTION_SET | LCD_FUNCTION_4BIT_INTERFACE | LCD_FUNCTION_2ROWS | LCD_FUNCTION_5X7_FONT);
//	LCD_send_command(lcd, LCD_CMD_FUNCTION_SET | LCD_FUNCTION_8BIT_INTERFACE | LCD_FUNCTION_2ROWS | LCD_FUNCTION_5X7_FONT);
	// clear display
	LCD_send_command(lcd, LCD_CMD_CLEAR_DISPLAY);
	lcd->delay_ms(2);
	// return cursor home
	LCD_send_command(lcd, LCD_CMD_CURSOR_RETURN);
	lcd->delay_ms(2);
	// display on, cursor off, blink off
	LCD_send_command(lcd, LCD_CMD_DISPLAY_ON_OFF_CONTROL | LCD_CONTROL_DISPLAY_ON | LCD_CONTROL_CURSOR_OFF | LCD_CONTROL_CURSOR_BLINK_OFF);

}

/**
 * @brief  Read busy flag and address counter.
 * @param	lcd*			pointer to LCD_Object
 * @retval	LCD_BusyFlagAndAddressCounter
 */
LCD_BusyFlagAndAddressCounter LCD_get_busy_flag_and_address_counter(LCD_Object *lcd) {

	data_arr[0] = LCD_I2C_RW | LCD_I2C_BACKLIGHT_EN;
	data_arr[1] = LCD_I2C_RW | LCD_I2C_BACKLIGHT_EN | LCD_I2C_ENABLE;
//	    data_arr[1] = LCD_I2C_RW|(is_data?LCD_I2C_RS:0)|LCD_I2C_BACKLIGHT_EN;
//	    data_arr[2] = LCD_I2C_RW|(is_data?LCD_I2C_RS:0)|LCD_I2C_BACKLIGHT_EN|LCD_I2C_ENABLE;
//	    data_arr[3] = LCD_I2C_RW|(is_data?LCD_I2C_RS:0)|LCD_I2C_BACKLIGHT_EN;

	lcd->pcf8574_obj.write_reg(lcd, 0, data_arr, 2);

	lcd->pcf8574_obj.read_reg(lcd, 0, &data_arr[2], 1);

	data_arr[0] = LCD_I2C_RW | LCD_I2C_BACKLIGHT_EN;
	lcd->pcf8574_obj.write_reg(lcd, 0, data_arr, 1);

	// read second nibble
	data_arr[0] = LCD_I2C_RW | LCD_I2C_BACKLIGHT_EN | LCD_I2C_ENABLE;
	lcd->pcf8574_obj.write_reg(lcd, 0, data_arr, 1);

	lcd->pcf8574_obj.read_reg(lcd, 0, &data_arr[3], 1);
	data_arr[0] = LCD_I2C_RW | LCD_I2C_BACKLIGHT_EN;
	lcd->pcf8574_obj.write_reg(lcd, 0, data_arr, 1);

	data_arr[0] = LCD_I2C_BACKLIGHT_EN;
	lcd->pcf8574_obj.write_reg(lcd, 0, data_arr, 1);
	data_arr[0]=(data_arr[2] & 0xf0) | ((data_arr[3] >> 4) & 0x0f);

	return *((LCD_BusyFlagAndAddressCounter*)&data_arr[0]);
}

/**
 * @brief  Send command or data to LCD module, through I2C.
 * @param	lcd*		pointer to LCD_Object
 * @param	data		Data/command to send.
 * @param	is_data		Boolean. true: is data, false: is command
 */
void LCD_send_internal(LCD_Object *lcd, uint8_t data, bool is_data) {
	uint8_t up = data & 0xF0;
	uint8_t lo = (data << 4) & 0xF0;

	data_arr[0] = up | (is_data ? LCD_I2C_RS : 0) | LCD_I2C_BACKLIGHT_EN | LCD_I2C_ENABLE;
	data_arr[1] = up | (is_data ? LCD_I2C_RS : 0) | LCD_I2C_BACKLIGHT_EN;
	data_arr[2] = lo | (is_data ? LCD_I2C_RS : 0) | LCD_I2C_BACKLIGHT_EN | LCD_I2C_ENABLE;
	data_arr[3] = lo | (is_data ? LCD_I2C_RS : 0) | LCD_I2C_BACKLIGHT_EN;

	lcd->pcf8574_obj.write_reg(lcd, 0, data_arr, 4);

//    HAL_Delay(5);
}

/**
 * @brief  Send command to LCD module.
 * @param	lcd*	pointer to LCD_Object
 * @param	cmd		Command to send.
 */
void LCD_send_command(LCD_Object *lcd, uint8_t cmd) {
	LCD_send_internal(lcd, cmd, false);
}

/**
 * @brief  Send data to LCD module.
 * @param	lcd*	pointer to LCD_Object
 * @param	data	Data to send.
 */
void LCD_send_data(LCD_Object *lcd, uint8_t data) {
	LCD_send_internal(lcd, data, true);
}

/**
 * @brief  Send string to LCD module.
 * @param	lcd*	pointer to LCD_Object
 * @param	str*	array of chars to send (string).
 */
void LCD_send_string(LCD_Object *lcd, char *str) {
	while (*str) {
		LCD_send_data(lcd, (uint8_t) (*str));
		str++;
	}
}

/**
 * @brief  Set position in LCD display, where to print character on display.
 * @param	lcd*	pointer to LCD_Object
 * @param	row		Row (values 0 to 1)
 * @param	col		Column (values 0 to 39)
 */
void LCD_set_position(LCD_Object *lcd, uint8_t row, uint8_t col) {
	LCD_send_command(lcd, LCD_CMD_SET_DDRAM_ADDRESS + (row * 0x40) + col);
}

/**
 * @brief  Set cursor.
 * @param	lcd*			pointer to LCD_Object
 * @param	show_cursor		true = cursor is displayed
 * @param	blink			true = cursor/character is blinking
 */
void LCD_set_cursor(LCD_Object *lcd, bool show_cursor, bool blink) {
	LCD_send_command(lcd, LCD_CMD_DISPLAY_ON_OFF_CONTROL | LCD_CONTROL_DISPLAY_ON | (show_cursor ? LCD_CONTROL_CURSOR_ON : LCD_CONTROL_CURSOR_OFF) | (blink ? LCD_CONTROL_CURSOR_BLINK_ON : LCD_CONTROL_CURSOR_BLINK_OFF));
}

/**
 * @brief  Clear LCD.
 * @param	lcd*	pointer to LCD_Object
 */
void LCD_clear(LCD_Object *lcd) {
	LCD_send_command(lcd, LCD_CMD_CLEAR_DISPLAY);
}

/**
 * @brief  Shift display contents or cursor.
 * @param	lcd*	pointer to LCD_Object
 * @param	sc		shift display data or cursor
 * @param	el		shift to left or right
 */
void LCD_shift(LCD_Object *lcd, LCD_ShiftObject sc, LCD_ShiftDirection rl) {
	LCD_send_command(lcd, LCD_CMD_CURSOR_DISPLAY_SHIFT | (sc == LCD_SHIFT__DISPLAY ? LCD_SHIFT_DISPLAY : LCD_SHIFT_CURSOR) | (rl == LCD_SHIFT_LEFT_TO_RIGHT ? LCD_SHIFT_RIGHT : LCD_SHIFT_LEFT));
}

/**
 * @brief  Define custom character. A command must follow this, not data write.
 * @param	lcd*	pointer to LCD_Object
 * @param	index	index of character position (values from 0 to 7)
 * @param	data*	array of 8 bytes. lowest 5 bits of each byte matter
 */
void LCD_custom_character(LCD_Object *lcd, uint8_t index, uint8_t *data) {
	if (index > 7) return;

	LCD_send_command(lcd, LCD_CMD_SET_CGRAM_ADDRESS + index * 8);
	LCD_send_data(lcd, data[0]);
	LCD_send_data(lcd, data[1]);
	LCD_send_data(lcd, data[2]);
	LCD_send_data(lcd, data[3]);
	LCD_send_data(lcd, data[4]);
	LCD_send_data(lcd, data[5]);
	LCD_send_data(lcd, data[6]);
	LCD_send_data(lcd, data[7]);
}
