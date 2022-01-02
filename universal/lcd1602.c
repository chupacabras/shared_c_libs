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
 * @param	lcd*		pointer to LCD_Handle
 * @param	data		Data to send (upper 4-bits from 8-bit data).
 */
static void LCD_send_internal_cmd8_nibble(LCD_Handle *lcd, uint8_t data) {
	uint8_t up = data & 0xF0;

	data_arr[0] = up | LCD_I2C_ENABLE;
	data_arr[1] = up;

	lcd->pcf8574_obj.write_reg(lcd, 0, data_arr, 2);
}

/**
 * @brief  Send command or data to LCD module, through I2C.
 * @param	lcd*		pointer to LCD_Handle
 * @param	data		Data/command to send.
 * @param	is_data		Boolean. true: is data, false: is command
 */
static void LCD_send_internal(LCD_Handle *lcd, uint8_t data, bool is_data) {
	uint8_t up = data & 0xF0;
	uint8_t lo = (data << 4) & 0xF0;

	data_arr[0] = up | (is_data ? LCD_I2C_RS : 0) | lcd->backlight_en | LCD_I2C_ENABLE;
	data_arr[1] = up | (is_data ? LCD_I2C_RS : 0) | lcd->backlight_en;
	data_arr[2] = lo | (is_data ? LCD_I2C_RS : 0) | lcd->backlight_en | LCD_I2C_ENABLE;
	data_arr[3] = lo | (is_data ? LCD_I2C_RS : 0) | lcd->backlight_en;

	lcd->pcf8574_obj.write_reg(lcd, 0, data_arr, 4);

//    HAL_Delay(5);
}

void LCD_init_i2c(LCD_Handle *lcd, uint8_t addr_pins, device_write_ptr write_reg, device_read_ptr read_reg, device_delay_ms_ptr delay_ms) {
	PCF8574_init(&(lcd->pcf8574_obj), addr_pins, write_reg, read_reg);
	lcd->delay_ms = delay_ms;
	lcd->backlight_en=LCD_I2C_BACKLIGHT_EN;

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

LCD_BusyFlagAndAddressCounter LCD_get_busy_flag_and_address_counter(LCD_Handle *lcd) {

	data_arr[0] = LCD_I2C_RW | lcd->backlight_en;
	data_arr[1] = LCD_I2C_RW | lcd->backlight_en | LCD_I2C_ENABLE;
//	    data_arr[1] = LCD_I2C_RW|(is_data?LCD_I2C_RS:0)|LCD_I2C_BACKLIGHT_EN;
//	    data_arr[2] = LCD_I2C_RW|(is_data?LCD_I2C_RS:0)|LCD_I2C_BACKLIGHT_EN|LCD_I2C_ENABLE;
//	    data_arr[3] = LCD_I2C_RW|(is_data?LCD_I2C_RS:0)|LCD_I2C_BACKLIGHT_EN;

	lcd->pcf8574_obj.write_reg(lcd, 0, data_arr, 2);

	lcd->pcf8574_obj.read_reg(lcd, 0, &data_arr[2], 1);

	data_arr[0] = LCD_I2C_RW | lcd->backlight_en;
	lcd->pcf8574_obj.write_reg(lcd, 0, data_arr, 1);

	// read second nibble
	data_arr[0] = LCD_I2C_RW | lcd->backlight_en | LCD_I2C_ENABLE;
	lcd->pcf8574_obj.write_reg(lcd, 0, data_arr, 1);

	lcd->pcf8574_obj.read_reg(lcd, 0, &data_arr[3], 1);
	data_arr[0] = LCD_I2C_RW | lcd->backlight_en;
	lcd->pcf8574_obj.write_reg(lcd, 0, data_arr, 1);

	data_arr[0] = lcd->backlight_en;
	lcd->pcf8574_obj.write_reg(lcd, 0, data_arr, 1);
	data_arr[0]=(data_arr[2] & 0xf0) | ((data_arr[3] >> 4) & 0x0f);

	return *((LCD_BusyFlagAndAddressCounter*)&data_arr[0]);
}

void LCD_send_command(LCD_Handle *lcd, uint8_t cmd) {
	LCD_send_internal(lcd, cmd, false);
}

void LCD_send_data(LCD_Handle *lcd, uint8_t data) {
	LCD_send_internal(lcd, data, true);
}

void LCD_send_string(LCD_Handle *lcd, char *str) {
	while (*str) {
		LCD_send_data(lcd, (uint8_t) (*str));
		str++;
	}
}

void LCD_set_position(LCD_Handle *lcd, uint8_t row, uint8_t col) {
	LCD_send_command(lcd, LCD_CMD_SET_DDRAM_ADDRESS + (row * 0x40) + col);
}

void LCD_set_cursor(LCD_Handle *lcd, bool show_cursor, bool blink) {
	LCD_send_command(lcd, LCD_CMD_DISPLAY_ON_OFF_CONTROL | LCD_CONTROL_DISPLAY_ON | (show_cursor ? LCD_CONTROL_CURSOR_ON : LCD_CONTROL_CURSOR_OFF) | (blink ? LCD_CONTROL_CURSOR_BLINK_ON : LCD_CONTROL_CURSOR_BLINK_OFF));
}

void LCD_clear(LCD_Handle *lcd) {
	LCD_send_command(lcd, LCD_CMD_CLEAR_DISPLAY);
}

void LCD_shift(LCD_Handle *lcd, LCD_ShiftObject sc, LCD_ShiftDirection rl) {
	LCD_send_command(lcd, LCD_CMD_CURSOR_DISPLAY_SHIFT | (sc == LCD_SHIFT__DISPLAY ? LCD_SHIFT_DISPLAY : LCD_SHIFT_CURSOR) | (rl == LCD_SHIFT_LEFT_TO_RIGHT ? LCD_SHIFT_RIGHT : LCD_SHIFT_LEFT));
}

void LCD_custom_character(LCD_Handle *lcd, uint8_t index, uint8_t *data) {
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

void LCD_backlight(LCD_Handle *lcd, bool enable) {
	lcd->backlight_en=enable?LCD_I2C_BACKLIGHT_EN:0;
	LCD_send_command(lcd, 0);
}

void LCD_display_off(LCD_Handle *lcd, bool off) {
	LCD_send_command(lcd, LCD_CMD_DISPLAY_ON_OFF_CONTROL | (off?LCD_CONTROL_DISPLAY_OFF:LCD_CONTROL_DISPLAY_ON));
}
