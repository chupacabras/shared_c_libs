#include "lcd1602.h"

LCD_Handle lcd;

void hw_delay_ms(uint16_t ms) {
	HAL_Delay(ms);
}

int8_t lcd_i2c_read(void *handle, uint8_t reg, uint8_t *data, uint16_t count) {
	HAL_I2C_Master_Receive(&hi2c1, ((LCD_Handle*)handle)->pcf8574_handle.addr, data, count, 100);
	return 0;
}

int8_t lcd_i2c_write(void *handle, uint8_t reg, uint8_t *data, uint16_t count) {
	HAL_I2C_Master_Transmit(&hi2c1, ((LCD_Handle*)handle)->pcf8574_handle.addr, data, count, 100);
	return 0;
}

int main(void) {
	// standard HAL initializations here
	...

	LCD_init_i2c(&lcd, LCD_I2C_DEFAULT_ADDR, lcd_i2c_write, lcd_i2c_read, hw_delay_ms);

	LCD_set_position(&lcd, 0, 0);
	LCD_send_string(&lcd, "LCD 1602 Demo");


	while (1) {
		
	}
}
