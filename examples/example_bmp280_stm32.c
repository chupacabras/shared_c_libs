#include "bmp280.h"

BMP280_Handle bmp280;

void hw_delay_ms(uint16_t ms) {
	HAL_Delay(ms);
}

int8_t bmp280_i2c_read(void *handle, uint8_t reg, uint8_t *data, uint16_t count) {
	return HAL_I2C_Mem_Read(&hi2c1, (*(BMP280_Handle*)handle).addr, reg, 1, data, count, 1000);
}
int8_t bmp280_i2c_write(void *handle, uint8_t reg, uint8_t *data, uint16_t count) {
	return HAL_I2C_Mem_Write(&hi2c1, (*(BMP280_Handle*)handle).addr, reg, 1, data, count, 1000);
}

int main(void) {
	// standard HAL initializations here
	...

	uint8_t bmp280found = BMP280_init(&bmp280, 0, bmp280_i2c_write, bmp280_i2c_read, hw_delay_ms);

	if (bmp280found) {
		BMP280_read_raw_temperature(&bmp280);
		int32_t t=BMP280_calc_temperature(&bmp280);

		BMP280_read_raw_pressure(&bmp280);
		uint32_t p=BMP280_calc_pressure(&bmp280);

	}


	while (1) {
		
	}
}
