#include "aht10.h"

AHT10_Handle aht10;
AHT10_Measurement aht10meas;

void hw_delay_ms(uint16_t ms) {
	HAL_Delay(ms);
}

int8_t aht10_i2c_read(void *handle, uint8_t reg, uint8_t *data, uint16_t count) {
	return HAL_I2C_Master_Receive(&hi2c1, AHT10_ADDRESS, data, count, 1000);
}
int8_t aht10_i2c_write(void *handle, uint8_t reg, uint8_t *data, uint16_t count) {
	if (count==0) {
		return HAL_I2C_Master_Transmit(&hi2c1, AHT10_ADDRESS, &reg, 1, 1000);
	} else {
		return HAL_I2C_Mem_Write(&hi2c1, AHT10_ADDRESS, reg, 1, data, count, 1000);
	}
}

int main(void) {
	// standard HAL initializations here
	...

	uint8_t aht10found=AHT10_init(&aht10, 0, aht10_i2c_write, aht10_i2c_read, hw_delay_ms);

	if (aht10found) {
		AHT10_trigger_measurement(&aht10);
		bool data_ok;
		uint16_t cnt = 0;
		while (true) {
			data_ok = AHT10_read_measurement(&aht10, &aht10meas);
			if (data_ok) {
				break;
			}
			cnt++;
			if (cnt > 600) {
				break;
			}
		}
		if (data_ok) {
			int16_t t = AHT10_convert_temperature(&aht10meas);
			uint16_t h = AHT10_convert_humidity(&aht10meas);

		}
	}


	while (1) {
		
	}
}
