#include "bl0942.h"

BL0942_Handle bl0942;

int8_t bl0942_read_data(uint8_t *data, uint8_t count) {
	return HAL_SPI_Receive(&hspi1, &data, count, 10);
}

int8_t bl0942_write_data(uint8_t *data, uint8_t count) {
	return HAL_SPI_Transmit(&hspi1, &data, count, 10);
}

int8_t bl0942_chip_select(bool cs) {
	return HAL_GPIO_WritePin(BL0942_GPIO_Port, BL0942_Pin, cs?GPIO_PIN_RESET:GPIO_PIN_SET);
}

int main(void) {
	// standard HAL initializations here
	...

	BL0942_init_SPI(&bl0942, bl0942_write_data, bl0942_read_data, bl0942_chip_select);

	BL0942_Status status;
	uint8_t ret=BL0942_read_status(&bl0942, &status);

	if (ret!=0) {
	  	UART_Printf("BL0942 error.\r\n");
	}

	...
}
