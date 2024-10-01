#include "bl0942.h"

BL0942_Handle bl0942;

int8_t bl0942_read_data(uint8_t *data, uint8_t count) {
	return HAL_UART_Receive(&huart1, data, count, 10);
}

int8_t bl0942_write_data(uint8_t *data, uint8_t count) {
	return HAL_UART_Transmit(&huart1, data, count, 10);
}

int main(void) {
	// standard HAL initializations here
	...

	BL0942_init_UART(&bl0942, 0, 0, bl0942_write_data, bl0942_read_data);

	BL0942_Status status;
	uint8_t ret=BL0942_read_status(&bl0942, &status);

	if (ret!=0) {
	  	UART_Printf("BL0942 error.\r\n");
	}

	...
}
