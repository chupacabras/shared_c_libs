#include "sx127x.h"

SX127X_Handle sx127x;

#define SX127X_SYNC_WORD		0x69

uint8_t receive_buf[255];


int8_t sx127x_spi_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) {
	HAL_SPI_Transmit(&hspi1, &reg, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(&hspi1, bufp, len, HAL_MAX_DELAY);

	return 0;
}

int8_t sx127x_spi_write(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) {
	HAL_SPI_Transmit(&hspi1, &reg, 1, HAL_MAX_DELAY);
	HAL_SPI_Transmit(&hspi1, bufp, len, HAL_MAX_DELAY);

	return 0;
}

uint8_t sx127x_get_dio0(void) {
	return HAL_GPIO_ReadPin(LORA_DIO0_GPIO_Port, LORA_DIO0_Pin);
}

void sx127x_set_nss(uint8_t d) {
	HAL_GPIO_WritePin(LORA_CS_GPIO_Port, LORA_CS_Pin, d);
}

void sx127x_set_nreset(uint8_t d) {
	HAL_GPIO_WritePin(LORA_RESET_GPIO_Port, LORA_RESET_Pin, d);
}

void hw_delay_ms(uint16_t ms) {
	HAL_Delay(ms);
}

int main(void) {
	// standard HAL initializations here
	...

	SX127X_init(&sx127x, sx127x_spi_write, sx127x_spi_read, hw_delay_ms, sx127x_set_nss, sx127x_set_nreset);
	SX127X_config_LoRa(&sx127x, 868000000, SX127X_LORA_SF_7, SX127X_LORA_CRC_ENABLED, SX127X_LORA_CR_4_5, SX127X_LORA_BW_125KHZ, SX127X_SYNC_WORD);

	uint8_t version=SX127X_getVersion(&sx127x);
	if (version==SX127X_DEFAULT_VERSION_VALUE) {
	  	UART_Printf("SX127X version OK.\r\n");
	}

	// default packet length set to 32, only used for implicit header (ie. spreading factor 6)
	SX127X_config_LoRa_Rx(&sx127x, 32);

	uint8_t received_len;
	uint8_t receive_buf[256];

	while (1) {
		// alternative A
		SX127X_Register_LoRa_RegIrqFlags intreq = SX127X_getLoRaIrq(&sx127x);
		if (intreq.RxDone) {
			UART_Printf("Package received...\n");
			received_len=SX127X_readReceivedPacket(&sx1276, receive_buf);
			// assume received packet is text
			receive_buf[receive_len]=0;
			UART_Printf("Package: [%s], length=%d\n", receive_buf, receive_len);
			SX127X_clearLoRaIrq(&sx127x);
		}

		// alternative B
		// RxDone is mapped to DIO0
		if (sx127x_get_dio0()) {
			UART_Printf("Package received...\n");
			received_len=SX127X_readReceivedPacket(&sx1276, receive_buf);
			// assume received packet is text
			receive_buf[receive_len]=0;
			UART_Printf("Package: [%s], length=%d\n", receive_buf, receive_len);
			SX127X_clearLoRaIrq(&sx127x);
		}

	}
}
