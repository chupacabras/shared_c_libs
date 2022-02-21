#include "nrf24l01.h"

NRF24_t nrf24;
uint8_t receive_buf[33];

void nrf24_setup_pins() {
	// setup already done in HAL
}

void nrf24_set_ce(uint8_t state) {
	HAL_GPIO_WritePin(NRF24_CE_GPIO_Port, NRF24_CE_Pin, state);
}

void nrf24_set_csn(uint8_t state) {
	HAL_GPIO_WritePin(NRF24_CS_GPIO_Port, NRF24_CS_Pin, state);
}

uint8_t nrf24_spi_transfer(uint8_t data) {
	uint8_t recv;
	HAL_SPI_TransmitReceive(&hspi1, &data, &recv, 1, HAL_MAX_DELAY);
	return recv;
}

void delay_us(uint16_t us) {
	__HAL_TIM_SET_COUNTER(&htim3, 0);  // set the counter value a 0; htim timer period set to 1us
	while (__HAL_TIM_GET_COUNTER(&htim3) < us);  // wait for the counter to reach the us input in the parameter
}

int main(void) {
	// standard HAL initializations here
	...


	// wait 100ms to power up nrf24 radio
	__delay_ms(100);

  	nrf24_init(&nrf24, nrf24_setup_pins, nrf24_spi_transfer, nrf24_set_csn, nrf24_set_ce, delay_us);
	nrf24_set_rf_channel(&nrf24, 10);

	uint8_t tx_addr[]={0x11,0x22,0x33,0x44,0x55};
	nrf24_config_tx(&nrf24, tx_addr);

	// set TX mode
	nrf24_set_operation_mode(&nrf24, NRF24_PTX);

	// power up radio
	nrf24_set_power_mode(&nrf24, NRF24_PWR_UP);

	// it takes 1.5ms to power up
	__delay_ms(2);


	while (1) {
		if (sendPacket) {
			nrf24_clear_irq_flags(&nrf24);
			nrf24_flush_tx(&nrf24);
			nrf24_write_tx_payload(&nrf24, send_buf, len);	// Write payload to radio TX FIFO

			// Toggle radio CE signal to start transmission 
			nrf24_pulse_ce(&nrf24);
		}



		// check status
		NRF24_Register_Status irq_status=nrf24_get_status(&nrf24);
		if (irq_status.MAX_RT) {
			// maximum retry count reached, message not sent
			UART_Printf("ACK failed.\r\n");

			nrf24_clear_irq_flag(&nrf24, NRF24_MAX_RT);
		}
		if (irq_status.TX_DS) {
			UART_Printf("Data sent and ACK received.\r\n");

			nrf24_clear_irq_flag(&nrf24, NRF24_TX_DS);
		}
		if (irq_status.RX_DR) {
			UART_Printf("Data received.\r\n");

			while (!nrf24_rx_fifo_empty(&nrf24)) {
				uint8_t len=nrf24_read_rx_payload(&nrf24, receive_buf);
				receive_buf[len]=0; // terminate string
				UART_Printf("Payload=[");
				UART_Printf((char*)receive_buf);
				UART_Printf("]\n");
			}

			nrf24_clear_irq_flag(&nrf24, NRF24_RX_DR);
		}

	}
}
