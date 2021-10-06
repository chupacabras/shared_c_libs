#include "nrf24l01.h"

NRF24_t nrf24;
uint8_t last_pipe_received;
bool data_received=false;
uint8_t data_length=0;
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

void nrf24_hw_delay_us(uint16_t us) {
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

	uint8_t addr0[]={0x11,0x22,0x33,0x44,0x55};
	uint8_t addr1[]={0x12,0x22,0x33,0x44,0x55};
	uint8_t addr2[]={0x13};
	uint8_t addr3[]={0x14};
	uint8_t addr4[]={0x15};
	uint8_t addr5[]={0x16};

	nrf24_config_rx_pipe(&nrf24, NRF24_PIPE0, addr0, true, 32);
	nrf24_config_rx_pipe(&nrf24, NRF24_PIPE1, addr1, true, 32);
	nrf24_config_rx_pipe(&nrf24, NRF24_PIPE2, addr2, true, 32);
	nrf24_config_rx_pipe(&nrf24, NRF24_PIPE3, addr3, true, 32);
	nrf24_config_rx_pipe(&nrf24, NRF24_PIPE4, addr4, true, 32);
	nrf24_config_rx_pipe(&nrf24, NRF24_PIPE5, addr5, true, 32);

	// set RX mode
	nrf24_set_operation_mode(&nrf24, NRF24_PRX);

	// power up radio
	nrf24_set_power_mode(&nrf24, NRF24_PWR_UP);

	// it takes 1.5ms to power up
	__delay_ms(2);

	// Enable receiver
	nrf24_set_ce(1);


	while (1) {

		if (data_received) {
			UART_Printf("Data received.\r\n");
			UART_Printf("Pipe=[%d]\n", last_pipe_received);
			UART_Printf("Payload=[");
			UART_Printf((char*)receive_buf);
			UART_Printf("]\n");

			data_received=false;
		}

	}
}

void EXTI15_10_IRQHandler(void) {
	if (__HAL_GPIO_EXTI_GET_IT(RFM69_DIO0_Pin) != 0x00u) {

		NRF24_Register_Status irq_status=nrf24_get_status(&nrf24);
		if (irq_status.RX_DR) {
			// data received
			data_length = nrf24_read_rx_payload_width(&nrf24);
			last_pipe_received = nrf24_get_rx_data_source(&nrf24);
			
			while (!nrf24_rx_fifo_empty(&nrf24)) {
				uint8_t len=nrf24_read_rx_payload(&nrf24, receive_buf);
				receive_buf[len]=0; // terminate string
				data_received=true;
			}

			// flush TX FIFO to prevent old status from being sent
			nrf24_flush_tx(&nrf24);

			// write payload for next ACK, just sending back payload in this case
			nrf24_write_ack_payload(&nrf24, last_pipe_received, receive_buf, data_length);
		}
		nrf24_clear_irq_flags(&nrf24);

	}

	// standard HAL interrupt handling
	...
}
