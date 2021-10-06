#include "nrf24l01.h"

NRF24_t nrf24;
bool ack_received=false;
bool data_sent=false;
bool sending_failed=false;
uint8_t data_length=0;
uint8_t receive_buf[33];

volatile uint8_t disabledInterruptLevels=0;

void disable_interrupts(void) {
//	__disable_irq();
	HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);

	disabledInterruptLevels++;
}
void enable_interrupts(void) {
//	disabledInterruptLevels--;
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
	if (disabledInterruptLevels==0) __enable_irq();
}

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

	disable_interrupts();
  	nrf24_init(&nrf24, nrf24_setup_pins, nrf24_spi_transfer, nrf24_set_csn, nrf24_set_ce, delay_us);
	nrf24_set_rf_channel(&nrf24, 10);

	uint8_t tx_addr[]={0x11,0x22,0x33,0x44,0x55};
	nrf24_config_tx(&nrf24, tx_addr);

	// set TX mode
	nrf24_set_operation_mode(&nrf24, NRF24_PTX);

	// power up radio
	nrf24_set_power_mode(&nrf24, NRF24_PWR_UP);
	enable_interrupts();

	// it takes 1.5ms to power up
	__delay_ms(2);


	while (1) {
		if (sendPacket) {
			disable_interrupts();	// disable interrupt on INT pin on every code that uses SPI bus that NRF24 is on, otherwise it will interfere with SPI bus usage in interrupt handling
			nrf24_clear_irq_flags(&nrf24);
			nrf24_flush_tx(&nrf24);
			nrf24_write_tx_payload(&nrf24, send_buf, len);	// Write payload to radio TX FIFO
			enable_interrupts();

			// Toggle radio CE signal to start transmission 
			nrf24_pulse_ce(&nrf24);
		}



		if (sending_failed) {
			// maximum retry count reached, message not sent
			UART_Printf("ACK failed.\r\n");
			sending_failed=false;
		}
		if (data_sent) {
			UART_Printf("Data sent and ACK received.\r\n");
			data_sent=false;
		}
		if (ack_received) {
			UART_Printf("Data received.\r\n");
			ack_received=false;
			UART_Printf("Payload=[");
			UART_Printf((char*)receive_buf);
			UART_Printf("]\n");
		}

	}
}


void EXTI15_10_IRQHandler(void) {
	if (__HAL_GPIO_EXTI_GET_IT(RFM69_DIO0_Pin) != 0x00u) {

		NRF24_Register_Status irq_status=nrf24_get_status(&nrf24);
		if (irq_status.MAX_RT) {
			// maximum retry count reached, message not sent
			sending_failed=true;
			data_sent=false;
		}
		if (irq_status.TX_DS) {
			// data sent and ACk received
			sending_failed=false;
			data_sent=true;
		}
		if (irq_status.RX_DR) {
			// data received
			while (!nrf24_rx_fifo_empty(&nrf24)) {
				uint8_t len=nrf24_read_rx_payload(&nrf24, receive_buf);
				receive_buf[len]=0; // terminate string
				ack_received=true;
			}
		}
		nrf24_clear_irq_flags(&nrf24);

	}

	// standard HAL interrupt handling
	...
}
