#include "rfm69.h"

#define RFM69_NODE_ADDRESS	0x01
RFM69_t rfm69;
uint8_t receive_buf[255];

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

int8_t rfm69_spi_read(void *handle, uint8_t *bufp, uint16_t len) {
	HAL_SPI_Receive(&hspi1, bufp, len, HAL_MAX_DELAY);

	return 0;
}
int8_t rfm69_spi_write(void *handle, uint8_t *bufp, uint16_t len) {
	if (len>0) HAL_SPI_Transmit(&hspi1, bufp, len, HAL_MAX_DELAY);

	return 0;
}
void rfm69_nss(uint8_t state) {
	HAL_GPIO_WritePin(RFM69_CS_GPIO_Port, RFM69_CS_Pin, state);
}
void rfm69_reset(uint8_t state) {
	HAL_GPIO_WritePin(RFM69_RESET_GPIO_Port, RFM69_RESET_Pin, state);
}
uint8_t rfm69_get_dio0(void) {
	return HAL_GPIO_ReadPin(RFM69_DIO0_GPIO_Port, RFM69_DIO0_Pin);
}
void rfm69_hw_delay_ms(uint16_t ms) {
	HAL_Delay(ms);
}

int main(void) {
	// standard HAL initializations here
	...

	RFM69_init(&rfm69, RFM69HW, 868000000, RFM69_NODE_ADDRESS, rfm69_spi_write, rfm69_spi_read, rfm69_hw_delay_ms, rfm69_get_dio0, rfm69_nss, rfm69_reset, receive_buf);
	uint8_t version=RFM69_get_version(&rfm69);
	if (version==RFM69_DEFAULT_VERSION) {
	  	UART_Printf("RFM69 version OK.\r\n");
	}
	RFM69_set_power_dbm(&rfm69, 10);
	RFM69_encrypt(&rfm69, "aaaabbbbccccdddd");	// 16 bytes AES key

	RFM69_start_listening(&rfm69);

	while (1) {
		disable_interrupts();	// disable interrupt on DIO0 pin on every code that uses SPI bus that RFM69 is on, otherwise it will interfere with SPI bus usage in interrupt handling
		RFM69_loop(&rfm69);
		enable_interrupts();

		if (rfm69.status==RFM69_STATUS_RECEIVED || rfm69.status==RFM69_STATUS_RECEIVED_AND_ACK) {
			UART_Printf("RFM69 received, length=%d, sender=%02X, recipient=%02X, rssi=%d, status=%d\r\n", rfm69.payload_length, rfm69.sender_addr, rfm69.target_addr, rfm69.received_rssi, rfm69.status);
			if (rfm69.status==RFM69_STATUS_RECEIVED_AND_ACK) {
				UART_Printf("RFM69 ACK sent.\r\n");
			}
			uint8_t q;
			for (q=0; q<rfm69.payload_length; q++) {
				UART_Printf("%02X ", receive_buf[q]);
			}
			UART_Printf("\r\n");

			disable_interrupts();
			RFM69_start_listening(&rfm69);
			enable_interrupts();
		}
	}
}


void EXTI15_10_IRQHandler(void) {
	if (__HAL_GPIO_EXTI_GET_IT(RFM69_DIO0_Pin) != 0x00u) {
		RFM69_interrupt_handler_io(&rfm69);
	}

	// standard HAL interrupt handling
	...
}

// 1ms timer
void TIM4_IRQHandler(void) {
	RFM69_interrupt_handler_timer(&rfm69);

	// standard HAL interrupt handling
	...
}