#include "dht.h"

DHT_Handle dht;

// 1us timer
TIM_HandleTypeDef htim3;

// DHT connected to PIN4 here
void dht_pin_output(void) {
//	(DHT_GPIO_Port)->CRH = (((DHT_GPIO_Port)->CRH) & 0xfffff0ff) | 0x00000700;
	(DHT_GPIO_Port)->MODER = (((DHT_GPIO_Port)->MODER) & 0xfffffcff) | 0x00000100;
}
void dht_pin_input(void) {
	__HAL_GPIO_EXTI_CLEAR_IT(DHT_Pin);
//	(DHT_GPIO_Port)->CRH = (((DHT_GPIO_Port)->CRH) & 0xfffff0ff) | 0x00000400;
	(DHT_GPIO_Port)->MODER = (((DHT_GPIO_Port)->MODER) & 0xfffffcff) | 0x00000000;
}
void dht_pin_low(void) {
	(DHT_GPIO_Port)->BSRR = (uint32_t)(((uint32_t)DHT_Pin) << 16);
}
void dht_pin_high(void) {
	(DHT_GPIO_Port)->BSRR = (uint32_t)(DHT_Pin);
}
uint8_t dht_pin_read(void) {
	return ((DHT_GPIO_Port)->IDR & DHT_Pin)>0;
}
void dht_reset_timer(void) {
	htim3.Instance->CNT=0;
}
uint16_t dht_read_timer(void) {
	return htim3.Instance->CNT;
}

int main(void) {
	// standard HAL initializations here
	...
	// external interrupt set od DHT pin, fall-edge trigger; EXTI4 used here for DHT pin
	// configure 1us timer, TIM3 used here

	DHT_init(&dht, dht_pin_output, dht_pin_input, dht_pin_low, dht_pin_high, dht_pin_read, dht_reset_timer, dht_read_timer);


	while (1) {
		HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

		// disable interrupts unrelated to DHT
		__HAL_TIM_DISABLE_IT(&htim14, TIM_IT_UPDATE);
		DHT_start_reading(&dht);
		// enable interrupts unrelated to DHT
		__HAL_TIM_ENABLE_IT(&htim14, TIM_IT_UPDATE);

		uint16_t temperature;
		uint16_t humidity;
		bool valid=DHT_get_value(&dht, &humidity, &temperature);

		if (valid) {
			uart("DHT OK.\n");


		} else {
			uart("DHT error.\n");
		}
		
	}
}

void EXTI4_15_IRQHandler(void) {
	if (__HAL_GPIO_EXTI_GET_IT(DHT_Pin) != 0x00u) {
		DHT_interrupt_handler_io(&dht);
	}

	// standard HAL interrupt handling
	...
}