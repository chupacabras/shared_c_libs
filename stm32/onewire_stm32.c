/**
  ******************************************************************************
  * @file    onewire_stm32.c
  * @author  Juraj Lonc (juraj.lonc@gmail.com)
  * @brief   One-Wire driver for STM32.
  ******************************************************************************
 */

/*
#include "onewire_stm32.h"

static GPIO_TypeDef* port;
static uint32_t pin;

void ow_pin_output(void) {
	// open-drain
cnf
	// pull-up resistor




	GPIO_InitTypeDef GPIO_InitStruct = {0};
			GPIO_InitStruct.Pin = E32_RXD_Pin;
			  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
			  GPIO_InitStruct.Pull = GPIO_PULLUP;
			  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
			  HAL_GPIO_Init(E32_RXD_GPIO_Port, &GPIO_InitStruct);
}
void ow_pin_input(void) {
	GPIO_InitTypeDef GPIO_InitStruct = {0};
			GPIO_InitStruct.Pin = E32_RXD_Pin;
			  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
			  GPIO_InitStruct.Pull = GPIO_PULLUP;
			  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
			  HAL_GPIO_Init(E32_RXD_GPIO_Port, &GPIO_InitStruct);
}
void ow_pin_low(void) {
	(port)->BSRR = (uint32_t)(pin << 16);
}
void ow_pin_high(void) {
	(port)->BSRR = (uint32_t)(pin);
}
uint8_t ow_pin_read(void) {
	return !((port->IDR & pin) == GPIO_PIN_RESET);
}
*/
