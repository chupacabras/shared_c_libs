#include "debouncer.h"

DebounceButton button1;
DebounceButton button2;

bool button1_is_pressed(void) {
	return HAL_GPIO_ReadPin(BUTTON1_GPIO_Port, BUTTON1_Pin) == GPIO_PIN_RESET;
}
bool button2_is_pressed(void) {
	return HAL_GPIO_ReadPin(BUTTON2_GPIO_Port, BUTTON2_Pin) == GPIO_PIN_RESET;
}

void button_is_down_callback(DebounceButton *btn) {

}
void button_is_released_callback(DebounceButton *btn) {

}

int main(void) {
	// standard HAL initializations here
	...

	deboucer_init(&button1, button1_is_pressed, button_is_down_callback, button_is_released_callback, DEBOUNCER_BUTTON_MODE_SINGLEPRESS);
	deboucer_init(&button2, button2_is_pressed, button_is_down_callback, button_is_released_callback, DEBOUNCER_BUTTON_MODE_MULTIPRESS);


	while (1) {
		// single press button mode
		if (button1.button_pressed_count > 0) {
			button1.button_pressed_count = 0;
			// button pressed

		}

		// multipress button mode (button held pressed)
		if (button2.button_pressed_count > 0) {
			count += deboucer_pull_pressed_count(&button2);
			// button pressed

		}
		
	}
}

// 1ms timer
void TIM4_IRQHandler(void) {
	deboucer_process(&button1);
	deboucer_process(&button2);

	// standard HAL interrupt handling
	...
}