/**
  ******************************************************************************
  * @file    debouncer.h
  * @author  Juraj Lonc (juraj.lonc@gmail.com)
  * @brief   Header file of Debouncing for buttons.
  ******************************************************************************
  */

#ifndef INC_DEBOUNCER_H_
#define INC_DEBOUNCER_H_

//#include "main.h"
#include <stdbool.h>
#include <stdint.h>


struct DebounceButton2;

/**
 * @brief  Returns status of button. Hardware dependent function.
 * @retval true = button is pressed, false = button is not pressed
 */
typedef bool(*debouncer_is_button_down) (void);

/**
 * @brief  Callback function for various events.
 * @field	DebounceButton		pointer to a button object that triggered event
 */
typedef void(*debouncer_button_callback) (struct DebounceButton2*);

typedef enum {
	DEBOUNCER_BUTTON_MODE_SINGLEPRESS,		// 1 button press, holding button is ignored
	DEBOUNCER_BUTTON_MODE_MULTIPRESS		// increment button presses by holding button
} DebouncerButtonMode;

/**
  * @brief  Debounce button handle Structure definition
  * @field	is_button_down		pointer to function that returns status of hardware button
  * @field	button_counter		internal counter, incremented every 1ms when button down
  * @field	button_status		button is currently pressed
  * @field	button_pressed_count	number of button presses (hold button -> adds more presses) - must be pulled/decremented in user code
  */
typedef struct DebounceButton2 {
	debouncer_is_button_down is_button_down;

	uint16_t button_counter;
	bool button_status;
	uint16_t button_pressed_count;
	DebouncerButtonMode mode;

	debouncer_button_callback button_pressed_cb;
	debouncer_button_callback button_released_cb;
} DebounceButton;

void deboucer_init(DebounceButton *btn, debouncer_is_button_down is_button_down, debouncer_button_callback button_pressed_cb, debouncer_button_callback button_released_cb, DebouncerButtonMode mode);
void deboucer_process(DebounceButton * btn);
void deboucer_set_button_mode(DebounceButton * btn, DebouncerButtonMode mode);
uint16_t deboucer_pull_pressed_count(DebounceButton *btn);


#endif /* INC_DEBOUNCER_H_ */
