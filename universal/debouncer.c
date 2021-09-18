/**
  ******************************************************************************
  * @file    debouncer.c
  * @author  Juraj Lonc (juraj.lonc@gmail.com)
  * @brief   Debouncing for buttons.
  ******************************************************************************
  */

#include "debouncer.h"

/**
 * @brief  Initialize button debouncer.
 * @param  btn:       pointer to a DebounceButton structure that contains
 *                    the information for the specified button.
 * @param  is_button_down: The function that returns hardware state of button
 * @param  button_pressed_cb: The callback function that is called when button is pressed. Can be NULL
 * @param  button_released_cb: The callback function that is called when button is released. Can be NULL
 * @param  mode: detection mode
 */
void deboucer_init(DebounceButton *btn, debouncer_is_button_down is_button_down, debouncer_button_callback button_pressed_cb, debouncer_button_callback button_released_cb, DebouncerButtonMode mode) {
	btn->is_button_down = is_button_down;
	btn->button_counter = 0;
	btn->button_status = false;
	btn->button_pressed_count = 0;
	btn->button_pressed_cb=button_pressed_cb;
	btn->button_released_cb=button_released_cb;
	btn->mode=mode;
}

/**
 * @brief  Process debouncing. Use in timer interrupt routine. Timer set for 1ms interval.
 * @param  btn:       pointer to a DebounceButton structure that contains
 *                    the information for the specified button.
 */
void deboucer_process(DebounceButton *btn) {
	if (btn->is_button_down && btn->is_button_down()) {
		if (btn->button_counter==0 && btn->button_pressed_cb) {
			btn->button_pressed_cb(btn);
		}
		btn->button_counter++;
		if (btn->button_counter == 60000) {
			btn->button_counter = 50000;
		}
		if (btn->mode == DEBOUNCER_BUTTON_MODE_SINGLEPRESS) {
			if (btn->button_counter == 6) {
				btn->button_status = true;
				btn->button_pressed_count++;
			}
		} else if (btn->mode == DEBOUNCER_BUTTON_MODE_MULTIPRESS) {
			if (btn->button_counter == 6) {
				btn->button_status = true;
				btn->button_pressed_count++;
			} else if (btn->button_counter < 700) {

			} else if (btn->button_counter < 2200) {
				if (btn->button_counter % 200 == 0) {
					btn->button_pressed_count++;
				}
			} else if (btn->button_counter < 4000) {
				if (btn->button_counter % 100 == 0) {
					btn->button_pressed_count++;
				}
			} else if (btn->button_counter < 6000) {
				if (btn->button_counter % 50 == 0) {
					btn->button_pressed_count++;
				}
			} else if (btn->button_counter < 8000) {
				if (btn->button_counter % 10 == 0) {
					btn->button_pressed_count++;
				}
			} else {
				btn->button_pressed_count++;
			}
		}

	} else {
		btn->button_status = false;
		btn->button_counter = 0;
		if (btn->button_released_cb) {
			btn->button_released_cb(btn);
		}
	}
}

/**
 * @brief  Pull count of button presses. And decrement count from btn handle.
 * @param  btn:       pointer to a DebounceButton structure that contains
 *                    the information for the specified button.
 * @retval count of button presses.
 */
uint16_t deboucer_pull_pressed_count(DebounceButton *btn) {
	uint16_t cnt = btn->button_pressed_count;
	btn->button_pressed_count = btn->button_pressed_count - cnt;

	return cnt;
}

/**
 * @brief  Set button mode.
 * @param  btn:       pointer to a DebounceButton structure that contains
 *                    the information for the specified button.
 * @param  mode:	button mode
 */
void deboucer_set_button_mode(DebounceButton * btn, DebouncerButtonMode mode) {
	btn->mode=mode;
}
