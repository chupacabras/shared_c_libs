/**
 ******************************************************************************
 * @file    ssd1306.c
 * @author  Juraj Lonc (juraj.lonc@gmail.com)
 * @brief   SSD1306 OLED driver.
 ******************************************************************************
 */

#include "ssd1306.h"

//const uint8_t FONT1[] = SSD1306_FONT1;
static uint8_t cmd[7];

/**
 * @brief  Send a single-byte command to SSD1306 OLED display.
 * @param  handle:       Pointer to a SSD1306_Handle structure that contains
 *                    the information for the display.
 * @param  c: command byte.
 */
static void SSD1306_command(SSD1306_Handle * handle, uint8_t c) {
	// 0b00000000 = control byte; Co (continuation bit) = 0, D/C# = 0

	handle->write_cmd(handle, 0b00000000, &c, 1);
}

/**
 * @brief  Send a multi-byte command to SSD1306 OLED display.
 * @param  handle:       Pointer to a SSD1306_Handle structure that contains
 *                    the information for the display.
 * @param  c: command/data bytes array to send. First byte of array is command byte
 * @param  count: count of bytes.
 */
static void SSD1306_command_list(SSD1306_Handle * handle, uint8_t * c, uint8_t count) {
	// 0b00000000 = control byte; Co (continuation bit) = 0, D/C# = 0
	handle->write_cmd(handle, 0b00000000, c, count);
}

/**
 * @brief  Check whether pixel is within screen or outside.
 * @param  handle:       Pointer to a SSD1306_Handle structure that contains
 *                    the information for the display.
 * @param  x: horizontal position, start with 0.
 * @param  y: vertical position, start with 0.
 * @retval true if pixel is within display.
 */
//static bool SSD1306_pixel_within_screen(SSD1306_Handle * handle, uint16_t x, uint16_t y) {
//	return ((x < handle->trans_lcd_width) && (y < handle->trans_lcd_height));
//}

static bool SSD1306_pixel_within_buffer(SSD1306_Handle * handle, int16_t x, int16_t y) {
	return ((x>=0) && (y>=0) && (x < handle->trans_buf_width) && (y < handle->trans_buf_height));
}

/**
 * @brief  Initialize SSD1306 OLED display and SSD1306 object/handler.
 * @param  handle:       Pointer to a SSD1306_Handle structure that contains
 *                    the information for the display.
 * @param  buffer: 	Pointer to the buffer of size (width*height/8).
 * 					First byte is always control byte and the rest is pixel map.
 * @param  font: 	Pointer to the array data of the font.
 * @param  buf_pix_width: Width of the buffer.
 * @param  buf_pix_height: Height of the buffer.
 * @param  chargepump: Value of type SSD1306_ChargePump. Use external VCC or charge pump.
 * @param  module: Value of type SSD1306_Module. Type of module (defined by resolution of display)
 * @param  i2c_address: I2C address of module (8-bit value). Depends on SA0 switch.
 * @param  write_cmd: 	The function that writes data/command to the module. Hardware dependent.
 * 						Usually I2C or SPI communication.
 */
void SSD1306_init(SSD1306_Handle * handle, uint8_t * buffer, uint8_t *font, uint8_t buf_pix_width, uint8_t buf_pix_height, SSD1306_ChargePump chargepump, SSD1306_Module module, uint8_t i2c_address, device_write_ptr write_cmd) {
	handle->charge_pump = chargepump;
	handle->address = i2c_address;
	handle->buffer = buffer;
	handle->buf_pix_width = buf_pix_width;
	handle->buf_pix_height = buf_pix_height;
	handle->module = module;
	handle->write_cmd = write_cmd;
	handle->font=font;

	if (handle->module == SSD1306_128_32) {
		handle->lcd_width = 128;
		handle->lcd_height = 32;
	} else if (handle->module == SSD1306_128_64) {
		handle->lcd_width = 128;
		handle->lcd_height = 64;
	} else if (handle->module == SSD1306_96_16) {
		handle->lcd_width = 96;
		handle->lcd_height = 16;
	}

	SSD1306_set_rotation(handle, SSD1306_ROTATION_0);

	handle->buf_bytes = (buf_pix_width * (uint16_t) buf_pix_height / 8);

#ifdef SSD1306_INIT_RESET
	SSD1306_RESET_ON();
	SSD1306_INIT_RESET;
	__delay_ms(10);
	SSD1306_RESET_OFF();
#endif



	SSD1306_command(handle, SSD1306_CMD_DISPLAY_OFF); // 0xAE


	cmd[0] = SSD1306_CMD_DISPLAY_CLOCK_DIV_FREQ; // 0xD5
	cmd[1] = 0x80; // default value after reset = 0x80
	SSD1306_command_list(handle, cmd, 2);

	cmd[0] = SSD1306_CMD_MULTIPLEX_RATIO; // 0xA8
	cmd[1] = handle->lcd_height - 1; // = rows-1
	SSD1306_command_list(handle, cmd, 2);

	cmd[0] = SSD1306_CMD_DISPLAY_OFFSET; // 0xD3
	cmd[1] = 0; // no offset, default value
	SSD1306_command_list(handle, cmd, 2);

	SSD1306_command(handle, SSD1306_CMD_DISPLAY_START_LINE | 0x00); // line 0, default value

	cmd[0] = SSD1306_CMD_CHARGEPUMP; // 0x8D
	cmd[1] = (handle->charge_pump == SSD1306_EXTERNALVCC) ? SSD1306_CHARGEPUMP_DISABLE : SSD1306_CHARGEPUMP_ENABLE;
	SSD1306_command_list(handle, cmd, 2);

	cmd[0] = SSD1306_CMD_MEMORY_ADDRESSING_MODE; // 0x20
	cmd[1] = SSD1306_MEMORY_MODE_HORIZ_ADDR;
	SSD1306_command_list(handle, cmd, 2);

	SSD1306_command(handle, SSD1306_CMD_SEGMENT_REMAP_ADDR127); // default value = SSD1306_CMD_SEGMENT_REMAP_ADDR0
	SSD1306_command(handle, SSD1306_CMD_COM_SCAN_DIR_REMAPPED); // default value = SSD1306_CMD_COM_SCAN_DIR_NORMAL

	if (handle->module == SSD1306_128_32) {
		cmd[0] = SSD1306_CMD_COM_PINS; // 0xDA
		cmd[1] = 0b00000010;
		SSD1306_command_list(handle, cmd, 2);

		cmd[0] = SSD1306_CMD_CONTRAST, // 0x81
			cmd[1] = 0x8F; // default = 0x7F
		SSD1306_command_list(handle, cmd, 2);

	} else if (handle->module == SSD1306_128_64) {
		cmd[0] = SSD1306_CMD_COM_PINS; // 0xDA
		cmd[1] = 0b00010010;
		SSD1306_command_list(handle, cmd, 2);

		cmd[0] = SSD1306_CMD_CONTRAST; // 0x81
		cmd[1] = (handle->charge_pump == SSD1306_EXTERNALVCC) ? 0x9F : 0xCF; // default = 0x7F
		SSD1306_command_list(handle, cmd, 2);

	} else if (handle->module == SSD1306_96_16) {
		cmd[0] = SSD1306_CMD_COM_PINS; // 0xDA
		cmd[1] = 0b00000010;
		SSD1306_command_list(handle, cmd, 2);

		cmd[0] = SSD1306_CMD_CONTRAST; // 0x81
		cmd[1] = (handle->charge_pump == SSD1306_EXTERNALVCC) ? 0x10 : 0xAF; // default = 0x7F
		SSD1306_command_list(handle, cmd, 2);

	}


	cmd[0] = SSD1306_CMD_PRECHARGE_PERIOD; // 0xD9
	cmd[1] = (handle->charge_pump == SSD1306_EXTERNALVCC) ? 0x22 : 0xF1; // default = 0x22
	SSD1306_command_list(handle, cmd, 2);

	cmd[0] = SSD1306_CMD_VCOM_DESELECT_LEVEL; // 0xDB
	cmd[1] = 0x40; // default value = 0x20,  0x00 = 0.65 VCC, 0x20 = 0.77 VCC, 0x30 = 0.83 VCC
	SSD1306_command_list(handle, cmd, 2);

	SSD1306_command(handle, SSD1306_CMD_DISPLAY_FROM_RAM); // 0xA4
	SSD1306_command(handle, SSD1306_CMD_NORMAL_DISPLAY); // 0xA6

	SSD1306_command(handle, SSD1306_CMD_SCROLL_DEACTIVATE);

	SSD1306_command(handle, SSD1306_CMD_DISPLAY_ON); // 0xAF

}

/**
 * @brief  Writes all buffer data to SSD1306 OLED display.
 * @param  handle:       Pointer to a SSD1306_Handle structure that contains
 *                    the information for the display.
 */
void SSD1306_display_buffer(SSD1306_Handle * handle) {

	cmd[0] = SSD1306_CMD_COLUMN_ADDRESS; // 0x21
	cmd[1] = 0; // column start address (reset = 0d)
	cmd[2] = handle->lcd_width - 1; // column end address (reset = 127d)
	SSD1306_command_list(handle, cmd, 3);

	cmd[0] = SSD1306_CMD_PAGE_ADDRESS; // 0x22
	cmd[1] = 0; // page start address (reset = 0d)
	cmd[2] = (handle->lcd_height >> 3) - 1; // page end address (reset = 7d)
	SSD1306_command_list(handle, cmd, 3);


	// 0b01000000 = control byte; Co (continuation bit) = 0, D/C# = 1
	handle->write_cmd(handle, 0b01000000, handle->buffer, handle->buf_bytes);

}

/**
 * @brief  Writes buffer data at specified position to SSD1306 OLED display. When buffer is smaller than screen.
 * @param  handle:       Pointer to a SSD1306_Handle structure that contains
 *                    the information for the display.
 * @param  x: x position of top left corner
 * @param  y: y position of top left corner
 */
void SSD1306_display_buffer_at(SSD1306_Handle * handle, uint8_t x, uint8_t y) {
	SSD1306_display_buffer_partial_at(handle, x, y, handle->trans_buf_width, handle->trans_buf_height);
}

/**
 * @brief  Writes buffer data at specified position to SSD1306 OLED display. When buffer is smaller than screen.
 * @param  handle:       Pointer to a SSD1306_Handle structure that contains
 *                    the information for the display.
 * @param  x: x position of top left corner
 * @param  y: y position of top left corner
 * @param  w: width of rectangle
 * @param  h: height of rectangle
 */
void SSD1306_display_buffer_partial_at(SSD1306_Handle * handle, uint8_t x, uint8_t y, uint8_t w, uint8_t h) {
	uint8_t trans_x = 0;
	uint8_t trans_y = 0;
	uint8_t trans_w = 0;
	uint8_t trans_h = 0;
	uint8_t page_start;
	uint8_t page_end;
	uint8_t i;

//	SSD1306_display_buffer_partial(handle, x, y, handle->buf_pix_width, handle->buf_pix_height);
	
//	uint8_t w=handle->trans_buf_width;
//	uint8_t h=handle->trans_buf_height;
	
	if ((handle->trans_lcd_width - x) < w) {
		w = handle->trans_lcd_width - x;
	}
	if ((handle->trans_lcd_height - y) < h) {
		h = handle->trans_lcd_height - y;
	}

	if (handle->rotation == SSD1306_ROTATION_0) {
		trans_x = x;
		trans_y = y;
		trans_w = w;
		trans_h = h;
	} else if (handle->rotation == SSD1306_ROTATION_90_CW) {
		trans_x = handle->lcd_width - y - 1 - (h - 1);
		trans_y = x;
		trans_w = h;
		trans_h = w;
	} else if (handle->rotation == SSD1306_ROTATION_180) {
		trans_x = handle->lcd_width - x - 1 - (w - 1);
		trans_y = handle->lcd_height - y - 1 - (h - 1);
		trans_w = w;
		trans_h = h;
	} else if (handle->rotation == SSD1306_ROTATION_270_CW) {
		trans_x = y;
		trans_y = handle->lcd_height - x - 1 - (w - 1);
		trans_w = h;
		trans_h = w;
	}

	
	page_start = trans_y / 8;
	page_end = (trans_y + trans_h - 1) / 8;

	cmd[0] = SSD1306_CMD_COLUMN_ADDRESS; // 0x21
	cmd[1] = trans_x; // column start address (reset = 0d)
	cmd[2] = trans_x + trans_w - 1; // column end address (reset = 127d)
	SSD1306_command_list(handle, cmd, 3);

	cmd[0] = SSD1306_CMD_PAGE_ADDRESS; // 0x22
	cmd[1] = page_start; // page start address (reset = 0d)
	cmd[2] = page_end; // page end address (reset = 7d)
	SSD1306_command_list(handle, cmd, 3);
	
	for (i = 0; i <= (page_end-page_start); i++) {
		// 0b01000000 = control byte; Co (continuation bit) = 0, D/C# = 1
		handle->write_cmd(handle, 0b01000000, &handle->buffer[i * handle->buf_pix_width], trans_w);
	}

//	handle->write_cmd(handle, 0b01000000, &handle->buffer[0], handle->buf_bytes);

}

/**
 * @brief  Writes buffer data within a specified rectangle to SSD1306 OLED display.
 * @param  handle:       Pointer to a SSD1306_Handle structure that contains
 *                    the information for the display.
 * @param  x: x position of top left corner
 * @param  y: y position of top left corner
 * @param  w: width of rectangle
 * @param  h: height of rectangle
 */
void SSD1306_display_buffer_partial(SSD1306_Handle * handle, uint8_t x, uint8_t y, uint8_t w, uint8_t h) {

	uint8_t trans_x = 0;
	uint8_t trans_y = 0;
	uint8_t trans_w = 0;
	uint8_t trans_h = 0;

	uint8_t page_start;
	uint8_t page_end;
	
	uint8_t i;

	if ((handle->trans_lcd_width - x) < w) {
		w = handle->trans_lcd_width - x;
	}
	if ((handle->trans_lcd_height - y) < h) {
		h = handle->trans_lcd_height - y;
	}

	if (handle->rotation == SSD1306_ROTATION_0) {
		trans_x = x;
		trans_y = y;
		trans_w = w;
		trans_h = h;
	} else if (handle->rotation == SSD1306_ROTATION_90_CW) {
		trans_x = handle->lcd_width - y - 1 - (h - 1);
		trans_y = x;
		trans_w = h;
		trans_h = w;
	} else if (handle->rotation == SSD1306_ROTATION_180) {
		trans_x = handle->lcd_width - x - 1 - (w - 1);
		trans_y = handle->lcd_height - y - 1 - (h - 1);
		trans_w = w;
		trans_h = h;
	} else if (handle->rotation == SSD1306_ROTATION_270_CW) {
		trans_x = y;
		trans_y = handle->lcd_height - x - 1 - (w - 1);
		trans_w = h;
		trans_h = w;
	}
	
	if (!SSD1306_pixel_within_buffer(handle, trans_x, trans_y)) return;

	page_start = trans_y / 8;
	page_end = (trans_y + trans_h - 1) / 8;

	cmd[0] = SSD1306_CMD_COLUMN_ADDRESS; // 0x21
	cmd[1] = trans_x; // column start address (reset = 0d)
	cmd[2] = trans_x + trans_w - 1; // column end address (reset = 127d)
	SSD1306_command_list(handle, cmd, 3);

	cmd[0] = SSD1306_CMD_PAGE_ADDRESS; // 0x22
	cmd[1] = page_start; // page start address (reset = 0d)
	cmd[2] = page_end; // page end address (reset = 7d)
	SSD1306_command_list(handle, cmd, 3);

	for (i = page_start; i <= page_end; i++) {
		// 0b01000000 = control byte; Co (continuation bit) = 0, D/C# = 1
		handle->write_cmd(handle, 0b01000000, &handle->buffer[i * handle->buf_pix_width + trans_x], trans_w);
	}

	//	uint8_t i;
	//	for (i=page_start; i<=page_end; i++) {
	//
	//		cmd[0]=SSD1306_CMD_COLUMN_ADDRESS; // 0x21
	//		cmd[1]=trans_x; // column start address (reset = 0d)
	//		cmd[2]=trans_x+trans_w-1; // column end address (reset = 127d)
	//		SSD1306_command_list(handle, cmd, 3);
	//
	//		cmd[0]=SSD1306_CMD_PAGE_ADDRESS; // 0x22
	//		cmd[1]=i; // page start address (reset = 0d)
	//		cmd[2]=i; // page end address (reset = 7d)
	//		SSD1306_command_list(handle, cmd, 3);
	//
	//
	//		// 0b01000000 = control byte; Co (continuation bit) = 0, D/C# = 1
	//		handle->write_cmd(handle, 0b01000000, &handle->buffer[i*handle->lcd_width+trans_x], trans_w);
	//
	//
	//	}



}

/**
 * @brief  Cleares buffer data.
 * @param  handle:       Pointer to a SSD1306_Handle structure that contains
 *                    the information for the display.
 */
void SSD1306_clear_buffer(SSD1306_Handle * handle) {
	uint16_t q;
	for (q = 0; q < handle->buf_bytes; q++) {
		handle->buffer[q] = 0;
	}

}

/**
 * @brief  Invert displayed data.
 * @param  handle:       Pointer to a SSD1306_Handle structure that contains
 *                    the information for the display.
 * @param  invert: 	true = invert, false = do not invert.
 */
void SSD1306_invert_display(SSD1306_Handle * handle, bool invert) {
	if (invert) {
		SSD1306_command(handle, SSD1306_CMD_INVERSE_DISPLAY);
	} else {
		SSD1306_command(handle, SSD1306_CMD_NORMAL_DISPLAY);
	}
}

/**
 * @brief  Dim display (set lowest contrast).
 * @param  handle:       Pointer to a SSD1306_Handle structure that contains
 *                    the information for the display.
 * @param  dim: true = dim, false = do not dim
 */
void SSD1306_dim(SSD1306_Handle * handle, bool dim) {
	uint8_t contrast;
	if (dim)
		contrast = 0; // Dimmed display
	else {
		if (handle->charge_pump == SSD1306_EXTERNALVCC)
			contrast = 0x9F;
		else
			contrast = 0xCF;
	}

	cmd[0] = SSD1306_CMD_CONTRAST; // 0x81
	cmd[1] = contrast;
	SSD1306_command_list(handle, cmd, 2);

}

/**
 * @brief  Set rotation of display.
 * @param  handle:       Pointer to a SSD1306_Handle structure that contains
 *                    the information for the display.
 * @param  rotation: type SSD1306_Rotation
 */
void SSD1306_set_rotation(SSD1306_Handle * handle, SSD1306_Rotation rotation) {
	if (rotation != SSD1306_ROTATION_0 && rotation != SSD1306_ROTATION_90_CW && rotation != SSD1306_ROTATION_180 && rotation != SSD1306_ROTATION_270_CW) {
		rotation = SSD1306_ROTATION_0;
	}
	handle->rotation = rotation;
	if (handle->rotation == SSD1306_ROTATION_0 || handle->rotation == SSD1306_ROTATION_180) {
		handle->trans_buf_height = handle->buf_pix_height;
		handle->trans_buf_width = handle->buf_pix_width;
		
		handle->trans_lcd_height = handle->lcd_height;
		handle->trans_lcd_width = handle->lcd_width;
	} else {
		handle->trans_buf_height = handle->buf_pix_width;
		handle->trans_buf_width = handle->buf_pix_height;
		
		handle->trans_lcd_height = handle->lcd_width;
		handle->trans_lcd_width = handle->lcd_height;
	}
}

/**
 * @brief  Is rotation set to vertical. Rotation 90 or 270.
 * @param  handle:       Pointer to a SSD1306_Handle structure that contains
 *                    the information for the display.
 * @retval returns true if rotation is vertical.
 */
bool SSD1306_is_vertical(SSD1306_Handle * handle) {
	return handle->rotation == SSD1306_ROTATION_90_CW || handle->rotation == SSD1306_ROTATION_270_CW;
}

/**
 * @brief  Setup and start scroll right of displayed data.
 * @param  handle:       Pointer to a SSD1306_Handle structure that contains
 *                    the information for the display.
 * @param  start_page: start page (page can be value between 0 and 7).
 * @param  stop_page: end page (page can be value between 0 and 7).
 */
void SSD1306_scroll_right(SSD1306_Handle * handle, uint8_t start_page, uint8_t stop_page) {

	cmd[0] = SSD1306_CMD_SCROLL_HORIZ_RIGHT; // 0x26
	cmd[1] = 0;
	cmd[2] = start_page; // 000b = PAGE0, 001b = PAGE1, 010b = PAGE2, ...., 111b = PAGE7
	cmd[3] = SSD1306_SCROLL_TIME_INTERVAL_5_FRAMES;
	cmd[4] = stop_page;
	cmd[5] = 0x00;
	cmd[6] = 0xff;
	SSD1306_command_list(handle, cmd, 7);

	SSD1306_command(handle, SSD1306_CMD_SCROLL_ACTIVATE); // 0x2F
}

/**
 * @brief  Setup and start scroll left of displayed data.
 * @param  handle:       Pointer to a SSD1306_Handle structure that contains
 *                    the information for the display.
 * @param  start_page: start page (page can be value between 0 and 7).
 * @param  stop_page: end page (page can be value between 0 and 7).
 */
void SSD1306_scroll_left(SSD1306_Handle * handle, uint8_t start_page, uint8_t stop_page) {

	cmd[0] = SSD1306_CMD_SCROLL_HORIZ_LEFT; // 0x27
	cmd[1] = 0;
	cmd[2] = start_page; // 000b = PAGE0, 001b = PAGE1, 010b = PAGE2, ...., 111b = PAGE7
	cmd[3] = SSD1306_SCROLL_TIME_INTERVAL_5_FRAMES;
	cmd[4] = stop_page;
	cmd[5] = 0x00;
	cmd[6] = 0xff;
	SSD1306_command_list(handle, cmd, 7);

	SSD1306_command(handle, SSD1306_CMD_SCROLL_ACTIVATE); // 0x2F
}

/**
 * @brief  Setup and start scroll vertical right of displayed data.
 * @param  handle:       Pointer to a SSD1306_Handle structure that contains
 *                    the information for the display.
 * @param  start_page: start page (page can be value between 0 and 7).
 * @param  stop_page: end page (page can be value between 0 and 7).
 */
void SSD1306_scroll_vert_right(SSD1306_Handle * handle, uint8_t start_page, uint8_t stop_page) {

	cmd[0] = SSD1306_CMD_VERT_SCROLL_VERT_AREA; // 0xA3
	cmd[1] = 0;
	cmd[2] = handle->lcd_height; // number of rows to scroll
	SSD1306_command_list(handle, cmd, 3);

	cmd[0] = SSD1306_CMD_SCROLL_HORIZVERT_RIGHT; // 0x29
	cmd[1] = 0;
	cmd[2] = start_page;
	cmd[3] = SSD1306_SCROLL_TIME_INTERVAL_5_FRAMES;
	cmd[4] = stop_page;
	cmd[5] = 0x01; // scrolling offset, 01h = 1 row, 3Fh = 63 rows
	SSD1306_command_list(handle, cmd, 6);

	SSD1306_command(handle, SSD1306_CMD_SCROLL_ACTIVATE); // 0x2F

}

/**
 * @brief  Setup and start scroll vertical left of displayed data.
 * @param  handle:       Pointer to a SSD1306_Handle structure that contains
 *                    the information for the display.
 * @param  start_page: start page (page can be value between 0 and 7).
 * @param  stop_page: end page (page can be value between 0 and 7).
 */
void SSD1306_scroll_vert_left(SSD1306_Handle * handle, uint8_t start_page, uint8_t stop_page) {

	cmd[0] = SSD1306_CMD_VERT_SCROLL_VERT_AREA; // 0xA3
	cmd[1] = 0;
	cmd[2] = handle->lcd_height; // number of rows to scroll
	SSD1306_command_list(handle, cmd, 3);

	cmd[0] = SSD1306_CMD_SCROLL_HORIZVERT_LEFT; // 0x2A
	cmd[1] = 0;
	cmd[2] = start_page;
	cmd[3] = SSD1306_SCROLL_TIME_INTERVAL_5_FRAMES;
	cmd[4] = stop_page;
	cmd[5] = 0x01; // scrolling offset, 01h = 1 row, 3Fh = 63 rows
	SSD1306_command_list(handle, cmd, 6);

	SSD1306_command(handle, SSD1306_CMD_SCROLL_ACTIVATE); // 0x2F

}

/**
 * @brief  Stop scrolling of displayed data.
 * @param  handle:       Pointer to a SSD1306_Handle structure that contains
 *                    the information for the display.
 */
void SSD1306_scroll_stop(SSD1306_Handle * handle) {
	SSD1306_command(handle, SSD1306_CMD_SCROLL_DEACTIVATE); // 0x2E
}

/**
 * @brief  Draw pixel. Unchecked. For internal use only.
 * @param  handle:       Pointer to a SSD1306_Handle structure that contains
 *                    the information for the display.
 * @param  x: horizontal position, start with 0.
 * @param  y: vertical position, start with 0.
 * @param  white: true = write white pixel, false = write black pixel.
 */
static void SSD1306_draw_pixel_unchecked(SSD1306_Handle * handle, uint8_t x, uint8_t y, bool white) {
	uint16_t i;

	uint8_t trans_x = 0;
	uint8_t trans_y = 0;

	if (handle->rotation == SSD1306_ROTATION_0) {
		trans_x = x;
		trans_y = y;
	} else if (handle->rotation == SSD1306_ROTATION_90_CW) {
		trans_x = handle->buf_pix_width - y - 1;
		trans_y = x;
	} else if (handle->rotation == SSD1306_ROTATION_180) {
		trans_x = handle->buf_pix_width - x - 1;
		trans_y = handle->buf_pix_height - y - 1;
	} else if (handle->rotation == SSD1306_ROTATION_270_CW) {
		trans_x = y;
		trans_y = handle->buf_pix_height - x - 1;
	}

	i = trans_x + (uint16_t) (trans_y >> 3) * handle->buf_pix_width;

	if (white) {
		handle->buffer[i] |= (1 << (trans_y & 7));
	} else {
		handle->buffer[i] &= ~(1 << (trans_y & 7));
	}
}

/**
 * @brief  Draw pixel.
 * @param  handle:       Pointer to a SSD1306_Handle structure that contains
 *                    the information for the display.
 * @param  x: horizontal position, start with 0.
 * @param  y: vertical position, start with 0.
 * @param  white: true = write white pixel, false = write black pixel.
 */
void SSD1306_draw_pixel(SSD1306_Handle * handle, int16_t x, int16_t y, bool white) {
	if (!SSD1306_pixel_within_buffer(handle, x, y)) return;

	SSD1306_draw_pixel_unchecked(handle, (uint8_t)x, (uint8_t)y, white);
}

/**
 * @brief  Draw line.
 * @param  handle:       Pointer to a SSD1306_Handle structure that contains
 *                    the information for the display.
 * @param  x1: horizontal position of start point, start with 0. can be outside of screen.
 * @param  y1: vertical position of start point, start with 0. can be outside of screen.
 * @param  x2: horizontal position of end point, start with 0. can be outside of screen.
 * @param  y2: vertical position of end point, start with 0. can be outside of screen.
 * @param  white: true = write white pixel, false = write black pixel.
 */
void SSD1306_draw_line(SSD1306_Handle * handle, int16_t x1, int16_t y1, int16_t x2, int16_t y2, bool white) {
	int16_t dx = (x1 > x2) ? (x1 - x2) : (x2 - x1);
	int16_t dy = (y1 > y2) ? (y1 - y2) : (y2 - y1);
	int16_t end;
	bool rev = dy>dx;
	int16_t tmp;
	int16_t p, x, y;
	
	if (rev) {
		tmp = dx;
		dx = dy;
		dy = tmp;

		tmp = x1;
		x1 = y1;
		y1 = tmp;

		tmp = x2;
		x2 = y2;
		y2 = tmp;

	}
	p = 2 * dy - dx;

	if (x1 > x2) {
		x = x2;
		y = y2;
		end = x1;
	} else {
		x = x1;
		y = y1;
		end = x2;
	}


	while (x <= end) {
		if (x >= 0 && y >= 0) {
			if (rev) {
				if (!SSD1306_pixel_within_buffer(handle, y, x)) break;
				SSD1306_draw_pixel_unchecked(handle, (uint8_t)y, (uint8_t)x, white);
			} else {
				if (!SSD1306_pixel_within_buffer(handle, x, y)) break;
				SSD1306_draw_pixel_unchecked(handle, (uint8_t)x, (uint8_t)y, white);
			}
		}

		if (p < 0) {
			x++;
			p = p + 2 * dy;
		} else {
			x++;
			y++;
			p = p + 2 * (dy - dx);
		}

	}
}

/**
 * @brief  Draw horizontal line.
 * @param  handle:       Pointer to a SSD1306_Handle structure that contains
 *                    the information for the display.
 * @param  x: horizontal position of start point, start with 0.
 * @param  y: vertical position of start point, start with 0.
 * @param  w: length of the line.
 * @param  white: true = write white pixel, false = write black pixel.
 */
void SSD1306_draw_horizontal_line(SSD1306_Handle * handle, uint8_t x, uint8_t y, uint8_t w, bool white) {
	uint8_t q;
	if (!SSD1306_pixel_within_buffer(handle, x, y)) return;
	for (q = x; q < (((x + w) >= handle->trans_buf_width) ? handle->trans_buf_width : (x + w)); q++) {
		SSD1306_draw_pixel_unchecked(handle, q, y, white);
	}
}

/**
 * @brief  Draw vertical line.
 * @param  handle:       Pointer to a SSD1306_Handle structure that contains
 *                    the information for the display.
 * @param  x: horizontal position of start point, start with 0.
 * @param  y: vertical position of start point, start with 0.
 * @param  h: length of the line.
 * @param  white: true = write white pixel, false = write black pixel.
 */
void SSD1306_draw_vertical_line(SSD1306_Handle *handle, uint8_t x, uint8_t y, uint8_t h, bool white) {
	uint8_t q;
	if (!SSD1306_pixel_within_buffer(handle, x, y)) return;
	for (q = 0; q < h; q++) {
		if (q>=handle->trans_buf_height) break;
		if ((q+y)>=handle->trans_lcd_height) break;
		SSD1306_draw_pixel_unchecked(handle, x, y+q, white);
	}
}

/**
 * @brief  Draw rectangle shape.
 * @param  handle:       Pointer to a SSD1306_Handle structure that contains
 *                    the information for the display.
 * @param  x: horizontal position of left top point, start with 0.
 * @param  y: vertical position of start point, start with 0. can be outside of screen.
 * @param  w: width of the rectangle.
 * @param  h: height of the rectangle.
 * @param  white: true = write white pixel, false = write black pixel.
 */
void SSD1306_draw_rect(SSD1306_Handle * handle, uint8_t x, uint8_t y, uint8_t w, uint8_t h, bool white) {
	SSD1306_draw_horizontal_line(handle, x, y, w, white);
	SSD1306_draw_horizontal_line(handle, x, y + h - 1, w, white);
	SSD1306_draw_vertical_line(handle, x, y, h, white);
	SSD1306_draw_vertical_line(handle, x + w - 1, y, h, white);
}

/**
 * @brief  Fill rectangle shape.
 * @param  handle:       Pointer to a SSD1306_Handle structure that contains
 *                    the information for the display.
 * @param  x: horizontal position of left top point, start with 0.
 * @param  y: vertical position of start point, start with 0. can be outside of screen.
 * @param  w: width of the rectangle.
 * @param  h: height of the rectangle.
 * @param  white: true = write white pixel, false = write black pixel.
 */
void SSD1306_fill_rect(SSD1306_Handle * handle, uint8_t x, uint8_t y, uint8_t w, uint8_t h, bool white) {
	uint16_t i;
	for (i = x; (i < x + w) && (i<=255); i++) {
		SSD1306_draw_vertical_line(handle, (uint8_t)i, y, h, white);
	}
}

/**
 * @brief  Draw circle pixels. 2 pixels for every quadrant. 8 pixels in total. For internal use only.
 * @param  handle:       Pointer to a SSD1306_Handle structure that contains
 *                    the information for the display.
 * @param  xc: x offset from center.
 * @param  yc: y offset from center.
 * @param  x: center of circle.
 * @param  y: center of circle.
 * @param  white: true = write white pixel, false = write black pixel.
 */
static void circle_draw_pixels(SSD1306_Handle * handle, int16_t xc, int16_t yc, int16_t x, int16_t y, bool white) {
	SSD1306_draw_pixel(handle, xc + x, yc + y, white);
	SSD1306_draw_pixel(handle, xc - x, yc + y, white);
	SSD1306_draw_pixel(handle, xc + x, yc - y, white);
	SSD1306_draw_pixel(handle, xc - x, yc - y, white);
	SSD1306_draw_pixel(handle, xc + y, yc + x, white);
	SSD1306_draw_pixel(handle, xc - y, yc + x, white);
	SSD1306_draw_pixel(handle, xc + y, yc - x, white);
	SSD1306_draw_pixel(handle, xc - y, yc - x, white);
}

/**
 * @brief  Draw circle.
 * 			http://staffwww.itn.liu.se/~stegu/circle/circlealgorithm.pdf
 * @param  handle:       Pointer to a SSD1306_Handle structure that contains
 *                    the information for the display.
 * @param  x0: center of circle.
 * @param  y0: center of circle.
 * @param  r: radius.
 * @param  white: true = write white pixels, false = write black pixels.
 */
void SSD1306_draw_circle(SSD1306_Handle * handle, int16_t x0, int16_t y0, uint16_t r, bool white) {
	int16_t x = 0;
	int16_t y = (int16_t)r;
	int16_t d = 5 - 4 * (int16_t)r;
	int16_t dA = 12;
	int16_t dB = 20 - 8 * (int16_t)r;
	
	while (x <= y) {
		circle_draw_pixels(handle, x0, y0, x, y, white);
		if (d < 0) {
			d = d + dA;
			dB = dB + 8;
		} else {
			y = y - 1;
			d = d + dB;
			dB = dB + 16;
		}
		x = x + 1;
		dA = dA + 8;
	}
}

/**
 * @brief  Draw circle filling lines. 4 lines in total. For internal use only.
 * @param  handle:       Pointer to a SSD1306_Handle structure that contains
 *                    the information for the display.
 * @param  xc: x offset from center.
 * @param  yc: y offset from center.
 * @param  x: center of circle.
 * @param  y: center of circle.
 * @param  white: true = write white pixel, false = write black pixel.
 */
static void circle_fill_pixels(SSD1306_Handle * handle, int16_t xc, int16_t yc, int16_t x, int16_t y, bool white) {
	SSD1306_draw_line(handle, xc + x, yc + y, xc + x, yc - y, white);
	SSD1306_draw_line(handle, xc - x, yc + y, xc - x, yc - y, white);

	SSD1306_draw_line(handle, xc + y, yc + x, xc + y, yc - x, white);
	SSD1306_draw_line(handle, xc - y, yc + x, xc - y, yc - x, white);
}

/**
 * @brief  Fill circle.
 * @param  handle:       Pointer to a SSD1306_Handle structure that contains
 *                    the information for the display.
 * @param  x0: center of circle.
 * @param  y0: center of circle.
 * @param  r: radius.
 * @param  white: true = write white pixels, false = write black pixels.
 */
void SSD1306_fill_circle(SSD1306_Handle * handle, int16_t x0, int16_t y0, uint16_t r, bool white) {
	int16_t x = 0;
	int16_t y = (int16_t)r;
	int16_t d = 5 - 4 * (int16_t)r;
	int16_t dA = 12;
	int16_t dB = 20 - 8 * (int16_t)r;
	
	while (x <= y) {
		circle_fill_pixels(handle, x0, y0, x, y, white);
		if (d < 0) {
			d = d + dA;
			dB = dB + 8;
		} else {
			y = y - 1;
			d = d + dB;
			dB = dB + 16;
		}
		x = x + 1;
		dA = dA + 8;
	}
}

/**
 * @brief  Print character. Use internal font, size 5x8 pixels.
 * @param  handle:       Pointer to a SSD1306_Handle structure that contains
 *                    the information for the display.
 * @param  x: horizontal position, start with 0.
 * @param  y: vertical position, start with 0.
 * @param  ch: character.
 * @param  text_size: multiplication factor. 1 = char size 5x8, 2 = 10x16, 3 = 15x24, etc.
 * @param  white: true = write white pixels, false = write black pixels.
 */
void SSD1306_print_char(SSD1306_Handle * handle, uint8_t x, uint8_t y, uint8_t ch, uint8_t text_size, bool white) {
	uint8_t q, w;
	bool colr;
	uint8_t b;
	
	for (q = 0; q < 5; q++) {
		b = handle->font[(ch - ' ') * 5 + q];
		for (w = 0; w < 8; w++) {
			colr = false;
			if (b & 0b00000001) {
				if (white) {
					colr = true;
				}
			} else {
				if (!white) {
					colr = true;
				}
			}
			if (text_size == 1)
				SSD1306_draw_pixel(handle, x + q, y + w, colr);
			else
				SSD1306_fill_rect(handle, x + (q * text_size), y + (w * text_size), text_size, text_size, colr);

			b >>= 1;
		}
	}
}

/**
 * @brief  Print string. Use internal font, size 5x8 pixels, with 1px spacing, so 1 char has effectively size 6x8 pixels.
 * @param  handle:       Pointer to a SSD1306_Handle structure that contains
 *                    the information for the display.
 * @param  x: horizontal position, start with 0.
 * @param  y: vertical position, start with 0.
 * @param  text: pointer to string (array of characters).
 * @param  text_size: multiplication factor. 1 = char size 5x8, 2 = 10x16, 3 = 15x24, etc.
 * @param  white: true = write white pixels, false = write black pixels.
 */
void SSD1306_print_string(SSD1306_Handle * handle, uint8_t x, uint8_t y, uint8_t * text, uint8_t text_size, bool white) {
	uint8_t xx = x;

	while (*text != '\0') {
		SSD1306_print_char(handle, xx, y, *text++, text_size, white);
		xx += 6 * text_size;
	}
}

/**
 * @brief  Draw image from pixel bitmap.
 * @param  handle:       Pointer to a SSD1306_Handle structure that contains
 *                    the information for the display.
 * @param  x: horizontal position of top left corner, start with 0.
 * @param  y: vertical position of top left corner, start with 0.
 * @param  bitmap: pointer to bitmap. big-endian
 * @param  w: width of image.
 * @param  h: height of image.
 */
void SSD1306_draw_image(SSD1306_Handle * handle, uint8_t x, uint8_t y, const uint8_t * bitmap, uint8_t w, uint8_t h) {
	uint8_t i, j, k;
	uint8_t b;
	uint8_t m;
	
	for (i = 0; i < h; i++) {
		for (j = 0; j < w / 8; j++) {
			b = bitmap[i * (w / 8) + j];
			m = 0b10000000;
			for (k = 0; k < 8; k++) {
				SSD1306_draw_pixel(handle, x + j * 8 + 7 - k, y + i, (b & m) != 0);
				m >>= 1;
			}
		}
	}
}
