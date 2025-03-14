/**
 ******************************************************************************
 * @file    ssd1306.h
 * @author  Juraj Lonc (juraj.lonc@gmail.com)
 * @brief   Header file of SSD1306 driver.
 ******************************************************************************
 */

#ifndef SSD1306_H
#define	SSD1306_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef __STM8S_H
#include <stdint.h>
#include <stdbool.h>
#endif /* __STM8S_H */

#include "device.h"

#define SSD1306_ADDRESS			0b01111000	// when SA0 is low
#define SSD1306_ADDRESS_ALT		0b01111010	// when SA0 is high

#define SSD1306_BUFFER_SIZE_128x64		1024		// 128*64/8; 128 columns, 64 rows
#define SSD1306_BUFFER_SIZE_128x32		512			// 128*32/8; 128 columns, 32 rows

// Fundamental Command Table
#define SSD1306_CMD_CONTRAST				0x81
#define SSD1306_CMD_DISPLAY_FROM_RAM		0xA4
#define SSD1306_CMD_ENTIRE_DISPLAY_ON		0xA5
#define SSD1306_CMD_NORMAL_DISPLAY			0xA6
#define SSD1306_CMD_INVERSE_DISPLAY			0xA7
#define SSD1306_CMD_DISPLAY_OFF				0xAE
#define SSD1306_CMD_DISPLAY_ON				0xAF

// Scrolling Command Table
#define SSD1306_CMD_SCROLL_HORIZ_RIGHT		0x26
#define SSD1306_CMD_SCROLL_HORIZ_LEFT		0x27
#define SSD1306_CMD_SCROLL_HORIZVERT_RIGHT	0x29
#define SSD1306_CMD_SCROLL_HORIZVERT_LEFT	0x2A
#define SSD1306_CMD_SCROLL_DEACTIVATE		0x2E
#define SSD1306_CMD_SCROLL_ACTIVATE			0x2F
#define SSD1306_CMD_VERT_SCROLL_VERT_AREA	0xA3

// Addressing Setting Command Table
#define SSD1306_CMD_SET_LOWER_COLUMN		0x00
#define SSD1306_CMD_SET_HIGHER_COLUMN		0x10
#define SSD1306_CMD_MEMORY_ADDRESSING_MODE	0x20
#define SSD1306_CMD_COLUMN_ADDRESS			0x21
#define SSD1306_CMD_PAGE_ADDRESS			0x22
#define SSD1306_CMD_PAGE_START				0xB0

// Hardware Configuration (Panel resolution & layout related) Command Table
#define SSD1306_CMD_DISPLAY_START_LINE		0x40
#define SSD1306_CMD_SEGMENT_REMAP_ADDR0		0xA0
#define SSD1306_CMD_SEGMENT_REMAP_ADDR127	0xA1
#define SSD1306_CMD_MULTIPLEX_RATIO			0xA8
#define SSD1306_CMD_COM_SCAN_DIR_NORMAL		0xC0
#define SSD1306_CMD_COM_SCAN_DIR_REMAPPED	0xC8
#define SSD1306_CMD_DISPLAY_OFFSET			0xD3
#define SSD1306_CMD_COM_PINS				0xDA

// Timing & Driving Scheme Setting Command Table
#define SSD1306_CMD_DISPLAY_CLOCK_DIV_FREQ	0xD5
#define SSD1306_CMD_PRECHARGE_PERIOD		0xD9
#define SSD1306_CMD_VCOM_DESELECT_LEVEL		0xDB
#define SSD1306_CMD_NOP						0xE3

// Charge Pump Command Table
#define SSD1306_CMD_CHARGEPUMP				0x8D

#define SSD1306_CHARGEPUMP_ENABLE			0x14
#define SSD1306_CHARGEPUMP_DISABLE			0x10

#define SSD1306_MEMORY_MODE_HORIZ_ADDR		0b00
#define SSD1306_MEMORY_MODE_VERT_ADDR		0b01
#define SSD1306_MEMORY_MODE_PAGE_ADDR		0b10

#define SSD1306_SCROLL_TIME_INTERVAL_5_FRAMES	0b000
#define SSD1306_SCROLL_TIME_INTERVAL_64_FRAMES	0b001
#define SSD1306_SCROLL_TIME_INTERVAL_128_FRAMES	0b010
#define SSD1306_SCROLL_TIME_INTERVAL_256_FRAMES	0b011
#define SSD1306_SCROLL_TIME_INTERVAL_3_FRAMES	0b100
#define SSD1306_SCROLL_TIME_INTERVAL_4_FRAMES	0b101
#define SSD1306_SCROLL_TIME_INTERVAL_25_FRAMES	0b110
#define SSD1306_SCROLL_TIME_INTERVAL_2_FRAMES	0b111

typedef enum tag_SSD1306_Module {
	SSD1306_128_64,
	SSD1306_128_32,
	SSD1306_96_16
} SSD1306_Module;

typedef enum tag_SSD1306_ChargePump {
	SSD1306_EXTERNALVCC = 0,
	SSD1306_CHARGEPUMP = 1
} SSD1306_ChargePump;

typedef enum tag_SSD1306_Rotation {
	SSD1306_ROTATION_0 = 0,
	SSD1306_ROTATION_90_CW = 9,
	SSD1306_ROTATION_180 = 18,
	SSD1306_ROTATION_270_CW = 27
} SSD1306_Rotation;

typedef struct tag_SSD1306_Handle {
	uint8_t address;
	SSD1306_ChargePump charge_pump;
	uint8_t * buffer;
	uint8_t buf_pix_width;
	uint8_t buf_pix_height;
	uint16_t buf_bytes;
	SSD1306_Module module;
	uint8_t lcd_width;
	uint8_t lcd_height;
	
	uint8_t * font;
	
	device_write_ptr write_cmd;

	SSD1306_Rotation rotation;
	uint8_t trans_buf_width; // translated value according to rotation
	uint8_t trans_buf_height; // translated value according to rotation
	
	uint8_t trans_lcd_width; // translated value according to rotation
	uint8_t trans_lcd_height; // translated value according to rotation

} SSD1306_Handle;


// basic functions
void SSD1306_init(SSD1306_Handle * handle, uint8_t * buffer, uint8_t *font, uint8_t buf_pix_width, uint8_t buf_pix_height, SSD1306_ChargePump chargepump, SSD1306_Module module, uint8_t i2c_address, device_write_ptr write_cmd);
void SSD1306_display_buffer(SSD1306_Handle * handle);
void SSD1306_display_buffer_partial(SSD1306_Handle * handle, uint8_t x, uint8_t y, uint8_t w, uint8_t h);
void SSD1306_display_buffer_at(SSD1306_Handle * handle, uint8_t x, uint8_t y);
void SSD1306_display_buffer_partial_at(SSD1306_Handle * handle, uint8_t x, uint8_t y, uint8_t w, uint8_t h);
void SSD1306_clear_buffer(SSD1306_Handle * handle);
void SSD1306_invert_display(SSD1306_Handle * handle, bool invert);
void SSD1306_dim(SSD1306_Handle * handle, bool dim);
void SSD1306_set_rotation(SSD1306_Handle * handle, SSD1306_Rotation rotation);
bool SSD1306_is_vertical(SSD1306_Handle * handle);

// scrolling
void SSD1306_scroll_right(SSD1306_Handle * handle, uint8_t start_page, uint8_t stop_page);
void SSD1306_scroll_left(SSD1306_Handle * handle, uint8_t start_page, uint8_t stop_page);
void SSD1306_scroll_vert_right(SSD1306_Handle * handle, uint8_t start_page, uint8_t stop_page);
void SSD1306_scroll_vert_left(SSD1306_Handle * handle, uint8_t start_page, uint8_t stop_page);
void SSD1306_scroll_stop(SSD1306_Handle * handle);

// draw graphics
void SSD1306_draw_pixel(SSD1306_Handle * handle, int16_t x, int16_t y, bool white);
void SSD1306_draw_line(SSD1306_Handle * handle, int16_t x1, int16_t y1, int16_t x2, int16_t y2, bool white);
void SSD1306_draw_horizontal_line(SSD1306_Handle * handle, uint8_t x, uint8_t y, uint8_t w, bool white);
void SSD1306_draw_vertical_line(SSD1306_Handle * handle, uint8_t x, uint8_t y, uint8_t h, bool white);
void SSD1306_draw_rect(SSD1306_Handle * handle, uint8_t x, uint8_t y, uint8_t w, uint8_t h, bool white);
void SSD1306_fill_rect(SSD1306_Handle * handle, uint8_t x, uint8_t y, uint8_t w, uint8_t h, bool white);
void SSD1306_draw_circle(SSD1306_Handle * handle, int16_t xc, int16_t yc, uint16_t r, bool white);
void SSD1306_fill_circle(SSD1306_Handle * handle, int16_t x0, int16_t y0, uint16_t r, bool white);

// print texts
void SSD1306_print_char(SSD1306_Handle * handle, uint8_t x, uint8_t y, uint8_t ch, uint8_t text_size, bool white);
void SSD1306_print_string(SSD1306_Handle * handle, uint8_t x, uint8_t y, uint8_t * text, uint8_t text_size, bool white);

// draw image
void SSD1306_draw_image(SSD1306_Handle * handle, uint8_t x, uint8_t y, const uint8_t * bitmap, uint8_t w, uint8_t h);


// default font
#define SSD1306_FONT1	{\
	0x00, 0x00, 0x00, 0x00, 0x00,	/*  	32*/\
	0x00, 0x00, 0x5F, 0x00, 0x00,	/* !	33*/\
	0x00, 0x07, 0x00, 0x07, 0x00,	/* "	34*/\
	0x14, 0x7F, 0x14, 0x7F, 0x14,	/* #	35*/\
	0x24, 0x2A, 0x7F, 0x2A, 0x12,	/* $	36*/\
	0x23, 0x13, 0x08, 0x64, 0x62,	/* %	37*/\
	0x36, 0x49, 0x56, 0x20, 0x50,	/* &	38*/\
	0x00, 0x08, 0x07, 0x03, 0x00,	/* '	39*/\
	0x00, 0x1C, 0x22, 0x41, 0x00,	/* (	40*/\
	0x00, 0x41, 0x22, 0x1C, 0x00,	/* )	41*/\
	0x2A, 0x1C, 0x7F, 0x1C, 0x2A,	/* *	42*/\
	0x08, 0x08, 0x3E, 0x08, 0x08,	/* +	43*/\
	0x00, 0x80, 0x70, 0x30, 0x00,	/* ,	44*/\
	0x08, 0x08, 0x08, 0x08, 0x08,	/* -	45*/\
	0x00, 0x00, 0x60, 0x60, 0x00,	/* .	46*/\
	0x20, 0x10, 0x08, 0x04, 0x02,	/* /	47*/\
	0x3E, 0x51, 0x49, 0x45, 0x3E,	/* 0	48*/\
	0x00, 0x42, 0x7F, 0x40, 0x00,	/* 1	49*/\
	0x42, 0x61, 0x51, 0x49, 0x46,	/* 2	50*/\
	0x22, 0x41, 0x49, 0x49, 0x36,	/* 3	51*/\
	0x18, 0x14, 0x12, 0x7F, 0x10,	/* 4	52*/\
	0x27, 0x45, 0x45, 0x45, 0x39,	/* 5	53*/\
	0x3E, 0x49, 0x49, 0x49, 0x30,	/* 6	54*/\
	0x61, 0x11, 0x09, 0x05, 0x03,	/* 7	55*/\
	0x36, 0x49, 0x49, 0x49, 0x36,	/* 8	56*/\
	0x06, 0x49, 0x49, 0x49, 0x3E,	/* 9	57*/\
	0x00, 0x00, 0x14, 0x00, 0x00,	/* :	58*/\
	0x00, 0x40, 0x34, 0x00, 0x00,	/* ;	59*/\
	0x00, 0x08, 0x14, 0x22, 0x41,	/* <	60*/\
	0x14, 0x14, 0x14, 0x14, 0x14,	/* =	61*/\
	0x00, 0x41, 0x22, 0x14, 0x08,	/* >	62*/\
	0x02, 0x01, 0x59, 0x09, 0x06,	/* ?	63*/\
	0x3E, 0x41, 0x5D, 0x59, 0x4E,	/* @	64*/\
	0x7E, 0x11, 0x11, 0x11, 0x7E,	/* A	65*/\
	0x7F, 0x49, 0x49, 0x49, 0x36,	/* B	66*/\
	0x3E, 0x41, 0x41, 0x41, 0x22,	/* C	67*/\
	0x7F, 0x41, 0x41, 0x41, 0x3E,	/* D	68*/\
	0x7F, 0x49, 0x49, 0x49, 0x41,	/* E	69*/\
	0x7F, 0x09, 0x09, 0x09, 0x01,	/* F	70*/\
	0x3E, 0x41, 0x51, 0x51, 0x72,	/* G	71*/\
	0x7F, 0x08, 0x08, 0x08, 0x7F,	/* H	72*/\
	0x00, 0x41, 0x7F, 0x41, 0x00,	/* I	73*/\
	0x20, 0x40, 0x41, 0x3F, 0x01,	/* J	74*/\
	0x7F, 0x08, 0x14, 0x22, 0x41,	/* K	75*/\
	0x7F, 0x40, 0x40, 0x40, 0x40,	/* L	76*/\
	0x7F, 0x02, 0x0C, 0x02, 0x7F,	/* M	77*/\
	0x7F, 0x04, 0x08, 0x10, 0x7F,	/* N	78*/\
	0x3E, 0x41, 0x41, 0x41, 0x3E,	/* O	79*/\
	0x7F, 0x09, 0x09, 0x09, 0x06,	/* P	80*/\
	0x3E, 0x41, 0x51, 0x21, 0x5E,	/* Q	81*/\
	0x7F, 0x09, 0x19, 0x29, 0x46,	/* R	82*/\
	0x26, 0x49, 0x49, 0x49, 0x32,	/* S	83*/\
	0x01, 0x01, 0x7F, 0x01, 0x01,	/* T	84*/\
	0x3F, 0x40, 0x40, 0x40, 0x3F,	/* U	85*/\
	0x1F, 0x20, 0x40, 0x20, 0x1F,	/* V	86*/\
	0x3F, 0x40, 0x38, 0x40, 0x3F,	/* W	87*/\
	0x63, 0x14, 0x08, 0x14, 0x63,	/* X	88*/\
	0x07, 0x08, 0x70, 0x08, 0x07,	/* Y	89*/\
	0x61, 0x51, 0x49, 0x45, 0x43,	/* Z	90*/\
	0x00, 0x7F, 0x41, 0x41, 0x41,	/* [	91*/\
	0x02, 0x04, 0x08, 0x10, 0x20,	/* 		92*/\
	0x00, 0x41, 0x41, 0x41, 0x7F,	/* ]	93*/\
	0x04, 0x02, 0x01, 0x02, 0x04,	/* ^	94*/\
	0x40, 0x40, 0x40, 0x40, 0x40,	/* _	95*/\
	0x00, 0x03, 0x07, 0x08, 0x00,	/* `	96*/\
	0x20, 0x54, 0x54, 0x54, 0x78,	/* a	97*/\
	0x7F, 0x44, 0x44, 0x44, 0x38,	/* b	98*/\
	0x38, 0x44, 0x44, 0x44, 0x28,	/* c	99*/\
	0x38, 0x44, 0x44, 0x44, 0x7F,	/* d	100*/\
	0x38, 0x54, 0x54, 0x54, 0x18,	/* e	101*/\
	0x00, 0x08, 0x7E, 0x09, 0x01,	/* f	102*/\
	0x18, 0xA4, 0xA4, 0xA4, 0x7C,	/* g	103*/\
	0x7F, 0x08, 0x04, 0x04, 0x78,	/* h	104*/\
	0x00, 0x44, 0x7D, 0x40, 0x00,	/* i	105*/\
	0x20, 0x40, 0x40, 0x3D, 0x00,	/* j	106*/\
	0x7F, 0x10, 0x28, 0x44, 0x00,	/* k	107*/\
	0x00, 0x41, 0x7F, 0x40, 0x00,	/* l	108*/\
	0x7C, 0x04, 0x78, 0x04, 0x78,	/* m	109*/\
	0x7C, 0x08, 0x04, 0x04, 0x78,	/* n	110*/\
	0x38, 0x44, 0x44, 0x44, 0x38,	/* o	111*/\
	0xFC, 0x24, 0x24, 0x24, 0x18,	/* p	112*/\
	0x18, 0x24, 0x24, 0x24, 0xFC,	/* q	113*/\
	0x7C, 0x08, 0x04, 0x04, 0x08,	/* r	114*/\
	0x48, 0x54, 0x54, 0x54, 0x24,	/* s	115*/\
	0x04, 0x04, 0x3F, 0x44, 0x24,	/* t	116*/\
	0x3C, 0x40, 0x40, 0x20, 0x7C,	/* u	117*/\
	0x1C, 0x20, 0x40, 0x20, 0x1C,	/* v	118*/\
	0x3C, 0x40, 0x30, 0x40, 0x3C,	/* w	119*/\
	0x44, 0x28, 0x10, 0x28, 0x44,	/* x	120*/\
	0x4C, 0x90, 0x90, 0x90, 0x7C,	/* y	121*/\
	0x44, 0x64, 0x54, 0x4C, 0x44,	/* z	122*/\
	0x00, 0x08, 0x36, 0x41, 0x00,	/* {	123*/\
	0x00, 0x00, 0x77, 0x00, 0x00,	/* |	124*/\
	0x00, 0x41, 0x36, 0x08, 0x00,	/* }	125*/\
	0x02, 0x01, 0x02, 0x04, 0x02,	/* ~	126*/\
	0x00, 0x00, 0x00, 0x00, 0x00,	/*  	127*/\
	\
	0x00, 0x00, 0x03, 0x03, 0x00,	/* degree 	128	0x80*/\
	0x00, 0xfc, 0x20, 0x20, 0x1c,	/* micro 	129	0x81*/\
	0x5c, 0x62, 0x02, 0x62, 0x5c,	/* ohm	 	130	0x82*/\
	0x00, 0x08, 0x1c, 0x3e, 0x00,	/* arrow left 	131	0x83*/\
	0x00, 0x3e, 0x1c, 0x08, 0x00,	/* arrow right 	132	0x84*/\
	0x10, 0x18, 0x1c, 0x18, 0x10,	/* arrow up 	133	0x85*/\
	0x04, 0x0c, 0x1c, 0x0c, 0x04,	/* arrow down 	134	0x86*/\
}


#ifdef __cplusplus
}
#endif

#endif	/* SSD1306_H */
