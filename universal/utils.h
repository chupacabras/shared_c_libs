/**
  ******************************************************************************
  * @file    utils.h
  * @author  Juraj Lonc (juraj.lonc@gmail.com)
  * @brief   Header file of Utilities.
  ******************************************************************************
  */

#ifndef __UTILS_H
#define __UTILS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <string.h>


void long2binary(uint8_t *buffer, uint32_t i, bool separated);
void int2binary(uint8_t *buffer, uint16_t i, bool separated);
void char2binary(uint8_t *buffer, uint8_t c, bool separated);
void long2hex(uint8_t *buffer, uint32_t i);
void int2hex(uint8_t *buffer, uint16_t i);
void char2hex(uint8_t *buffer, uint8_t c);
uint8_t long2string(uint8_t *buffer, int32_t i);
void double2string_fixed(uint8_t *buffer, double d, uint8_t decimals, uint8_t length);
void double2string(uint8_t *buffer, double d, uint8_t decimals);
void long2string_fixed(uint8_t *buffer, int32_t n, uint8_t decimals, uint8_t length);

uint8_t bcd_to_num(uint8_t bcd);
uint8_t num_to_bcd(uint8_t num);

void roundDouble(double d, int32_t* rounded);
void ceilDouble(double d, int32_t* ceiled);

#ifdef __cplusplus
}
#endif

#endif
