/**
  ******************************************************************************
  * @file    crc16.h
  * @author  Juraj Lonc (juraj.lonc@gmail.com)
  * @brief   Header file of CRC16 function.
  ******************************************************************************
  */

#ifndef INC_CRC16_H_
#define INC_CRC16_H_

//#include "stdio.h"
#include "stdbool.h"
#include "stdint.h"


bool check_crc16(uint8_t* input, uint16_t len, uint16_t crc_to_check);
uint16_t crc16(uint8_t* input, uint16_t len);


#endif /* INC_CRC16_H_ */
