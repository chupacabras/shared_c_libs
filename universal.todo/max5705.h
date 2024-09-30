/**
  ******************************************************************************
  * @file    max5705.h
  * @author  Juraj Lonc (juraj.lonc@gmail.com)
  * @brief   Header file of MAX5705 driver.
  ******************************************************************************
  */

#ifndef INC_MAX5705_H_
#define INC_MAX5705_H_

typedef struct {
	device_write_ptr write_cmd;
	device_read_ptr read_cmd;
} MAX5705_Handle;

void MAX5705_setData(MAX5705_Handle *dac, unsigned char command, unsigned long val);
void MAX5705_sendCommand(MAX5705_Handle *dac, unsigned char command, unsigned char val);

#endif /* INC_MAX5705_H_ */