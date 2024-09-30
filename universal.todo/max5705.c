#include "max5705.h"

static unsigned char c0, c1, c2;
void MAX5705_setDataDAC(MAX5705_Handle *dac, unsigned char command, unsigned long val) {
	c1=(val>>4) & 0xff;
	c2=(val<<4) & 0xf0;
	
	while (MAX5705_SPIRBE==0) c0=MAX5705_SPIBUF;
	
	MAX5705_SPIBUF = command;
	while (!MAX5705_SPIRBF);
	c0 = MAX5705_SPIBUF;
	
	MAX5705_SPIBUF = c1;
	while (!MAX5705_SPIRBF);
	c0 = MAX5705_SPIBUF;
	
	MAX5705_SPIBUF = c2;
	while (!MAX5705_SPIRBF);
	c0 = MAX5705_SPIBUF;
	
}
void MAX5705_sendCommandDAC(MAX5705_Handle *dac, unsigned char command, unsigned char val) {
	while (MAX5705_SPIRBE==0) c0=MAX5705_SPIBUF;
	
	MAX5705_SPIBUF = command;
	while (!MAX5705_SPIRBF);
	c0 = MAX5705_SPIBUF;
	
	MAX5705_SPIBUF = 0;
	while (!MAX5705_SPIRBF);
	c0 = MAX5705_SPIBUF;

	MAX5705_SPIBUF = val;
	while (!MAX5705_SPIRBF);
	c0 = MAX5705_SPIBUF;
}

