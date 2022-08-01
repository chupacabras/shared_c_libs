#include "battery_liion.h"

// input: voltage in mV (4100 = 4.100 V)
// output: promile (890 = 89.0%)
uint16_t convertBatteryLevel(uint16_t voltage) {
//	if (voltage>4190) {
	if (voltage>4150) {
        return 1000;
    } else if (voltage>3950) {
        // 100-80%
//        return -2360+(4*voltage)/5;
        return -3150+voltage;
    } else if (voltage>3810) {
        // 80-60%
        return -4843+(10*voltage)/7;
    } else if (voltage>3680) {
        // 60-20%
	return -11123+(((uint32_t)40)*voltage)/13;
    } else if (voltage>3620) {
        // 20-13.3%
        return -3885+(((uint32_t)111)*voltage)/100;
    } else if (voltage>3.52) {
        // 13.3-6.6%
        return -2278+(((uint32_t)333)*voltage)/500;
    } else if (voltage>3.00) {
        // 6.6-0%
        return -384+(((uint32_t)333)*voltage)/2600;
    } else {
        return 0;
    }
}



/*
float convertBatteryLevel(float voltage) {
    if (voltage>4.2) {
        return 100.0;
    } else if (voltage>3.95) {
        // 100-80%
        return 100.0-20.0*(4.2-voltage)/(4.2-3.95);
    } else if (voltage>3.81) {
        // 80-60%
        return 80.0-20.0*(3.95-voltage)/(3.95-3.81);
    } else if (voltage>3.68) {
        // 60-20%
        return 60.0-40.0*(3.81-voltage)/(3.81-3.68);
    } else if (voltage>3.62) {
        // 20-13.3%
        return 20.0-6.66*(3.68-voltage)/(3.68-3.62);
    } else if (voltage>3.52) {
        // 13.3-6.6%
        return 13.33-6.66*(3.62-voltage)/(3.62-3.52);
    } else if (voltage>3.00) {
        // 6.6-0%
        return 6.66-6.66*(3.52-voltage)/(3.52-3.00);
    } else {
        return 0;
    }
}
*/
