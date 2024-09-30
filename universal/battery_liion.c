#include "battery_liion.h"

// input: voltage in mV (4100 = 4.100 V)
// output: promile (890 = 89.0%)
uint16_t convertBatteryLevel(uint16_t voltage) {
//	if (voltage>4190) {
	if (voltage>4150) {
        return 1000;
    } else if (voltage>3700) {
        // 100-10%
/*
4150-3700=450
90%

130+90*(U-3700)/450
130+U*90/450-90*3700/450
130-740+U*1/5
-610+U*1/5
*/

        return -610+voltage/5;
    } else if (voltage>3450) {
        // 10-5%
/*
3700-3450=250
5%

50+5*(U-3450)/250
50+5*U/250-5*3450/250
50-69+1*U/50
-19+1*U/50
*/

        return -19+(voltage)/50;
    } else if (voltage>3240) {
        // 5-2%
/*
3450-3240=210
3%

20+3*(U-3240)/210
20+3*U/210-3*3240/210
20-46+1*U/70
-26+1*U/70
*/

	return -26+(voltage)/70;
    } else if (voltage>3000) {
        // 2-0%
/*
3240-3000=240
2%

0+2*(U-3000)/240
0+2*U/240-2*3000/240
0-25+1*U/120
-25+1*U/120
*/

        return -25+(voltage)/120;
    } else {
        return 0;
    }
}
