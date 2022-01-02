#include "ntc.h"

int16_t convert_temperature(uint32_t val, const uint16_t *NTC_ARR) {
    unsigned int q;
    if (val > NTC_ARR[0]) return -4000;
    for (q = 1; q < 34; q++) {
        if (val > NTC_ARR[q]) {
            return -4000 + 500 * (q - 1)+((500 * ((int32_t)(NTC_ARR[q - 1] - val))) / ((int32_t)(NTC_ARR[q - 1] - NTC_ARR[q])));
        }
    }

    return 13000;
}