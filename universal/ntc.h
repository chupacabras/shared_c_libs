#ifndef INC_NTC_H
#define	INC_NTC_H

#include <stdint.h>

// 2322 640-10k, 10 kOhm, B=3977K
#define NTC_ARR_10K_B3977 {332100, 240000, 175200, 129300,  96360, 72500, 55050, 42160, 32560, 25340, 19870, 15700, 12490, 10000, 8059, 6536, 5330, 4372, 3606, 2989, 2490, 2084, 1753, 1481, 1256, 1070, 915, 786, 677, 586, 508, 443, 387, 339}

// NCP21XV103J03RA, 10 kOhm, B=3900K, Murata
// starts at -40C, to +125C
#define NTC_ARR_10K_B3900_NCP21 {328996, 237387, 173185, 127773,  95327, 71746, 54564, 41813, 32330, 25194, 19785, 15651, 12468, 10000, 8072, 6556, 5356, 4401, 3635, 3019, 2521, 2115, 1781, 1509, 1284, 1097, 941, 810, 701, 608, 530, 463, 406, 358}


// generic NTC, 10 kOhm, B=3950K
// starts at -40C, to +125C
#define NTC_ARR_10K_B3950 {401860, 281577, 200204, 144317, 105385, 77898, 58246, 44026, 33621, 25925, 20175, 15837, 12535, 10000, 8037, 6506, 5301, 4348, 3588, 2978, 2486, 2086, 1760, 1492, 1270, 1087, 934, 805, 698, 606, 529, 463, 407, 359}
// 10k:10k divider, 10bit ADC
#define NTC_ARR_10K_B3950_10ADC {999, 989, 975, 958, 935, 908, 874, 834, 789, 739, 685, 628, 570, 512, 456, 404, 355, 310, 270, 235, 204, 177, 153, 133, 115, 100, 87, 76, 67, 59, 51, 45, 40, 35}
// 10k:10k divider, 12bit ADC
#define NTC_ARR_10K_B3950_12ADC {3997,3956,3901,3831,3741,3630,3496,3338,3157,2956,2739,2511,2278,2048,1825,1614,1419,1241,1082,940,816,707,613,532,462,402,350,305,267,234,206,181,160,142}


// MF52 A, 10 kOhm, B=3950K, Nanjing Shiheng Elec
// starts at -40C, to +125C
#define NTC_ARR_10K_B3950_MF52A {283730, 207454, 154827, 117280,  89682, 68982, 53280, 41306, 32116, 25152, 19783, 15652, 12461, 10000, 8047, 6523, 5318, 4357, 3588, 2968, 2466, 2058, 1725, 1452, 1228, 1044, 890, 763, 657, 567, 491, 427, 371, 324}


// MF52 B/D, 10 kOhm, B=3950K, Nanjing Shiheng Elec, D=black wires, B=red wires
// starts at -40C, to +125C
#define NTC_ARR_10K_B3950_MF52B {304880, 222326, 163799, 121950,  91728, 69666, 53380, 41229, 32040, 25116, 19788, 15679, 12493, 10000, 8060, 6527, 5313, 4348, 3578, 2958, 2459, 2053, 1723, 1452, 1229, 1045, 891, 763, 656, 565, 489, 424, 369, 323}
#define NTC_ARR_10K_B3950_MF52D NTC_ARR_10K_B3950_MF52B
// 10k:10k divider, 10bit ADC
#define NTC_ARR_10K_B3950_MF52B_10ADC {999, 989, 975, 958, 935, 908, 874, 834, 789, 739, 685, 628, 570, 512, 456, 404, 355, 310, 270, 235, 204, 177, 153, 133, 115, 100, 87, 76, 67, 59, 51, 45, 40, 35}
#define NTC_ARR_10K_B3950_MF52D_10ADC NTC_ARR_10K_B3950_MF52B_10ADC
// 10k:10k divider, 12bit ADC
#define NTC_ARR_10K_B3950_MF52B_12ADC {3966,3920,3860,3786,3693,3582,3450,3296,3122,2930,2721,2501,2275,2048,1828,1618,1421,1241,1079,935,808,698,602,519,448,388,335,290,252,219,191,167,146,128}
#define NTC_ARR_10K_B3950_MF52D_12ADC NTC_ARR_10K_B3950_MF52B_12ADC


// MF52 B/D, 10 kOhm (changed to 9.7k), B=3950K, Nanjing Shiheng Elec, D=black wires, B=red wires
// starts at -40C, to +125C
#define NTC_ARR_9K7_B3950_MF52D {295734,215656,158885,118292,88976,67576,51779,39992,31079,24363,19194,15209,12118,9700,7818,6331,5154,4218,3471,2869,2385,1991,1671,1408,1192,1014,864,740,636,548,474,411,358,313}
// 10k:9.7k divider, 12bit ADC
#define NTC_ARR_9K7_B3950_MF52D_12ADC {3962,3914,3853,3777,3682,3568,3433,3277,3099,2904,2693,2471,2244,2017,1797,1588,1393,1215,1055,913,789,680,586,506,436,377,326,282,245,213,185,162,142,124}

// MF52 E, 10 kOhm, B=3950K
// starts at -40C, to +125C
#define NTC_ARR_10K_B3950_MF52E {345230, 248904, 181700, 133300,  98880, 74100, 56060, 42800, 98960, 25580, 20000, 15760, 12510, 10000, 8048, 6518, 5312, 4354, 3588, 2974, 2476, 2072, 1743, 1473, 1250, 1065, 911, 782, 674, 584, 507, 443, 387, 339}
// 10k:10k divider, 12bit ADC
#define NTC_ARR_10K_B3950_MF52E_12ADC {3981,3938,3882,3810,3720,3609,3476,3320,3720,2945,2731,2506,2276,2048,1826,1616,1421,1242,1082,939,813,703,608,526,455,394,342,297,259,226,198,174,153,134}

#define NTC_ARR_10K25_B3950_MF52D_12ADC {3969,3924,3866,3793,3702,3593,3463,3312,3140,2950,2743,2525,2300,2073,1853,1642,1444,1263,1099,953,825,712,615,531,458,396,343,297,258,224,196,171,149,131}
#define NTC_ARR_10K20_B3950_MF52D_12ADC {3968,3923,3865,3791,3700,3591,3460,3309,3136,2946,2739,2520,2295,2068,1848,1637,1440,1258,1095,949,821,709,612,528,456,395,341,296,257,223,195,170,149,131}
#define NTC_ARR_10K10_B3950_MF52D_12ADC {3967,3921,3863,3788,3697,3586,3455,3303,3129,2938,2730,2511,2285,2058,1838,1627,1430,1250,1087,942,815,703,607,524,452,391,338,293,255,221,193,168,147,129}
#define NTC_ARR_10K05_B3950_MF52D_12ADC {3967,3921,3861,3787,3695,3584,3452,3300,3125,2934,2726,2506,2280,2053,1833,1623,1426,1246,1083,939,812,701,605,522,450,389,337,292,253,220,192,167,146,129}


// input: resistance value in ohms, or ADC value (NTC at low side, resistor at high side)
// input: NTC_ARR, array of resistance values, or array of ADC values, between -40C and 125C, in 5C steps
// output: hundreths od deg C (13000 = 130.00 C)
int16_t convert_temperature(uint32_t val, const uint16_t *NTC_ARR);


#endif	/* INC_NTC_H */
