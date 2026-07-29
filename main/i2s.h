#pragma once
#include "driver/i2s_std.h"

#define BCLK   GPIO_NUM_4   // old: 4
#define WS      GPIO_NUM_16     // odl: 7 older: 46
#define DIN     GPIO_NUM_17      // old: 6

#define DOUT_1    GPIO_NUM_25 //regular esp does not have gpio46 old: 46; older: 9
#define DOUT_2    GPIO_NUM_18 // old: 5

#define samples 480
#define EXAMPLE_BUFF_SIZE              samples * 2 * sizeof(int32_t)// stereo -> *2
#define CLOCK   48000    

void init_i2s(void);


extern i2s_chan_handle_t                tx_chan_1;
extern i2s_chan_handle_t                tx_chan_2;       