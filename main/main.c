//#include <stdint.h>
//#include <stdlib.h>
//#include <string.h>
#include <tgmath.h>

#include "freertos/FreeRTOS.h"
//#include "freertos/task.h"

#include "driver/i2s_std.h"
//#include "driver/gpio.h"
//#include "esp_check.h"
//#include "sdkconfig.h"

#include "esp_adc/adc_oneshot.h"
//#include "esp_adc/adc_cali.h"
//#include "esp_adc/adc_cali_scheme.h"


#define BCLK   GPIO_NUM_4   // old: 4
#define WS      GPIO_NUM_7     // old: 46
#define DIN     GPIO_NUM_6      // old: 6

#define DOUT_1    GPIO_NUM_46 // old: 9
#define DOUT_2    GPIO_NUM_5 // old: 5

#define EXAMPLE_BUFF_SIZE              4 * sizeof(int32_t)//64
#define CLOCK   48000    

//-----High Coeffs------//
const int64_t HighCoeffs_A_HP[2] = { 1395166348, -523407902 }; //hp3887 30bit
const int64_t HighCoeffs_B_HP[3] = { 748297431, -1496594861, 748297431 };

const int64_t HighCoeffs_A_LP[2] = { -1947171197, -892451235 }; //lp23000 30bit
const int64_t HighCoeffs_B_LP[3] = { 978532682, 1957065364, 978532682 };

//-----Mid Coeffs------//
const int64_t MidCoeffs_A_HP[2] = { 2106887749, -1033695662 }; //hp203 30bit
const int64_t MidCoeffs_B_HP[3] = { 1053623222, -2107246444, 1053623222 };

const int64_t MidCoeffs_A_LP[2] = { 1470521965, -563754365 }; // lp3483Hz 30bit
const int64_t MidCoeffs_B_LP[3] = { 41814974, 83629947, 41814974 };

//-----Low Coeffs------//
const int64_t LowCoeffs_A_HP = 1070347903; //hp17 n=1
const int64_t LowCoeffs_B_HP[2] = { 1072044923, -1072044923 };

const int64_t LowCoeffs_A_LP[2] = { 2111633663, -1038474472 }; //lp178 30bit 
const int64_t LowCoeffs_B_LP[3] = { 143309, 286617, 143309 };

// a * y
// b * x

int32_t lastX_high[4] = {0};
int32_t lastX_midR[4] = {0};
int32_t lastX_midL[4] = {0};
int32_t lastX_low[4] = {0};

int32_t lastYHigh_0[4] = {0};
int32_t lastYHigh_1[4] = {0};

int32_t lastYMid_R_0[4] = {0};
int32_t lastYMid_R_1[4] = {0};

int32_t lastYMid_L_0[4] = {0};
int32_t lastYMid_L_1[4] = {0};

int32_t lastYLow_0[4] = {0};
int32_t lastYLow_1[4] = {0};

const int shift = 30;
//int32_t max = 2147483647;          //real max: 2147418112 (3.12V). Using 2^31 - 1 (3.12V)

static inline void IRAM_ATTR calculateY(int32_t *restrict M_Buf, int32_t *restrict H_L_Buf){
    // for (int i = 5; i >= 2; i--) {
    //     lastYHigh_0[i] = lastYHigh_0[i - 2];
    //     lastYHigh_1[i] = lastYHigh_1[i - 2];

    //     lastYMid_R_0[i] = lastYMid_R_0[i - 2];
    //     lastYMid_R_1[i] = lastYMid_R_1[i - 2];

    //     lastYMid_L_0[i] = lastYMid_L_0[i - 2];
    //     lastYMid_L_1[i] = lastYMid_L_1[i - 2];

    //     lastYLow_0[i] = lastYLow_0[i - 2];
    //     lastYLow_1[i] = lastYLow_1[i - 2];
    // }
    // memmove(&lastYHigh_0[2], &lastYHigh_0[0], 4 * sizeof(lastYHigh_0[0]));
    // memmove(&lastYHigh_1[2], &lastYHigh_1[0], 4 * sizeof(lastYHigh_1[0]));
    // memmove(&lastYMid_R_0[2], &lastYMid_R_0[0], 4 * sizeof(lastYMid_R_0[0]));
    // memmove(&lastYMid_R_1[2], &lastYMid_R_1[0], 4 * sizeof(lastYMid_R_1[0]));
    // memmove(&lastYMid_L_0[2], &lastYMid_L_0[0], 4 * sizeof(lastYMid_L_0[0]));
    // memmove(&lastYMid_L_1[2], &lastYMid_L_1[0], 4 * sizeof(lastYMid_L_1[0]));
    // memmove(&lastYLow_0[2],  &lastYLow_0[0],  4 * sizeof(lastYLow_0[0]));
    // memmove(&lastYLow_1[2],  &lastYLow_1[0],  4 * sizeof(lastYLow_1[0]));

    lastYHigh_0[3] = lastYHigh_0[1]; lastYHigh_0[2] = lastYHigh_0[0];
    lastYHigh_1[3] = lastYHigh_1[1]; lastYHigh_1[2] = lastYHigh_1[0];
    lastYMid_R_0[3] = lastYMid_R_0[1]; lastYMid_R_0[2] = lastYMid_R_0[0];
    lastYMid_R_1[3] = lastYMid_R_1[1]; lastYMid_R_1[2] = lastYMid_R_1[0];
    lastYMid_L_0[3] = lastYMid_L_0[1]; lastYMid_L_0[2] = lastYMid_L_0[0];
    lastYMid_L_1[3] = lastYMid_L_1[1]; lastYMid_L_1[2] = lastYMid_L_1[0];
    lastYLow_0[3]   = lastYLow_0[1];   lastYLow_0[2]   = lastYLow_0[0];
    lastYLow_1[3]   = lastYLow_1[1];   lastYLow_1[2]   = lastYLow_1[0];


    //-----High Calculation------//
    //--HP_203Hz--//
    lastYHigh_0[1] = ((lastX_high[1] * HighCoeffs_B_HP[0]) >> shift) + ((lastX_high[2] * HighCoeffs_B_HP[1]) >> shift) + ((lastX_high[3] * HighCoeffs_B_HP[2]) >> shift)
         + ((lastYHigh_0[2] * HighCoeffs_A_HP[0]) >> shift) + ((lastYHigh_0[3] * HighCoeffs_A_HP[1]) >> shift);

    lastYHigh_0[0] = ((lastX_high[0] * HighCoeffs_B_HP[0]) >> shift) + ((lastX_high[1] * HighCoeffs_B_HP[1]) >> shift) + ((lastX_high[2] * HighCoeffs_B_HP[2]) >> shift)
         + ((lastYHigh_0[1] * HighCoeffs_A_HP[0]) >> shift) + ((lastYHigh_0[2] * HighCoeffs_A_HP[1]) >> shift);

    //--LP_3483Hz--//
    lastYHigh_1[1] = ((lastYHigh_0[1] * HighCoeffs_B_LP[0]) >> shift) + ((lastYHigh_0[2] * HighCoeffs_B_LP[1]) >> shift) + ((lastYHigh_0[3] * HighCoeffs_B_LP[2]) >> shift)
         + ((lastYHigh_1[2] * HighCoeffs_A_LP[0]) >> shift) + ((lastYHigh_1[3] * HighCoeffs_A_LP[1]) >> shift);

    lastYHigh_1[0] = ((lastYHigh_0[0] * HighCoeffs_B_LP[0]) >> shift) + ((lastYHigh_0[1] * HighCoeffs_B_LP[1]) >> shift) + ((lastYHigh_0[2] * HighCoeffs_B_LP[2]) >> shift)
         + ((lastYHigh_1[1] * HighCoeffs_A_LP[0]) >> shift) + ((lastYHigh_1[2] * HighCoeffs_A_LP[1]) >> shift);


    //-----Mid Calculation------//
    //--HP_203Hz--//
    //--Right--//
    lastYMid_R_0[1] = ((lastX_midR[1] * MidCoeffs_B_HP[0]) >> shift) + ((lastX_midR[2] * MidCoeffs_B_HP[1]) >> shift) + ((lastX_midR[3] * MidCoeffs_B_HP[2]) >> shift)
         + ((lastYMid_R_0[2] * MidCoeffs_A_HP[0]) >> shift) + ((lastYMid_R_0[3] * MidCoeffs_A_HP[1]) >> shift);

    lastYMid_R_0[0] = ((lastX_midR[0] * MidCoeffs_B_HP[0]) >> shift) + ((lastX_midR[1] * MidCoeffs_B_HP[1]) >> shift) + ((lastX_midR[2] * MidCoeffs_B_HP[2]) >> shift)
         + ((lastYMid_R_0[1] * MidCoeffs_A_HP[0]) >> shift) + ((lastYMid_R_0[2] * MidCoeffs_A_HP[1]) >> shift);
    //--Left--//
    lastYMid_L_0[1] = ((lastX_midL[1] * MidCoeffs_B_HP[0]) >> shift) + ((lastX_midL[2] * MidCoeffs_B_HP[1]) >> shift) + ((lastX_midL[3] * MidCoeffs_B_HP[2]) >> shift)
         + ((lastYMid_L_0[2] * MidCoeffs_A_HP[0]) >> shift) + ((lastYMid_L_0[3] * MidCoeffs_A_HP[1]) >> shift);

    lastYMid_L_0[0] = ((lastX_midL[0] * MidCoeffs_B_HP[0]) >> shift) + ((lastX_midL[1] * MidCoeffs_B_HP[1]) >> shift) + ((lastX_midL[2] * MidCoeffs_B_HP[2]) >> shift)
         + ((lastYMid_L_0[1] * MidCoeffs_A_HP[0]) >> shift) + ((lastYMid_L_0[2] * MidCoeffs_A_HP[1]) >> shift);


    //--LP_3483Hz--//
    //--Right--//
    lastYMid_R_1[1] = ((lastYMid_R_0[1] * MidCoeffs_B_LP[0]) >> shift) + ((lastYMid_R_0[2] * MidCoeffs_B_LP[1]) >> shift) + ((lastYMid_R_0[3] * MidCoeffs_B_LP[2]) >> shift)
         + ((lastYMid_R_1[2] * MidCoeffs_A_LP[0]) >> shift) + ((lastYMid_R_1[3] * MidCoeffs_A_LP[1]) >> shift);

    lastYMid_R_1[0] = ((lastYMid_R_0[0] * MidCoeffs_B_LP[0]) >> shift) + ((lastYMid_R_0[1] * MidCoeffs_B_LP[1]) >> shift) + ((lastYMid_R_0[2] * MidCoeffs_B_LP[2]) >> shift)
         + ((lastYMid_R_1[1] * MidCoeffs_A_LP[0]) >> shift) + ((lastYMid_R_1[2] * MidCoeffs_A_LP[1]) >> shift);
    //--Left--//
    lastYMid_L_1[1] = ((lastYMid_L_0[1] * MidCoeffs_B_LP[0]) >> shift) + ((lastYMid_L_0[2] * MidCoeffs_B_LP[1]) >> shift) + ((lastYMid_L_0[3] * MidCoeffs_B_LP[2]) >> shift)
         + ((lastYMid_L_1[2] * MidCoeffs_A_LP[0]) >> shift) + ((lastYMid_L_1[3] * MidCoeffs_A_LP[1]) >> shift);

    lastYMid_L_1[0] = ((lastYMid_L_0[0] * MidCoeffs_B_LP[0]) >> shift) + ((lastYMid_L_0[1] * MidCoeffs_B_LP[1]) >> shift) + ((lastYMid_L_0[2] * MidCoeffs_B_LP[2]) >> shift)
         + ((lastYMid_L_1[1] * MidCoeffs_A_LP[0]) >> shift) + ((lastYMid_L_1[2] * MidCoeffs_A_LP[1]) >> shift);


    //-----Low Calculation------//  
    lastYLow_0[1] = ((lastX_low[1] * LowCoeffs_B_HP[0]) >> shift) + ((lastX_low[2] * LowCoeffs_B_HP[1]) >> shift)
         + ((lastYLow_0[2] * LowCoeffs_A_HP) >> shift);

    lastYLow_0[0] = ((lastX_low[0] * LowCoeffs_B_HP[0]) >> shift) + ((lastX_low[1] * LowCoeffs_B_HP[1]) >> shift)
         + ((lastYLow_0[1] * LowCoeffs_A_HP) >> shift);

    //--LP_178Hz--//
    lastYLow_1[1] = ((lastYLow_0[1] * LowCoeffs_B_LP[0]) >> shift) + ((lastYLow_0[2] * LowCoeffs_B_LP[1]) >> shift) + ((lastYLow_0[3] * LowCoeffs_B_LP[2]) >> shift)
         + ((lastYLow_1[2] * LowCoeffs_A_LP[0]) >> shift) + ((lastYLow_1[3] * LowCoeffs_A_LP[1]) >> shift);

    lastYLow_1[0] = ((lastYLow_0[0] * LowCoeffs_B_LP[0]) >> shift) + ((lastYLow_0[1] * LowCoeffs_B_LP[1]) >> shift) + ((lastYLow_0[2] * LowCoeffs_B_LP[2]) >> shift)
         + ((lastYLow_1[1] * LowCoeffs_A_LP[0]) >> shift) + ((lastYLow_1[2] * LowCoeffs_A_LP[1]) >> shift);


    // lastYLow_1[0] = lastX_low[0];
    // lastYLow_1[1] = lastX_low[1];

    // lastYMid_R_1[0] = lastX_midR[0];
    // lastYMid_R_1[1] = lastX_midR[1];

    // lastYMid_L_1[0] = lastX_midL[0];
    // lastYMid_L_1[1] = lastX_midL[1];

    // lastYHigh_1[0] = lastX_high[0];
    // lastYHigh_1[1] = lastX_high[1];
}

int16_t main_volume;
int16_t high_volume;
int16_t mid_volume;
int16_t low_volume;

static inline void IRAM_ATTR pushNewX(int32_t *newSample){ // clocks: 954, wenn filter funktion auskommentiert
    // for (int i = 5; i >= 2; --i) {
    //     lastX_high[i] = lastX_high[i-2];
    //     lastX_midR[i] = lastX_midR[i-2];
    //     lastX_midL[i] = lastX_midL[i-2];
    //     lastX_low[i]  = lastX_low[i-2];
    // }
    // memmove(&lastX_high[2], &lastX_high[0], 4 * sizeof(lastX_high[0]));
    // memmove(&lastX_midR[2], &lastX_midR[0], 4 * sizeof(lastX_midR[0]));
    // memmove(&lastX_midL[2], &lastX_midL[0], 4 * sizeof(lastX_midL[0]));
    // memmove(&lastX_low[2],  &lastX_low[0],  4 * sizeof(lastX_low[0]));

    lastX_high[3] = lastX_high[1]; lastX_high[2] = lastX_high[0];
    lastX_midR[3] = lastX_midR[1]; lastX_midR[2] = lastX_midR[0];
    lastX_midL[3] = lastX_midL[1]; lastX_midL[2] = lastX_midL[0];
    lastX_low[3]  = lastX_low[1];  lastX_low[2]  = lastX_low[0];

    {
        int32_t L = newSample[0];
        int32_t R = newSample[1];

        int64_t mono  = ((((int64_t)L + (int64_t)R) >> 1) * main_volume) >> 12;
        int64_t left  = ((int64_t)L * main_volume) >> 12;
        int64_t right = ((int64_t)R * main_volume) >> 12;

        lastX_high[1] = (int32_t)((mono  * high_volume) >> 12);
        lastX_low[1]  = (int32_t)((mono  * low_volume)  >> 12);
        lastX_midL[1] = (int32_t)((left  * mid_volume)  >> 12);
        lastX_midR[1] = (int32_t)((right * mid_volume)  >> 12);
    }

    {
        int32_t L = newSample[3];
        int32_t R = newSample[2];

        int64_t mono  = ((((int64_t)L + (int64_t)R) >> 1) * main_volume) >> 12;
        int64_t left  = ((int64_t)L * main_volume) >> 12;
        int64_t right = ((int64_t)R * main_volume) >> 12;

        lastX_high[0] = (int32_t)((mono  * high_volume) >> 12);
        lastX_low[0]  = (int32_t)((mono  * low_volume)  >> 12);
        lastX_midL[0] = (int32_t)((left  * mid_volume)  >> 12);
        lastX_midR[0] = (int32_t)((right * mid_volume) >> 12);
    }
}


static inline void IRAM_ATTR mapOutput(int32_t *M_Buf, int32_t *H_L_Buf){
    H_L_Buf[0] = (int32_t)lastYHigh_1[1];
    M_Buf[0] = (int32_t)lastYMid_R_1[1];
    M_Buf[1] = (int32_t)lastYMid_L_1[1];
    H_L_Buf[1] = (int32_t)lastYLow_1[1];

    H_L_Buf[2] = (int32_t)lastYHigh_1[0];
    M_Buf[2] = (int32_t)lastYMid_R_1[0];
    M_Buf[3] = (int32_t)lastYMid_L_1[0];
    H_L_Buf[3] = (int32_t)lastYLow_1[0];
}


static i2s_chan_handle_t                tx_chan_1;
static i2s_chan_handle_t                tx_chan_2;       
static i2s_chan_handle_t                rx_chan;

// uint32_t clocks[4096] = {0};
// uint64_t sum = 0;
// int it = 0;

static inline void fiter(void *args)
{
    int32_t *r_buf = (int32_t *)malloc(EXAMPLE_BUFF_SIZE);
    int32_t *M_Buf = (int32_t *)malloc(EXAMPLE_BUFF_SIZE);
    int32_t *H_L_Buf = (int32_t *)malloc(EXAMPLE_BUFF_SIZE);
    
    size_t r_bytes = 0;
    size_t w_bytes = 0;

    ESP_ERROR_CHECK(i2s_channel_enable(rx_chan));
    ESP_ERROR_CHECK(i2s_channel_enable(tx_chan_1));
    ESP_ERROR_CHECK(i2s_channel_enable(tx_chan_2));    

    while (true) {
        if (i2s_channel_read(rx_chan, r_buf, EXAMPLE_BUFF_SIZE, &r_bytes, 1000) == ESP_OK) {
            uint32_t start = esp_cpu_get_cycle_count();
            pushNewX(r_buf);
            calculateY(M_Buf, H_L_Buf);
            mapOutput(M_Buf, H_L_Buf);

            // clocks[it] = esp_cpu_get_cycle_count() - start;
            // it++;
            // if (it == 4095)
            // {
            //     for (size_t i = 1; i < 4096; i++)
            //     {
            //         sum += clocks[i];
            //     }
            //     printf("Avg calc cycles: %llu\n", sum >> 12);
            //     //printf("Avg calc cycles: %lu\n", clocks[4096]);
            //     sum = 0;
            //     it = 0;
            // }

            i2s_channel_write(tx_chan_1, M_Buf, EXAMPLE_BUFF_SIZE, &w_bytes, 1000);
            i2s_channel_write(tx_chan_2, H_L_Buf, EXAMPLE_BUFF_SIZE, &w_bytes, 1000);

            // r_buf[1] = r_buf[1] * -1;
            // r_buf[3] = r_buf[3] * -1;
            
            // memcpy(w_buf1, r_buf, EXAMPLE_BUFF_SIZE);
            // memcpy(w_buf2, r_buf, EXAMPLE_BUFF_SIZE);
            // i2s_channel_write(tx_chan_1, w_buf1, EXAMPLE_BUFF_SIZE, &w_bytes, 1000);
            // i2s_channel_write(tx_chan_2, w_buf2, EXAMPLE_BUFF_SIZE, &w_bytes, 1000);
        } 
        else {
            printf("Read Task: i2s read failed\n");
        }
        //vTaskDelay(pdMS_TO_TICKS(200));
    }
    free(r_buf);
    free(M_Buf);
    free(H_L_Buf);
    vTaskDelete(NULL);
}

static void init_i2s(void)
{
    //i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_SLAVE);
    i2s_chan_config_t d_chan_cfg = {
        .dma_desc_num = 6,
        .dma_frame_num = 240,
        .id = I2S_NUM_0,
        .role = I2S_ROLE_SLAVE,
        .intr_priority = 0,
        .auto_clear = true,
    };
    ESP_ERROR_CHECK(i2s_new_channel(&d_chan_cfg, &tx_chan_1, &rx_chan));

    i2s_chan_config_t s_chan_cfg = {
        .dma_desc_num = 6,
        .dma_frame_num = 240,
        .id = I2S_NUM_1,
        .role = I2S_ROLE_SLAVE,
        .intr_priority = 0,
        .auto_clear = true,
    };
    ESP_ERROR_CHECK(i2s_new_channel(&s_chan_cfg, &tx_chan_2, NULL));

    i2s_std_config_t d_cfg = {
        .clk_cfg  = {
            .sample_rate_hz = CLOCK,
            .clk_src = I2S_CLK_SRC_PLL_160M,
            .mclk_multiple = 256U,
        },                          
        //.clk_cfg  =I2S_STD_CLK_DEFAULT_CONFIG(CLOCK),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,    
            .bclk = BCLK,
            .ws   = WS,
            .dout = DOUT_1,
            .din  = DIN,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
    };

    i2s_std_config_t s_cfg = {
        .clk_cfg  = {
            .sample_rate_hz = CLOCK,
            .clk_src = I2S_CLK_SRC_PLL_160M,
            .mclk_multiple = 256U,
        },                          
        //.clk_cfg  =I2S_STD_CLK_DEFAULT_CONFIG(CLOCK),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,    
            .bclk = BCLK,
            .ws   = WS,
            .dout = DOUT_2,
            .din  = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
    };
    /* Initialize the channels */
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_chan, &d_cfg));
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_chan_1, &d_cfg));
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_chan_2, &s_cfg));
}


adc_oneshot_unit_handle_t adc_handle;

void init_adc() {
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC_UNIT_2,
    };
    adc_oneshot_new_unit(&unit_cfg, &adc_handle);

    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12,
    };

    adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_3, &chan_cfg); // Pin 14
    adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_2, &chan_cfg); // Pin 13
    adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_1, &chan_cfg); // Pin 12
    adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_0, &chan_cfg); // Pin 11
}

const float PotiCoeffs_A[2] = { 1.64745998,  -0.70089678 }; // lp wn=2Hz fs=50Hz
const float PotiCoeffs_B[3] = { 0.0133592, 0.0267184, 0.0133592 };

int16_t lastPotiMain_Y[3] = {0};
int16_t lastPotiHigh_Y[3] = {0};
int16_t lastPotiMid_Y[3] = {0};
int16_t lastPotiLow_Y[3] = {0};

int16_t lastPotiMain_X[3] = {0};
int16_t lastPotiHigh_X[3] = {0};
int16_t lastPotiMid_X[3] = {0};
int16_t lastPotiLow_X[3] = {0};

void filterPoti(int16_t *Xarr, int16_t *Yarr){// should be changed to integer logic
    for (int i = 2; i >= 1; i--) {
        Yarr[i] = Yarr[i - 1];
    }

    Yarr[0] = (Xarr[0] * PotiCoeffs_B[0]) + (Xarr[1] * PotiCoeffs_B[1]) + (Xarr[2] * PotiCoeffs_B[2])
            + (Yarr[1] * PotiCoeffs_A[0]) + (Yarr[2] * PotiCoeffs_A[1]);
}

void filterPotis(void){
    filterPoti(lastPotiMain_X, lastPotiMain_Y);
    filterPoti(lastPotiHigh_X, lastPotiHigh_Y);
    filterPoti(lastPotiMid_X, lastPotiMid_Y);
    filterPoti(lastPotiLow_X, lastPotiLow_Y);
}

void shiftPotiArr(int16_t main, int16_t high, int16_t mid, int16_t low){
    for (int i = 2; i >= 1; i--) {
        lastPotiMain_X[i] = lastPotiMain_X[i - 1];
        lastPotiHigh_X[i] = lastPotiHigh_X[i - 1];
        lastPotiMid_X[i] = lastPotiMid_X[i - 1];
        lastPotiLow_X[i] = lastPotiLow_X[i - 1];
    }
    lastPotiMain_X[0] = main;
    lastPotiHigh_X[0] = high;
    lastPotiMid_X[0] = mid;
    lastPotiLow_X[0] = low;
}

int16_t read_adc(adc_channel_t channel) {
    int raw;
    adc_oneshot_read(adc_handle, channel, &raw);
    return raw;
}

int16_t make_exponential(int16_t x){
    return 4095 * pow(((float)x/4095), 2);
}

const float high_Limit = 0.3;
const float mid_Limit = 0.3;
const float low_Limit = 0.3;

// 1 -> no change; -1 -> inverting
const int invertHigh = 1;
const int invertMid = 1;
const int invertLow = 1;

int16_t clamp12bit(int16_t x){
    if(x >= 4095){
        return 4095;
    }
    else if(x <= 0){
        return 0;
    }
    else{
        return x;
    }
}

void get_volumes(){
    int16_t mainADC = read_adc(ADC_CHANNEL_3);// Pin 14 will be changed mabye 
    int16_t highADC = read_adc(ADC_CHANNEL_2);// Pin 13
    int16_t midADC = read_adc(ADC_CHANNEL_1);// Pin 12
    int16_t lowADC = read_adc(ADC_CHANNEL_0);// Pin 11

    shiftPotiArr(mainADC, highADC, midADC, lowADC);

    filterPotis();

    // printf("X: %d\n", clamp12bit(lastPotiMain_X[0]));
    // printf("Y: %d\n\n", clamp12bit(lastPotiMain_Y[0]));
    main_volume = make_exponential(clamp12bit(lastPotiMain_Y[0]));  
    high_volume = clamp12bit(lastPotiHigh_Y[0]) * high_Limit * invertHigh;  
    mid_volume = clamp12bit(lastPotiMid_Y[0]) * mid_Limit * invertMid;  
    low_volume = clamp12bit(lastPotiLow_Y[0]) * low_Limit * invertLow;  

    // printf("Main: %d\n", main_volume);
}

void volumeControll(){
    while (true)
    {
        get_volumes();
        vTaskDelay(pdMS_TO_TICKS(20));
        // printf("main: %d\n", main_volume);
        // printf("high: %d\n", high_volume);
        // printf("mid: %d\n", mid_volume);
        // printf("low: %d\n", low_volume);
    }
}

void app_main(void)
{    
    init_i2s();
    init_adc();
    
    xTaskCreatePinnedToCore(volumeControll, "volumeControll", 4096, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(fiter, "filter", 4096, NULL, 0, NULL, 1);
}