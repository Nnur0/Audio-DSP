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

#define samples 480 // max: 960
#define EXAMPLE_BUFF_SIZE              samples * 2 * sizeof(int32_t)//64
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

volatile int16_t main_volume;
volatile int16_t high_volume;
volatile int16_t mid_volume;
volatile int16_t low_volume;

#define Q_SHIFT 30

typedef struct {
    int32_t x1, x2;
    int32_t y1, y2;
} biquad_state_t;

typedef struct {
    int32_t x1;
    int32_t y1;
} iir1_state_t;

// ---- High band ----
static biquad_state_t high_hp;
static biquad_state_t high_lp;

// ---- Mid band ----
static biquad_state_t midR_hp;
static biquad_state_t midR_lp;
static biquad_state_t midL_hp;
static biquad_state_t midL_lp;

// ---- Low band ----
static iir1_state_t  low_hp;   // 1st order HP @17Hz
static biquad_state_t low_lp;  // 2nd order LP

static inline int32_t IRAM_ATTR biquad_process(
    biquad_state_t *s,
    int32_t x,
    const int64_t *B,   // B[3]
    const int64_t *A    // A[2]
) {
    int64_t y =
        (int64_t)x     * B[0] +
        (int64_t)s->x1 * B[1] +
        (int64_t)s->x2 * B[2] +
        (int64_t)s->y1 * A[0] +
        (int64_t)s->y2 * A[1];

    y >>= Q_SHIFT;

    s->x2 = s->x1;
    s->x1 = x;
    s->y2 = s->y1;
    s->y1 = (int32_t)y;

    return (int32_t)y;
}

static inline int32_t IRAM_ATTR iir1_process(
    iir1_state_t *s,
    int32_t x,
    const int64_t *B,   // B[2]
    int64_t A           // scalar
) {
    int64_t y =
        (int64_t)x     * B[0] +
        (int64_t)s->x1 * B[1] +
        (int64_t)s->y1 * A;

    y >>= Q_SHIFT;

    s->x1 = x;
    s->y1 = (int32_t)y;

    return (int32_t)y;
}

static inline void IRAM_ATTR process_block(
    const int32_t *in_buf,
    int32_t *M_Buf,
    int32_t *H_L_Buf,
    size_t frames
) {
    for (size_t i = 0; i < frames; i++) {

        int32_t L = in_buf[2*i];
        int32_t R = in_buf[2*i + 1];

        // ---- Volume ----
        L = ((int64_t)L * main_volume)>> 12;
        R = ((int64_t)R * main_volume)>> 12;
        int64_t mono = ((int64_t)L + R) >> 1;

        int32_t midL = ((int64_t)L * mid_volume) >> 12;
        int32_t midR = ((int64_t)R * mid_volume) >> 12;
        int32_t high = (mono * high_volume) >> 12;
        int32_t low  = (mono * low_volume)  >> 12;

        // ---- High band ----
        int32_t h = biquad_process(&high_hp, high,
                                   HighCoeffs_B_HP, HighCoeffs_A_HP);
        h = biquad_process(&high_lp, h,
                           HighCoeffs_B_LP, HighCoeffs_A_LP);

        // ---- Mid band ----
        int32_t mR = biquad_process(&midR_hp, midR,
                                    MidCoeffs_B_HP, MidCoeffs_A_HP);
        mR = biquad_process(&midR_lp, mR,
                            MidCoeffs_B_LP, MidCoeffs_A_LP);

        int32_t mL = biquad_process(&midL_hp, midL,
                                    MidCoeffs_B_HP, MidCoeffs_A_HP);
        mL = biquad_process(&midL_lp, mL,
                            MidCoeffs_B_LP, MidCoeffs_A_LP);

        // ---- Low band ----
        int32_t l = iir1_process(&low_hp, low,
                                 LowCoeffs_B_HP, LowCoeffs_A_HP);
        l = biquad_process(&low_lp, l,
                           LowCoeffs_B_LP, LowCoeffs_A_LP);

        // ---- OUTPUT MAPPING ----
        M_Buf[2*i]     = mL;
        M_Buf[2*i + 1] = mR;

        H_L_Buf[2*i]     = h;
        H_L_Buf[2*i + 1] = l;
    }
}


static i2s_chan_handle_t                tx_chan_1;
static i2s_chan_handle_t                tx_chan_2;       
static i2s_chan_handle_t                rx_chan;

uint32_t clocks[4096] = {0};
uint64_t sum = 0;
int it = 0;

static inline void fiter(void *args)
{
    int32_t *in_buf = (int32_t *)malloc(EXAMPLE_BUFF_SIZE);
    int32_t *M_Buf = (int32_t *)malloc(EXAMPLE_BUFF_SIZE);
    int32_t *H_L_Buf = (int32_t *)malloc(EXAMPLE_BUFF_SIZE);
    
    size_t r_bytes = 0;
    size_t w_bytes = 0;

    ESP_ERROR_CHECK(i2s_channel_enable(rx_chan));
    ESP_ERROR_CHECK(i2s_channel_enable(tx_chan_1));
    ESP_ERROR_CHECK(i2s_channel_enable(tx_chan_2));    

    while (true) {
        if (i2s_channel_read(rx_chan, in_buf, EXAMPLE_BUFF_SIZE, &r_bytes, 1000) == ESP_OK) {
            // uint32_t start = esp_cpu_get_cycle_count();
            // if (r_bytes != EXAMPLE_BUFF_SIZE) printf("Bytes: %d\n", r_bytes);
            size_t frames = r_bytes / (2 * sizeof(int32_t));
            process_block(in_buf, M_Buf, H_L_Buf, frames);

            i2s_channel_write(tx_chan_1, M_Buf, r_bytes, &w_bytes, portMAX_DELAY);
            i2s_channel_write(tx_chan_2, H_L_Buf, r_bytes, &w_bytes, portMAX_DELAY);
            
            
            // pushNewX(in_buf);
            // calculateY(M_Buf, H_L_Buf);
            // mapOutput(M_Buf, H_L_Buf);

            // clocks[it] = esp_cpu_get_cycle_count() - start;
            // it++;
            // if (it == 1023)
            // {
            //     for (size_t i = 0; i < 1024; i++)
            //     {
            //         sum += clocks[i];
            //     }
            //     printf("Avg calc cycles: %llu\n", sum >> 10);
            //     //printf("Avg calc cycles: %lu\n", clocks[4096]);
            //     sum = 0;
            //     it = 0;
            // }

            //i2s_channel_write(tx_chan_1, in_buf, EXAMPLE_BUFF_SIZE, &w_bytes, 10); // <---- M_buf
            //i2s_channel_write(tx_chan_2, in_buf, EXAMPLE_BUFF_SIZE, &w_bytes, 1000);

            // in_buf[1] = in_buf[1] * -1;
            // in_buf[3] = in_buf[3] * -1;
            
            // memcpy(w_buf1, in_buf, EXAMPLE_BUFF_SIZE);
            // memcpy(w_buf2, in_buf, EXAMPLE_BUFF_SIZE);
            // i2s_channel_write(tx_chan_1, w_buf1, EXAMPLE_BUFF_SIZE, &w_bytes, 1000);
            // i2s_channel_write(tx_chan_2, w_buf2, EXAMPLE_BUFF_SIZE, &w_bytes, 1000);
        } 
        else {
            printf("Read Task: i2s read failed\n");
        }
        //vTaskDelay(pdMS_TO_TICKS(200));
    }
    free(in_buf);
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