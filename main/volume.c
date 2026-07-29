#include "esp_adc/adc_oneshot.h"
#include "volume.h"
#include <tgmath.h>
#include "freertos/FreeRTOS.h"

volatile int main_volume;
volatile int high_volume;
volatile int mid_volume;
volatile int low_volume;

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

static const float PotiCoeffs_A[2] = { 1.64745998,  -0.70089678 }; // lp wn=2Hz fs=50Hz
static const float PotiCoeffs_B[3] = { 0.0133592, 0.0267184, 0.0133592 };

static int16_t lastPotiMain_Y[3] = {0};
static int16_t lastPotiHigh_Y[3] = {0};
static int16_t lastPotiMid_Y[3] = {0};
static int16_t lastPotiLow_Y[3] = {0};

static int16_t lastPotiMain_X[3] = {0};
static int16_t lastPotiHigh_X[3] = {0};
static int16_t lastPotiMid_X[3] = {0};
static int16_t lastPotiLow_X[3] = {0};

static void filterPoti(int16_t *Xarr, int16_t *Yarr){// should be changed to integer logic
    for (int i = 2; i >= 1; i--) {
        Yarr[i] = Yarr[i - 1];
    }

    Yarr[0] = (Xarr[0] * PotiCoeffs_B[0]) + (Xarr[1] * PotiCoeffs_B[1]) + (Xarr[2] * PotiCoeffs_B[2])
            + (Yarr[1] * PotiCoeffs_A[0]) + (Yarr[2] * PotiCoeffs_A[1]);
}

static void filterPotis(void){
    filterPoti(lastPotiMain_X, lastPotiMain_Y);
    filterPoti(lastPotiHigh_X, lastPotiHigh_Y);
    filterPoti(lastPotiMid_X, lastPotiMid_Y);
    filterPoti(lastPotiLow_X, lastPotiLow_Y);
}

static void shiftPotiArr(int16_t main, int16_t high, int16_t mid, int16_t low){
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

static int16_t read_adc(adc_channel_t channel) {
    int raw;
    adc_oneshot_read(adc_handle, channel, &raw);
    return raw;
}

static int16_t make_exponential(int16_t x){
    return 4095 * pow(((float)x/4095), 2);
}

static const float high_Limit = 0.45; // 2000/2^10 = 0.4883
static const float mid_Limit = 0.45; // 2000/2^10 = 0.4883
static const float low_Limit = 0.38; // 1580/2^10 = 0.3857

// 1 -> no change; -1 -> inverting
static const int invertHigh = 1;
static const int invertMid = 1;
static const int invertLow = 1;

static int16_t clamp12bit(int16_t x){
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

static void get_volumes(){
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