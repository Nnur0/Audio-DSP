#include "driver/i2s_std.h"
#include "freertos/FreeRTOS.h"

#include "filter.h"
#include "volume.h"
#include "i2s.h"

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
biquad_state_t high_hp;
biquad_state_t high_lp;

// ---- Mid band ----
biquad_state_t midR_hp;
biquad_state_t midR_lp;
biquad_state_t midL_hp;
biquad_state_t midL_lp;

// ---- Low band ----
iir1_state_t  low_hp;   // 1st order HP @17Hz
biquad_state_t low_lp;  // 2nd order LP

int32_t IRAM_ATTR biquad_process(
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

inline int32_t IRAM_ATTR iir1_process(
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

void IRAM_ATTR process_block(
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


uint32_t clocks[4096] = {0};
uint64_t sum = 0;
int it = 0;

void filter(void *args)
{
    int32_t *in_buf = (int32_t *)malloc(EXAMPLE_BUFF_SIZE);
    int32_t *M_Buf = (int32_t *)malloc(EXAMPLE_BUFF_SIZE);
    int32_t *H_L_Buf = (int32_t *)malloc(EXAMPLE_BUFF_SIZE);
    
    size_t r_bytes = 0;
    size_t w_bytes = 0;

    //ESP_ERROR_CHECK(i2s_channel_enable(rx_chan));
    ESP_ERROR_CHECK(i2s_channel_enable(tx_chan_1));
    ESP_ERROR_CHECK(i2s_channel_enable(tx_chan_2));    

    while (true) {
        // if (i2s_channel_read(rx_chan, in_buf, EXAMPLE_BUFF_SIZE, &r_bytes, 1000) == ESP_OK) {
        //     // uint32_t start = esp_cpu_get_cycle_count();
        //     // if (r_bytes != EXAMPLE_BUFF_SIZE) printf("Bytes: %d\n", r_bytes);
        //     size_t frames = r_bytes / (2 * sizeof(int32_t));
        //     process_block(in_buf, M_Buf, H_L_Buf, frames);

        //     i2s_channel_write(tx_chan_1, M_Buf, r_bytes, &w_bytes, portMAX_DELAY);
        //     i2s_channel_write(tx_chan_2, H_L_Buf, r_bytes, &w_bytes, portMAX_DELAY);
            
            
        //     // pushNewX(in_buf);
        //     // calculateY(M_Buf, H_L_Buf);
        //     // mapOutput(M_Buf, H_L_Buf);

        //     // clocks[it] = esp_cpu_get_cycle_count() - start;
        //     // it++;
        //     // if (it == 1023)
        //     // {
        //     //     for (size_t i = 0; i < 1024; i++)
        //     //     {
        //     //         sum += clocks[i];
        //     //     }
        //     //     printf("Avg calc cycles: %llu\n", sum >> 10);
        //     //     //printf("Avg calc cycles: %lu\n", clocks[4096]);
        //     //     sum = 0;
        //     //     it = 0;
        //     // }

        //     //i2s_channel_write(tx_chan_1, in_buf, EXAMPLE_BUFF_SIZE, &w_bytes, 10); // <---- M_buf
        //     //i2s_channel_write(tx_chan_2, in_buf, EXAMPLE_BUFF_SIZE, &w_bytes, 1000);

        //     // in_buf[1] = in_buf[1] * -1;
        //     // in_buf[3] = in_buf[3] * -1;
            
        //     // memcpy(w_buf1, in_buf, EXAMPLE_BUFF_SIZE);
        //     // memcpy(w_buf2, in_buf, EXAMPLE_BUFF_SIZE);
        //     // i2s_channel_write(tx_chan_1, w_buf1, EXAMPLE_BUFF_SIZE, &w_bytes, 1000);
        //     // i2s_channel_write(tx_chan_2, w_buf2, EXAMPLE_BUFF_SIZE, &w_bytes, 1000);
        // } 
        // else {
        //     printf("Read Task: i2s read failed\n");
        // }
        printf("test\n");
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    free(in_buf);
    free(M_Buf);
    free(H_L_Buf);
    vTaskDelete(NULL);
}