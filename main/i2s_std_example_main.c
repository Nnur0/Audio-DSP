#include <stdint.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "sdkconfig.h"
#include <tgmath.h>

#include <string.h>

#define BCLK   GPIO_NUM_4     
#define WS      GPIO_NUM_46
#define DIN     GPIO_NUM_6

#define DOUT_1    GPIO_NUM_9
#define DOUT_2    GPIO_NUM_5       

#define EXAMPLE_BUFF_SIZE              4 * sizeof(int32_t)//64
#define CLOCK   48000

static i2s_chan_handle_t                tx_chan_1;
static i2s_chan_handle_t                tx_chan_2;       
static i2s_chan_handle_t                rx_chan;        

//-----High Coeffs------//
int32_t HighCoeffs_A_HP[2] = { 1395166348, -523407902 }; //hp3887 30bit
int32_t HighCoeffs_B_HP[3] = { 748297431, -1496594861, 748297431 };

int32_t HighCoeffs_A_LP[2] = { -1947171197, -892451235 }; //lp23000 30bit
int32_t HighCoeffs_B_LP[3] = { 978532682, 1957065364, 978532682 };

//-----Mid Coeffs------//
int32_t MidCoeffs_A_HP[2] = { 2106887749, -1033695662 }; //hp203 30bit
int32_t MidCoeffs_B_HP[3] = { 1053623222, -2107246444, 1053623222 };

int32_t MidCoeffs_A_LP[2] = { 1470521965, -563754365 }; // lp3483Hz 30bit
int32_t MidCoeffs_B_LP[3] = { 41814974, 83629947, 41814974 };

//-----Low Coeffs------//
float LowCoeffs_A_HP[2] = { 1.99685296051001, -0.99685790466517 }; //hp17
float LowCoeffs_B_HP[3] = { 0.998427716293794, -1.99685543258759, 0.998427716293794 }; 

int32_t LowCoeffs_A_LP[2] = { 2111633663, -1038474472 }; //lp178 30bit 
int32_t LowCoeffs_B_LP[3] = { 143309, 286617, 143309 };

// a * y
// b * x


int64_t lastXR[6] = {0};
int64_t lastXL[6] = {0};

int64_t lastYHigh_0[6] = {0};
int64_t lastYHigh_1[6] = {0};

int64_t lastYMid_R_0[6] = {0};
int64_t lastYMid_R_1[6] = {0};

int64_t lastYMid_L_0[6] = {0};
int64_t lastYMid_L_1[6] = {0};

int64_t lastYLow_0[6] = {0};
int64_t lastYLow_1[6] = {0};

const int shift = 30;
uint64_t  clocks[4096];
uint64_t sum = 0;
int it = 0;
int32_t max = 2147483647;          //real max: 2147418112 (3.12V). Using 2^31 - 1 (3.12V)
void calculateY(int32_t *M_Buf, int32_t *H_L_Buf){

    for (size_t i = 5; i >= 2; i--) {
        lastYHigh_0[i] = lastYHigh_0[i - 1];
        lastYHigh_1[i] = lastYHigh_1[i - 1];

        lastYMid_R_0[i] = lastYMid_R_0[i - 1];
        lastYMid_R_1[i] = lastYMid_R_1[i - 1];

        lastYMid_L_0[i] = lastYMid_L_0[i - 1];
        lastYMid_L_1[i] = lastYMid_L_1[i - 1];

        lastYLow_0[i] = lastYLow_0[i - 1];
        lastYLow_1[i] = lastYLow_1[i - 1];
    }

    //-----High Calculation------//
    //--HP_203Hz--//
    lastYHigh_0[1] = ((lastXR[1] * HighCoeffs_B_HP[0]) >> shift) + ((lastXR[2] * HighCoeffs_B_HP[1]) >> shift) + ((lastXR[3] * HighCoeffs_B_HP[2]) >> shift)
         + ((lastYHigh_0[2] * HighCoeffs_A_HP[0]) >> shift) + ((lastYHigh_0[3] * HighCoeffs_A_HP[1]) >> shift);

    lastYHigh_0[0] = ((lastXR[0] * HighCoeffs_B_HP[0]) >> shift) + ((lastXR[1] * HighCoeffs_B_HP[1]) >> shift) + ((lastXR[2] * HighCoeffs_B_HP[2]) >> shift)
         + ((lastYHigh_0[1] * HighCoeffs_A_HP[0]) >> shift) + ((lastYHigh_0[2] * HighCoeffs_A_HP[1]) >> shift);

    //--LP_3483Hz--//
    lastYHigh_1[1] = ((lastYHigh_0[1] * HighCoeffs_B_LP[0]) >> shift) + ((lastYHigh_0[2] * HighCoeffs_B_LP[1]) >> shift) + ((lastYHigh_0[3] * HighCoeffs_B_LP[2]) >> shift)
         + ((lastYHigh_1[2] * HighCoeffs_A_LP[0]) >> shift) + ((lastYHigh_1[3] * HighCoeffs_A_LP[1]) >> shift);

    lastYHigh_1[0] = ((lastYHigh_0[0] * HighCoeffs_B_LP[0]) >> shift) + ((lastYHigh_0[1] * HighCoeffs_B_LP[1]) >> shift) + ((lastYHigh_0[2] * HighCoeffs_B_LP[2]) >> shift)
         + ((lastYHigh_1[1] * HighCoeffs_A_LP[0]) >> shift) + ((lastYHigh_1[2] * HighCoeffs_A_LP[1]) >> shift);



    //-----Mid Calculation------//
    //--HP_203Hz--//
    //--Right--//
    lastYMid_R_0[1] = ((lastXR[1] * MidCoeffs_B_HP[0]) >> shift) + ((lastXR[2] * MidCoeffs_B_HP[1]) >> shift) + ((lastXR[3] * MidCoeffs_B_HP[2]) >> shift)
         + ((lastYMid_R_0[2] * MidCoeffs_A_HP[0]) >> shift) + ((lastYMid_R_0[3] * MidCoeffs_A_HP[1]) >> shift);

    lastYMid_R_0[0] = ((lastXR[0] * MidCoeffs_B_HP[0]) >> shift) + ((lastXR[1] * MidCoeffs_B_HP[1]) >> shift) + ((lastXR[2] * MidCoeffs_B_HP[2]) >> shift)
         + ((lastYMid_R_0[1] * MidCoeffs_A_HP[0]) >> shift) + ((lastYMid_R_0[2] * MidCoeffs_A_HP[1]) >> shift);
    //--Left--//
    lastYMid_L_0[1] = ((lastXL[1] * MidCoeffs_B_HP[0]) >> shift) + ((lastXL[2] * MidCoeffs_B_HP[1]) >> shift) + ((lastXL[3] * MidCoeffs_B_HP[2]) >> shift)
         + ((lastYMid_L_0[2] * MidCoeffs_A_HP[0]) >> shift) + ((lastYMid_L_0[3] * MidCoeffs_A_HP[1]) >> shift);

    lastYMid_L_0[0] = ((lastXL[0] * MidCoeffs_B_HP[0]) >> shift) + ((lastXL[1] * MidCoeffs_B_HP[1]) >> shift) + ((lastXL[2] * MidCoeffs_B_HP[2]) >> shift)
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
    //--HP_17Hz--//
    lastYLow_0[1] = (lastXR[1] * LowCoeffs_B_HP[0]) + (lastXR[2] * LowCoeffs_B_HP[1]) + (lastXR[3] * LowCoeffs_B_HP[2])
            + (lastYLow_0[2] * LowCoeffs_A_HP[0]) + (lastYLow_0[3] * LowCoeffs_A_HP[1]);

    lastYLow_0[0] = (lastXR[0] * LowCoeffs_B_HP[0]) + (lastXR[1] * LowCoeffs_B_HP[1]) + (lastXR[2] * LowCoeffs_B_HP[2])
            + (lastYLow_0[1] * LowCoeffs_A_HP[0]) + (lastYLow_0[2] * LowCoeffs_A_HP[1]); 

    //--LP_178Hz--//
    lastYLow_1[1] = ((lastYLow_0[1] * LowCoeffs_B_LP[0]) >> shift) + ((lastYLow_0[2] * LowCoeffs_B_LP[1]) >> shift) + ((lastYLow_0[3] * LowCoeffs_B_LP[2]) >> shift)
         + ((lastYLow_1[2] * LowCoeffs_A_LP[0]) >> shift) + ((lastYLow_1[3] * LowCoeffs_A_LP[1]) >> shift);

    lastYLow_1[0] = ((lastYLow_0[0] * LowCoeffs_B_LP[0]) >> shift) + ((lastYLow_0[1] * LowCoeffs_B_LP[1]) >> shift) + ((lastYLow_0[2] * LowCoeffs_B_LP[2]) >> shift)
         + ((lastYLow_1[1] * LowCoeffs_A_LP[0]) >> shift) + ((lastYLow_1[2] * LowCoeffs_A_LP[1]) >> shift);

    

    //clocks[it] = esp_cpu_get_cycle_count() - clocks[it] -1;

    // lastYHigh_1[0] = lastXR[0];
    // lastYHigh_1[1] = lastXR[1];

    // lastYMid_R_1[0] = lastXR[0];
    // lastYMid_R_1[1] = lastXR[1];

    // lastYMid_L_1[0] =lastXR[0];
    // lastYMid_L_1[1] =lastXR[1];

    //printf("calc cycles: %llu\n", clocks[it]);
    // it++;
    // if (it == 4095)
    // {
    //     for (size_t i = 0; i < 4096; i++)
    //     {
    //         sum += clocks[i];
    //     }
    //     printf("Avg calc cycles: %llu\n", sum >> 12);
    //     sum = 0;
    //     it = 0;
    // }

    // if (lastXR[1] > max)
    // {
    //     max = lastXR[1];
    //     printf("new Max: %ld\n", max);
    // }
}

void pushNewX(int32_t *newSample){
    for (size_t i = 5; i >= 2; i--)
    {
        lastXR[i] = lastXR[i-2];
        lastXL[i] = lastXL[i-2];
    }
    lastXL[1] = newSample[0];
    lastXR[1] = newSample[1];
    
    lastXR[0] = newSample[2];
    lastXL[0] = newSample[3];
}

void mapOutput(int32_t *M_Buf, int32_t *H_L_Buf){
    H_L_Buf[0] = lastYHigh_1[1];
    M_Buf[0] = lastYMid_R_1[1];
    M_Buf[1] = lastYMid_L_1[1];
    H_L_Buf[1] = lastYLow_1[1];

    H_L_Buf[2] = lastYHigh_1[0];
    M_Buf[2] = lastYMid_R_1[0];
    M_Buf[3] = lastYMid_L_1[0];
    H_L_Buf[3] = lastYLow_1[0];
}

static void i2s_example_read_task(void *args)
{
    int32_t *r_buf = (int32_t *)malloc(EXAMPLE_BUFF_SIZE);
    int32_t *w_buf1 = (int32_t *)malloc(EXAMPLE_BUFF_SIZE);
    
    int32_t *testBuf = (int32_t *)malloc(EXAMPLE_BUFF_SIZE);
    int32_t *w_buf2 = (int32_t *)malloc(EXAMPLE_BUFF_SIZE);

    assert(r_buf); 
    assert(w_buf2);
    assert(w_buf1);
    assert(testBuf);

    int32_t *M_Buf = (int32_t *)malloc(EXAMPLE_BUFF_SIZE);
    int32_t *H_L_Buf = (int32_t *)malloc(EXAMPLE_BUFF_SIZE);
        
    assert(M_Buf); 
    assert(H_L_Buf);

    size_t r_bytes = 0;
    size_t w_bytes = 0;

    ESP_ERROR_CHECK(i2s_channel_enable(rx_chan));
    ESP_ERROR_CHECK(i2s_channel_enable(tx_chan_1));
    ESP_ERROR_CHECK(i2s_channel_enable(tx_chan_2));
    while (1) {
        if (i2s_channel_read(rx_chan, r_buf, EXAMPLE_BUFF_SIZE, &r_bytes, 1000) == ESP_OK) {

            pushNewX(r_buf);
            calculateY(M_Buf, H_L_Buf);
            mapOutput(M_Buf, H_L_Buf);

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
    //free(r_buf2);
    vTaskDelete(NULL);
}

static void i2s_example_init_std_duplex(void)
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

void app_main(void)
{    
    i2s_example_init_std_duplex();
    //xTaskCreate(i2s_example_read_task, "i2s_example_read_task", 4096, NULL, 0, NULL);
    xTaskCreatePinnedToCore(i2s_example_read_task, "i2s_example_read_task", 4096, NULL, 0, NULL, 1);
}