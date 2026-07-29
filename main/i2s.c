#include "i2s.h"

i2s_chan_handle_t                tx_chan_1;
i2s_chan_handle_t                tx_chan_2;       
i2s_chan_handle_t                rx_chan;

void init_i2s(void)
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