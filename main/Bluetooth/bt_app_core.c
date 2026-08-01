/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/xtensa_api.h"
#include "freertos/FreeRTOSConfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "bt_app_core.h"
// #include "driver/i2s.h"
#include "driver/i2s_std.h"
#include "freertos/ringbuf.h"

#include "../filter.h"
#include "../i2s.h"

#ifndef min
#define min(a, b) ((a) < (b) ? (a) : (b))
#endif

static void bt_app_task_handler(void *arg);
static bool bt_app_send_msg(bt_app_msg_t *msg);
static void bt_app_work_dispatched(bt_app_msg_t *msg);

static QueueHandle_t s_bt_app_task_queue = NULL;
static TaskHandle_t s_bt_app_task_handle = NULL;
static TaskHandle_t s_bt_i2s_task_handle = NULL;
static RingbufHandle_t s_ringbuf_i2s = NULL;
;

bool bt_app_work_dispatch(bt_app_cb_t p_cback, uint16_t event, void *p_params, int param_len, bt_app_copy_cb_t p_copy_cback)
{
    ESP_LOGD(BT_APP_CORE_TAG, "%s event 0x%x, param len %d", __func__, event, param_len);

    bt_app_msg_t msg;
    memset(&msg, 0, sizeof(bt_app_msg_t));

    msg.sig = BT_APP_SIG_WORK_DISPATCH;
    msg.event = event;
    msg.cb = p_cback;

    if (param_len == 0)
    {
        return bt_app_send_msg(&msg);
    }
    else if (p_params && param_len > 0)
    {
        if ((msg.param = malloc(param_len)) != NULL)
        {
            memcpy(msg.param, p_params, param_len);
            /* check if caller has provided a copy callback to do the deep copy */
            if (p_copy_cback)
            {
                p_copy_cback(&msg, msg.param, p_params);
            }
            return bt_app_send_msg(&msg);
        }
    }

    return false;
}

static bool bt_app_send_msg(bt_app_msg_t *msg)
{
    if (msg == NULL)
    {
        return false;
    }

    if (xQueueSend(s_bt_app_task_queue, msg, 10 / portTICK_PERIOD_MS) != pdTRUE)
    {
        ESP_LOGE(BT_APP_CORE_TAG, "%s xQueue send failed", __func__);
        // return false;
    }
    return true;
}

static void bt_app_work_dispatched(bt_app_msg_t *msg)
{
    if (msg->cb)
    {
        msg->cb(msg->event, msg->param);
    }
}

static void bt_app_task_handler(void *arg)
{
    bt_app_msg_t msg;
    for (;;)
    {
        if (pdTRUE == xQueueReceive(s_bt_app_task_queue, &msg, (TickType_t)portMAX_DELAY))
        {
            ESP_LOGD(BT_APP_CORE_TAG, "%s, sig 0x%x, 0x%x", __func__, msg.sig, msg.event);
            switch (msg.sig)
            {
            case BT_APP_SIG_WORK_DISPATCH:
                bt_app_work_dispatched(&msg);
                break;
            default:
                ESP_LOGW(BT_APP_CORE_TAG, "%s, unhandled sig: %d", __func__, msg.sig);
                break;
            } // switch (msg.sig)

            if (msg.param)
            {
                free(msg.param);
            }
        }
    }
}

void bt_app_task_start_up(void)
{
    s_bt_app_task_queue = xQueueCreate(10, sizeof(bt_app_msg_t));
    xTaskCreate(bt_app_task_handler, "BtAppT", 3072, NULL, configMAX_PRIORITIES - 3, &s_bt_app_task_handle);
    return;
}

void bt_app_task_shut_down(void)
{
    if (s_bt_app_task_handle)
    {
        vTaskDelete(s_bt_app_task_handle);
        s_bt_app_task_handle = NULL;
    }
    if (s_bt_app_task_queue)
    {
        vQueueDelete(s_bt_app_task_queue);
        s_bt_app_task_queue = NULL;
    }
}


static void bt_i2s_task_handler(void *arg)
{
    // size_t bytes_written = 0;
    uint8_t *data = NULL;
    size_t item_size = 0;

    int32_t *in_buf = (int32_t *)malloc(EXAMPLE_BUFF_SIZE);
    int32_t *M_Buf = (int32_t *)malloc(EXAMPLE_BUFF_SIZE);
    int32_t *H_L_Buf = (int32_t *)malloc(EXAMPLE_BUFF_SIZE);

    ESP_ERROR_CHECK(i2s_channel_enable(tx_chan_1));
    ESP_ERROR_CHECK(i2s_channel_enable(tx_chan_2));

    size_t w_bytes = 0;

    for (;;)
    {
        data = (uint8_t *)xRingbufferReceive(s_ringbuf_i2s, &item_size, (TickType_t)portMAX_DELAY);
        if (item_size != 0)
        {
            // todo data to in_buf
            int16_t *src = (int16_t *)data;
            size_t frames = item_size / (2 * sizeof(int16_t));

            for (size_t i = 0; i < frames * 2; i++)
            {
                in_buf[i] = ((int32_t)src[i]) << 16;
            }


            process_block(in_buf, M_Buf, H_L_Buf, frames);
            
            size_t out_bytes = frames * 2 * sizeof(int32_t);

            i2s_channel_write(tx_chan_1, M_Buf, out_bytes, &w_bytes, portMAX_DELAY);
            i2s_channel_write(tx_chan_2, H_L_Buf, out_bytes, &w_bytes, portMAX_DELAY);
        }
    }
}

void bt_i2s_task_start_up(void)
{
    s_ringbuf_i2s = xRingbufferCreate(/*32 8*/ 8 * 1024, RINGBUF_TYPE_BYTEBUF);
    if (s_ringbuf_i2s == NULL)
    {
        return;
    }

    xTaskCreatePinnedToCore(bt_i2s_task_handler, "BtI2ST", 2 * 4096, NULL, configMAX_PRIORITIES - 3, &s_bt_i2s_task_handle, 0);
    return;
}

void bt_i2s_task_shut_down(void)
{
    if (s_bt_i2s_task_handle)
    {
        vTaskDelete(s_bt_i2s_task_handle);
        s_bt_i2s_task_handle = NULL;
    }

    if (s_ringbuf_i2s)
    {
        vRingbufferDelete(s_ringbuf_i2s);
        s_ringbuf_i2s = NULL;
    }
}

size_t write_ringbuf(const uint8_t *data, size_t size)
{
    BaseType_t done = xRingbufferSend(s_ringbuf_i2s, (void *)data, size, (TickType_t)portMAX_DELAY);
    if (done)
    {
        return size;
    }
    else
    {
        return 0;
    }
}
