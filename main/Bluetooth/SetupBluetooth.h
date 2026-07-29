#ifndef SETUP_BLUETOOTH_H
#define SETUP_BLUETOOTH_H

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "esp_log.h"

#include "esp_bt.h"
#include "bt_app_core.h"
#include "bt_app_av.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"
#include "driver/i2s_std.h"


/*************************************************************************************************************
 *
 *  Bluetooth Setup
 *  ===============
 *
 *  This library relies heavily on the official example repository by Espressif in the version 4.3.
 *  Check out the github repository for more information:
 *     https://github.com/espressif/esp-idf/blob/v4.3/examples/bluetooth/bluedroid/classic_bt/a2dp_sink/main
 *
 *  The library is untouched for the most part and handles all bluetooth relevant tasks for the whole project.
 * 
 *  The original library uses a deprecated version of I2S. Therefore, the library was adjusted to make use of
 *  the latest I2S version (driver/i2s_std.h).
 *  With the old I2S version, multi channel processing would not be possible.
 * 
 *  The following files are part of the bluetooth library:
 *     - bt_app_av.h/.c
 *     - bt_app_core.h/.c
 * 
 *  The interface between the bluetooth library and the rest of the project is in the file "bt_app_core.c"
 *  in the function "bt_i2s_task_handler".
 *
 *************************************************************************************************************/



/*********************************************************************
 *
 *  Espressif library specific function definitions
 *
 *********************************************************************/

/**
 *  event for handler "bt_av_hdl_stack_up
 */
enum
{
    BT_APP_EVT_STACK_UP = 0,
};

static void bt_av_hdl_stack_evt(uint16_t event, void *p_param);

void bt_app_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);

/*********************************************************************
 *
 *  SpatialAudioESP specific function definitions
 *
 *********************************************************************/

/**
 *  @brief initializes the bluetooth functionality by processing the Espressif workflow
 *  @details check out the code or the github repository linked above for more information
 */
void initBluetooth();

#endif