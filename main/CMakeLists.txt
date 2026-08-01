idf_component_register(
    SRCS 
        "filter.c" 
        "volume.c" 
        "i2s.c" 
        "main.c"
        "Bluetooth/bt_app_core.c"
        "Bluetooth/bt_app_av.c"
        "Bluetooth/SetupBluetooth.c"
                    
    PRIV_REQUIRES 
        bt #bredr_app_common_utils #bt_app_core_utils
        #a2dp_sink_common_utils
        nvs_flash driver
        ${EXTRA_COMPONENTS}

        esp_driver_i2s 
        esp_driver_gpio 
        esp_adc
                    
    INCLUDE_DIRS 
        "Bluetooth"
        "." 
        "C:/Users/savel/esp/v5.4/esp-idf/examples/peripherals/i2s/common"
    )
