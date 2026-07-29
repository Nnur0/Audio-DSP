idf_component_register(SRCS "filter.c" "volume.c" "i2s.c" "main.c"
                    PRIV_REQUIRES esp_driver_i2s esp_driver_gpio esp_adc
                    INCLUDE_DIRS "." "C:/Users/savel/esp/v5.4/esp-idf/examples/peripherals/i2s/common")
