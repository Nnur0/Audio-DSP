#include "i2s.h"
#include "volume.h"
#include "filter.h"
#include "freertos/FreeRTOS.h"
#include "Bluetooth/SetupBluetooth.h"

void app_main(void)
{    
    init_i2s();
    init_adc();

    initBluetooth();
    
    xTaskCreatePinnedToCore(volumeControll, "volumeControll", 4096, NULL, 2, NULL, 0);
    // xTaskCreatePinnedToCore(filter_old, "filter", 4096, NULL, 0, NULL, 1);
}