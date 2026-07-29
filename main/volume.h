#pragma once

extern volatile int main_volume;
extern volatile int high_volume;
extern volatile int mid_volume;
extern volatile int low_volume;

void volumeControll();
void init_adc();