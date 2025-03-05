#ifndef SENSORS_H
#define SENSORS_H

#include "hardware/adc.h"
#include "quadrature_encoder.pio.h"


typedef struct {
	PIO pio;
	uint sm;
	uint prev_count;
	uint64_t prev_time_us;
	uint ppm;
	int8_t direction;
} Encoder;

void sensor_init();
bool get_lift_hardstop();
Encoder *init_encoder(uint pinA, uint pinB, uint ppm, int direction);
float get_battery_voltage();

#endif
