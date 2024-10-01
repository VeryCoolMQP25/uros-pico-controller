#ifndef ACTUATORS_H
#define ACTUATORS_H
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "quadrature_encoder.pio.h"
#include <stdio.h>

#define DT_L_ENCODER_A 6
#define DT_L_ENCODER_B 7
#define DT_R_ENCODER_A 8
#define DT_R_ENCODER_B 9

typedef struct {
	PIO pio;
	uint sm; 
} Encoder;

typedef struct {
	uint pin_num;
	uint slice_num;
	Encoder *enc;
} Motor;

void init_motors();

#endif
