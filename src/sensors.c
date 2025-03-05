#include "sensors.h"
#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "pins.h"
#include "uart_logging.h"
#include "tunables.h"

const float voltage_convert_ratio = 3.3f / (1 << 12);

// return index of next unallocated PIO state machine
static int get_next_sm()
{
	// each bit represents availability of nth state machine
	static char pioAvail = 0xff;
	for (int i = 0; i < 8; i++)
	{
		// ith pio state machine is available
		if ((pioAvail >> i) & 0x1)
		{
			// mark SM as taken
			pioAvail &= ~(0x1 << i);
			return i;
		}
	}
	return -1;
}

void sensor_init(){
    // lift limit sensor
    gpio_init(LIFT_LIMIT_PIN);
	gpio_pull_up(LIFT_LIMIT_PIN);
	uart_log(LEVEL_DEBUG, "Starting Prog. I/O init");
	// load pio program into both PIOs
	pio_add_program(pio0, &quadrature_encoder_program);
	pio_add_program(pio1, &quadrature_encoder_program);
	// battery voltage sensor
	adc_init();
	adc_gpio_init(VBATT_SENSE_PIN);
	adc_select_input(0); // PIN26


}

Encoder *init_encoder(uint pinA, uint pinB, uint ppm, int direction)
{
	if (abs(pinA - pinB) != 1)
	{
		uart_log(LEVEL_ERROR, "Encoder pin A and B must be sequential! Aborting enc init");
		return NULL;
	}
	Encoder *enc = malloc(sizeof(Encoder));
	// find an available state machine
	int sm_idx = get_next_sm();
	if (sm_idx == -1)
	{
		return NULL;
	}
	if (sm_idx < 4)
	{
		enc->pio = pio0;
		enc->sm = sm_idx;
	}
	else
	{
		enc->pio = pio1;
		enc->sm = sm_idx - 4;
	}
	quadrature_encoder_program_init(enc->pio, enc->sm, pinA, 0);
	enc->prev_count = 0;
	enc->prev_time_us = 0;
	char asdf[40];
	snprintf(asdf, 40, "Encoder on pin (%d, %d) allocated SM %d", pinA, pinB, sm_idx);
	uart_log(LEVEL_DEBUG, asdf);
	enc->ppm = ppm;
	enc->direction = direction;
	return enc;
}

bool get_lift_hardstop()
{
	return !gpio_get(LIFT_LIMIT_PIN);
}

float get_battery_voltage()
{
    adc_select_input(0); // ensure batt input is selected
	uint16_t raw = adc_read();
	return raw * voltage_convert_ratio;
}
