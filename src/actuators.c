#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/timer.h"
#include "actuators.h"
#include "pins.h"
#include "uart_logging.h"
#include "tunables.h"

Motor drivetrain_left;
Motor drivetrain_right;
Motor lift_motor;

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

static Encoder *init_encoder(uint pinA, uint pinB, uint ppm, int direction)
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

void init_motor(char *name, int pin, Motor *motor_struct, bool (*killfunc)(void))
{
	motor_struct->name = name;
	motor_struct->pin_num = pin;
	gpio_set_function(pin, GPIO_FUNC_PWM);
	uint slice = pwm_gpio_to_slice_num(pin);
	motor_struct->slice_num = slice;
	// configure machinery to operate at 333Hz
	float div = clock_get_hz(clk_sys) / (TALON_PWM_FREQ * (TALON_PWM_WRAP - 1)); // should be ~25
	pwm_config config = pwm_get_default_config();
	pwm_config_set_clkdiv(&config, div);
	pwm_config_set_wrap(&config, TALON_PWM_WRAP);
	// set PWM to neutral before start
	pwm_set_gpio_level(pin, TALON_DEADCTR);
	motor_struct->curpower = 0;
	// init and start PWM channel
	pwm_init(slice, &config, true);
	motor_struct->enc = NULL;
	motor_struct->velocity = 0.0;
	motor_struct->position = 0.0;
	motor_struct->enabled = false;
	motor_struct->killfunc = killfunc;
	pwm_power(motor_struct, true);
	char debugbuff[100];
	snprintf(debugbuff, sizeof(debugbuff), "initialized motor %s with pin %d, slice %d", name, pin, slice);
	uart_log(LEVEL_INFO, debugbuff);
}

void init_motor_with_encoder(char *name, int pin, Motor *motor_struct, int enc_pin_A, int enc_pin_B, bool (*killfunc)(void), int ppm, int direction)
{
	init_motor(name, pin, motor_struct, killfunc);
	motor_struct->enc = init_encoder(enc_pin_A, enc_pin_B, ppm, direction);
	if (motor_struct->enc == NULL)
	{
		uart_log(LEVEL_ERROR, "Could not init encoder!");
	}
}

// sets the power level of a motor via PWM.
// accepts integer power level [-100, 100]
bool set_motor_power(Motor *motor, int power)
{
	//make sure motor is on
	pwm_power(motor, true);
	bool ok = true;
	if (abs(power) > MOTOR_POWER_MAX)
	{
		power = MOTOR_POWER_MAX * (power / abs(power));
		ok = false;
	}
	// check if motor has a defined cutout function
	if (motor->killfunc != NULL)
	{
		if (motor->killfunc())
		{
			uart_log(LEVEL_DEBUG, "motor kill funciton active");
			power = 0;
			ok = false;
		}
	}
	int setpoint = (TALON_DEADCTR + power * (TALON_FULL_FWD - TALON_DEADCTR) / 100);
	if (setpoint > TALON_FULL_FWD || setpoint < TALON_FULL_REV)
	{
		char dbgbuf[60];
		snprintf(dbgbuf, 60, "Rejecting pwm setpoint %d from power %d!", setpoint, power);
		uart_log(LEVEL_WARN, dbgbuf);
		return false;
	}
	pwm_set_gpio_level(motor->pin_num, setpoint);
	motor->curpower = power;
	return ok;
}



void init_all_motors()
{
	uart_log(LEVEL_DEBUG, "Starting Prog. I/O init");
	// load pio program into both PIOs
	pio_add_program(pio0, &quadrature_encoder_program);
	pio_add_program(pio1, &quadrature_encoder_program);
	uart_log(LEVEL_DEBUG, "Starting motor init");
	init_motor_with_encoder("DT_L", DT_L_PWM, &drivetrain_left, DT_L_ENCODER_A, DT_L_ENCODER_B, NULL, DT_ENCODER_PPM_L, -1);
	init_motor_with_encoder("DT_R", DT_R_PWM, &drivetrain_right, DT_R_ENCODER_A, DT_R_ENCODER_B, NULL, DT_ENCODER_PPM_R, 1);
	init_motor("LIFT", LIFT_PWM, &lift_motor, get_lift_hardstop);
	// initialize GPIO hardstop sensor
	gpio_init(LIFT_LIMIT_PIN);
	gpio_pull_up(LIFT_LIMIT_PIN);
	uart_log(LEVEL_DEBUG, "Motor & Encoder init finished.");
}

void kill_all_actuators()
{
	uart_log(LEVEL_INFO, "Actuators killed");
	set_motor_power(&drivetrain_right, 0);
	set_motor_power(&drivetrain_left, 0);
	set_motor_power(&lift_motor, 0);
}

void disable_all_actuators()
{
	uart_log(LEVEL_INFO, "Actuators disabled");
	pwm_power(&drivetrain_left,0);
	pwm_power(&drivetrain_right,0);
	pwm_power(&lift_motor,0);
}

void update_motor_encoder(Motor *mot)
{
	Encoder *encoder = mot->enc;
	// skip function if encoder did not init
	if (encoder == NULL)
	{
		uart_log(LEVEL_WARN, "Encoder is NULL!!");
		return;
	}
	int32_t raw = quadrature_encoder_get_count(encoder->pio, encoder->sm)*encoder->direction;
	int32_t dist_delta_pulse = raw - encoder->prev_count;
	uint64_t curtime = time_us_64();
	uint64_t delta_time_us = curtime - encoder->prev_time_us;
	encoder->prev_count = raw;
	encoder->prev_time_us = curtime;
	float pulse_per_sec = (1000000.0 * (float)dist_delta_pulse) / (float)delta_time_us;
	float velocity = pulse_per_sec / encoder->ppm;
	mot->velocity = velocity;
	mot->position = (float)raw / encoder->ppm;
}

bool get_lift_hardstop()
{
	return !gpio_get(LIFT_LIMIT_PIN);
}

// halt actual PWM signal
void pwm_power(Motor *motor, bool enable){
	if (enable == motor->enabled){
		return;
	}
	pwm_set_enabled(motor->slice_num, enable);
	motor->enabled = enable;
}
