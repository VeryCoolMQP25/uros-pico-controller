#ifndef ACTUATORS_H
#define ACTUATORS_H

#include "pins.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "quadrature_encoder.pio.h"
#include <stdint.h>

/** Talon SR speed controller specifics
 * 333Hz signal
 * 2.037ms = full "forward"
 * 1.539ms = the "high end" of the deadband range
 * 1.513ms = center of the deadband range (off)
 * 1.487ms = the "low end" of the deadband range
 * 0.989ms = full "reverse"
*/

#define TALON_PWM_FREQ 333
#define TALON_PWM_WRAP 15015
#define TALON_FULL_FWD 10184
#define TALON_FULL_REV 4945
#define TALON_DEADCTR  7565

/* Servo PWM signaling:
* 50Hz signal
* 20ms period
* 500us = full "forward"
* 1500us = center of the deadband range (off)
* 2500us = full "reverse"
*/
#define SERVO_PWM_FREQ     50
#define SERVO_PWM_WRAP     24999
#define SERVO_MIN_PWM      1250  // Servo 1000µs in counter ticks (1000 * 25000 / 20000)
#define SERVO_MAX_PWM      3125
#define SERVO_MIN_POS_DEG  0
#define SERVO_MAX_POS_DEG  180

typedef struct {
	PIO pio;
	uint sm;
	uint prev_count;
	uint64_t prev_time_us;
	uint ppm;
	int8_t direction;
} Encoder;

typedef struct {
	char *name;
	uint pin_num;
	uint slice_num;
	int curpower;
	Encoder *enc;
	float velocity;
	float position;
	bool enabled;
	bool (*killfunc)(void);
} Motor;

extern Motor drivetrain_left;
extern Motor drivetrain_right;
extern Motor lift_motor;


void init_all_motors();
bool set_motor_power(Motor*, int);
void kill_all_actuators();
void disable_all_actuators();


void update_motor_encoder(Motor*);
bool get_lift_hardstop();
void pwm_power(Motor *motor, bool enable);
#endif
