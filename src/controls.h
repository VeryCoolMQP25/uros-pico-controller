#ifndef CONTROLS_H
#define CONTROLS_H

#define DRIVETRAIN_TIMEOUT 200000 //microseconds

#include "pins.h"
#include "actuators.h"
#include "message_types.h"

typedef enum {
	dm_raw,
	dm_twist,
	dm_halt
} DriveMode;

typedef enum {
	pid_position,
	pid_velocity
} PIDMode;

typedef struct {
	float Kp;
	float Ki;
	float Kd;
	float previous_error;
	float integral;
	float tolerance;
	float target;
	PIDMode mode;
	uint64_t last_tick_us;
} PIDController;

void pid_setup();

void calibrate_pid(char, float);

PIDController init_pid_control(float Kp, float Ki, float Kd, float tolerance, PIDMode pmode);

void twist_callback(const void*);

void raw_lift_callback(const void*);

void set_lift_power(int pwr);

void do_drivetrain_pid_v();

void run_pid(Motor*, PIDController*);

DriveMode drive_mode_from_ros();

void lift_timeout_check();

void die();

#endif
