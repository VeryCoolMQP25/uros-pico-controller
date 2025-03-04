#ifndef CONTROLS_H
#define CONTROLS_H

#include "pins.h"
#include "actuators.h"
#include "message_types.h"
#include "tunables.h"

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

void update_motor_encoders();

void twist_callback(const void*);

void raw_lift_callback(const void*);

void set_lift_power(int pwr);

void set_pid(bool);

bool do_drivetrain_pid_v(struct repeating_timer*);

void run_pid(Motor*, PIDController*);

DriveMode drive_mode_from_ros();
void lift_timeout_check();

void set_rsl(unsigned char);

void die();

#endif
