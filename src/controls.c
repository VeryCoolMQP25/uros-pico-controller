#include <complex.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <stdio.h>
#include <math.h>
#include "actuators.h"
#include "pico/stdlib.h"
#include "pico/malloc.h"
#include "uart_logging.h"
#include "controls.h"
#include "tunables.h"
#include "message_types.h"
#include "nav.h"

// globals
static PIDController pid_v_left;
static PIDController pid_v_right;
static DriveMode drive_mode_global = dm_halt;
unsigned long last_twist_msg = 0;
unsigned long last_lift_msg = 0;
unsigned long last_pid_cb_run = 0;
bool do_pid_debug = false;
bool do_pid = false;


void pid_setup()
{
	pid_v_left = init_pid_control(PID_DT_V_KP, PID_DT_V_KI, PID_DT_V_KD, PID_DT_TOL, pid_velocity);
	pid_v_right = init_pid_control(PID_DT_V_KP, PID_DT_V_KI, PID_DT_V_KD, PID_DT_TOL, pid_velocity);
}
/* change PID values in runtime
*/
void calibrate_pid(char which, float k){
	switch (which)
	{
	case 'p':
		pid_v_left.Kp = k;
		pid_v_right.Kp = k;
		break;
	case 'i':
		pid_v_left.Ki = k;
		pid_v_right.Ki = k;
		break;
	case 'd':
		pid_v_left.Kd = k;
		pid_v_right.Kd = k;
		break;
	default:
		return;
	}
	do_pid_debug = true;
	char dbg[24];
	snprintf(dbg, 24, "set K%c to %f", which, k);
	uart_log(LEVEL_DEBUG, dbg);
}

PIDController init_pid_control(float Kp, float Ki, float Kd, float tolerance, PIDMode pmode)
{
	PIDController controller;
	controller.Kp = Kp;
	controller.Ki = Ki;
	controller.Kd = Kd;
	controller.previous_error = 0.0;
	controller.integral = 0.0;
	controller.target = 0.0;
	controller.tolerance = tolerance;
	controller.mode = pmode;
	controller.last_tick_us = time_us_64();
	critical_section_init(&controller.cs);
	return controller;
}

void update_motor_encoders(){
	update_motor_encoder(&drivetrain_left);
	update_motor_encoder(&drivetrain_right);
	update_odometry();
}

void twist_callback(const void *msgin)
{
	const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
	float linear = msg->linear.x;	// m/s
	float angular = msg->angular.z; // rad/sec
	last_twist_msg = time_us_64();
	critical_section_enter_blocking(&pid_v_right.cs);
	critical_section_enter_blocking(&pid_v_left.cs);
	// boost angular velocity when rotating in place
	if (fabs(linear) < ROTATE_INPLACE_THRESHOLD)
	{
		pid_v_right.Ki = PID_DT_V_KI * ROTATE_INPLACE_IMULT;
		pid_v_right.Kp = PID_DT_V_KP * ROTATE_INPLACE_PMULT;

		pid_v_left.Ki = PID_DT_V_KI * ROTATE_INPLACE_IMULT;
		pid_v_left.Kp = PID_DT_V_KP * ROTATE_INPLACE_PMULT;		
	}
	else {
		pid_v_right.Ki = PID_DT_V_KI;
		pid_v_right.Kp = PID_DT_V_KP;
		
		pid_v_left.Ki = PID_DT_V_KI;
		pid_v_left.Kp = PID_DT_V_KP;
	}
	pid_v_left.target = linear - (WHEELBASE_M * angular) / 2;
	critical_section_exit(&pid_v_left.cs);
	
	pid_v_right.target = linear + (WHEELBASE_M * angular) / 2;
	critical_section_exit(&pid_v_right.cs);
}

void lift_callback(const void *msgin)
{
	const std_msgs__msg__Int16 *msg = (const std_msgs__msg__Int16 *)msgin;
	int pow = (int)(msg->data);
	set_lift_power(pow);
	last_lift_msg = time_us_64();
}

void set_lift_power(int pwr)
{
	if (get_lift_hardstop() && pwr < 0)
	{
		pwr = 0;
		uart_log(LEVEL_INFO, "Halted downward lift motion due to limit sw");
	}
	pwm_power(&lift_motor, true);
	set_motor_power(&lift_motor, pwr);
}

void set_pid(bool setpoint){
	do_pid = setpoint;
	set_rsl(!setpoint);
}

bool do_drivetrain_pid_v(__unused struct repeating_timer *tmr)
{
	static uint8_t led_state = 0;
	static bool led_lit = false;
	last_pid_cb_run = time_us_64();
	if (do_pid){
		run_pid(&drivetrain_left, &pid_v_left);
		run_pid(&drivetrain_right, &pid_v_right);
	}
	gpio_put(LED_PIN, 0);
	if (led_state++ >= 3){
		led_state = 0;
		gpio_put(LED_PIN, led_lit);
		led_lit = !led_lit;
	}
	return 1; // instruct timer to keep repeating
}

uint64_t get_last_pid_update(){
	return last_pid_cb_run;
}

void run_pid(Motor *motor, PIDController *pid)
{
	static unsigned short printctr = 0;
	uint64_t curtime = time_us_64();
	float delta_time_s = (curtime - pid->last_tick_us) / 1000000.0;
	if (delta_time_s < 1e-6 || delta_time_s > 0.1)
	{
		uart_log(LEVEL_WARN, "PID delta time out of bounds!");
		delta_time_s = 1e-6;
	}
	critical_section_enter_blocking(&pid->cs);
	pid->last_tick_us = curtime;
	float error;
	switch (pid->mode)
	{
	case pid_velocity:
		error = pid->target - motor->velocity;
		pid->integral += error * delta_time_s;
		// reset integral when stopping
		if (fabs(error) < pid->tolerance && fabs(pid->target) < 2.0*pid->tolerance){
			pid->integral = 0.0;
		}
		if (fabs(pid->integral) > PID_DT_I_CAP)
		{
			if (printctr > 5){
				uart_log_nonblocking(LEVEL_INFO, "Capping integral");
			}
			pid->integral = PID_DT_I_CAP * (pid->integral / fabs(pid->integral));
		}
		break;
	case pid_position:
		pid->integral += error * delta_time_s;
		break;
	default:
		uart_log(LEVEL_ERROR, "Invalid PID mode!");
		critical_section_exit(&pid->cs);
		return;
	}

	float cur_P = pid->Kp * error;
	float cur_I = pid->Ki * pid->integral;
	float cur_D = pid->Kd * ((error - pid->previous_error) / delta_time_s);

	// PID output
	int output = 100 * (cur_P + cur_I + cur_D);
	if (output > 100)
	{
		output = 100;
	}
	else if (output < -100)
	{
		output = -100;
	}
	if (do_pid_debug && printctr++ == 4){
		char debugbuff[110];
		snprintf(debugbuff, 110, "[%s] P:%f, I:%f, D:%f\t\ttgt: %f, act: %f\t\tPwr: %d%%",
		motor->name, cur_P, cur_I, cur_D, pid->target, motor->velocity, output);
		uart_log_nonblocking(LEVEL_DEBUG, debugbuff);
		printctr = 0;
	}
	pid->previous_error = error;
	critical_section_exit(&pid->cs);
	set_motor_power(motor, output);
}

void reset_integral(){
	pid_v_left.integral = 0.0;
	pid_v_right.integral = 0.0;
	drivetrain_left.position = 0.0;
	drivetrain_right.position = 0.0;
	pid_v_left.target = 0.0;
	pid_v_right.target = 0.0;
	pid_v_left.previous_error = 0.0;
	pid_v_right.previous_error = 0.0;
	pid_v_left.last_tick_us = time_us_64();
	pid_v_right.last_tick_us = time_us_64();
}

DriveMode drive_mode_from_ros()
{
	// disable timeout if in PID debug mode

	if (time_us_64() - last_twist_msg > DRIVETRAIN_TIMEOUT)
	{
		return dm_halt;
	}
	return dm_twist;
}

// check if previous lift command was more than 500ms ago, and de-init lift PWM signaling
void lift_timeout_check(){
	static bool doPrint = true;
	if (time_us_64() - last_lift_msg > 500000){
		pwm_power(&lift_motor, false);
		if (doPrint){
			uart_log(LEVEL_INFO, "disabling lift");
			doPrint = false;
		}
	}
	else if (!doPrint){
		doPrint = true;
	}
}

// prevent repeated setting of same val
void set_rsl(unsigned char value){
	static unsigned char prev = 1;
	if (value != prev){
		gpio_put(RSL_PIN, !value);
		prev = value;
	}
}

void die()
{
	// kill drivtrain control core (prevent WDT updates)
	multicore_lockout_start_blocking();
	kill_all_actuators();
	while (1)
	{
	   disable_all_actuators();
	   uart_log(LEVEL_ERROR, "KILL ME");
	}
}
