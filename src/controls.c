#include <std_msgs/msg/int32_multi_array.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "uart_logging.h"
#include "controls.h"
#include "actuators.h"
//globals
static int left_power = 0;
static int right_power = 0;
unsigned long dt_raw_last_update = 0;

void dt_power_callback(const void *indata) {
    const std_msgs__msg__Int32MultiArray * msg = (const std_msgs__msg__Int32MultiArray *)indata;
    if (msg->data.size < 2){
    	uart_log(LEVEL_WARN,"drivetrain power message wrong len! Discarded.");
    }
	if (abs(msg->data.data[0]) > 100 || abs(msg->data.data[1]) > 100){
		uart_log(LEVEL_WARN,"bad drivetrain power message. Discarded.");
		return;
	}
    left_power = msg->data.data[0];
    right_power = msg->data.data[1];
    char printbuff[30];
    snprintf(printbuff, 30, "got new motor powers: %d %d", left_power, right_power);
    uart_log(LEVEL_DEBUG, printbuff);
    dt_raw_last_update = time_us_64();
}

int get_left_power(){
	return left_power;
}

int get_right_power(){
	return right_power;
}

void set_drivetrain_power(int l_power, int r_power){
	set_motor_power(&drivetrain_left, l_power);
	set_motor_power(&drivetrain_right, r_power);
}

void drivetrain_power_from_ros(){
	set_drivetrain_power(left_power, right_power);
}

void set_drivetrain_speed(float l_speed, float r_speed){
	uart_log(LEVEL_ERROR,"Speed control not implemented!");
}
