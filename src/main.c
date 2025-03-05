#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <math.h>
#include "hardware/watchdog.h"
#include "pico/stdlib.h"
#include "pico/malloc.h"
#include "pico_ros_usb.h"
#include "uart_logging.h"
#include "actuators.h"
#include "controls.h"
#include "pins.h"
#include "message_types.h"
#include "nav.h"
#include "sensors.h"

// version numbering: <term>-<day>.ver
#define VERSION "B-53.3.1"
#define RCL_CONTEXT_COUNT 6

// globals
const char *namespace = "";
DriveMode drive_mode = dm_halt;
uint8_t do_encoder_debug = 0;
uint8_t do_core1_healthcheck = 0;
int core1_stage = 0;

/// support for encoder publisher
rcl_publisher_t odometry_publisher;
nav_msgs__msg__Odometry odometry_message;
// support for battery voltage publisher
rcl_publisher_t battery_publisher;
std_msgs__msg__Float32 battery_message;

// callback to publish encoder data (processed into timestamped twists)

void publish_encoder(rcl_timer_t *timer, int64_t last_call_time)
{
    watchdog_update();
	//fill in up-to-date values for odom
	populate_odometry(&odometry_message);
	// Publish messages
	if (rcl_publish(&odometry_publisher, &odometry_message, NULL))
	{
		uart_log(LEVEL_WARN, "Odom publish failed!");
	}
	if (do_encoder_debug)
	{
	   char encoderbuff[100];
		snprintf(encoderbuff, sizeof(encoderbuff), "Encoder positions: %f %f\n", drivetrain_left.position, drivetrain_right.position);
		uart_log(LEVEL_DEBUG, encoderbuff);
	}
}

void publish_battery(rcl_timer_t *timer, int64_t last_call_time)
{
    battery_message.data = get_battery_voltage();
	if (rcl_publish(&battery_publisher, &battery_message, NULL))
	{
		uart_log(LEVEL_WARN, "Battery publish failed!");
	}
}

// checks if we have comms with serial agent
void check_connectivity(rcl_timer_t *timer, int64_t last_call_time)
{
	// uart_log(LEVEL_DEBUG, "connectivity CB run");
	bool ok = (rmw_uros_ping_agent(50, 1) == RCL_RET_OK);
	gpio_put(LED_PIN, ok);
	if (!ok)
	{
		drive_mode = dm_halt;
		uart_log(LEVEL_ERROR, "Disconnected from uROS!");
		die();
	}
	watchdog_update();
}

void uart_input_handler(rcl_timer_t *timer, int64_t last_call_time)
{
	static char recbuff[UART_READBUFF_SIZE];
	if (uart_getline(recbuff))
	{
		switch (recbuff[0])
		{
		// intentional fallthroughs
		case 'p':
		case 'i':
		case 'd':
			if (strlen(recbuff) < 2)
			{
				uart_log(LEVEL_WARN, "Bad PID command! ignoring...");
				return;
			}
			calibrate_pid(recbuff[0], atoff(recbuff + 1));
			break;
		case 'R':
			reset_odometry();
			do_encoder_debug = 1;
			break;
		case 'v':
		{
			char buf[60];
			snprintf(buf, sizeof(buf), "Battery voltage: %fV", get_battery_voltage());
			uart_log(LEVEL_INFO, buf);
			break;
		}
		case 'h':
		{
	        uart_log(LEVEL_WARN, "System is going down now!");
			die();
			break;
		}
		case 't':
		    if (do_core1_healthcheck == 2){
				uart_log(LEVEL_ERROR, "Core1 exited");
				}
		    do_core1_healthcheck = 1;
			char stage[50];
			snprintf(stage, 50, "Core1 check requested, stage: %d",core1_stage);
			uart_log(LEVEL_INFO, stage);
			break;
		default:
			uart_log(LEVEL_WARN, "Unrecognized command!");
			uart_log(LEVEL_DEBUG, recbuff);
		}
	}
}

// creates and returns a timer, configuring it to call specified callback. Returns timer handle
rcl_timer_t *create_timer_callback(rclc_executor_t *executor, rclc_support_t *support, uint period_ms, rcl_timer_callback_t cb)
{
	rcl_timer_t *timer = malloc(sizeof(rcl_timer_t));
	rclc_timer_init_default(timer, support, RCL_MS_TO_NS(period_ms), cb);
	rclc_executor_add_timer(executor, timer);
	uart_log(LEVEL_DEBUG, "registered timer cb");
	return timer;
}

void core1task()
{
    core1_stage = 0;
	uart_log(LEVEL_DEBUG, "Started core 1 task");
	multicore_lockout_victim_init();
	int motor_kill_ctr = 0;
	alarm_pool_t *pid_pool = malloc(sizeof(alarm_id_t));
	pid_pool = alarm_pool_create_with_unused_hardware_alarm(1);
	repeating_timer_t *pid_timer = malloc(sizeof(repeating_timer_t));
	if (!alarm_pool_add_repeating_timer_ms(pid_pool, 10, do_drivetrain_pid_v, NULL, pid_timer)){
		uart_log(LEVEL_ERROR, "Cannot init PID timer!!");
	}
	else {
		uart_log(LEVEL_INFO, "Started recurring PID interrupt timer");
	}
	while (true)
	{
	   core1_stage = 1;
	    if(do_core1_healthcheck){
			uart_log(LEVEL_INFO, "Core1 is up!");
			do_core1_healthcheck = false;
			}
		drive_mode = drive_mode_from_ros();
		core1_stage = 2;
		lift_timeout_check();
		core1_stage = 3;
		update_motor_encoders();
		core1_stage = 4;
		switch (drive_mode)
		{
		case dm_raw:
		{
			set_pid(false);
			set_motor_power(&drivetrain_left, 55);
			set_motor_power(&drivetrain_right, 55);
			update_motor_encoders(&drivetrain_left);
			update_motor_encoders(&drivetrain_right);
			char velocity_dbg[30];
			snprintf(velocity_dbg, 30, "Velocities: (%f, %f)", drivetrain_left.velocity, drivetrain_right.velocity);
			uart_log(LEVEL_DEBUG, velocity_dbg);
			motor_kill_ctr = 0;
			break;
		}
		case dm_halt:
			set_pid(false);
			if (motor_kill_ctr++ < 500)
			{
				set_motor_power(&drivetrain_left, 0);
				set_motor_power(&drivetrain_right, 0);
			}
			else
			{
				pwm_power(&drivetrain_left, false);
				pwm_power(&drivetrain_right, false);
				if(motor_kill_ctr == 500){
					uart_log(LEVEL_INFO, "Disabling drivetrain.");
				}
			}

			break;
		case dm_twist:
			set_pid(true);
			motor_kill_ctr = 0;
			break;
		default:
			uart_log(LEVEL_WARN, "Invalid drive state!");
			drive_mode = dm_halt;
		}
		core1_stage = 5;
		sleep_us(500);
	}
	alarm_pool_destroy(pid_pool); // kill PID timer
	uart_log(LEVEL_ERROR, "Exiting core1 task!");
	kill_all_actuators();
	do_core1_healthcheck = 2;
}

int main()
{
	// init uart0 debugging iface
	uart_setup();
	char *ver_str = malloc(40);
	snprintf(ver_str, 40, "--Robot Software Version %s--", VERSION);
	uart_log(LEVEL_INFO, ver_str);
	free(ver_str);
	if (watchdog_caused_reboot())
	{
		uart_log(LEVEL_WARN, "Rebooted by watchdog!");
		// in case of unclean boot, make sure actuators are off
		kill_all_actuators();
	}
	else
	{
		uart_log(LEVEL_DEBUG, "Boot was clean.");
	}
	uart_log(LEVEL_INFO, "Starting watchdog...");
	watchdog_enable(300, 1);
	// init USB serial comms
	rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_serial_transport_open,
		pico_serial_transport_close,
		pico_serial_transport_write,
		pico_serial_transport_read);

	// setup on-board status LED
	gpio_init(LED_PIN);
	gpio_set_dir(LED_PIN, GPIO_OUT);
	// setup external RSL control
	gpio_init(RSL_PIN);
	gpio_set_dir(RSL_PIN, GPIO_OUT);
	gpio_put(RSL_PIN, 0);

	uart_log(LEVEL_INFO, "Waiting for agent...");

	// try 50 times to ping, 50ms timeout each ping
	for (int i = 0; i < 50; i++)
	{
		watchdog_update();
		if (rmw_uros_ping_agent(1, 80) == RCL_RET_OK)
		{
			uart_log(LEVEL_INFO, "Connected to host.");
			watchdog_update();
			break;
		}
		char outbuff[25];
		snprintf(outbuff, 25, "Ping %d/50 failed.", i + 1);
		uart_log(LEVEL_DEBUG, outbuff);
		if (i == 49)
		{
			uart_log(LEVEL_ERROR, "Cannot contact USB Serial Agent! Bailing!");
			// wait for watchdog to reset board
			while (1)
				;
		}
	}

	// --init uros--
	rcl_node_t node;
	rcl_allocator_t allocator;
	rclc_support_t support;
	rclc_executor_t executor;

	allocator = rcl_get_default_allocator();
	rclc_support_init(&support, 0, NULL, &allocator);
	rclc_node_init_default(&node, "primary_pico", namespace, &support);
	rclc_executor_init(&executor, &support.context, RCL_CONTEXT_COUNT, &allocator);

	// --create timed events--
	create_timer_callback(&executor, &support, 10, publish_encoder);
	create_timer_callback(&executor, &support, 200, check_connectivity);
	create_timer_callback(&executor, &support, 800, uart_input_handler);
	create_timer_callback(&executor, &support, 5000, publish_battery);
	watchdog_update();

	// --create publishers--
	rclc_publisher_init_default(
		&odometry_publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
		"odom");
	// setup static components of message
	init_odom_message(&odometry_message);

	rclc_publisher_init_default(
		&battery_publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
		"battery_voltage");

	// --create subscribers--
	// twist command subscriber
	rcl_subscription_t twist_subscriber;
	geometry_msgs__msg__Twist twist_msg;
	rclc_subscription_init_default(
		&twist_subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
		"cmd_vel");
	rclc_executor_add_subscription(&executor, &twist_subscriber, &twist_msg, &twist_callback, ON_NEW_DATA);
	watchdog_update();
	// Lift command subscriber
	rcl_subscription_t lift_subscriber;
	std_msgs__msg__Int16 lift_msg;
	rclc_subscription_init_default(
		&lift_subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
		"lift_power");
	rclc_executor_add_subscription(&executor, &lift_subscriber, &lift_msg, &lift_callback, ON_NEW_DATA);
	watchdog_update();
	// -- general inits --
	init_all_motors();
	sensor_init();
	pid_setup();
	init_odometry();
	uart_log(LEVEL_DEBUG, "Finished init, starting exec");

	multicore_launch_core1(core1task);
	rclc_executor_spin(&executor);
	uart_log(LEVEL_ERROR, "Executor exited! Emergency Stop.");
	gpio_put(LED_PIN, 0);
	// wait to be killed by watchdog
	die();
}
