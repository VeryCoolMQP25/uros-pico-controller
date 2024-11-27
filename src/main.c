#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include "hardware/watchdog.h"
#include "pico/stdlib.h"
#include "pico/malloc.h"
#include "pico_ros_usb.h"
#include "uart_logging.h"
#include "actuators.h"
#include "controls.h"
#include "pins.h"
#include "message_types.h"
#include 

// version numbering: <term>-<day>.ver
#define VERSION "B-27.1"

// globals
const char *namespace = "";
DriveMode drive_mode = dm_halt;

/// support for encoder publisher
rcl_publisher_t odometry_publisher;
nav_msgs__msg__Odometry odom_message;

// callback to publish encoder data (processed into timestamped twists)
void publish_odometry(rcl_timer_t *timer, int64_t last_call_time)
{
	// mutate message
	// Publish message
	if (rcl_publish(&odometry_publisher, &odom_message, NULL))
	{
		uart_log(LEVEL_WARN, "Odom publish failed!");
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
	uart_log(LEVEL_DEBUG, "Started core 1 task");
	multicore_lockout_victim_init();
	int motor_kill_ctr = 0;
	while (true)
	{
		watchdog_update();
		drive_mode = drive_mode_from_ros();
		lift_timeout_check();
		update_motor_encoders();
		switch (drive_mode)
		{
		case dm_raw:
		{
			set_motor_power(&drivetrain_left, 25);
			set_motor_power(&drivetrain_right, 25);
			update_motor_encoders(&drivetrain_left);
			update_motor_encoders(&drivetrain_right);
			char velocity_dbg[30];
			snprintf(velocity_dbg, 30, "Velocities: (%f, %f)", drivetrain_left.velocity, drivetrain_right.velocity);
			uart_log(LEVEL_DEBUG, velocity_dbg);
			motor_kill_ctr = 0;
			break;
		}
		case dm_halt:
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
			do_drivetrain_pid_v();
			motor_kill_ctr = 0;
			break;
		default:
			uart_log(LEVEL_WARN, "Invalid drive state!");
			drive_mode = dm_halt;
		}
		sleep_ms(4);
	}
	uart_log(LEVEL_ERROR, "Exiting core1 task!");
	kill_all_actuators();
}

int main()
{
	// init uart0 debugging iface
	uart_setup();
	uart_log(LEVEL_INFO, "Started UART comms");
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
		uart_log(LEVEL_INFO, "Boot was clean.");
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

	uart_log(LEVEL_INFO, "Waiting for agent...");

	// try 50 times to ping, 50ms timeout each ping
	for (int i = 0; i < 50; i++)
	{
		watchdog_update();
		if (rmw_uros_ping_agent(1, 45) == RCL_RET_OK)
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
	rclc_node_init_default(&node, "pico_node", namespace, &support);
	rclc_executor_init(&executor, &support.context, 5, &allocator);

	// --create timed events--
	create_timer_callback(&executor, &support, 25, publish_odometry);
	create_timer_callback(&executor, &support, 300, check_connectivity);
	create_timer_callback(&executor, &support, 1000, uart_input_handler);
	watchdog_update();

	// --create publishers--
	rclc_publisher_init_default(
		&odometry_publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
		"odom");
	// setup static components of message
	odom_message.header.frame_id.data = "base_link";
	odom_message.header.frame_id.size = strlen(odom_message.header.frame_id.data);
	odom_message.header.frame_id.capacity = odom_message.header.frame_id.size+1;

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
	std_msgs__msg__Float32 lift_msg;
	rclc_subscription_init_default(
		&lift_subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
		"lift_raw");
	rclc_executor_add_subscription(&executor, &lift_subscriber, &lift_msg, &raw_lift_callback, ON_NEW_DATA);
	watchdog_update();
	// -- general inits --
	init_all_motors();
	pid_setup();
	uart_log(LEVEL_DEBUG, "Finished init, starting exec");

	multicore_launch_core1(core1task);
	rclc_executor_spin(&executor);
	uart_log(LEVEL_ERROR, "Executor exited! Emergency Stop.");
	gpio_put(LED_PIN, 0);
	// wait to be killed by watchdog
	die();
}
