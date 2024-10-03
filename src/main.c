#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "pico_ros_usb.h"
#include "uart_logging.h"
#include "actuators.h"
#include "controls.h"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){uart_log_fatal(__LINE__,(int)temp_rc); return 1;}}

// globals
const char *namespace = "";
// system states
CommState comm_state = cs_connected;
DriveMode drive_mode = dm_raw;

int comm_fail_counter = 0;
// onboard green LED
const uint LED_PIN = 25;

void amogus_cb(rcl_timer_t *timer, int64_t last_call_time)
{
	uart_log_nonblocking(LEVEL_DEBUG,"amogus");
	return;
	//TODO
}

void publish_all_cb(rcl_timer_t *timer, int64_t last_call_time)
{
	uart_send("p");
	return;
	//TODO
}

// checks if we have comms with serial agent
void check_connectivity(rcl_timer_t *timer, int64_t last_call_time)
{
	uart_log_nonblocking(LEVEL_DEBUG,"connectivity check run");
	bool ok = (rmw_uros_ping_agent(30, 1) == RCL_RET_OK);
	gpio_put(LED_PIN, ok);
	if (!ok){
		comm_state = cs_disconnected;
	}
}

void core1task(){
	uart_log(LEVEL_DEBUG, "Started core 1 task");
	while(true){
		switch(drive_mode){
			case dm_halt:
				kill_all_actuators();
				break;
			case dm_raw:
				drivetrain_power_from_ros();
				break;
			default:
				uart_log(LEVEL_WARN, "Invalid drive state!");
				drive_mode = dm_halt;
		}
		sleep_ms(10);
	}
	uart_log(LEVEL_ERROR, "Exiting core1 task!");
	kill_all_actuators();	
}

int main()
{
	// init USB serial comms
    rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_serial_transport_open,
		pico_serial_transport_close,
		pico_serial_transport_write,
		pico_serial_transport_read
	);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

	// init uart0 debugging iface
    uart_setup();
    uart_log(LEVEL_DEBUG, "Started UART comms");
    uart_log(LEVEL_INFO, "Waiting for agent...");

    rcl_ret_t ret = rmw_uros_ping_agent(50, 200);

    if (ret != RCL_RET_OK)
    {
        uart_log(LEVEL_ERROR, "Cannot contact USB Serial Agent! Bailing out!");
        return ret;
    }

	// init uros
    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor;

    allocator = rcl_get_default_allocator();	
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "pico_node", namespace, &support);
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    
    // create timed events
    rcl_timer_t mogtimer;
    RCCHECK(rclc_timer_init_default(&mogtimer, &support, RCL_MS_TO_NS(1200), amogus_cb));
    rclc_executor_add_timer(&executor, &mogtimer);

    rcl_timer_t pubtimer;
    RCCHECK(rclc_timer_init_default(&pubtimer, &support, RCL_MS_TO_NS(1000), publish_all_cb));
    rclc_executor_add_timer(&executor, &pubtimer);
    
    rcl_timer_t conntimer;
    RCCHECK(rclc_timer_init_default(&conntimer, &support, RCL_MS_TO_NS(500), check_connectivity));
    rclc_executor_add_timer(&executor, &conntimer);    
    
	std_msgs__msg__Int32MultiArray dt_pwr_msg;
	std_msgs__msg__Int32MultiArray__init(&dt_pwr_msg); // Initialize the messag
    // create message subscribers
    rcl_subscription_t dt_pwr_sub;
    rclc_subscription_init_default(
        &dt_pwr_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
        "drivetrain_powers");    
    rclc_executor_add_subscription(&executor, &dt_pwr_sub, &dt_pwr_msg, &dt_power_callback, ON_NEW_DATA);
    init_all_motors();
    uart_log(LEVEL_DEBUG, "Finished init, starting exec");
    multicore_launch_core1(core1task);
    uart_log(LEVEL_DEBUG,"main loop entry");
    rclc_executor_spin(&executor);
    uart_log(LEVEL_DEBUG,"fortnite");
	while (true){
		switch(comm_state){
			case cs_down: {
				uart_log(LEVEL_DEBUG,"Link down");
				kill_all_actuators();
				if (rmw_uros_ping_agent(10, 5) == RCL_RET_OK){
					uart_log(LEVEL_INFO, "trying to re-connect...");
					comm_state = cs_disconnected;
					comm_fail_counter = 0;
				}
				break;
			}
			case cs_connected: {
				rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
				break;
			}
			case cs_disconnected: {
				uart_log(LEVEL_DEBUG,"Link disconnected");
				if(rmw_uros_ping_agent(50, 1) == RCL_RET_OK){
					uart_log(LEVEL_INFO, "Connected to ROS agent");
					comm_state = cs_connected;
					comm_fail_counter = 0;
				}
				else if(++comm_fail_counter > COMM_FAIL_THRESH){
					comm_state = cs_down;
					uart_log(LEVEL_WARN, "No connection to ROS agent!");
				}
				break;
			}
			default:
				uart_log(LEVEL_ERROR, "Illegal state!");
				return 1;
		}
	}
	uart_log(LEVEL_ERROR, "Executor exited!");
	
    return 0;
}
