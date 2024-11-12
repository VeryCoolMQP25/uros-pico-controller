#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <geometry_msgs/msg/twist.h>
#include <rmw_microros/rmw_microros.h>

#include "hardware/watchdog.h"
#include "pico/stdlib.h"
#include "pico/malloc.h"
#include <malloc.h>
#include "pico_ros_usb.h"
#include "uart_logging.h"
#include "actuators.h"
#include "controls.h"
#include "pins.h"

// globals
const char *namespace = "";
DriveMode drive_mode = dm_halt;
rcl_publisher_t encoder_raw_publisher;
std_msgs__msg__Int32MultiArray encoder_raw_message;


void publish_all_cb(rcl_timer_t *timer, int64_t last_call_time){
	// update encoder values
	update_motor_encoders(&drivetrain_left);
	update_motor_encoders(&drivetrain_right);
	// publish raw encoder readings
	encoder_raw_message.data.data[0] = drivetrain_left.enc->prev_count;
	encoder_raw_message.data.data[1] = drivetrain_right.enc->prev_count;
	int res = rcl_publish(&encoder_raw_publisher, &encoder_raw_message, NULL);
	#ifdef DEBUG_ENCODERS
		char debugbuff[80];
		snprintf(debugbuff, 80, "Encoder data: (%d, %d) [%d]", drivetrain_left.enc->prev_count, drivetrain_right.enc->prev_count, res);
		uart_log(LEVEL_DEBUG, debugbuff);
	#endif //DEBUG_ENCODERS
	#ifdef DEBUG_MEM
		struct mallinfo info = mallinfo();
		char membuff[32];
		snprintf(membuff, 32, "malloc: %d, mfree: %d",info.uordblks,info.fordblks);
		uart_log_nonblocking(LEVEL_INFO, membuff);
	#endif // DEBUG_MEM
}

// checks if we have comms with serial agent
void check_connectivity(rcl_timer_t *timer, int64_t last_call_time){
	//uart_log(LEVEL_DEBUG, "connectivity CB run");
	bool ok = (rmw_uros_ping_agent(50, 1) == RCL_RET_OK);
	gpio_put(LED_PIN, ok);
	if (!ok){
		drive_mode = dm_halt;
	}
	watchdog_update();
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

void core1task(){
	uart_log(LEVEL_DEBUG, "Started core 1 task");
	while(true){
		watchdog_update();
		drive_mode = drive_mode_from_ros();
		switch(drive_mode){
			case dm_halt:
				kill_all_actuators();
				break;
			case dm_twist:
				do_drivetrain_pid_v();
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
	// in case of unclean boot, make sure actuators are off
	kill_all_actuators();
	// init USB serial comms
    rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_serial_transport_open,
		pico_serial_transport_close,
		pico_serial_transport_write,
		pico_serial_transport_read
	);
	
	// setup on-board status LED
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

	// init uart0 debugging iface
    uart_setup();
    uart_log(LEVEL_DEBUG, "Started UART comms");
    if (watchdog_caused_reboot()){
    	uart_log(LEVEL_WARN, "Rebooted by watchdog!");
    }
    else {
    	uart_log(LEVEL_INFO, "Boot was clean.");
    }
    uart_log(LEVEL_INFO, "Starting watchdog...");
    watchdog_enable(300, 1);
    
    uart_log(LEVEL_INFO, "Waiting for agent...");
    
    // try 50 times to ping, 50ms timeout each ping
	for (int i = 0; i < 50; i++){
		if (rmw_uros_ping_agent(1, 50) == RCL_RET_OK){
			uart_log(LEVEL_INFO,"Connected to host.");
			break;
		}
		char outbuff[20];
		snprintf(outbuff, 20, "Ping #%d failed.", i);
		uart_log(LEVEL_DEBUG, outbuff);
		if (i == 49){
			uart_log(LEVEL_ERROR, "Cannot contact USB Serial Agent! Bailing!");
			//wait for watchdog to reset board
			while(1);
		}
		watchdog_update();
	}

	// --init uros--
    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor;

    allocator = rcl_get_default_allocator();	
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "pico_node", namespace, &support);
    rclc_executor_init(&executor, &support.context, 3, &allocator);
    
    // --create timed events--
    create_timer_callback(&executor, &support, 100, publish_all_cb);
    create_timer_callback(&executor, &support, 200, check_connectivity);

	// --create publishers--
	if (rclc_publisher_init_default(&encoder_raw_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray), "/encoder_raw_counts") != RCL_RET_OK){
		while (1){
			uart_log(LEVEL_ERROR,"Publisher init failed!!");
		}
	}
	encoder_raw_message.data.data = malloc(sizeof(int32_t)*2);
	encoder_raw_message.data.size = 2;
	encoder_raw_message.data.capacity = 2;
	
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
    // -- general inits --
    init_all_motors();
    pid_setup();
    uart_log(LEVEL_DEBUG, "Finished init, starting exec");
    
    multicore_launch_core1(core1task);
   	rclc_executor_spin(&executor);
	uart_log(LEVEL_ERROR, "Executor exited! Emergency Stop.");
	gpio_put(LED_PIN, 0);
	// wait to be killed by watchdog
    while(1) {
    	kill_all_actuators();
    	uart_log(LEVEL_ERROR,"KILL ME");
    }
}
