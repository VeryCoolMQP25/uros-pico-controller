#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/timer.h"
#include "actuators.h"
#include "uart_logging.h"

Motor drivetrain_left;
Motor drivetrain_right;
Motor lift_motor;

//return index of next unallocated PIO state machine
static int get_next_sm(){
	// each bit represents availability of nth state machine
	static uint8_t pioAvail = 0xff;
	for (int i=0; i<8; i++){
		// ith pio state machine is available
		if ((pioAvail >> i) & 0x1){
			// mark SM as taken
			pioAvail &= ~(0x1 << i);
			return i;
		}
	}
	return -1;
}

static Encoder *init_encoder(uint pinA, uint pinB){
	if (pinA-pinB != 1){
		uart_log(LEVEL_ERROR, "Encoder pin A and B must be sequential! Aborting enc init");
		return NULL;
	}
	Encoder *enc = malloc(sizeof(Encoder));
	// find an available state machine
	int sm_idx = get_next_sm();
	if (sm_idx == -1) {
		return NULL;
	}
	if (sm_idx < 4) {
		enc->pio = pio0;
		enc->sm = sm_idx;
	}
	else {
		enc->pio = pio1;
		enc->sm = sm_idx-4;
	}
	quadrature_encoder_program_init(enc->pio, enc->sm, pinA, 0);
	return enc;
}

void init_motor(int pin, Motor *motor_struct){
	motor_struct->pin_num = pin;
	gpio_set_function(pin, GPIO_FUNC_PWM);
	uint slice = pwm_gpio_to_slice_num(pin);
	motor_struct->slice_num = slice;
	// configure machinery to operate at 333Hz
	float div = clock_get_hz(clk_sys) / (TALON_PWM_FREQ * (TALON_PWM_WRAP-1)); // should be ~25
	pwm_config config = pwm_get_default_config();
	pwm_config_set_clkdiv(&config, div);
	pwm_config_set_wrap(&config, TALON_PWM_WRAP);
	// set PWM to neutral before start
	pwm_set_gpio_level(pin, TALON_DEADCTR);
	motor_struct->curpower = 0;
	// init and start PWM channel
	pwm_init(slice, &config, true);
	motor_struct->enc = NULL;
}

void init_motor_with_encoder(int pin, Motor *motor_struct, int enc_pin_A, int enc_pin_B){
	init_motor(pin, motor_struct);
	motor_struct->enc = init_encoder(enc_pin_A, enc_pin_B);
	if (motor_struct->enc == NULL){
		uart_log(LEVEL_ERROR, "Could not init encoder!");
	}
}

// accepts integer power level [-100, 100]
bool set_motor_power(Motor *motor, int power){
	if(abs(power) > 100){
		char buff[40];
		snprintf(buff, 40, "Requsted motor power %d is invalid! Ignoring.", power);
		uart_log(LEVEL_WARN, buff);
		return false;
	}
	int setpoint = (TALON_DEADCTR+power*(TALON_FULL_FWD-TALON_DEADCTR));
	pwm_set_gpio_level(motor->pin_num, setpoint);
	motor->curpower = power;
	return true;
}

void init_all_motors(){
	uart_log(LEVEL_DEBUG, "Starting Prog. I/O init");
	// load pio program into both PIOs
	pio_add_program(pio0, &quadrature_encoder_program);
	pio_add_program(pio1, &quadrature_encoder_program);
	uart_log(LEVEL_DEBUG, "Starting motor init");
	init_motor(DT_L_PWM, &drivetrain_left);
	init_motor(DT_R_PWM, &drivetrain_right);
	init_motor(LIFT_PWM, &lift_motor);
	uart_log(LEVEL_DEBUG, "Motor & Encoder init finished.");
}

void kill_all_actuators(){
	set_motor_power(&drivetrain_right, 0);
	set_motor_power(&drivetrain_left, 0);
	set_motor_power(&lift_motor, 0);
}
