#ifndef TUNABLES_H
#define TUNABLES_H

// distance between front wheels in meters
#define WHEELBASE_M 0.51

#define PID_DT_V_KP	0.52 
#define PID_DT_V_KI	0.45 //.35
#define PID_DT_V_KD	0.0001
#define PID_DT_TOL	0.02
#define PID_DT_I_CAP	6.0

#define DT_ENCODER_PPM_L	25530
#define DT_ENCODER_PPM_R    25024
#define MOTOR_POWER_MAX	    100
#define MOTOR_DEADZONE	    12

#define ROTATE_INPLACE_THRESHOLD 0.1 //m/s
#define ROTATE_INPLACE_MULT 1.5

#define DRIVETRAIN_TIMEOUT 200000 //microseconds

#endif
