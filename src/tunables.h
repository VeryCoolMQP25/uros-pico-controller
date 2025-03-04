#ifndef TUNABLES_H
#define TUNABLES_H

// distance between front wheels in meters
#define WHEELBASE_M 0.51

#define PID_DT_V_KP	0.45
#define PID_DT_V_KI	0.12
#define PID_DT_V_KD	0.0001
#define PID_DT_TOL	0.06
#define PID_DT_I_CAP	6.0

#define PID_LFT_KP	10.0
#define PID_LFT_KI	0.1
#define PID_LFT_KD	0.05
#define PID_LFT_TOL	0.05

#define DT_ENCODER_PPM_L	25530
#define DT_ENCODER_PPM_R    25024
#define MOTOR_POWER_MAX	    75

#define DRIVETRAIN_TIMEOUT 200000 //microseconds

#endif
