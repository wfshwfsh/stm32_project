#include <stdio.h>
#include <stdint.h>
#include <math.h>


#define PID_CH1_ADJUST (+29)
#define PID_CH2_ADJUST (-13)

//#define FUSION_RATE_PID 0.65
#define FUSION_RATE_PID 0.20

extern float Kp;
extern float Ki;
extern float Kd;

extern float error, lastError;
extern float targetAngle;
extern float integral;
extern float pidOutput, prev_pidOutput, prev_pidAdjust;


void pid_calculate();
