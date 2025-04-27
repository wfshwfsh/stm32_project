#include <stdio.h>
#include <stdint.h>
#include <math.h>


#define PID_CH1_ADJUST (+8)
#define PID_CH2_ADJUST (-8)



extern float Kp;
extern float Ki;
extern float Kd;

extern float error, lastError;
extern float targetAngle;
extern float integral;
extern float pidOutput, prev_pidOutput, prev_pidAdjust;


void pid_calculate();
float map_float(float x, float in_min, float in_max, float out_min, float out_max);
int compensatePIDOutput(float pid_output, float angle);
