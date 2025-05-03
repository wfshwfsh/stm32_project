#include "pid.h"
#include "mpu6050.h"
extern float dt_tim1_intr;

// PID parameter
float Kp = 4.125*0.6;//2.250*0.6;
float Ki = 0.125;
float Kd = 21.58*0.6;//2.88*0.6;


float error=0.0, lastError=0.0;
float	targetAngle=0.0;
float integral=0.0, derivative=0.0;
float pidOutput=0.0, prev_pidOutput=0.0, prev_pidAdjust=0.0;
extern float Gyro_Xdps, Gyro_Ydps;

static int count=0;
void pid_calculate()
{
  error = curAngle-targetAngle;
	
	// pid_i:
	//if(error > -3.0 && error < 3.0){
		integral += error*(dt_tim1_intr/1000);
	//}
	
	// pid_d:
  //derivative = (error-lastError)/(dt_tim1_intr/1000);
	derivative = Gyro_Ydps;
	if(derivative > 10){
		derivative = 10;
	}else if(derivative < -10){
		derivative = -10;
	}
	float PID = Kp * error + Ki * integral + Kd * derivative;
  
	//pidOutput = FUSION_RATE_PID*prev_pidOutput + (1-FUSION_RATE_PID)*PID;
	pidOutput = PID;
	//printf("prev_pidOutput:%f pidOutput:%f PIDm:%f\r\n", prev_pidOutput, pidOutput, PID);
	//prev_pidOutput = pidOutput;
	lastError = error;
  
  //printf("%f\r\n", pidOutput);
	if(count++ % 20 == 0){
		printf("curAngle:%f\t P:%f\t I:%f\t D:%f\r\n", curAngle, Kp * error, Ki * integral, Kd * derivative);
	}
	//printf("curAngle:%f PID:%f\r\n", curAngle, PID);
}


float map_float(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int compensatePIDOutput(float pid_output, float angle) {
    int compensation = 0;

    // +++ : 0 ~ +20 => 0 ~ +27
    if (angle > 0.0f) {
        if (angle > 20.0f) angle = 19.3f;
        compensation = map_float((angle * 100.0f), 0, 1930, 0, PID_CH1_ADJUST);
    }
    // --- : 0 ~ -15 => 0 ~ -11
    else if (angle < 0.0f) {
        if (angle < -15.0f) angle = -15.0f;
        compensation = map_float((-angle * 100.0f), 0, 1500, 0, PID_CH2_ADJUST);
    }
		
		//printf("compensation:%d\n", compensation);
    // PID
    int compensated_output = pid_output + compensation;
    return compensated_output;
}