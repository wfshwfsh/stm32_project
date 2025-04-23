#include "pid.h"
#include "mpu6050.h"

// PID parameter
float Kp = 0.176;//0.102;//0.0320;
float Ki = 0.000210;//0.05;
float Kd = 0;//0.35;//4.15;


float error=0.0, lastError=0.0;
float	targetAngle=0.0;
float integral=0.0;
float pidOutput=0.0, prev_pidOutput=0.0, prev_pidAdjust=0.0;

void pid_calculate()
{
	if(0 == dt) return;
  error = curAngle-targetAngle;
	
	// pid_i:
	//if(error > -3.0 && error < 3.0){
		integral += error*dt;
	//}
	
	// pid_d:
  float derivative = (error-lastError)/dt;
	float PID = Kp * error + Ki * integral + Kd * derivative;
  
	//pidOutput = FUSION_RATE_PID*prev_pidOutput + (1-FUSION_RATE_PID)*PID;
	pidOutput = PID;
	//printf("prev_pidOutput:%f pidOutput:%f PID:%f\r\n", prev_pidOutput, pidOutput, PID);
	//prev_pidOutput = pidOutput;
	lastError = error;
  
  //printf("%f\r\n", pidOutput);
	printf("curAngle:%f\t P:%f\t I:%f\t D:%f\r\n", curAngle, Kp * error, Ki * integral, Kd * derivative);
	//printf("curAngle:%f PID:%f\r\n", curAngle, PID);
}
