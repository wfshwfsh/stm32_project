#include "pwm.h"


void start_pwm(int ch) {
	HAL_TIM_PWM_Start(&htim3,ch);
}

void stop_pwm(int ch) {
	HAL_TIM_PWM_Stop(&htim3, ch);
}

void set_pwm(int ch, int pwmVal) {
	//if(TIM_CHANNEL_2 == ch)
	//	pwmVal = pwmVal+A2212_CH2_Base;
	__HAL_TIM_SetCompare(&htim3, ch, pwmVal);
}

float pwm_filter(float val)
{
	if(val <= 1000){
		return 1000;
	}else if(val >= 1600){
		return 1600;
	}else{
		return val;
	}
}

