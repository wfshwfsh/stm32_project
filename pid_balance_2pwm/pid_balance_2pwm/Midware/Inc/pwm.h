#include "tim.h"

#define A2212_CH2_Base 100
#define A2212_CH2_Offset (+12.9)
#define A2212_CH2_RATE 1.0445

void start_pwm(int ch);
void stop_pwm(int ch);
void set_pwm(int ch, int pwmVal);
float pwm_filter(float val);

