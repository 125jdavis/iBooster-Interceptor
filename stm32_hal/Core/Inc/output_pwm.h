#ifndef OUTPUT_PWM_H
#define OUTPUT_PWM_H /* Include guard for output_pwm.h */

#include "main.h"

void output_pwm_init_safe_state(TIM_HandleTypeDef *htim);
void output_pwm_set_duty_pct(TIM_HandleTypeDef *htim, float s2_duty_pct, float s4_duty_pct);

#endif
