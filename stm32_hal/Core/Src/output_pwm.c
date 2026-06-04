#include "output_pwm.h"

#include "app_config.h"
#include "control.h"

/* Converts duty percentage to a timer compare value using current ARR limits. */
static uint32_t compare_from_duty_pct(TIM_HandleTypeDef *htim, float duty_pct)
{
    const uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim);
    const float clamped_duty = control_clamp_pct(duty_pct);
    const float scaled = (clamped_duty / 100.0f) * (float)(arr + 1U);

    if (scaled <= 0.0f) {
        return 0U;
    }
    if (scaled >= (float)(arr + 1U)) {
        return arr;
    }
    return (uint32_t)scaled;
}

/* Initializes PWM outputs to the configured safe/rest duty state. */
void output_pwm_init_safe_state(TIM_HandleTypeDef *htim)
{
    output_pwm_set_duty_pct(htim, S2_REST_DUTY_PCT, S4_REST_DUTY_PCT);
}

/* Updates both PWM channels with requested S2/S4 duty percentages. */
void output_pwm_set_duty_pct(TIM_HandleTypeDef *htim, float s2_duty_pct, float s4_duty_pct)
{
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, compare_from_duty_pct(htim, s4_duty_pct));
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, compare_from_duty_pct(htim, s2_duty_pct));
}
