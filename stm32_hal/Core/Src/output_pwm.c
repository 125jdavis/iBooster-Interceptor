#include "output_pwm.h"

#include "app_config.h"
#include "control.h"

/* Converts duty percentage to a timer compare value using current ARR limits.
 *
 * TIM3 is configured in PWM mode 1: the output is high while CNT < CCR and low
 * while CNT >= CCR.  At CCR == 0 the output is always low (0 % duty); at
 * CCR == ARR the output is high for ARR out of ARR+1 counts, which gives the
 * maximum non-100 % duty representable by the counter.  Using (ARR+1) as the
 * full-scale divisor ensures 100 % input maps to ARR rather than ARR+1, which
 * would wrap around.  The actual period and PWM frequency depend on the TIM3
 * prescaler and ARR set during CubeMX initialisation and should be verified on
 * a scope before connecting to the iBooster ECU. */
static uint32_t compare_from_duty_pct(TIM_HandleTypeDef *htim, float duty_pct)
{
    const uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim); /* timer auto-reload value that sets PWM resolution */
    const float clamped_duty = control_clamp_pct(duty_pct); /* requested duty constrained to the valid range */
    const float scaled = (clamped_duty / 100.0f) * (float)(arr + 1U); /* floating-point compare value before clamping */

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
