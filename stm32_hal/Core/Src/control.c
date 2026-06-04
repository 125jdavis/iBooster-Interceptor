#include "control.h"

#include <math.h>

#include "app_config.h"

/* Performs linear interpolation between two values using normalized factor t. */
static float lerp(float a, float b, float t)
{
    return a + ((b - a) * t);
}

/* Clamps percentage values to the valid 0..100 range. */
float control_clamp_pct(float value_pct)
{
    if (value_pct < 0.0f) {
        return 0.0f;
    }
    if (value_pct > 100.0f) {
        return 100.0f;
    }
    return value_pct;
}

/* Initializes filtered duty signals to known rest defaults. */
void control_filter_init(control_filter_state_t *state)
{
    state->s2_filtered_pct = S2_REST_DUTY_PCT;
    state->s4_filtered_pct = S4_REST_DUTY_PCT;
}

/* Applies exponential smoothing to valid raw S2/S4 duty measurements. */
void control_update_filtered_duties(control_filter_state_t *state,
                                    float raw_s2_pct,
                                    bool s2_valid,
                                    float raw_s4_pct,
                                    bool s4_valid)
{
    if (s2_valid) {
        state->s2_filtered_pct = (EMA_ALPHA * raw_s2_pct) + ((1.0f - EMA_ALPHA) * state->s2_filtered_pct);
    }

    if (s4_valid) {
        state->s4_filtered_pct = (EMA_ALPHA * raw_s4_pct) + ((1.0f - EMA_ALPHA) * state->s4_filtered_pct);
    }
}

/* Converts paired duty signals into a normalized pedal travel percentage. */
float control_compute_travel_pct(float s2_pct, float s4_pct)
{
    const float s2_travel = (S2_REST_DUTY_PCT - s2_pct) / (S2_REST_DUTY_PCT - S2_FULL_DUTY_PCT) * 100.0f;
    const float s4_travel = (s4_pct - S4_REST_DUTY_PCT) / (S4_FULL_DUTY_PCT - S4_REST_DUTY_PCT) * 100.0f;

    return control_clamp_pct((s2_travel + s4_travel) * 0.5f);
}

/* Checks whether current pedal travel is within the configured rest tolerance. */
bool control_pedal_at_rest(float travel_pct)
{
    return travel_pct <= REST_TOLERANCE_PCT;
}

/* Maps measured travel through the configured calibration curve segments. */
float control_lookup_curve_pct(float travel_pct)
{
    const float clamped = control_clamp_pct(travel_pct);

    for (uint32_t index = 0U; index < (CAL_POINT_COUNT - 1U); ++index) {
        if (clamped <= CAL_IN_PCT[index + 1U]) {
            const float span = CAL_IN_PCT[index + 1U] - CAL_IN_PCT[index];
            const float t = (span > 0.0f) ? ((clamped - CAL_IN_PCT[index]) / span) : 0.0f;
            return lerp(CAL_OUT_PCT[index], CAL_OUT_PCT[index + 1U], t);
        }
    }

    return CAL_OUT_PCT[CAL_POINT_COUNT - 1U];
}

/* Converts normalized travel percentage back into S2/S4 output duty targets. */
void control_travel_to_duty_pct(float travel_pct, float *s2_duty_pct, float *s4_duty_pct)
{
    const float normalized = control_clamp_pct(travel_pct) / 100.0f;

    *s2_duty_pct = S2_REST_DUTY_PCT + (normalized * (S2_FULL_DUTY_PCT - S2_REST_DUTY_PCT));
    *s4_duty_pct = S4_REST_DUTY_PCT + (normalized * (S4_FULL_DUTY_PCT - S4_REST_DUTY_PCT));
}

/* Slews command-mode target travel using a bounded ramp rate per tick. */
void control_update_command_ramp(control_command_state_t *state, float dt_seconds)
{
    const float max_step = COMMAND_RAMP_RATE_PCT_PER_SEC * dt_seconds;

    if (state->current_travel_pct < state->target_travel_pct) {
        state->current_travel_pct = fminf(state->current_travel_pct + max_step, state->target_travel_pct);
    } else if (state->current_travel_pct > state->target_travel_pct) {
        state->current_travel_pct = fmaxf(state->current_travel_pct - max_step, state->target_travel_pct);
    }
}
