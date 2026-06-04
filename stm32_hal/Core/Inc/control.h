#ifndef CONTROL_H
#define CONTROL_H /* Include guard for control.h */

#include <stdbool.h>

#include "app_types.h"

void control_filter_init(control_filter_state_t *state);
void control_update_filtered_duties(control_filter_state_t *state,
                                    float raw_s2_pct,
                                    bool s2_valid,
                                    float raw_s4_pct,
                                    bool s4_valid);
float control_compute_travel_pct(float s2_pct, float s4_pct);
bool control_pedal_at_rest(float travel_pct);
float control_lookup_curve_pct(float travel_pct);
void control_travel_to_duty_pct(float travel_pct, float *s2_duty_pct, float *s4_duty_pct);
void control_update_command_ramp(control_command_state_t *state, float dt_seconds);
float control_clamp_pct(float value_pct);

#endif
