#ifndef SAFETY_H
#define SAFETY_H

#include <stdbool.h>
#include <stdint.h>

#include "app_types.h"

typedef struct {
    system_state_t state;
    uint32_t fault_bits;
    uint32_t startup_valid_elapsed_ms;
    uint32_t plausibility_bad_elapsed_ms;
    uint32_t stale_events;
    uint32_t plausibility_violations;
    bool startup_complete;
    bool relay_commanded_on;
    bool level_shifter_enabled;
} safety_state_t;

void safety_init(safety_state_t *state);
void safety_set_level_shifter_enabled(safety_state_t *state, bool enabled);
void safety_update(safety_state_t *state,
                   functional_mode_t requested_mode,
                   bool inputs_valid,
                   bool s2_fresh,
                   bool s4_fresh,
                   bool s2_valid,
                   bool s4_valid,
                   bool pedal_at_rest,
                   float plausibility_error_pct,
                   uint32_t dt_ms);
bool safety_fault_latched(const safety_state_t *state);
bool safety_relay_allowed(const safety_state_t *state);

#endif
