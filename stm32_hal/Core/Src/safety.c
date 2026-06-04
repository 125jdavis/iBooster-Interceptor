#include "safety.h"

#include <math.h>

#include "app_config.h"

/* Latches a fault condition and forces the system into fail-safe state. */
static void latch_fault(safety_state_t *state, uint32_t fault_bit)
{
    state->fault_bits |= fault_bit;
    state->relay_commanded_on = false;
    state->state = SYSTEM_STATE_FAULT_LATCHED;
}

/* Initializes all safety-state fields to conservative boot defaults. */
void safety_init(safety_state_t *state)
{
    state->state = SYSTEM_STATE_BOOT;
    state->fault_bits = FAULT_NONE;
    state->startup_valid_elapsed_ms = 0U;
    state->plausibility_bad_elapsed_ms = 0U;
    state->stale_events = 0U;
    state->plausibility_violations = 0U;
    state->startup_complete = false;
    state->relay_commanded_on = false;
    state->level_shifter_enabled = false;
}

/* Tracks whether the output level shifter is currently enabled. */
void safety_set_level_shifter_enabled(safety_state_t *state, bool enabled)
{
    state->level_shifter_enabled = enabled;
}

/* Evaluates startup, validity, plausibility, and mode rules to command safety outputs. */
void safety_update(safety_state_t *state,
                   functional_mode_t requested_mode,
                   bool inputs_valid,
                   bool s2_fresh,
                   bool s4_fresh,
                   bool s2_valid,
                   bool s4_valid,
                   bool pedal_at_rest,
                   float plausibility_error_pct,
                   uint32_t dt_ms)
{
    if (safety_fault_latched(state)) {
        state->relay_commanded_on = false;
        state->state = SYSTEM_STATE_FAULT_LATCHED;
        return;
    }

    if (!state->startup_complete) {
        if (inputs_valid && pedal_at_rest) {
            if (state->startup_valid_elapsed_ms < STARTUP_VALID_MS) {
                state->startup_valid_elapsed_ms += dt_ms;
            }
            if (state->startup_valid_elapsed_ms >= STARTUP_VALID_MS) {
                state->startup_complete = true;
            }
        } else {
            state->startup_valid_elapsed_ms = 0U;
        }
        state->plausibility_bad_elapsed_ms = 0U;
        state->relay_commanded_on = false;
        state->state = SYSTEM_STATE_WAIT_FOR_VALID_INPUT;
        return;
    }

    if (!s2_valid || !s4_valid) {
        latch_fault(state, FAULT_INVALID_PERIOD);
        return;
    }

    if (!s2_fresh || !s4_fresh) {
        state->stale_events++;
        latch_fault(state, FAULT_STALE_INPUT);
        return;
    }

    if (fabsf(plausibility_error_pct) > PLAUSIBILITY_TOLERANCE_PCT) {
        if (state->plausibility_bad_elapsed_ms < PLAUSIBILITY_FAULT_MS) {
            state->plausibility_bad_elapsed_ms += dt_ms;
        }
        if (state->plausibility_bad_elapsed_ms >= PLAUSIBILITY_FAULT_MS) {
            state->plausibility_violations++;
            latch_fault(state, FAULT_PLAUSIBILITY);
            return;
        }
    } else {
        state->plausibility_bad_elapsed_ms = 0U;
    }

    if ((requested_mode == FUNCTIONAL_MODE_ACTIVE) || (requested_mode == FUNCTIONAL_MODE_COMMAND)) {
        if (!state->relay_commanded_on && pedal_at_rest) {
            state->relay_commanded_on = true;
        }
    } else {
        state->relay_commanded_on = false;
    }

    if ((requested_mode == FUNCTIONAL_MODE_ACTIVE) && state->relay_commanded_on) {
        state->state = SYSTEM_STATE_ACTIVE;
    } else if ((requested_mode == FUNCTIONAL_MODE_COMMAND) && state->relay_commanded_on) {
        state->state = SYSTEM_STATE_COMMAND;
    } else {
        state->state = SYSTEM_STATE_PASSTHROUGH_READY;
    }
}

/* Returns true when any safety fault has been latched. */
bool safety_fault_latched(const safety_state_t *state)
{
    return state->fault_bits != FAULT_NONE;
}

/* Returns true when relay engagement is allowed by current safety state. */
bool safety_relay_allowed(const safety_state_t *state)
{
    return state->relay_commanded_on && !safety_fault_latched(state) && state->startup_complete;
}
