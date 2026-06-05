#include "heartbeat.h"

#include <stdbool.h>
#include <stddef.h>

#include "board_pins.h"

typedef struct {
    const uint16_t *durations_ms;
    const bool *levels_on;
    size_t length;
} heartbeat_pattern_t;

static const uint16_t wait_durations_ms[] = { 500U, 500U };
static const bool wait_levels_on[] = { true, false };

static const uint16_t active_durations_ms[] = { 100U, 200U, 100U, 200U, 400U, 400U };
static const bool active_levels_on[] = { true, false, true, false, true, false };

static const uint16_t command_durations_ms[] = { 200U, 200U, 200U, 800U, 800U };
static const bool command_levels_on[] = { true, false, true, false, true };

static const uint16_t fault_durations_ms[] = { 100U, 100U };
static const bool fault_levels_on[] = { true, false };

static const heartbeat_pattern_t wait_pattern = {
    wait_durations_ms,
    wait_levels_on,
    sizeof(wait_durations_ms) / sizeof(wait_durations_ms[0]),
};

static const heartbeat_pattern_t active_pattern = {
    active_durations_ms,
    active_levels_on,
    sizeof(active_durations_ms) / sizeof(active_durations_ms[0]),
};

static const heartbeat_pattern_t command_pattern = {
    command_durations_ms,
    command_levels_on,
    sizeof(command_durations_ms) / sizeof(command_durations_ms[0]),
};

static const heartbeat_pattern_t fault_pattern = {
    fault_durations_ms,
    fault_levels_on,
    sizeof(fault_durations_ms) / sizeof(fault_durations_ms[0]),
};

static system_state_t active_state = SYSTEM_STATE_BOOT;
static size_t step_index = 0U;
static uint32_t step_elapsed_ms = 0U;

static void set_led(bool on)
{
    HAL_GPIO_WritePin(HEARTBEAT_LED_GPIO_Port,
                      HEARTBEAT_LED_Pin,
                      on ? HEARTBEAT_LED_ON_STATE : HEARTBEAT_LED_OFF_STATE);
}

static const heartbeat_pattern_t *pattern_for_state(system_state_t state)
{
    if (state == SYSTEM_STATE_ACTIVE) {
        return &active_pattern;
    }

    if (state == SYSTEM_STATE_COMMAND) {
        return &command_pattern;
    }

    if (state == SYSTEM_STATE_FAULT_LATCHED) {
        return &fault_pattern;
    }

    return &wait_pattern;
}

void heartbeat_init(void)
{
    active_state = SYSTEM_STATE_BOOT;
    step_index = 0U;
    step_elapsed_ms = 0U;
    set_led(wait_levels_on[0]);
}

void heartbeat_update(system_state_t state, uint32_t dt_ms)
{
    const heartbeat_pattern_t *pattern = pattern_for_state(state);

    if (pattern->length == 0U) {
        return;
    }

    if (state != active_state) {
        active_state = state;
        step_index = 0U;
        step_elapsed_ms = 0U;
        set_led(pattern->levels_on[step_index]);
    }

    if (dt_ms == 0U) {
        return;
    }

    step_elapsed_ms += dt_ms;
    while (step_elapsed_ms >= pattern->durations_ms[step_index]) {
        step_elapsed_ms -= pattern->durations_ms[step_index];
        step_index++;
        if (step_index >= pattern->length) {
            step_index = 0U;
        }
        set_led(pattern->levels_on[step_index]);
    }
}
