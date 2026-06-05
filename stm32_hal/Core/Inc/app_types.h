#ifndef APP_TYPES_H
#define APP_TYPES_H /* Include guard for app_types.h */

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    FUNCTIONAL_MODE_PASSTHROUGH = 0,
    FUNCTIONAL_MODE_ACTIVE,
    FUNCTIONAL_MODE_COMMAND,
} functional_mode_t;

typedef enum {
    SYSTEM_STATE_BOOT = 0,
    SYSTEM_STATE_WAIT_FOR_VALID_INPUT,
    SYSTEM_STATE_PASSTHROUGH_READY,
    SYSTEM_STATE_ACTIVE,
    SYSTEM_STATE_COMMAND,
    SYSTEM_STATE_FAULT_LATCHED,
} system_state_t;

enum {
    FAULT_NONE                = 0U,
    FAULT_PLAUSIBILITY        = 1U << 0,
    FAULT_STALE_INPUT         = 1U << 1,
    FAULT_INVALID_CAPTURE     = 1U << 2,
};

typedef struct {
    float s2_filtered_pct;
    float s4_filtered_pct;
} control_filter_state_t;

typedef struct {
    float target_travel_pct;
    float current_travel_pct;
} control_command_state_t;

#endif
