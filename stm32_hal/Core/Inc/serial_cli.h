#ifndef SERIAL_CLI_H
#define SERIAL_CLI_H /* Include guard for serial_cli.h */

#include <stdbool.h>
#include <stdint.h>

#include "app_types.h"
#include "input_capture.h"
#include "safety.h"

typedef struct {
    functional_mode_t mode;
    system_state_t state;
    uint32_t fault_bits;
    bool relay_on;
    bool level_shifter_on;
    bool s2_valid;
    bool s4_valid;
    bool s2_fresh;
    bool s4_fresh;
    uint32_t s2_high_us;
    uint32_t s2_period_us;
    uint32_t s4_high_us;
    uint32_t s4_period_us;
    float s2_duty_pct;
    float s4_duty_pct;
    float travel_pct;
    float plausibility_error_pct;
    uint32_t startup_valid_elapsed_ms;
    uint32_t stale_events;
    uint32_t rejected_periods;
    uint32_t rejected_pulses;
    uint32_t plausibility_violations;
    uint32_t exti_edges;
} telemetry_snapshot_t;

void serial_cli_init(void);
void serial_cli_poll(functional_mode_t *requested_mode,
                     bool *manual_override,
                     control_command_state_t *command_state,
                     bool *reset_auto_state);
void serial_cli_print_banner(void);
void serial_cli_write_telemetry(const telemetry_snapshot_t *snapshot);
const char *serial_cli_mode_name(functional_mode_t mode);
const char *serial_cli_state_name(system_state_t state);

int serial_cli_transport_read(uint8_t *byte);
void serial_cli_transport_write(const uint8_t *data, uint16_t length);

#endif
