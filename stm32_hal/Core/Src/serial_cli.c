#include "serial_cli.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "app_config.h"
#include "control.h"

static char command_buffer[SERIAL_CMD_BUFFER_SIZE];
static uint32_t command_length;
static bool discarding_long_command;

/* Writes raw text to the CLI transport backend. */
static void transport_write_string(const char *text)
{
    serial_cli_transport_write((const uint8_t *)text, (uint16_t)strlen(text));
}

/* Writes one CRLF-terminated line to the CLI transport. */
static void write_line(const char *text)
{
    transport_write_string(text);
    transport_write_string("\r\n");
}

/* Returns a short display label for each functional operating mode. */
const char *serial_cli_mode_name(functional_mode_t mode)
{
    switch (mode) {
    case FUNCTIONAL_MODE_ACTIVE:
        return "ACTIVE";
    case FUNCTIONAL_MODE_COMMAND:
        return "COMMAND";
    case FUNCTIONAL_MODE_PASSTHROUGH:
    default:
        return "PASSTHRU";
    }
}

/* Returns a short display label for each system safety state. */
const char *serial_cli_state_name(system_state_t state)
{
    switch (state) {
    case SYSTEM_STATE_BOOT:
        return "BOOT";
    case SYSTEM_STATE_WAIT_FOR_VALID_INPUT:
        return "WAIT_VALID";
    case SYSTEM_STATE_PASSTHROUGH_READY:
        return "READY";
    case SYSTEM_STATE_ACTIVE:
        return "ACTIVE";
    case SYSTEM_STATE_COMMAND:
        return "COMMAND";
    case SYSTEM_STATE_FAULT_LATCHED:
        return "FAULT";
    default:
        return "UNKNOWN";
    }
}

/* Switches into command mode and resets ramp state on first entry. */
static void enter_command_mode(functional_mode_t *requested_mode,
                               control_command_state_t *command_state)
{
    if (*requested_mode != FUNCTIONAL_MODE_COMMAND) {
        command_state->current_travel_pct = 0.0f;
    }
    *requested_mode = FUNCTIONAL_MODE_COMMAND;
}

/* Parses one received CLI line and applies mode/target commands. */
static void process_command(const char *line,
                            functional_mode_t *requested_mode,
                            bool *manual_override,
                            control_command_state_t *command_state,
                            bool *reset_auto_state)
{
    while ((*line == ' ') || (*line == '\t')) {
        ++line;
    }

    if (*line == '\0') {
        return;
    }

    if (((line[0] == 'p') || (line[0] == 'P')) && (line[1] == '\0')) {
        *manual_override = true;
        *requested_mode = FUNCTIONAL_MODE_PASSTHROUGH;
        *reset_auto_state = true;
        write_line("# MANUAL: PASSTHROUGH");
        return;
    }

    if (((line[0] == 'a') || (line[0] == 'A')) && (line[1] == '\0')) {
        *manual_override = true;
        *requested_mode = FUNCTIONAL_MODE_ACTIVE;
        write_line("# MANUAL: ACTIVE");
        return;
    }

    if (((line[0] == 'r') || (line[0] == 'R')) && (line[1] == '\0')) {
        *manual_override = false;
        *requested_mode = FUNCTIONAL_MODE_PASSTHROUGH;
        *reset_auto_state = true;
        write_line("# AUTO: zero-detect resumed");
        return;
    }

    if (((line[0] == 'c') || (line[0] == 'C')) && (line[1] == '\0')) {
        *manual_override = true;
        enter_command_mode(requested_mode, command_state);
        write_line("# MANUAL: COMMAND");
        return;
    }

    const char *parse_start = line;
    if ((*parse_start == 'c') || (*parse_start == 'C')) {
        ++parse_start;
        while ((*parse_start == ' ') || (*parse_start == '\t')) {
            ++parse_start;
        }
    }

    char *end_ptr = NULL;
    const float requested_travel = strtof(parse_start, &end_ptr);
    if (end_ptr != parse_start) {
        while ((*end_ptr == ' ') || (*end_ptr == '\t')) {
            ++end_ptr;
        }
        if (*end_ptr == '\0') {
            char response[48];

            *manual_override = true;
            enter_command_mode(requested_mode, command_state);
            command_state->target_travel_pct = control_clamp_pct(requested_travel);
            (void)snprintf(response, sizeof(response), "# MANUAL: COMMAND target=%.1f", command_state->target_travel_pct);
            write_line(response);
            return;
        }
    }

    write_line("# Unknown command");
}

/* Initializes command parser state for a fresh serial session. */
void serial_cli_init(void)
{
    command_length = 0U;
    discarding_long_command = false;
}

/* Consumes serial bytes, assembles lines, and dispatches parsed commands. */
void serial_cli_poll(functional_mode_t *requested_mode,
                     bool *manual_override,
                     control_command_state_t *command_state,
                     bool *reset_auto_state)
{
    uint8_t byte;

    *reset_auto_state = false;
    while (serial_cli_transport_read(&byte) != 0) {
        const char character = (char)byte;
        if (character == '\r') {
            continue;
        }

        if (discarding_long_command) {
            if (character == '\n') {
                discarding_long_command = false;
            }
            continue;
        }

        if (character == '\n') {
            command_buffer[command_length] = '\0';
            process_command(command_buffer, requested_mode, manual_override, command_state, reset_auto_state);
            command_length = 0U;
            continue;
        }

        if (command_length < (SERIAL_CMD_BUFFER_SIZE - 1U)) {
            command_buffer[command_length++] = character;
        } else {
            command_length = 0U;
            discarding_long_command = true;
            write_line("# Command too long");
        }
    }
}

/* Prints startup help and telemetry column headers. */
void serial_cli_print_banner(void)
{
    write_line("# Commands: P=passthrough  A=active  C=command  R=resume auto");
    write_line("# Command target: send 0..100 (or 'c <value>'), ramps at 25%/sec");
    write_line("state\tmode\trelay\tlvl\ts2_high\ts2_period\ts2_duty\ts4_high\ts4_period\ts4_duty\ttravel\tplaus\tfaults");
}

/* Formats and emits one tab-delimited telemetry sample line. */
void serial_cli_write_telemetry(const telemetry_snapshot_t *snapshot)
{
    char line[256];

    (void)snprintf(line,
                   sizeof(line),
                   "%s\t%s\t%u\t%u\t%lu\t%lu\t%.2f\t%lu\t%lu\t%.2f\t%.2f\t%.2f\t0x%08lX\r\n",
                   serial_cli_state_name(snapshot->state),
                   serial_cli_mode_name(snapshot->mode),
                   snapshot->relay_on ? 1U : 0U,
                   snapshot->level_shifter_on ? 1U : 0U,
                   (unsigned long)snapshot->s2_high_us,
                   (unsigned long)snapshot->s2_period_us,
                   snapshot->s2_duty_pct,
                   (unsigned long)snapshot->s4_high_us,
                   (unsigned long)snapshot->s4_period_us,
                   snapshot->s4_duty_pct,
                   snapshot->travel_pct,
                   snapshot->plausibility_error_pct,
                   (unsigned long)snapshot->fault_bits);
    transport_write_string(line);
}

/* Default weak transport read stub for integration override. */
__attribute__((weak)) int serial_cli_transport_read(uint8_t *byte)
{
    (void)byte;
    return 0;
}

/* Default weak transport write stub for integration override. */
__attribute__((weak)) void serial_cli_transport_write(const uint8_t *data, uint16_t length)
{
    (void)data;
    (void)length;
}
