#include "main.h"

#include <stdbool.h>

#include "app_config.h"
#include "board_pins.h"
#include "control.h"
#include "input_capture.h"
#include "output_pwm.h"
#include "safety.h"
#include "serial_cli.h"

extern TIM_HandleTypeDef htim2; /* free-running timestamp timer for input capture */
extern TIM_HandleTypeDef htim3; /* dual-channel PWM output timer */
extern TIM_HandleTypeDef htim10; /* periodic control-loop scheduler timer */
extern IWDG_HandleTypeDef hiwdg; /* watchdog refreshed by the main control loop */

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_TIM2_Init(void);
void MX_TIM3_Init(void);
void MX_TIM10_Init(void);
void MX_IWDG_Init(void);

static volatile bool control_tick_pending; /* set by TIM10 ISR when a control update is due */

static functional_mode_t requested_mode = FUNCTIONAL_MODE_PASSTHROUGH; /* currently requested operating mode */
static bool manual_override = false; /* true when CLI has overridden automatic mode switching */
static control_command_state_t command_state = { 0.0f, 0.0f }; /* command-mode target and ramp progress */
static control_filter_state_t filter_state; /* filtered duty-cycle measurements for both pedal channels */
static safety_state_t safety_state; /* current safety state machine data and fault flags */
static telemetry_snapshot_t telemetry_snapshot; /* latest telemetry sample published to the CLI */
static uint32_t rest_hold_elapsed_ms = 0U; /* accumulated rest time before auto-enabling active mode */
static uint32_t last_telemetry_ms = 0U; /* timestamp of the most recent telemetry print */

/* Converts a validated PWM capture snapshot into duty-cycle percentage. */
static float duty_from_capture(const pwm_capture_snapshot_t *capture, bool *valid)
{
    if (!capture->valid || (capture->period_us == 0U) || (capture->high_us >= capture->period_us)) {
        *valid = false;
        return 0.0f;
    }

    *valid = true;
    return ((float)capture->high_us * 100.0f) / (float)capture->period_us;
}

/* Drives relay GPIO to either active interception or passthrough state. */
static void set_relay(bool enabled)
{
    HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, enabled ? RELAY_ACTIVE_STATE : RELAY_INACTIVE_STATE);
}

/* Controls the level shifter power state and mirrors status into safety state. */
static void set_level_shifter(bool enabled)
{
    HAL_GPIO_WritePin(LEVEL_SHIFTER_GPIO_Port,
                      LEVEL_SHIFTER_Pin,
                      enabled ? LEVEL_SHIFTER_ACTIVE_STATE : LEVEL_SHIFTER_INACTIVE_STATE);
    safety_set_level_shifter_enabled(&safety_state, enabled);
}

/* Populates the telemetry structure with the latest measured and derived values. */
static void update_telemetry(const pwm_capture_snapshot_t *s2_capture,
                             const pwm_capture_snapshot_t *s4_capture,
                             bool s2_valid,
                             bool s4_valid,
                             bool s2_fresh,
                             bool s4_fresh,
                             float travel_pct,
                             float plausibility_error_pct)
{
    telemetry_snapshot.mode = requested_mode;
    telemetry_snapshot.state = safety_state.state;
    telemetry_snapshot.fault_bits = safety_state.fault_bits;
    telemetry_snapshot.relay_on = safety_relay_allowed(&safety_state);
    telemetry_snapshot.level_shifter_on = safety_state.level_shifter_enabled;
    telemetry_snapshot.s2_valid = s2_valid;
    telemetry_snapshot.s4_valid = s4_valid;
    telemetry_snapshot.s2_fresh = s2_fresh;
    telemetry_snapshot.s4_fresh = s4_fresh;
    telemetry_snapshot.s2_high_us = s2_capture->high_us;
    telemetry_snapshot.s2_period_us = s2_capture->period_us;
    telemetry_snapshot.s4_high_us = s4_capture->high_us;
    telemetry_snapshot.s4_period_us = s4_capture->period_us;
    telemetry_snapshot.s2_duty_pct = filter_state.s2_filtered_pct;
    telemetry_snapshot.s4_duty_pct = filter_state.s4_filtered_pct;
    telemetry_snapshot.travel_pct = travel_pct;
    telemetry_snapshot.plausibility_error_pct = plausibility_error_pct;
    telemetry_snapshot.startup_valid_elapsed_ms = safety_state.startup_valid_elapsed_ms;
    telemetry_snapshot.stale_events = safety_state.stale_events;
    telemetry_snapshot.rejected_periods = input_capture_get_total_rejected_periods();
    telemetry_snapshot.rejected_pulses = input_capture_get_total_rejected_pulses();
    telemetry_snapshot.plausibility_violations = safety_state.plausibility_violations;
    telemetry_snapshot.exti_edges = input_capture_get_total_edge_count();
}

/* Executes one 1 kHz control iteration from input sampling through output update. */
static void run_control_tick(void)
{
    pwm_capture_snapshot_t s2_capture; /* latest copied capture state for the S2 input */
    pwm_capture_snapshot_t s4_capture; /* latest copied capture state for the S4 input */
    bool s2_valid; /* true when the S2 capture converts into a usable duty cycle */
    bool s4_valid; /* true when the S4 capture converts into a usable duty cycle */
    const uint32_t now_ms = HAL_GetTick(); /* current millisecond tick used for telemetry scheduling */
    uint32_t now_us; /* current microsecond timer count used for freshness checks */
    float raw_s2_pct; /* raw S2 duty cycle derived from the latest capture snapshot */
    float raw_s4_pct; /* raw S4 duty cycle derived from the latest capture snapshot */
    float travel_pct; /* normalized pedal travel computed from filtered sensor duties */
    float plausibility_error_pct; /* summed sensor mismatch used by the safety plausibility check */
    float output_s2_pct; /* commanded S2 output duty after mode and safety selection */
    float output_s4_pct; /* commanded S4 output duty after mode and safety selection */
    bool s2_fresh; /* true when S2 has a recent valid sample */
    bool s4_fresh; /* true when S4 has a recent valid sample */

    /* Snapshot raw input captures and convert them into duty-cycle percentages. */
    input_capture_copy_snapshot(&s2_capture, &s4_capture);
    now_us = input_capture_get_timestamp_us();
    raw_s2_pct = duty_from_capture(&s2_capture, &s2_valid);
    raw_s4_pct = duty_from_capture(&s4_capture, &s4_valid);

    /* Determine whether each channel has recent valid data within stale timeout. */
    s2_fresh = s2_valid && ((now_us - s2_capture.last_valid_us) <= (INPUT_STALE_TIMEOUT_MS * 1000U));
    s4_fresh = s4_valid && ((now_us - s4_capture.last_valid_us) <= (INPUT_STALE_TIMEOUT_MS * 1000U));

    /* Filter inputs and derive travel/plausibility terms used by control and safety. */
    control_update_filtered_duties(&filter_state, raw_s2_pct, s2_valid, raw_s4_pct, s4_valid);
    travel_pct = control_compute_travel_pct(filter_state.s2_filtered_pct, filter_state.s4_filtered_pct);
    plausibility_error_pct = (filter_state.s2_filtered_pct + filter_state.s4_filtered_pct) - 100.0f;

    /* Auto-promote from passthrough to active mode after sustained pedal rest. */
    if (!manual_override && (requested_mode == FUNCTIONAL_MODE_PASSTHROUGH)) {
        if (control_pedal_at_rest(travel_pct)) {
            if (rest_hold_elapsed_ms < REST_HOLD_MS) {
                rest_hold_elapsed_ms += CONTROL_TICK_MS;
            }
            if (rest_hold_elapsed_ms >= REST_HOLD_MS) {
                requested_mode = FUNCTIONAL_MODE_ACTIVE;
            }
        } else {
            rest_hold_elapsed_ms = 0U;
        }
    }

    /* Advance command-mode ramp toward the operator-requested travel target. */
    if (requested_mode == FUNCTIONAL_MODE_COMMAND) {
        control_update_command_ramp(&command_state, 1.0f / (float)CONTROL_TICK_HZ);
    }

    /* Update safety state machine before selecting requested output behavior. */
    safety_update(&safety_state,
                  requested_mode,
                  s2_valid && s4_valid && s2_fresh && s4_fresh,
                  s2_fresh,
                  s4_fresh,
                  s2_valid,
                  s4_valid,
                  control_pedal_at_rest(travel_pct),
                  plausibility_error_pct,
                  CONTROL_TICK_MS);

    /* Choose output duties based on fault status and selected functional mode. */
    if (safety_fault_latched(&safety_state)) {
        output_s2_pct = S2_REST_DUTY_PCT;
        output_s4_pct = S4_REST_DUTY_PCT;
    } else if (requested_mode == FUNCTIONAL_MODE_ACTIVE) {
        const float spoof_travel_pct = control_lookup_curve_pct(travel_pct); /* remapped travel for assisted output */
        control_travel_to_duty_pct(spoof_travel_pct, &output_s2_pct, &output_s4_pct);
    } else if (requested_mode == FUNCTIONAL_MODE_COMMAND) {
        control_travel_to_duty_pct(command_state.current_travel_pct, &output_s2_pct, &output_s4_pct);
    } else {
        output_s2_pct = filter_state.s2_filtered_pct;
        output_s4_pct = filter_state.s4_filtered_pct;
    }

    /* Apply outputs and capture a telemetry snapshot for periodic diagnostics. */
    output_pwm_set_duty_pct(&htim3, output_s2_pct, output_s4_pct);
    set_relay(safety_relay_allowed(&safety_state));
    update_telemetry(&s2_capture,
                     &s4_capture,
                     s2_valid,
                     s4_valid,
                     s2_fresh,
                     s4_fresh,
                     travel_pct,
                     plausibility_error_pct);

    /* Emit telemetry at the configured reporting interval. */
    if ((now_ms - last_telemetry_ms) >= TELEMETRY_PRINT_INTERVAL_MS) {
        serial_cli_write_telemetry(&telemetry_snapshot);
        last_telemetry_ms = now_ms;
    }
}

/* Initializes MCU peripherals and runs the foreground loop scheduler. */
int main(void)
{
    /* Initialize HAL, clocks, and generated peripheral instances. */
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM10_Init();
    MX_IWDG_Init();

    /* Initialize application modules after hardware setup. */
    input_capture_init();
    control_filter_init(&filter_state);
    safety_init(&safety_state);
    serial_cli_init();

    /* Force safe output state before enabling timers and power paths. */
    set_relay(false);
    set_level_shifter(false);

    /* Start timers and drive PWM outputs to safe startup duty values. */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    output_pwm_init_safe_state(&htim3);
    HAL_TIM_Base_Start(&htim2);
    HAL_TIM_Base_Start_IT(&htim10);

    /* Enable level shifter and print CLI banner for operator interaction. */
    set_level_shifter(true);
    last_telemetry_ms = HAL_GetTick();
    serial_cli_print_banner();

    /* Poll CLI continuously and run the control task when tick flag is set. */
    while (1) {
        bool reset_auto_state = false; /* requests reset of zero-detect auto-activation timing */

        serial_cli_poll(&requested_mode, &manual_override, &command_state, &reset_auto_state);
        if (reset_auto_state) {
            rest_hold_elapsed_ms = 0U;
        }

        if (control_tick_pending) {
            control_tick_pending = false;
            run_control_tick();
            HAL_IWDG_Refresh(&hiwdg);
        }
    }
}

/* Forwards GPIO EXTI interrupts to the PWM input-capture module. */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    input_capture_handle_gpio_exti(GPIO_Pin);
}

/* Sets the control tick flag on each TIM10 update interrupt. */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM10) {
        control_tick_pending = true;
    }
}
