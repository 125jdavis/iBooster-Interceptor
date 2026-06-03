#include "main.h"

#include <stdbool.h>

#include "app_config.h"
#include "board_pins.h"
#include "control.h"
#include "input_capture.h"
#include "output_pwm.h"
#include "safety.h"
#include "serial_cli.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim10;
extern IWDG_HandleTypeDef hiwdg;

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_TIM2_Init(void);
void MX_TIM3_Init(void);
void MX_TIM10_Init(void);
void MX_IWDG_Init(void);

static volatile bool control_tick_pending;

static functional_mode_t requested_mode = FUNCTIONAL_MODE_PASSTHROUGH;
static bool manual_override;
static control_command_state_t command_state;
static control_filter_state_t filter_state;
static safety_state_t safety_state;
static telemetry_snapshot_t telemetry_snapshot;
static uint32_t rest_hold_elapsed_ms;
static uint32_t last_telemetry_ms;

static float duty_from_capture(const pwm_capture_snapshot_t *capture, bool *valid)
{
    if (!capture->valid || (capture->period_us == 0U) || (capture->high_us >= capture->period_us)) {
        *valid = false;
        return 0.0f;
    }

    *valid = true;
    return ((float)capture->high_us * 100.0f) / (float)capture->period_us;
}

static void set_relay(bool enabled)
{
    HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, enabled ? RELAY_ACTIVE_STATE : RELAY_INACTIVE_STATE);
}

static void set_level_shifter(bool enabled)
{
    HAL_GPIO_WritePin(LEVEL_SHIFTER_GPIO_Port,
                      LEVEL_SHIFTER_Pin,
                      enabled ? LEVEL_SHIFTER_ACTIVE_STATE : LEVEL_SHIFTER_INACTIVE_STATE);
    safety_set_level_shifter_enabled(&safety_state, enabled);
}

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

static void run_control_tick(void)
{
    pwm_capture_snapshot_t s2_capture;
    pwm_capture_snapshot_t s4_capture;
    bool s2_valid;
    bool s4_valid;
    const uint32_t now_ms = HAL_GetTick();
    uint32_t now_us;
    float raw_s2_pct;
    float raw_s4_pct;
    float travel_pct;
    float plausibility_error_pct;
    float output_s2_pct;
    float output_s4_pct;
    bool s2_fresh;
    bool s4_fresh;

    input_capture_copy_snapshot(&s2_capture, &s4_capture);
    now_us = input_capture_get_timestamp_us();
    raw_s2_pct = duty_from_capture(&s2_capture, &s2_valid);
    raw_s4_pct = duty_from_capture(&s4_capture, &s4_valid);

    s2_fresh = s2_valid && ((now_us - s2_capture.last_valid_us) <= (INPUT_STALE_TIMEOUT_MS * 1000U));
    s4_fresh = s4_valid && ((now_us - s4_capture.last_valid_us) <= (INPUT_STALE_TIMEOUT_MS * 1000U));

    control_update_filtered_duties(&filter_state, raw_s2_pct, s2_valid, raw_s4_pct, s4_valid);
    travel_pct = control_compute_travel_pct(filter_state.s2_filtered_pct, filter_state.s4_filtered_pct);
    plausibility_error_pct = (filter_state.s2_filtered_pct + filter_state.s4_filtered_pct) - 100.0f;

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

    if (requested_mode == FUNCTIONAL_MODE_COMMAND) {
        control_update_command_ramp(&command_state, 1.0f / (float)CONTROL_TICK_HZ);
    }

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

    if (safety_fault_latched(&safety_state)) {
        output_s2_pct = S2_REST_DUTY_PCT;
        output_s4_pct = S4_REST_DUTY_PCT;
    } else if (requested_mode == FUNCTIONAL_MODE_ACTIVE) {
        const float spoof_travel_pct = control_lookup_curve_pct(travel_pct);
        control_travel_to_duty_pct(spoof_travel_pct, &output_s2_pct, &output_s4_pct);
    } else if (requested_mode == FUNCTIONAL_MODE_COMMAND) {
        control_travel_to_duty_pct(command_state.current_travel_pct, &output_s2_pct, &output_s4_pct);
    } else {
        output_s2_pct = filter_state.s2_filtered_pct;
        output_s4_pct = filter_state.s4_filtered_pct;
    }

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

    if ((now_ms - last_telemetry_ms) >= TELEMETRY_PRINT_INTERVAL_MS) {
        serial_cli_write_telemetry(&telemetry_snapshot);
        last_telemetry_ms = now_ms;
    }
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM10_Init();
    MX_IWDG_Init();

    input_capture_init();
    control_filter_init(&filter_state);
    safety_init(&safety_state);
    serial_cli_init();

    set_relay(false);
    set_level_shifter(false);

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    output_pwm_init_safe_state(&htim3);
    HAL_TIM_Base_Start(&htim2);
    HAL_TIM_Base_Start_IT(&htim10);

    set_level_shifter(true);
    serial_cli_print_banner();

    while (1) {
        bool reset_auto_state = false;

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

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    input_capture_handle_gpio_exti(GPIO_Pin);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM10) {
        control_tick_pending = true;
    }
}
