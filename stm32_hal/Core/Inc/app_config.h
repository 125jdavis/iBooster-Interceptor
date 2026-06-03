#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <stdint.h>

#define S2_REST_DUTY_PCT                    83.0f
#define S2_FULL_DUTY_PCT                    61.0f
#define S4_REST_DUTY_PCT                    17.0f
#define S4_FULL_DUTY_PCT                    39.0f

#define REST_TOLERANCE_PCT                  2.0f
#define REST_HOLD_MS                        100U
#define STARTUP_VALID_MS                    50U
#define PLAUSIBILITY_TOLERANCE_PCT          3.0f
#define PLAUSIBILITY_FAULT_MS               500U
#define INPUT_STALE_TIMEOUT_MS              10U

#define PWM_PERIOD_MIN_US                   900U
#define PWM_PERIOD_MAX_US                   1100U
#define CONTROL_TICK_HZ                     1000U
#define CONTROL_TICK_MS                     1U
#define TELEMETRY_PRINT_INTERVAL_MS         20U
#define COMMAND_RAMP_RATE_PCT_PER_SEC       25.0f
#define EMA_ALPHA                           0.8f
#define SERIAL_CMD_BUFFER_SIZE              32U

#define CAL_POINT_COUNT                     6U

static const float CAL_IN_PCT[CAL_POINT_COUNT] = { 0.0f, 20.0f, 40.0f, 60.0f, 80.0f, 100.0f };
static const float CAL_OUT_PCT[CAL_POINT_COUNT] = { 0.0f, 20.0f, 40.0f, 60.0f, 80.0f, 100.0f };

#endif
