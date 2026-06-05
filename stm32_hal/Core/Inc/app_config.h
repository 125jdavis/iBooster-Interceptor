#ifndef APP_CONFIG_H
#define APP_CONFIG_H /* Include guard for app_config.h */

#include <stdint.h>

#define S2_REST_DUTY_PCT                    83.0f /* Sensor 2 duty at pedal rest (%) */
#define S2_FULL_DUTY_PCT                    61.0f /* Sensor 2 duty at full pedal (%) */
#define S4_REST_DUTY_PCT                    17.0f /* Sensor 4 duty at pedal rest (%) */
#define S4_FULL_DUTY_PCT                    39.0f /* Sensor 4 duty at full pedal (%) */

#define REST_TOLERANCE_PCT                  2.0f /* Allowed deviation when considering pedal at rest (%) */
#define REST_HOLD_MS                        100U /* Required stable rest time before state transitions (ms) */
#define STARTUP_VALID_MS                    50U /* Minimum valid-input duration needed at startup (ms) */
#define PLAUSIBILITY_TOLERANCE_PCT          3.0f /* Max allowed S2/S4 travel disagreement (%) */
#define PLAUSIBILITY_FAULT_MS               500U /* Time threshold before latching plausibility fault (ms) */
#define INPUT_STALE_TIMEOUT_MS              10U /* Max age of input sample before marking stale (ms) */

#define PWM_PERIOD_MIN_US                   900U /* Minimum accepted input PWM period (us) */
#define PWM_PERIOD_MAX_US                   1100U /* Maximum accepted input PWM period (us) */
#define CONTROL_TICK_HZ                     1000U /* Control-loop update frequency (Hz) */
#define CONTROL_TICK_MS                     1U /* Control-loop period (ms) */
#define DIAGNOSTIC_PRINT_INTERVAL_MS        20U /* Serial diagnostics print interval (ms) */
#define COMMAND_RAMP_RATE_PCT_PER_SEC       25.0f /* Commanded travel ramp limit (% per second) */
#define EMA_ALPHA                           0.8f /* Exponential moving average smoothing factor */
#define SERIAL_CMD_BUFFER_SIZE              32U /* CLI command input buffer size (bytes) */

#define CAL_POINT_COUNT                     6U /* Number of calibration curve points */

extern const float CAL_IN_PCT[CAL_POINT_COUNT];
extern const float CAL_OUT_PCT[CAL_POINT_COUNT];

#endif
