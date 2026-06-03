#ifndef INPUT_CAPTURE_H
#define INPUT_CAPTURE_H

#include <stdbool.h>
#include <stdint.h>

#include "main.h"

typedef struct {
    uint32_t last_rise_us;
    uint32_t last_fall_us;
    uint32_t period_us;
    uint32_t high_us;
    uint32_t last_edge_us;
    uint32_t last_valid_us;
    uint32_t edge_count;
    uint32_t rejected_periods;
    uint32_t rejected_high_pulses;
    bool valid;
    bool new_sample;
} pwm_capture_snapshot_t;

void input_capture_init(void);
void input_capture_handle_gpio_exti(uint16_t gpio_pin);
void input_capture_copy_snapshot(pwm_capture_snapshot_t *s2, pwm_capture_snapshot_t *s4);
uint32_t input_capture_get_timestamp_us(void);
uint32_t input_capture_get_total_edge_count(void);
uint32_t input_capture_get_total_rejected_periods(void);
uint32_t input_capture_get_total_rejected_pulses(void);

#endif
