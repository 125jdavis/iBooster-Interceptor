#include "input_capture.h"

#include <string.h>

#include "app_config.h"
#include "board_pins.h"

extern TIM_HandleTypeDef htim2;

typedef struct {
    volatile pwm_capture_snapshot_t snapshot;
} pwm_capture_channel_t;

static pwm_capture_channel_t s2_channel;
static pwm_capture_channel_t s4_channel;
static volatile uint32_t total_edge_count;
static volatile uint32_t total_rejected_periods;
static volatile uint32_t total_rejected_pulses;

static void reset_channel(pwm_capture_channel_t *channel)
{
    memset((void *)&channel->snapshot, 0, sizeof(channel->snapshot));
}

static void handle_edge(pwm_capture_channel_t *channel, GPIO_TypeDef *gpio_port, uint16_t gpio_pin, uint32_t now_us)
{
    const GPIO_PinState pin_state = HAL_GPIO_ReadPin(gpio_port, gpio_pin);

    channel->snapshot.last_edge_us = now_us;
    channel->snapshot.edge_count++;
    total_edge_count++;

    if (pin_state == GPIO_PIN_SET) {
        if (channel->snapshot.last_rise_us != 0U) {
            const uint32_t period_us = now_us - channel->snapshot.last_rise_us;
            if ((period_us >= PWM_PERIOD_MIN_US) && (period_us <= PWM_PERIOD_MAX_US)) {
                channel->snapshot.period_us = period_us;
            } else {
                channel->snapshot.rejected_periods++;
                total_rejected_periods++;
            }
        }

        channel->snapshot.last_rise_us = now_us;
        return;
    }

    if ((channel->snapshot.last_rise_us == 0U) || (channel->snapshot.period_us == 0U)) {
        return;
    }

    const uint32_t high_us = now_us - channel->snapshot.last_rise_us;
    if ((high_us > 0U) && (high_us < channel->snapshot.period_us)) {
        channel->snapshot.last_fall_us = now_us;
        channel->snapshot.high_us = high_us;
        channel->snapshot.last_valid_us = now_us;
        channel->snapshot.valid = true;
        channel->snapshot.new_sample = true;
    } else {
        channel->snapshot.rejected_high_pulses++;
        total_rejected_pulses++;
    }
}

void input_capture_init(void)
{
    __disable_irq();
    reset_channel(&s2_channel);
    reset_channel(&s4_channel);
    total_edge_count = 0U;
    total_rejected_periods = 0U;
    total_rejected_pulses = 0U;
    __enable_irq();
}

void input_capture_handle_gpio_exti(uint16_t gpio_pin)
{
    const uint32_t now_us = __HAL_TIM_GET_COUNTER(&htim2);

    if (gpio_pin == S2_IN_Pin) {
        handle_edge(&s2_channel, S2_IN_GPIO_Port, S2_IN_Pin, now_us);
    } else if (gpio_pin == S4_IN_Pin) {
        handle_edge(&s4_channel, S4_IN_GPIO_Port, S4_IN_Pin, now_us);
    }
}

void input_capture_copy_snapshot(pwm_capture_snapshot_t *s2, pwm_capture_snapshot_t *s4)
{
    __disable_irq();
    *s2 = s2_channel.snapshot;
    *s4 = s4_channel.snapshot;
    s2_channel.snapshot.new_sample = false;
    s4_channel.snapshot.new_sample = false;
    __enable_irq();
}

uint32_t input_capture_get_timestamp_us(void)
{
    return __HAL_TIM_GET_COUNTER(&htim2);
}

uint32_t input_capture_get_total_edge_count(void)
{
    return total_edge_count;
}

uint32_t input_capture_get_total_rejected_periods(void)
{
    return total_rejected_periods;
}

uint32_t input_capture_get_total_rejected_pulses(void)
{
    return total_rejected_pulses;
}
