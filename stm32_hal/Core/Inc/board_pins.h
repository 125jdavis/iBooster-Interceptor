#ifndef BOARD_PINS_H
#define BOARD_PINS_H /* Include guard for board_pins.h */

#include "main.h"

#define S2_IN_GPIO_Port          GPIOB /* GPIO port for S2 PWM input */
#define S2_IN_Pin                GPIO_PIN_1 /* GPIO pin for S2 PWM input */
#define S4_IN_GPIO_Port          GPIOB /* GPIO port for S4 PWM input */
#define S4_IN_Pin                GPIO_PIN_0 /* GPIO pin for S4 PWM input */
#define RELAY_GPIO_Port          GPIOB /* GPIO port driving safety relay */
#define RELAY_Pin                GPIO_PIN_10 /* GPIO pin driving safety relay */
#define LEVEL_SHIFTER_GPIO_Port  GPIOA /* GPIO port enabling output level shifter */
#define LEVEL_SHIFTER_Pin        GPIO_PIN_5 /* GPIO pin enabling output level shifter */
#define HEARTBEAT_LED_GPIO_Port  GPIOC /* GPIO port for heartbeat LED */
#define HEARTBEAT_LED_Pin        GPIO_PIN_13 /* GPIO pin for heartbeat LED */

#define RELAY_ACTIVE_STATE            GPIO_PIN_SET /* Relay command state for active/intercept mode */
#define RELAY_INACTIVE_STATE          GPIO_PIN_RESET /* Relay command state for passive/passthrough mode */
#define LEVEL_SHIFTER_ACTIVE_STATE    GPIO_PIN_SET /* Level shifter state when outputs are enabled */
#define LEVEL_SHIFTER_INACTIVE_STATE  GPIO_PIN_RESET /* Level shifter state when outputs are disabled */
#define HEARTBEAT_LED_ON_STATE        GPIO_PIN_RESET /* Blackpill PC13 LED is active-low */
#define HEARTBEAT_LED_OFF_STATE       GPIO_PIN_SET /* Blackpill PC13 LED off state */

#endif
