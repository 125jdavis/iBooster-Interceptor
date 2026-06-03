#ifndef BOARD_PINS_H
#define BOARD_PINS_H

#include "main.h"

#define S2_IN_GPIO_Port          GPIOB
#define S2_IN_Pin                GPIO_PIN_1
#define S4_IN_GPIO_Port          GPIOB
#define S4_IN_Pin                GPIO_PIN_0
#define RELAY_GPIO_Port          GPIOB
#define RELAY_Pin                GPIO_PIN_10
#define LEVEL_SHIFTER_GPIO_Port  GPIOA
#define LEVEL_SHIFTER_Pin        GPIO_PIN_5

#define RELAY_ACTIVE_STATE            GPIO_PIN_SET
#define RELAY_INACTIVE_STATE          GPIO_PIN_RESET
#define LEVEL_SHIFTER_ACTIVE_STATE    GPIO_PIN_SET
#define LEVEL_SHIFTER_INACTIVE_STATE  GPIO_PIN_RESET

#endif
