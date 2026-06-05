#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim10;
extern UART_HandleTypeDef huart1;
extern IWDG_HandleTypeDef hiwdg;

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_TIM2_Init(void);
void MX_TIM3_Init(void);
void MX_TIM10_Init(void);
void MX_USART1_UART_Init(void);
void MX_IWDG_Init(void);

void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif
