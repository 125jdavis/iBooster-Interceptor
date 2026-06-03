# iBooster-Interceptor

Signal interceptor to allow calibration of the iBooster brake pedal assist curve.

## Repository layout

- `/stm32_hal` - new STM32F411CEU6 HAL firmware source tree based on the documentation in `/Documentation`
- `/archive/arduino/interceptor.ino` - archived Arduino Nano implementation
- `/Documentation` - STM32 firmware specification and CubeMX/HAL implementation notes

## STM32 firmware notes

The STM32 implementation follows the documented split-timer architecture:

- TIM2 free-running 1 MHz timestamp base for EXTI edge measurement
- TIM3 dual-channel 1 kHz PWM output generation on PA6/PA7
- TIM10 1 kHz control-loop tick
- fault-latched safety logic with relay-default passthrough behavior
- preserved legacy serial command interface (`p`, `a`, `r`, `c`, and numeric command targets)

The source tree is organized so it can be dropped into a CubeMX-generated STM32F4 HAL project and wired to the generated peripheral initialization code.
