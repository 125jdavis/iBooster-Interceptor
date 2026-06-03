# iBooster Interceptor Firmware Specification
Target MCU: **STM32F411CEU6**  
Implementation style: **STM32 HAL**  
Numeric strategy: **integer timing in ISR/capture paths, floating point in control logic**  
Primary safety posture: **default passthrough, fault latch until reset**

---

## 1. Purpose

This firmware measures two PWM pedal sensor signals from the iBooster (`S2`, `S4`), validates them, determines pedal travel, and optionally generates modified PWM outputs according to a calibration curve. A relay selects either:

- **Passthrough**: original sensor signals routed directly through hardware
- **Active/Command**: MCU-generated modified PWM routed to output

The STM32 implementation is intended to eliminate timing problems observed on the Arduino Nano version, where interrupt collisions occasionally corrupted measured high-time values.

---

## 2. Design Goals

1. **Eliminate timing interference between channels**
2. **Keep fail-safe behavior hardware-first**
3. **Use deterministic timing architecture**
4. **Preserve current control behavior initially**
5. **Retain current serial command interface**
6. **Support future calibration/configuration over serial**
7. **Provide strong debug visibility for bench validation**

---

## 3. Key Design Decisions and Rationale

### 3.1 Split-timer architecture
**Decision:** Separate output generation, input timestamping, and control scheduling.

**Why:**
- avoids the Nano-era problem of competing timing responsibilities
- easier to understand and debug than a one-timer mixed-mode design
- avoids coupling PWM generation configuration to input measurement timing
- safer and more maintainable

### 3.2 EXTI + free-running timer for input measurement
**Decision:** Use EXTI on `PB0/PB1` and timestamp edges from a dedicated 1 MHz timer.

**Why:**
- much more deterministic than Arduino `attachInterrupt()` + `micros()`
- simpler bring-up than mixed hardware capture on shared timer channels
- enough precision for ~1 kHz PWM inputs
- keeps ISR work minimal and integer-only

### 3.3 TIM3 for generated PWM
**Decision:** Use TIM3 CH1/CH2 on `PA6/PA7` for output PWM.

**Why:**
- native AF2 mapping
- paired channels
- clean preload-based synchronous updates
- good fit for fixed-frequency dual-channel output

### 3.4 Float math only in control layer
**Decision:** Use integer microsecond timing in low-level measurement, float in control calculations.

**Why:**
- preserves current algorithm behavior with minimal conversion risk
- STM32F411 performance is sufficient
- avoids float use in ISR paths while keeping calibration logic readable

### 3.5 Hardware-safe boot behavior
**Decision:** Boot in passthrough with relay de-energized and activate modified path only after validated startup conditions.

**Why:**
- brake-related path should fail to original sensor routing
- avoids switching into modified output until signals are known-good
- aligns with board relay NC safe state

---

## 4. Hardware Interface

### 4.1 Pin assignment
- `PB1` = `S2_IN`
- `PB0` = `S4_IN`
- `PA7` = `S2_OUT`
- `PA6` = `S4_OUT`
- `PB10` = `PIN_SWT_SRC` / relay control
- `PA5` = level shifter enable

### 4.2 Electrical behavior
- `S2_IN` / `S4_IN`: 5 V sensor PWM translated to 3.3 V MCU domain
- `S2_OUT` / `S4_OUT`: 3.3 V MCU PWM translated to 5 V ECU domain
- level shifter OE:
  - low = disabled
  - high = enabled
- relay safe default:
  - unpowered = NC = passthrough

---

## 5. Signal Assumptions

### 5.1 Input signals
- nominal frequency: **1000 Hz**
- observed drift: small, on order of a few Hz
- S2 duty:
  - rest ≈ **83%**
  - full ≈ **61%**
- S4 duty:
  - rest ≈ **17%**
  - full ≈ **39%**
- S2 and S4:
  - always present together
  - phase independent
  - no fixed phase relationship

### 5.2 Plausibility rule
Expected:
- `S2 duty + S4 duty ≈ 100%`

Fault condition:
- if deviation exceeds **±3%**
- continuously for more than **500 ms**
- latch fault until reset

---

## 6. Firmware Modes and States

## 6.1 Functional modes
- `PASSTHROUGH`
- `ACTIVE`
- `COMMAND`

These mirror current firmware behavior.

### 6.2 System states
Recommended implementation:
- `BOOT`
- `WAIT_FOR_VALID_INPUT`
- `PASSTHROUGH_READY`
- `ACTIVE`
- `COMMAND`
- `FAULT_LATCHED`

State purpose:
- `BOOT`: initialize peripherals safely
- `WAIT_FOR_VALID_INPUT`: level shifter enabled, waiting for stable valid signals
- `PASSTHROUGH_READY`: valid signals observed, still hardware passthrough
- `ACTIVE`: relay energized, generated output follows lookup curve
- `COMMAND`: relay energized, generated output follows commanded target
- `FAULT_LATCHED`: relay off, modified path disallowed until reset

---

## 7. Timer and Interrupt Architecture

### 7.1 TIM3 — PWM output
Use:
- `TIM3_CH1 -> PA6 -> S4_OUT`
- `TIM3_CH2 -> PA7 -> S2_OUT`

Configuration:
- fixed PWM frequency near **1 kHz**
- preload enabled on CCR registers
- update new duty values at timer rollover

Justification:
- synchronous updates prevent output glitches
- output generation is fully hardware-timed

### 7.2 TIM2 — free-running timestamp timer
Use:
- free-running counter at **1 MHz**
- 1 tick = 1 µs

Justification:
- simple period/high-time math
- stable hardware timebase
- no dependence on HAL tick or `micros()`-style software abstraction

### 7.3 EXTI — input edge detection
Use:
- `PB0` EXTI for `S4_IN`
- `PB1` EXTI for `S2_IN`
- interrupt on both rising and falling edges

ISR work:
- read `TIM2` count immediately
- determine pin state / edge polarity
- update raw measurement bookkeeping
- set flags only

Do **not** do:
- float math
- serial output
- calibration lookup
- mode changes

### 7.4 Control loop timer
Use:
- dedicated 1 kHz periodic interrupt (preferred: `TIM10`)
- acceptable fallback: `SysTick`

Purpose:
- periodic deterministic control/update loop
- separates control rate from asynchronous edge events

---

## 8. Startup Sequence

1. reset occurs
2. relay output forced **off**
3. level shifter forced **disabled**
4. initialize GPIO
5. initialize TIM3 output PWM with safe/rest values
6. initialize TIM2 free-running 1 MHz counter
7. initialize EXTI on `PB0/PB1`
8. initialize 1 kHz control timer
9. enable watchdog
10. enable level shifter (`PA5 = high`)
11. begin input validation window
12. require:
    - valid captures continuously for at least **50 ms**
    - pedal at rest
    - no fault active
13. once satisfied, allow transition from passthrough to active mode when logic requests it

Notes:
- relay remains off during startup validation
- passthrough is maintained physically through the NC relay path

---

## 9. Fault Handling

### 9.1 Fault response
On any latched fault:
- de-energize relay
- enter `FAULT_LATCHED`
- reject `ACTIVE` and `COMMAND`
- keep reporting diagnostics
- clear only on reset/power cycle

### 9.2 Fault sources
Minimum required:
1. **Plausibility fault**
   - `abs((S2 + S4) - 100) > 3%` for >500 ms
2. **Stale input fault**
   - no fresh valid sample within timeout window
3. **Invalid period fault**
   - input period outside accepted bounds for sustained interval
4. **Startup validation failure**
   - inputs never become stable/valid within implementation-defined timeout, if desired

### 9.3 Recommended stale timeout
Use a timeout on order of **5–10 ms** for missing fresh samples.

Reason:
- nominal input is 1 kHz, so this allows multiple missed cycles before faulting

---

## 10. Input Measurement Strategy

### 10.1 Per-channel raw data
Each channel shall maintain:
- last rising timestamp
- last falling timestamp
- latest valid period (µs)
- latest valid high time (µs)
- last edge timestamp
- valid flag
- new sample flag
- reject/fault counters

### 10.2 Edge handling
On rising edge:
- `period = now - previous_rise`
- if plausible, update `period_us`
- store `last_rise_us`

On falling edge:
- `high = now - last_rise_us`
- if plausible and `< period_us`, update `high_us`
- mark new sample ready

### 10.3 Period acceptance
Initial implementation should use a tolerance window around 1 kHz, for example:
- lower bound: configurable
- upper bound: configurable

Because actual drift range is not fully characterized, these bounds should be easy to adjust.

---

## 11. Control Loop Behavior

Runs at **1 kHz**.

Per tick:
1. atomically copy latest raw channel measurements
2. verify freshness and validity
3. compute raw duty % for each channel
4. apply EMA filtering
5. compute pedal travel
6. evaluate plausibility/fault logic
7. update startup-valid timer
8. execute mode/state machine
9. compute desired output duties
10. update TIM3 compare registers
11. update relay command

---

## 12. Control Algorithm

### 12.1 Existing constants to preserve
Initial STM32 version shall preserve:
- `S2_REST`
- `S2_FULL`
- `S4_REST`
- `S4_FULL`
- `REST_TOLERANCE`
- `CAL_IN`
- `CAL_OUT`
- EMA behavior
- command ramp behavior

### 12.2 Travel calculation
Use existing method:
- derive travel from both S2 and S4
- average the two
- constrain to 0–100%

### 12.3 Filtering
Preserve existing EMA behavior, but apply it in the control loop, not inside edge ISRs.

Justification:
- removes float operations from interrupt context
- easier to debug and tune

### 12.4 Output generation
- `ACTIVE`: output lookup-curve transformed travel
- `COMMAND`: output commanded travel target with ramp
- `PASSTHROUGH`: generated outputs may continue running but relay remains off, so they are electrically unused

---

## 13. Relay Control Policy

### 13.1 Safe rule
Relay shall be energized only if all are true:
- no fault latched
- startup validation complete
- pedal at rest at activation time
- mode requires modified path (`ACTIVE` or `COMMAND`)

### 13.2 Relay behavior
- boot: off
- fault: off
- passthrough: off
- active/command: on only when allowed

### 13.3 Startup delay
Minimum startup stability requirement before relay activation:
- **50 ms of valid signal monitoring**

---

## 14. Level Shifter Policy

### 14.1 Boot behavior
- reset/early boot: disabled
- enable after peripherals are configured

### 14.2 Runtime behavior
- enabled during normal operation so sensor inputs can be read
- optional to disable on latched fault, but not required if relay already forces passthrough

Recommended initial behavior:
- leave enabled after startup unless a specific reason emerges to disable on fault

---

## 15. Serial Interface

### 15.1 Initial requirement
Keep current serial command interface functionally unchanged.

Commands to preserve:
- `p` -> manual passthrough
- `a` -> manual active
- `r` -> resume automatic behavior
- `c` -> command mode
- numeric target / `c <value>` -> set command target

### 15.2 Future extension
Reserve room for:
- serial calibration updates
- configuration storage
- debug/fault counters
- telemetry streaming modes

Recommended future-ready strategy:
- line-oriented command parser
- keyword/value commands in addition to single-letter legacy commands

---

## 16. Watchdog

### 16.1 Requirement
Enable watchdog from first STM32 version.

### 16.2 Policy
- feed watchdog only from healthy main/control execution path
- do not feed from low-level ISR only
- ensure a hung control loop results in reset

Recommended:
- feed in main loop or periodic health-supervised context

---

## 17. Debug and Telemetry Requirements

The first firmware version shall expose enough information for bench validation.

Minimum debug data:
- mode/state
- relay state
- level shifter state
- S2 high time / period / duty
- S4 high time / period / duty
- computed travel
- valid/stale flags
- plausibility deviation
- startup-valid timer status
- fault cause bitmask
- counters for:
  - stale events
  - rejected periods
  - rejected pulse widths
  - plausibility violations
  - EXTI edge counts

Justification:
- required to validate that timing issues are truly resolved
- required to tune period bounds and startup logic on real hardware

---

## 18. Suggested Software Modules

### `board_pins.h`
- pin names
- GPIO polarity
- safe defaults

### `input_capture.h/.c`
- EXTI handling
- TIM2 timestamp reads
- raw measurement structures
- validity bookkeeping

### `output_pwm.h/.c`
- TIM3 setup
- CCR update functions
- duty-to-compare conversion

### `control.h/.c`
- EMA filtering
- travel computation
- calibration lookup
- command ramp

### `safety.h/.c`
- startup validation
- fault detection/latching
- relay permission logic

### `serial_cli.h/.c`
- command parsing
- diagnostics output

### `main.c`
- init order
- main loop
- watchdog servicing
- integration glue

---

## 19. Data Structures

Minimum per-channel capture structure:

```c
typedef struct {
    volatile uint32_t last_rise_us;
    volatile uint32_t last_fall_us;
    volatile uint32_t period_us;
    volatile uint32_t high_us;
    volatile uint32_t last_edge_us;
    volatile uint8_t valid;
    volatile uint8_t new_sample;
} pwm_capture_t;
```

Recommended fault/state structure:
- fault bitmask
- fault latched flag
- startup valid elapsed ms
- plausibility bad elapsed ms
- relay allowed flag
- current system state
- current functional mode

---

## 20. Acceptance Criteria

Firmware is acceptable when all are true:

1. boots into physical passthrough
2. level shifter enables only after MCU initialization
3. input measurement remains stable with no Nano-style duty spikes
4. startup requires at least 50 ms of valid signals before modified path is allowed
5. active path engages only when pedal is at rest
6. plausibility fault latches when `S2 + S4` deviates from `100 ± 3%` for >500 ms
7. fault drops relay to passthrough and remains latched until reset
8. command mode and existing serial interface behave as before
9. watchdog is active
10. debug telemetry exposes sufficient data for bench validation

---

## 21. Implementation Notes

- keep ISR code extremely short
- avoid floats in ISR paths
- use preload on PWM outputs
- treat all timing values internally as microseconds where possible
- make thresholds configurable constants
- prefer explicit state transitions over implicit mode changes
- document fault causes in a bitmask for diagnostics

---

## 22. Summary

This firmware shall replace the Arduino Nano interrupt-driven timing approach with a more deterministic STM32 architecture:

- **TIM3** generates output PWM
- **TIM2** provides a 1 MHz hardware timestamp base
- **EXTI** measures input edges with low ISR overhead
- **1 kHz control loop** performs filtering, travel calculation, safety checks, and output updates
- system defaults to **hardware passthrough**
- faults **latch until reset**

This architecture is selected because it minimizes timing interference, preserves safety behavior, and remains straightforward to implement and debug using STM32 HAL.