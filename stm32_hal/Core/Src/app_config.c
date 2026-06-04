#include "app_config.h"

/* Defines pedal travel calibration input breakpoints used by the lookup curve. */
const float CAL_IN_PCT[CAL_POINT_COUNT] = { 0.0f, 20.0f, 40.0f, 60.0f, 80.0f, 100.0f }; /* input travel breakpoints for curve interpolation */
/* Defines calibrated output travel values mapped from the input breakpoints. */
const float CAL_OUT_PCT[CAL_POINT_COUNT] = { 0.0f, 20.0f, 40.0f, 60.0f, 80.0f, 100.0f }; /* output travel values paired with each breakpoint */
