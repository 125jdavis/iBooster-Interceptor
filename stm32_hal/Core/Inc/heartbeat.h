#ifndef HEARTBEAT_H
#define HEARTBEAT_H /* Include guard for heartbeat.h */

#include <stdint.h>

#include "app_types.h"

void heartbeat_init(void);
void heartbeat_update(system_state_t state, uint32_t dt_ms);

#endif
