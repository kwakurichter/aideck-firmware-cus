#pragma once
#include "vehicle_state.h"
#include <stdbool.h>

// Creates and starts the FreeRTOS task that runs the custom mission.
void mission_start(VehicleState_t *vehicle_state);

bool mission_is_start_commanded(void);

void mission_signal_start(void);