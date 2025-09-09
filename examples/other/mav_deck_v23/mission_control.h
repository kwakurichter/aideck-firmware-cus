#pragma once
#include "vehicle_state.h"
#include <stdbool.h>

void autonomous_mission_task(void *parameters);

bool mission_is_start_commanded(void);

void mission_signal_start(void);