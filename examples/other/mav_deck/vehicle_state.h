#pragma once
#include <stdbool.h>

// Holds all relevant state information received from the drone.
// A pointer to this struct can be passed to any task that needs it.
typedef struct {
    bool is_armed;
    float position_ned[3]; // North, East, Down in meters
    float altitude_m;      // Calculated from z position
    // Add other state variables here as needed (e.g., attitude, battery)
} VehicleState_t;