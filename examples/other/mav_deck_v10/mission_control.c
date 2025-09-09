#include "pmsis.h"
#include "mission_control.h"
#include "mavlink_interface.h" // Use our MAVLink API
#include <stdio.h>
#include "cpx.h"

static volatile bool g_start_mission_commanded = false;

// --- ArduCopter Flight Modes ---
#define COPTER_MODE_GUIDED 4
#define COPTER_MODE_RTL 6

// --- Mission Parameters ---
#define TAKEOFF_ALT 1.0f
#define FORWARD_DIST 3.0f

bool mission_is_start_commanded(void) {
    return g_start_mission_commanded;
}

void mission_signal_start(void) {
    g_start_mission_commanded = true;
}

// Define the states for your mission
typedef enum {
    MISSION_WAIT_FOR_COMMAND,
    MISSION_WAIT_FOR_ARM,
    MISSION_SET_ORIGIN,
    MISSION_SET_MODE_GUIDED,
    MISSION_COMMAND_TAKEOFF,
    MISSION_WAIT_FOR_CLIMB,
    MISSION_GOTO_FORWARD,
    MISSION_SET_MODE_RTL,
    MISSION_DONE    
} MissionState_t;

// The mission task function
void autonomous_mission_task(void *parameters) {
    VehicleState_t *state = (VehicleState_t *)parameters;
    MissionState_t current_state = MISSION_WAIT_FOR_COMMAND;

    cpxPrintToConsole(LOG_TO_WIFI, "Mission task started. Waiting for start command via WiFi...\n");

    while (current_state != MISSION_DONE) {
        switch (current_state) {
            case MISSION_WAIT_FOR_COMMAND:
                if (mission_is_start_commanded()) {
                    cpxPrintToConsole(LOG_TO_WIFI, "Mission: Start command received! Waiting for arm...\n");
                    current_state = MISSION_WAIT_FOR_ARM;
                }
                break;            
            case MISSION_WAIT_FOR_ARM:
                if (mavlink_is_armed()) { // Use the cleaner API call
                    cpxPrintToConsole(LOG_TO_WIFI, "Mission: Vehicle is armed.\n");
                    current_state = MISSION_SET_ORIGIN;
                }
                break;
            case MISSION_SET_ORIGIN:
                mavlink_set_ekf_origin(-353632640, 1491652352, 584090);
                current_state = MISSION_SET_MODE_GUIDED;
                break;
            case MISSION_SET_MODE_GUIDED:
                mavlink_set_mode(COPTER_MODE_GUIDED);
                // Note: You would normally wait for an acknowledgement here
                current_state = MISSION_COMMAND_TAKEOFF;
                break;
            case MISSION_COMMAND_TAKEOFF:
                mavlink_command_takeoff(TAKEOFF_ALT);
                current_state = MISSION_WAIT_FOR_CLIMB;
                break;
                case MISSION_WAIT_FOR_CLIMB:
                if (state->altitude_m >= TAKEOFF_ALT * 0.9) {
                    cpxPrintToConsole(LOG_TO_WIFI, "Mission: Climb complete.\n");
                    current_state = MISSION_GOTO_FORWARD;
                }
                break;
            case MISSION_GOTO_FORWARD:
                // For this simple script, moving forward is along the Y (East) axis
                // A real script would use the current yaw to fly "forward".
                mavlink_set_local_target(0, FORWARD_DIST, -TAKEOFF_ALT);
                current_state = MISSION_SET_MODE_RTL;
                break;
            case MISSION_SET_MODE_RTL:
                mavlink_rtl();
                cpxPrintToConsole(LOG_TO_WIFI, "Mission: Switching to RTL. Mission complete.\n");
                current_state = MISSION_DONE;
                break;
            case MISSION_DONE:
                // Loop forever or self-terminate the task
                break;                            
        }
        vTaskDelay(500 / portTICK_PERIOD_MS); // Loop twice per second
    }
    vTaskDelete(NULL);
}