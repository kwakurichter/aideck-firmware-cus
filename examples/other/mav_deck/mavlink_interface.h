#pragma once
#include "vehicle_state.h"
#include "ardupilotmega/mavlink.h"

// --- Initialization ---
// Starts the UART and the MAVLink receive/transmit tasks.
// Returns a pointer to the globally managed vehicle state.
VehicleState_t* mavlink_interface_init(void);

void send_mavlink_message(const mavlink_message_t *msg);
void send_heartbeat();

void mavlink_request_data_stream(uint32_t message_id, float frequency_hz);
void mavlink_request_param_read(const char *param_id);
void mavlink_send_command_long(uint16_t command, float p1, float p2, float p3, float p4, float p5, float p6, float p7);

// --- Command Functions ---
void mavlink_command_takeoff(float altitude_m);
void mavlink_set_mode(uint8_t custom_mode);
void mavlink_arm_vehicle();
void mavlink_set_ekf_origin(int32_t lat, int32_t lon, int32_t alt);
void mavlink_disarm_vehicle(void);
void mavlink_set_local_target(float x_north, float y_east, float z_down);
void mavlink_rtl(void);

// --- State Access Functions (optional but good practice) ---
bool mavlink_is_armed(void);
float mavlink_get_altitude(void);