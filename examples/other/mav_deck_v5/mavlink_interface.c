#include "mavlink_interface.h"
#include "pmsis.h" // For printf
#include "ardupilotmega/mavlink.h"
#include "cpx.h"


#define GAP8_SYSTEM_ID 1
#define GAP8_COMPONENT_ID 1


/**
 * @brief Public function to send a MAVLink message.
 * This function is thread-safe.
 */
void send_mavlink_message(const mavlink_message_t *msg) {
    if (mavlink_tx_queue != NULL) {
        // Send the message to the transmit queue.
        // The uart_transmit_task will handle the actual sending.
        xQueueSend(mavlink_tx_queue, msg, (TickType_t)0);
    }
}

/**
 * @brief Creates and sends a MAVLink HEARTBEAT message.
 */
void send_heartbeat() {
    mavlink_message_t msg;
    mavlink_heartbeat_t hb;

    hb.type = MAV_TYPE_ONBOARD_CONTROLLER;
    hb.autopilot = MAV_AUTOPILOT_GENERIC;
    hb.base_mode = 0;
    hb.custom_mode = 0;
    hb.system_status = MAV_STATE_ACTIVE;

    mavlink_msg_heartbeat_encode(GAP8_SYSTEM_ID, GAP8_COMPONENT_ID, &msg, &hb);
    send_mavlink_message(&msg);
    printf("-> Sent HEARTBEAT to host.\n");
}

/**
 * @brief Requests a MAVLink data stream from the vehicle.
 * @param message_id The ID of the message to stream (e.g., MAVLINK_MSG_ID_ATTITUDE).
 * @param frequency_hz The desired frequency in Hz. Use 0 to stop the stream.
 */
void mavlink_request_data_stream(uint32_t message_id, float frequency_hz) {
    mavlink_message_t msg;

    // Calculate the interval in microseconds.
    // Drones expect an interval, not a frequency.
    // A frequency of 0 Hz means stop the stream (-1 value).
    int32_t interval_us = (frequency_hz > 0) ? (1000000 / frequency_hz) : -1;

    mavlink_msg_command_long_pack(
        GAP8_SYSTEM_ID, GAP8_COMPONENT_ID, &msg,
        1, 1, // target_system, target_component
        MAV_CMD_SET_MESSAGE_INTERVAL,
        0, // confirmation
        (float)message_id, // param1: The MAVLink message ID
        (float)interval_us, // param2: The interval in microseconds
        0, 0, 0, 0, 0 // params 3-7 not used
    );

    send_mavlink_message(&msg);
    cpxPrintToConsole(LOG_TO_WIFI, "-> Sent stream request for MSG ID %ld at %.1f Hz\n", message_id, frequency_hz);
}

/**
 * @brief Requests the value of a single parameter from the vehicle.
 * @param param_id The null-terminated string ID of the parameter (e.g., "SYSID_THISMAV").
 */
void mavlink_request_param_read(const char *param_id) {
    mavlink_message_t msg;

    mavlink_msg_param_request_read_pack(
        GAP8_SYSTEM_ID, GAP8_COMPONENT_ID, &msg,
        1, 1, // target_system, target_component
        param_id,
        -1 // Use -1 for parameter index when using a name
    );

    send_mavlink_message(&msg);
    cpxPrintToConsole(LOG_TO_WIFI, "-> Sent PARAM_REQUEST_READ for %s.\n", param_id);
}

/**
 * @brief Sends a generic MAVLink command to the vehicle.
 * @param command   The MAV_CMD_xxx ID of the command.
 * @param p1-p7     The 7 parameters for the command.
 */
void mavlink_send_command_long(uint16_t command, float p1, float p2, float p3, float p4, float p5, float p6, float p7) {
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(
        GAP8_SYSTEM_ID, GAP8_COMPONENT_ID, &msg,
        1, 1, // target_system, target_component
        command,
        0, // confirmation
        p1, p2, p3, p4, p5, p6, p7
    );
    send_mavlink_message(&msg);
}

// Wrapper for MAV_CMD_NAV_TAKEOFF
void mavlink_command_takeoff(float altitude_m) {
    cpxPrintToConsole(LOG_TO_WIFI, "-> Commanding takeoff to %.1f meters.\n", altitude_m);
    mavlink_send_command_long(MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, altitude_m);
}

// Wrapper for MAV_CMD_DO_SET_MODE
void mavlink_set_mode(uint8_t custom_mode) {
    cpxPrintToConsole(LOG_TO_WIFI, "-> Setting mode to %d.\n", custom_mode);
    mavlink_send_command_long(MAV_CMD_DO_SET_MODE, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, (float)custom_mode, 0, 0, 0, 0, 0);
}

// Wrapper for component arm/disarm
void mavlink_arm_vehicle() {
    cpxPrintToConsole(LOG_TO_WIFI, "-> Sending ARM command.\n");
    mavlink_send_command_long(MAV_CMD_COMPONENT_ARM_DISARM, 1.0f, 0, 0, 0, 0, 0, 0);
}

/**
 * @brief Sets the EKF origin / Home position. Crucial for non-GPS flight.
 * @param lat Latitude in degrees * 1e7
 * @param lon Longitude in degrees * 1e7
 * @param alt Altitude in millimeters
 */
void mavlink_set_ekf_origin(int32_t lat, int32_t lon, int32_t alt) {
    mavlink_message_t msg;
    cpxPrintToConsole(LOG_TO_WIFI, "-> Setting EKF origin.\n");

    // Create a zero-filled quaternion array as required by the function
    float q[4] = {0};

    // This message sets the vehicle's home position, which also serves as the EKF origin.
    mavlink_msg_set_home_position_pack(
        GAP8_SYSTEM_ID, GAP8_COMPONENT_ID, &msg,
        1, // target_system
        lat, lon, alt,
        0, 0, 0,    // x, y, z position
        q,          // quaternion
        0, 0, 0,    // approach vector
        0           // time_usec (0 to let FC handle it)
    );
    send_mavlink_message(&msg);
}

/**
 * @brief Sends a command to disarm the vehicle.
 */
void mavlink_disarm_vehicle(void) {
    cpxPrintToConsole(LOG_TO_WIFI, "-> Sending DISARM command.\n");
    // Uses the generic command function. Param 1 = 0.0f for DISARM.
    mavlink_send_command_long(MAV_CMD_COMPONENT_ARM_DISARM, 0.0f, 0, 0, 0, 0, 0, 0);
}

/**
 * @brief Commands the vehicle to fly to a target in the local NED frame.
 * @param x_north Target position North in meters.
 * @param y_east  Target position East in meters.
 * @param z_down  Target position Down in meters (negative for altitude).
 */
void mavlink_set_local_target(float x_north, float y_east, float z_down) {
    mavlink_message_t msg;
    cpxPrintToConsole(LOG_TO_WIFI, "-> Setting local NED target to N:%.1f, E:%.1f, D:%.1f\n", x_north, y_east, z_down);

    // Bitmask tells the drone to only use the position fields (x,y,z) and ignore others.
    uint16_t type_mask = POSITION_TARGET_TYPEMASK_VX_IGNORE |
                           POSITION_TARGET_TYPEMASK_VY_IGNORE |
                           POSITION_TARGET_TYPEMASK_VZ_IGNORE |
                           POSITION_TARGET_TYPEMASK_AX_IGNORE |
                           POSITION_TARGET_TYPEMASK_AY_IGNORE |
                           POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                           POSITION_TARGET_TYPEMASK_YAW_IGNORE |
                           POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;

    mavlink_msg_set_position_target_local_ned_pack(
        GAP8_SYSTEM_ID, GAP8_COMPONENT_ID, &msg,
        0, // time_boot_ms (0 for now)
        1, 1, // target_system, target_component
        MAV_FRAME_LOCAL_NED, // Use the local frame of reference
        type_mask,
        x_north, y_east, z_down,
        0, 0, 0, // velocity (ignored)
        0, 0, 0, // acceleration (ignored)
        0, 0     // yaw & yaw rate (ignored)
    );
    send_mavlink_message(&msg);
}

/**
 * @brief Commands the vehicle to enter Return-To-Launch (RTL) mode.
 */
void mavlink_rtl(void) {
    cpxPrintToConsole(LOG_TO_WIFI, "-> Commanding RTL.\n");
    // ArduCopter's custom mode number for RTL is 6.
    mavlink_set_mode(6);
}

// Implementations for state access functions
bool mavlink_is_armed(void) {
    return g_vehicle_state.is_armed;
}

float mavlink_get_altitude(void) {
    return g_vehicle_state.altitude_m;
}