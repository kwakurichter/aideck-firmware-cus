#include "pmsis.h"
#include "bsp/bsp.h"
#include "cpx.h"
#include "ardupilotmega/mavlink.h"
#include "wifi.h" // Include the WiFi control header
#include <math.h>

#define GAP8_SYSTEM_ID 10
#define GAP8_COMPONENT_ID 1
#define LED_PIN (2)

#define TAKEOFF_ALT 1.0f
#define FORWARD_DIST 3.0f

// --- Global Variables ---
static struct pi_device uart_device;
static pi_device_t led_device;
static CPXPacket_t txp; // Packet for sending CPX commands
static CPXPacket_t rxp; // Packet for receiving CPX commands
uint8_t send_buffer[MAVLINK_MAX_PACKET_LEN];

// Startup flags
static volatile bool is_mission_start = false;
static volatile bool is_mission_reset = false;
static volatile bool is_mission_terminate = false;
static volatile bool g_fc_connected = false;
static volatile bool g_pos_stream_active = false;
static volatile bool g_att_stream_active = false;
static volatile bool g_range_stream_active = false;
static volatile bool g_ahrs_stream_active = false;
static volatile bool g_p2p_data_received_once = false; // Becomes true once we get the first p2p ATTITUDE message
static volatile TickType_t g_last_p2p_message_tick = 0; // Timestamp for timeout

// ACK declarations
static SemaphoreHandle_t g_ack_semaphore;
static volatile uint16_t g_last_ack_command = 0;
static volatile uint8_t g_last_ack_result = 0;

typedef struct {
    bool is_armed;
    float position_ned[3]; // North, East, Down in meters
    float altitude_m;      // Calculated from z position
    float roll;
    float pitch;
    float yaw;
    // Add other state variables here as needed (e.g., attitude, battery)
} VehicleState_t;

typedef struct {
    float roll;
    float pitch;
    float yaw;
} PeerState_t;

VehicleState_t g_vehicle_state = {0}; // A single, shared structure to hold the vehicle's state
PeerState_t g_peer_state = {0}; // Holds the last known attitude of the peer

// Define the states for your mission
typedef enum {
    MISSION_WAIT_FOR_FC_CONNECT,
    MISSION_WAIT_FOR_STREAMS,
    MISSION_WAIT_FOR_COMMAND,
    MISSION_SET_EKF_ORIGIN,
    MISSION_SET_MODE_GUIDED,     
    MISSION_ARM_VEHICLE,
    MISSION_WAIT_FOR_ARM,
    MISSION_COMMAND_TAKEOFF,
    MISSION_WAIT_FOR_CLIMB,
    MISSION_HOVER,
    VOODOO_CONTROL,
    MISSION_COMMAND_LAND,
    MISSION_DONE    
} MissionState_t;

volatile MissionState_t g_mission_state = MISSION_WAIT_FOR_FC_CONNECT;

// Helper function to convert mission state enum to a string for printing
const char* mission_state_to_string(MissionState_t state) {
    switch (state) {
        case MISSION_WAIT_FOR_FC_CONNECT:   return "WAIT_FOR_FC_CONNECT";
        case MISSION_WAIT_FOR_STREAMS:      return "WAIT_FOR_STREAMS";
        case MISSION_WAIT_FOR_COMMAND:      return "WAIT_FOR_COMMAND";
        case MISSION_SET_EKF_ORIGIN:        return "SET_EKF_ORIGIN";
        case MISSION_SET_MODE_GUIDED:       return "SET_MODE_GUIDED";        
        case MISSION_ARM_VEHICLE:           return "ARM_VEHICLE";
        case MISSION_WAIT_FOR_ARM:          return "WAIT_FOR_ARM";
        case MISSION_COMMAND_TAKEOFF:       return "COMMAND_TAKEOFF";
        case MISSION_WAIT_FOR_CLIMB:        return "WAIT_FOR_CLIMB";
        case MISSION_HOVER:                 return "HOVER";
        case VOODOO_CONTROL:                return "VOODOO";
        case MISSION_COMMAND_LAND:          return "COMMAND_LAND";
        case MISSION_DONE:                  return "DONE";
        default:                            return "UNKNOWN_STATE";
    }
}

/**
 * @brief Configures the ESP32 to start its own WiFi Access Point.
 */
void setupWiFi(void) {
    static char ssid[] = "AideckDebugAP";
    cpxPrintToConsole(LOG_TO_WIFI, "Setting up WiFi AP with SSID: %s\n", ssid);

    cpxInitRoute(CPX_T_GAP8, CPX_T_ESP32, CPX_F_WIFI_CTRL, &txp.route);
    WiFiCTRLPacket_t *wifiCtrl = (WiFiCTRLPacket_t*) txp.data;

    // Command to set the SSID
    wifiCtrl->cmd = WIFI_CTRL_SET_SSID;
    memcpy(wifiCtrl->data, ssid, sizeof(ssid));
    txp.dataLength = sizeof(ssid) + sizeof(wifiCtrl->cmd);
    cpxSendPacketBlocking(&txp);

    // Command to connect (i.e., start the Access Point)
    wifiCtrl->cmd = WIFI_CTRL_WIFI_CONNECT;
    wifiCtrl->data[0] = 0x01; // 0x01 indicates AP mode
    txp.dataLength = 2;
    cpxSendPacketBlocking(&txp);

    cpxPrintToConsole(LOG_TO_WIFI, "WiFi AP setup commands sent.\n");
}

/**
 * @brief A simple task to blink the onboard LED.
 */
void led_blinky_task(void *parameters)
{
  pi_gpio_pin_configure(&led_device, LED_PIN, PI_GPIO_OUTPUT);
  while (1)
  {
    pi_gpio_pin_write(&led_device, LED_PIN, 1);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    pi_gpio_pin_write(&led_device, LED_PIN, 0);
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

/**
 * @brief Sends a MAVLink message structure over the UART.
 */
void send_mavlink_message(const mavlink_message_t *msg) {
    uint16_t len = mavlink_msg_to_send_buffer(send_buffer, msg);
    pi_uart_write(&uart_device, send_buffer, len);
}

/**
 * @brief Waits for a specific COMMAND_ACK message from the flight controller.
 * @param command_id The MAV_CMD ID we are waiting for an ACK for.
 * @param timeout_ms The maximum time to wait in milliseconds.
 * @return The MAV_RESULT from the ACK, or -1 on timeout.
 */
int mavlink_wait_for_ack(uint16_t command_id, uint32_t timeout_ms) {
    // Drain any stale signals from the semaphore before waiting.
    // This ensures we are waiting for a NEW ack, not an old one.
    xSemaphoreTake(g_ack_semaphore, 0); 

    // Clear any stale ACK information
    g_last_ack_command = 0;
    g_last_ack_result = 0;

    // Try to take the semaphore, waiting for the specified timeout
    if (xSemaphoreTake(g_ack_semaphore, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
        // We received an ACK. Check if it's the one we were waiting for.
        if (g_last_ack_command == command_id) {
            return g_last_ack_result;
        } else {
            // We got an ACK, but for a different command. Treat as failure.
            cpxPrintToConsole(LOG_TO_WIFI, "DEBUG: Waited for ACK %d but got %d\n", command_id, g_last_ack_command);
            return -1;             
        }
    }

    // If we timed out or got the wrong ACK, return -1 (timeout/failure)
    return -1;
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
 * @brief NEW: Creates and sends a request to read a parameter from the drone.
 */
void request_parameter() {
    mavlink_message_t msg;
    // Request the drone's own System ID. This is a good, simple test parameter.
    const char *param_id = "SYSID_THISMAV";
    
    // Target the drone's system and component IDs. Using 1, 1 is standard for the first flight controller.
    uint8_t target_system = 1;
    uint8_t target_component = 1;

    mavlink_msg_param_request_read_pack(GAP8_SYSTEM_ID, GAP8_COMPONENT_ID, &msg, target_system, target_component, param_id, -1);
    send_mavlink_message(&msg);
    printf("-> Sent PARAM_REQUEST_READ for %s.\n", param_id);
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
 * @brief Requests the essential data streams from the flight controller at startup.
 */
 void mavlink_init_data_streams(void) {
    printf("-> Requesting essential data streams...\n");

    // Request LOCAL_POSITION_NED at 10 Hz
    // This provides the x, y, z position needed to calculate altitude.
    mavlink_request_data_stream(MAVLINK_MSG_ID_LOCAL_POSITION_NED, 10.0f);

    // You can add requests for other streams here if needed, for example:
    //mavlink_request_data_stream(MAVLINK_MSG_ID_ATTITUDE, 10.0f);

    // Request DISTANCE_SENSOR at 10 Hz for altitude
    mavlink_request_data_stream(MAVLINK_MSG_ID_RANGEFINDER, 10.0f);

    // Request AHRS2 at 10 Hz for attitude data
    mavlink_request_data_stream(MAVLINK_MSG_ID_AHRS2, 10.0f);    
}

/**
 * @brief Task to send periodic messages over UART.
 */
 void uart_transmit_task(void *parameters) {
    uint32_t loop_counter = 0;
    while (1) {
        send_heartbeat();

        // Every 5 seconds, request a parameter
        if (loop_counter % 5 == 0) {
            //request_parameter();
        }
        
        loop_counter++;
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Run loop every second
    }
}


/**
 * @brief Task to listen for and parse incoming MAVLink messages.
 */
void uart_receive_task(void *parameters) {
    mavlink_message_t received_msg;
    mavlink_status_t status = {0};
    uint8_t rx_byte;

    while(1) {
        pi_uart_read(&uart_device, &rx_byte, 1);

        if (mavlink_parse_char(MAVLINK_COMM_0, rx_byte, &received_msg, &status)) {
            // Use a switch to handle different message types
            switch (received_msg.msgid) {
                case MAVLINK_MSG_ID_HEARTBEAT: {
                    mavlink_heartbeat_t hb;
                    mavlink_msg_heartbeat_decode(&received_msg, &hb);
                    g_vehicle_state.is_armed = (hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED);
                    
                    if (!g_fc_connected) {
                        g_fc_connected = true;
                        cpxPrintToConsole(LOG_TO_WIFI, "<- First HEARTBEAT received. FC is connected.\n");
                    }    

                    //cpxPrintToConsole(LOG_TO_WIFI, "<- HEARTBEAT from SysID: %d\n", received_msg.sysid);
                    break;
                }
            case MAVLINK_MSG_ID_STATUSTEXT: {
                    mavlink_statustext_t status;
                    mavlink_msg_statustext_decode(&received_msg, &status);
                    // Print the error message over WiFi
                    cpxPrintToConsole(LOG_TO_WIFI, "FC Status: %s\n", status.text);
                    break;
                }                
                case MAVLINK_MSG_ID_AHRS2: {
                    if (!g_ahrs_stream_active) {
                        g_ahrs_stream_active = true;
                    }

                    static int ahrs_msg_count = 0;
                    ahrs_msg_count++;

                    mavlink_ahrs2_t ahrs2;
                    mavlink_msg_ahrs2_decode(&received_msg, &ahrs2);
                    // Update the global state with new attitude data
                    g_vehicle_state.roll = ahrs2.roll;
                    g_vehicle_state.pitch = ahrs2.pitch;
                    g_vehicle_state.yaw = ahrs2.yaw;

                    // Every 10 messages, print the data
                    if (ahrs_msg_count % 10 == 0) {
                        // Print the attitude data to the WiFi console
                        cpxPrintToConsole(LOG_TO_WIFI, "<- AHRS: Roll=%.2f Pitch=%.2f Yaw=%.2f\n", ahrs2.roll, ahrs2.pitch, ahrs2.yaw);
                    }                    
                    break;
                }
                case MAVLINK_MSG_ID_PARAM_VALUE: {
                    // We received a parameter value, decode it
                    mavlink_param_value_t param;
                    mavlink_msg_param_value_decode(&received_msg, &param);
                    
                    // Print the received parameter name and its value over WiFi
                    cpxPrintToConsole(LOG_TO_WIFI, "<- PARAM_VALUE: %s = %f\n", param.param_id, param.param_value);
                    break;
                }
                case MAVLINK_MSG_ID_ATTITUDE: {
                    // We assume any ATTITUDE message is from the peer drone
                    if (!g_p2p_data_received_once) {
                        g_p2p_data_received_once = true;
                    }                    
                    
                    // Keep track of how many attitude messages we've received
                    static int attitude_msg_count = 0;
                    attitude_msg_count++;

                    // Decode the message
                    mavlink_attitude_t peer_attitude;
                    mavlink_msg_attitude_decode(&received_msg, &peer_attitude);
                    
                    // Update the global peer state
                    g_peer_state.roll = peer_attitude.roll;
                    g_peer_state.pitch = peer_attitude.pitch;
                    g_peer_state.yaw = peer_attitude.yaw;
                    
                    g_last_p2p_message_tick = xTaskGetTickCount(); // Record time of this message

                    // Every 10 messages, print the data
                    if (attitude_msg_count % 10 == 0) {
                        // Print the attitude data to the WiFi console
                        cpxPrintToConsole(LOG_TO_WIFI, "<- P2P ATTITUDE: Roll=%.2f Pitch=%.2f Yaw=%.2f\n", peer_attitude.roll, peer_attitude.pitch, peer_attitude.yaw);
                    }
                    break;
                }
                case MAVLINK_MSG_ID_LOCAL_POSITION_NED: {
                    if (!g_pos_stream_active) {
                        g_pos_stream_active = true;
                    }                    
                    
                    // Keep track of how many attitude messages we've received
                    static int pos_msg_count = 0;
                    pos_msg_count++;

                    // We received a position, decode it
                    mavlink_local_position_ned_t pos;
                    mavlink_msg_local_position_ned_decode(&received_msg, &pos);
                    g_vehicle_state.position_ned[0] = pos.x; // North
                    g_vehicle_state.position_ned[1] = pos.y; // East
                    g_vehicle_state.position_ned[2] = pos.z; // Down
                    
                    if (pos_msg_count % 10 == 0) {
                        // Print the received position over WiFi
                        cpxPrintToConsole(LOG_TO_WIFI, "<- POS: X=%.2f Y=%.2f Z=%.2f\n",
                            pos.x, pos.y, pos.z);                        
                    }
                    break;
                }
                case MAVLINK_MSG_ID_RANGEFINDER: {
                    if (!g_range_stream_active) {
                        g_range_stream_active = true;
                    }

                    static int range_msg_count = 0;
                    range_msg_count++;   

                    mavlink_rangefinder_t range;
                    mavlink_msg_rangefinder_decode(&received_msg, &range);
                    // Update altitude from the rangefinder (convert cm to m)
                    g_vehicle_state.altitude_m = range.distance;

                    if (range_msg_count % 10 == 0) {
                        // Print the received position over WiFi
                        cpxPrintToConsole(LOG_TO_WIFI, "<- ALTITUDE: %.2f m\n", range.distance);                        
                    }                    
                    break;
                }                
                case MAVLINK_MSG_ID_COMMAND_ACK: {
                    mavlink_command_ack_t ack;
                    mavlink_msg_command_ack_decode(&received_msg, &ack);

                    // Store the command and result in our global variables
                    g_last_ack_command = ack.command;
                    g_last_ack_result = ack.result;

                    // Give the semaphore to signal that an ACK has been received
                    xSemaphoreGive(g_ack_semaphore);
                    break;
                }                                
                default:
                    // Optional: Log other messages if needed for debugging
                    // cpxPrintToConsole(LOG_TO_WIFI, "<- RX MSG with ID: %d\n", received_msg.msgid);
                    break;
            }
        }
    }
}

/**
 * @brief Task to send periodic debug messages over WiFi.
 */
void wifi_debug_task(void *parameters) {
    while(1) {
        cpxPrintToConsole(LOG_TO_WIFI, "WiFi task alive... Mission State: %s\n", mission_state_to_string(g_mission_state));
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}

/**
 * @brief Task to listen for and process commands received over WiFi.
 */
void wifi_command_receive_task(void *parameters) {
    CPXPacket_t rxp;
    // This task doesn't need to initialize a route, it only receives.

    while(1) {
        // Block until a packet is received on the WIFI_CTRL function channel
        cpxReceivePacketBlocking(CPX_F_WIFI_CTRL, &rxp);

        // Check if the packet came from the WiFi Host
        if (rxp.route.source == CPX_T_WIFI_HOST && rxp.dataLength > 0) {
            WiFiCTRLPacket_t *wifiCtrl = (WiFiCTRLPacket_t*) rxp.data;

            // Check if it's a user command
            if (wifiCtrl->cmd == WIFI_CTRL_USER_COMMAND) {
                // Define the command string we are looking for
                const char* start_cmd = "start_mission";
                const char* reset_cmd = "reset_mission";
                const char* term_cmd = "terminate";
                int received_len = rxp.dataLength - sizeof(wifiCtrl->cmd);
                
                // Check if the received command matches "start_mission"
                if (strlen(start_cmd) == received_len && strncmp(start_cmd, (const char*)wifiCtrl->data, received_len) == 0) {
                    cpxPrintToConsole(LOG_TO_WIFI, "<- Received START MISSION command!\n");
                    is_mission_start = true;
                } else if (strlen(reset_cmd) == received_len && strncmp(reset_cmd, (const char*)wifiCtrl->data, received_len) == 0) {
                    cpxPrintToConsole(LOG_TO_WIFI, "<- Received RESET MISSION command!\n");
                    is_mission_reset = true;
                } else if (strlen(term_cmd) == received_len && strncmp(term_cmd, (const char*)wifiCtrl->data, received_len) == 0) {
                    cpxPrintToConsole(LOG_TO_WIFI, "<- Received TERMINATE MISSION command!\n");
                    is_mission_terminate = true;
                } else {
                    // Print any other commands for debugging
                    cpxPrintToConsole(LOG_TO_WIFI, "<- Received command: %.*s\n", received_len, wifiCtrl->data);
                }
            }
        }
    }
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

/**
 * @brief Sets the EKF origin / Home position. Crucial for non-GPS flight.
 * @param lat Latitude in degrees * 1e7
 * @param lon Longitude in degrees * 1e7
 * @param alt Altitude in millimeters
 */
void mavlink_set_ekf_origin(int32_t lat, int32_t lon, int32_t alt) {
    cpxPrintToConsole(LOG_TO_WIFI, "-> Setting EKF origin.\n");
    // MAV_CMD_DO_SET_HOME uses parameters 5, 6, and 7 for lat, lon, and alt.
    // Parameter 1 = 1 means it will set the current location as home.
    // We want to specify a location, so we set param 1 to 0.
    mavlink_send_command_long(
        MAV_CMD_DO_SET_HOME,
        0,                      // p1: 0 to use lat/lon/alt params
        0,                      // p2: empty
        0,                      // p3: empty
        0,                      // p4: empty
        (float)(lat / 1.0e7),   // p5: latitude in degrees
        (float)(lon / 1.0e7),   // p6: longitude in degrees
        (float)(alt / 1000.0)   // p7: altitude in meters
    );
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

// Wrapper for component arm
void mavlink_arm_vehicle() {
    cpxPrintToConsole(LOG_TO_WIFI, "-> Sending ARM command.\n");
    mavlink_send_command_long(MAV_CMD_COMPONENT_ARM_DISARM, 1.0f, 0, 0, 0, 0, 0, 0);
}

void mavlink_disarm_vehicle() {
    cpxPrintToConsole(LOG_TO_WIFI, "-> Sending DISARM command.\n");
    // Param 1 = 0.0f for DISARM
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
 * @brief Commands the vehicle to a specific attitude.
 * @param roll      The desired roll angle in radians.
 * @param pitch     The desired pitch angle in radians.
 * @param yaw_rate  The desired rate of yaw change in radians/sec.
 * @param thrust    The collective thrust from 0.0 (idle) to 1.0 (full). 0.5 is hover.
 */
void mavlink_set_attitude_target(float roll, float pitch, float yaw_rate, float thrust) {
    mavlink_message_t msg;

    // This bitmask tells the flight controller to use the quaternion for attitude
    // and the yaw_rate, while ignoring the body roll and pitch rates.
    uint8_t type_mask = (1 << 0) | (1 << 1); // MAV_ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE | MAV_ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE

    // --- Convert Euler angles (roll, pitch, yaw) to quaternion ---
    // For a simple hover or forward flight, we can command a target yaw angle of 0.
    float target_yaw = 0.0f;

    float cr = cos(roll * 0.5f);
    float sr = sin(roll * 0.5f);
    float cp = cos(pitch * 0.5f);
    float sp = sin(pitch * 0.5f);
    float cy = cos(target_yaw * 0.5f);
    float sy = sin(target_yaw * 0.5f);

    float q[4];
    q[0] = cr * cp * cy + sr * sp * sy; // w
    q[1] = sr * cp * cy - cr * sp * sy; // x
    q[2] = cr * sp * cy + sr * cp * sy; // y
    q[3] = cr * cp * sy - sr * sp * cy; // z

    // This parameter is for 3D thrust vectoring; unused in this case.
    float thrust_body[3] = {0};

    // Pack the message with all 13 arguments as per the function definition
    mavlink_msg_set_attitude_target_pack(
        GAP8_SYSTEM_ID,    // system_id
        GAP8_COMPONENT_ID, // component_id
        &msg,              // msg
        0,                 // time_boot_ms
        1, 1,              // target_system, target_component
        type_mask,
        q,                 // quaternion
        0,                 // body_roll_rate (ignored by mask)
        0,                 // body_pitch_rate (ignored by mask)
        yaw_rate,
        thrust,
        thrust_body        // The missing argument
    );

    send_mavlink_message(&msg);
}

/**
 * @brief Commands the vehicle to enter Return-To-Launch (RTL) mode.
 */
void mavlink_rtl(void) {
    cpxPrintToConsole(LOG_TO_WIFI, "-> Commanding RTL.\n");
    // ArduCopter's custom mode number for RTL is 6.
    mavlink_set_mode(COPTER_MODE_RTL);
}

void mavlink_command_land() {
    cpxPrintToConsole(LOG_TO_WIFI, "-> Commanding LAND.\n");
    // Parameters for MAV_CMD_NAV_LAND are typically unused for a standard landing.
    mavlink_send_command_long(MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0);
}

/**
 * @brief Sends a MANUAL_CONTROL MAVLink message.
 * @param x Pitch control (-1000 to 1000). Forward is positive.
 * @param y Roll control (-1000 to 1000). Right is positive.
 * @param z Thrust control (0 to 1000).
 * @param r Yaw control (-1000 to 1000). Clockwise is positive.
 */
void mavlink_send_manual_control(int16_t x, int16_t y, int16_t z, int16_t r) {
    mavlink_message_t msg;

    // This function call includes all 19 arguments required by your MAVLink version.
    // The extra fields for buttons and auxiliary controls are set to 0 as they are unused.
    mavlink_msg_manual_control_pack(
        GAP8_SYSTEM_ID,    // system_id
        GAP8_COMPONENT_ID, // component_id
        &msg,              // msg
        1,                 // target system
        x,
        y,
        z,
        r,
        0,                 // buttons
        0,                 // buttons2
        0,                 // enabled_extensions
        0,                 // s (pitch-only axis)
        0,                 // t (roll-only axis)
        0,                 // aux1
        0,                 // aux2
        0,                 // aux3
        0,                 // aux4
        0,                 // aux5
        0                  // aux6
    );

    send_mavlink_message(&msg);
}


// The mission task function
void autonomous_mission_task(void *parameters) {
    VehicleState_t *state = (VehicleState_t *)parameters;

    // --- State-specific counters ---
    int stream_retries = 0;
    const int MAX_STREAM_RETRIES = 6; // 3-second timeout
    int arm_retries = 0;
    const int MAX_ARM_RETRIES = 10; // 5-second timeout
    int hover_counter = 0;
    const int HOVER_DURATION = 10; // 20 retries * 500ms = 10 second timeout
    static int16_t s_takeoff_throttle = 500;
    const float VOODOO_PITCH_THRESHOLD_RAD = 0.35f; // ~20 degrees
    const float VOODOO_ROLL_THRESHOLD_RAD = 0.35f;  // ~20 degrees
    const int16_t VOODOO_MOVE_STRENGTH = 200;       // Command strength for manual_control (20% stick input)
    const int16_t VOODOO_YAW_RATE_STRENGTH = 150;   // Command strength for yaw
    const uint32_t VOODOO_TIMEOUT_MS = 5000;        // 5-second timeout    

    cpxPrintToConsole(LOG_TO_WIFI, "Mission task started. Waiting for start command via WiFi...\n");

    while (1) {
        if (is_mission_terminate) {
            cpxPrintToConsole(LOG_TO_WIFI, "Mission terminated by user. Disarming...\n");
            mavlink_disarm_vehicle();
            is_mission_terminate = false; // Clear the flag
            g_mission_state = MISSION_DONE;
            // Continue to the switch to immediately enter the DONE state
        }        
        switch (g_mission_state) {
            case MISSION_WAIT_FOR_FC_CONNECT:
                if (g_fc_connected) {
                    cpxPrintToConsole(LOG_TO_WIFI, "FC connected. Requesting data streams...\n");
                    mavlink_init_data_streams();
                    g_mission_state = MISSION_WAIT_FOR_STREAMS;
                } else {
                    vTaskDelay(500 / portTICK_PERIOD_MS);
                    cpxPrintToConsole(LOG_TO_WIFI, "Waiting for FC connection...\n");
                }
                break;
            case MISSION_WAIT_FOR_STREAMS:
                if (g_pos_stream_active) {
                    cpxPrintToConsole(LOG_TO_WIFI, "Data streams active.\n");
                    g_mission_state = MISSION_WAIT_FOR_COMMAND;
                } else {
                    if (stream_retries >= MAX_STREAM_RETRIES) {
                        cpxPrintToConsole(LOG_TO_WIFI, "FAILED to receive data stream. Aborting.\n");
                        g_mission_state = MISSION_DONE;
                    } else {
                        mavlink_init_data_streams();
                        cpxPrintToConsole(LOG_TO_WIFI, "Waiting for data stream... (attempt %d/%d)\n", stream_retries + 1, MAX_STREAM_RETRIES);
                        stream_retries++;
                    }
                }
                break;                       
            case MISSION_WAIT_FOR_COMMAND:
                if (is_mission_start) {
                    cpxPrintToConsole(LOG_TO_WIFI, "Mission: Start command received! Waiting for EKF to settle...\n");
                    arm_retries = 0; // Reset counter before starting
                    g_mission_state = MISSION_SET_EKF_ORIGIN;
                }
                break;
            case MISSION_SET_EKF_ORIGIN: {
                    mavlink_set_ekf_origin(-353632640, 1491652352, 584090);
    
                    int result = mavlink_wait_for_ack(MAV_CMD_DO_SET_HOME, 2000);
    
                    if (result == MAV_RESULT_ACCEPTED) {
                        cpxPrintToConsole(LOG_TO_WIFI, "Mission: Origin set.\n");
                        g_mission_state = MISSION_SET_MODE_GUIDED;
                    } else {
                       cpxPrintToConsole(LOG_TO_WIFI, "Mission: Origin set FAILED (result: %d). Aborting.\n", result);
                       g_mission_state = MISSION_DONE;
                    }                
                    break;
                }                        
            case MISSION_SET_MODE_GUIDED: {
                    mavlink_set_mode(COPTER_MODE_GUIDED);
                    
                    int result = mavlink_wait_for_ack(MAV_CMD_DO_SET_MODE, 2000);
    
                    if (result == MAV_RESULT_ACCEPTED) {
                        cpxPrintToConsole(LOG_TO_WIFI, "Mission: Guided mode set successfully.\n");
                        g_mission_state = MISSION_ARM_VEHICLE;
                    } else {
                        cpxPrintToConsole(LOG_TO_WIFI, "Mission: FAILED to set Guided mode (result: %d). Aborting.\n", result);
                        g_mission_state = MISSION_DONE;
                    }                
                    break;
                }                         
            case MISSION_ARM_VEHICLE: {
                    // First, send the command
                    mavlink_arm_vehicle();
    
                    // Immediately wait for the ACK
                    int result = mavlink_wait_for_ack(MAV_CMD_COMPONENT_ARM_DISARM, 2000); // 2-second timeout
    
                    if (result == MAV_RESULT_ACCEPTED) {
                        cpxPrintToConsole(LOG_TO_WIFI, "Mission: Vehicle armed successfully.\n");
                        g_mission_state = MISSION_WAIT_FOR_ARM;
                    } else {
                        if (arm_retries >= MAX_ARM_RETRIES) {
                            cpxPrintToConsole(LOG_TO_WIFI, "Mission: FAILED to arm (result: %d). Aborting.\n", result);
                            g_mission_state = MISSION_DONE;
                        } else {
                            cpxPrintToConsole(LOG_TO_WIFI, "Mission: Arming failed, retrying... (attempt %d/%d)\n", arm_retries + 1, MAX_ARM_RETRIES);
                            arm_retries++;
                            // The loop will re-enter this state on the next iteration to retry
                        }
                    }
                break;
            }
            case MISSION_WAIT_FOR_ARM:
                cpxPrintToConsole(LOG_TO_WIFI, "Waiting...\n");

                vTaskDelay(500 / portTICK_PERIOD_MS);
                g_mission_state = MISSION_COMMAND_TAKEOFF;
                break;            
            case MISSION_COMMAND_TAKEOFF: {
                mavlink_command_takeoff(TAKEOFF_ALT);

                int result = mavlink_wait_for_ack(MAV_CMD_NAV_TAKEOFF, 5000); // 5s timeout for takeoff

                if (result == MAV_RESULT_ACCEPTED) {
                        cpxPrintToConsole(LOG_TO_WIFI, "Mission: Takeoff command accepted.\n");
                        g_mission_state = MISSION_WAIT_FOR_CLIMB;
                } else {
                    cpxPrintToConsole(LOG_TO_WIFI, "Mission: Takeoff command FAILED (result: %d). Aborting.\n", result);
                    g_mission_state = MISSION_DONE;
                }
                break;
            }
            case MISSION_WAIT_FOR_CLIMB:
                if (state->altitude_m >= TAKEOFF_ALT * 0.95) {
                    cpxPrintToConsole(LOG_TO_WIFI, "Mission: Climb complete. Hovering...\n");
                    hover_counter = 0;
                    g_mission_state = MISSION_HOVER;
                }
                break;
            case MISSION_HOVER:
                // Check if we have ever received peer data
                if (g_p2p_data_received_once) {
                    cpxPrintToConsole(LOG_TO_WIFI, "Mission: Peer data detected. Entering Voodoo Control.\n");
                    g_last_p2p_message_tick = xTaskGetTickCount(); // Reset timeout timer
                    g_mission_state = VOODOO_CONTROL;
                }                
                else if (hover_counter >= HOVER_DURATION) {
                    cpxPrintToConsole(LOG_TO_WIFI, "Mission: Hover complete (no peer data). Initiating RTL.\n");
                    g_mission_state = MISSION_COMMAND_LAND;
                } else {
                    cpxPrintToConsole(LOG_TO_WIFI, "Hovering... waiting for peer data (%d/%d)\n", hover_counter + 1, HOVER_DURATION);
                    hover_counter++;
                }
                break;
            case VOODOO_CONTROL: {
                // SAFETY: Check for data timeout first
                TickType_t current_tick = xTaskGetTickCount();
                if ((current_tick - g_last_p2p_message_tick) * portTICK_PERIOD_MS > VOODOO_TIMEOUT_MS) {
                    cpxPrintToConsole(LOG_TO_WIFI, "Voodoo: Peer data timeout! Landing.\n");
                    g_mission_state = MISSION_COMMAND_LAND;
                    break; // Exit the state immediately
                }

                // Local copies of peer attitude for this loop
                float peer_roll = g_peer_state.roll;
                float peer_pitch = g_peer_state.pitch;

                // --- Single-Action-Priority Logic ---
                if (peer_pitch < -VOODOO_PITCH_THRESHOLD_RAD) {
                    // Move Forward (Pitch stick forward is negative X)
                    cpxPrintToConsole(LOG_TO_WIFI, "Voodoo: Move FORWARD\n");
                    mavlink_send_manual_control(-VOODOO_MOVE_STRENGTH, 0, 500, 0);
                }
                else if (peer_pitch > VOODOO_PITCH_THRESHOLD_RAD) {
                    // Move Backward (Pitch stick back is positive X)
                    cpxPrintToConsole(LOG_TO_WIFI, "Voodoo: Move BACKWARD\n");
                    mavlink_send_manual_control(VOODOO_MOVE_STRENGTH, 0, 500, 0);
                }
                else if (peer_roll > VOODOO_ROLL_THRESHOLD_RAD) {
                    // Move Right (Roll stick right is positive Y)
                    cpxPrintToConsole(LOG_TO_WIFI, "Voodoo: Move RIGHT\n");
                    mavlink_send_manual_control(0, VOODOO_MOVE_STRENGTH, 500, 0);
                }
                else if (peer_roll < -VOODOO_ROLL_THRESHOLD_RAD) {
                    // Move Left (Roll stick left is negative Y)
                    cpxPrintToConsole(LOG_TO_WIFI, "Voodoo: Move LEFT\n");
                    mavlink_send_manual_control(0, -VOODOO_MOVE_STRENGTH, 500, 0);
                }
                else {
                    // No thresholds met, default to a stable hover
                    mavlink_send_manual_control(0, 0, 500, 0);
                }
                break;
            }                     
            case MISSION_COMMAND_LAND: {
                mavlink_rtl();

                int result = mavlink_wait_for_ack(MAV_CMD_DO_SET_MODE, 2000);

                if (result == MAV_RESULT_ACCEPTED) {
                    cpxPrintToConsole(LOG_TO_WIFI, "Mission: Switching to RTL. Mission complete.\n");
                    g_mission_state = MISSION_DONE;
                } else {
                    cpxPrintToConsole(LOG_TO_WIFI, "Mission: FAILED to set RTL (result: %d). Aborting.\n", result);
                    g_mission_state = MISSION_DONE;
                }    
                break;
            }    
            case MISSION_DONE:
                if (is_mission_reset) {
                    cpxPrintToConsole(LOG_TO_WIFI, "Mission state is being reset.\n");
                    // Reset all counters and flags
                    stream_retries = 0;
                    arm_retries = 0;
                    hover_counter = 0;
                    is_mission_reset = false; // <-- IMPORTANT: Clear the reset flag
                    // Go back to the very first state
                    g_mission_state = MISSION_WAIT_FOR_FC_CONNECT;
                }
                break;                            
        }
        vTaskDelay(500 / portTICK_PERIOD_MS); // Loop twice per second
    }
}

/**
 * @brief The main entry point for the application.
 */
void start_main_application(void *parameters) {
    g_ack_semaphore = xSemaphoreCreateBinary();
    if (g_ack_semaphore == NULL) {
        printf("FATAL: Could not create ACK semaphore!\n");
        pmsis_exit(-1);
    }

    cpxInit();

    #ifdef SETUP_WIFI_AP
        setupWiFi();
    #endif

    struct pi_uart_conf conf;
    pi_uart_conf_init(&conf);
    conf.enable_tx = 1;
    conf.enable_rx = 1;
    conf.baudrate_bps = 115200;
    pi_open_from_conf(&uart_device, &conf);
    if (pi_uart_open(&uart_device))
    {
        cpxPrintToConsole(LOG_TO_WIFI, "FATAL: UART open failed!\n");
        pmsis_exit(-1);
    }
    cpxPrintToConsole(LOG_TO_WIFI, "UART opened successfully.\n");

    cpxPrintToConsole(LOG_TO_WIFI, "-- GAP8 MAVLink WiFi Test --\n");

    // Enable the CPX function for WiFi control commands
    cpxEnableFunction(CPX_F_WIFI_CTRL);

    BaseType_t xTask;
    xTask = xTaskCreate(uart_transmit_task, "uart_transmit_task", configMINIMAL_STACK_SIZE * 8, NULL, tskIDLE_PRIORITY + 1, NULL);
    if (xTask != pdPASS) {
        cpxPrintToConsole(LOG_TO_WIFI, "FATAL: UART transmit task creation failed!\n");
    }

    xTask = xTaskCreate(autonomous_mission_task, "mission_task", configMINIMAL_STACK_SIZE * 8, &g_vehicle_state, tskIDLE_PRIORITY + 1, NULL);
    if (xTask != pdPASS) {
        cpxPrintToConsole(LOG_TO_WIFI, "FATAL: Mission task creation failed!\n");
    }      

    xTask = xTaskCreate(uart_receive_task, "uart_receive_task", configMINIMAL_STACK_SIZE * 4, NULL, tskIDLE_PRIORITY + 1, NULL);
    if (xTask != pdPASS) {
        cpxPrintToConsole(LOG_TO_WIFI, "FATAL: UART receive task creation failed!\n");
    }

    xTask = xTaskCreate(wifi_debug_task, "wifi_debug_task", configMINIMAL_STACK_SIZE * 2, NULL, tskIDLE_PRIORITY + 1, NULL);
    if (xTask != pdPASS) {
        cpxPrintToConsole(LOG_TO_WIFI, "FATAL: WiFi debug task creation failed!\n");
    }

    xTask = xTaskCreate(led_blinky_task, "led_debug_task", configMINIMAL_STACK_SIZE * 2, NULL, tskIDLE_PRIORITY + 1, NULL);
    if (xTask != pdPASS) {
        cpxPrintToConsole(LOG_TO_WIFI, "FATAL: LED debug task creation failed!\n");
    }

    xTask = xTaskCreate(wifi_command_receive_task, "wifi_cmd_rx_task", configMINIMAL_STACK_SIZE * 2, NULL, tskIDLE_PRIORITY + 1, NULL);
    if (xTask != pdPASS) {
        cpxPrintToConsole(LOG_TO_WIFI, "FATAL: WiFi command receive task creation failed!\n");
    }  

    while(1) {
        pi_yield();
    }
}

int main(void) {
    pi_bsp_init();
    pi_freq_set(PI_FREQ_DOMAIN_FC, 250000000);
    __pi_pmu_voltage_set(PI_PMU_DOMAIN_FC, 1200);
    return pmsis_kickoff((void *)start_main_application);
}