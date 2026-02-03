#include "pmsis.h"
#include "bsp/bsp.h"
#include "cpx.h"
#include "ardupilotmega/mavlink.h"
#include "wifi.h" // Include the WiFi control header
#include <math.h>
#include <string.h>
#include <stddef.h>

#define GAP8_SYSTEM_ID 255
#define GAP8_COMPONENT_ID 1
#define STM32_SYSTEM_ID 1
#define STM32_COMPONENT_ID 1
#define PEER_SYSTEM_ID 20
#define LED_PIN (2)

#define MISSION_HZ 20
#define RC_CENTER 1500
#define RC_MAX_DELTA 200      // <-- your limit
#define RC_DEADBAND 15        // small deadband to prevent jitter
#define WP_RADIUS_M 0.15f     // arrival radius (10 cm)
#define WP_SETTLE_MS 300      // must stay inside radius for this long
#define SETTLE_TICKS (WP_SETTLE_MS / (1000 / MISSION_HZ))

// Guidance
#define LOOKAHEAD_M       0.30f    // 10–20 cm works well indoors
#define END_RADIUS_M      0.08f    // when to switch to next corner
#define LINE_KP_POS2RC    220.0f   // RC delta per meter (start 180–260)
#define KP  250.0f   // stronger (progress)
#define XTRACK_BLEND    0.30f  // start 0.25–0.35

// RC limits
#define RC_MAX   170
#define RC_MAX_DELTA_YAW  120
#define RC_DEADBAND_XY    10

#define SLEW_STEP_PITCH 10
#define SLEW_STEP_ROLL 10
#define SLEW_STEP_YAW 12

// Yaw hold (optional but recommended)
#define YAW_HOLD_ENABLE   1
#define YAW_KP_RAD2RC     180.0f   // RC delta per rad (start 120–220)
#define YAW_ERR_DEADBAND  0.05f    // rad (~3 deg)

// Waypoints for a 1x1 square (meters)
#define SQUARE_SIZE_M 0.5f

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
static volatile bool is_origin_set = false;
static volatile bool g_fc_connected = false;
static volatile bool g_pos_stream_active = false;
static volatile bool g_att_stream_active = false;
static volatile bool g_range_stream_active = false;
static volatile bool g_ahrs_stream_active = false;

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
    int mode;
    float flow_comp_m_x;
    float flow_comp_m_y;
    uint8_t quality;    
    // Add other state variables here as needed (e.g., attitude, battery)
} VehicleState_t;

VehicleState_t g_vehicle_state = {0}; // A single, shared structure to hold the vehicle's state


typedef struct {
    float origin_n;
    float origin_e;
    bool  origin_set;
    float yaw0;
    int seg;
} PathTracker_t;

static PathTracker_t g_path = {0};

// Define the states for your mission
typedef enum {
    MISSION_WAIT_FOR_LOITER,
    MISSION_WAIT_FOR_COMMAND,
    MISSION_ARM,
    MISSION_TAKEOFF,
    MISSION_HOVER,
    MISSION_PATH_EAST,
    MISSION_PATH_NORTH,
    MISSION_PATH_WEST,
    MISSION_PATH_SOUTH,
    MISSION_COMMAND_LAND,
    MISSION_WAIT_FOR_LAND,
    MISSION_DISARM,
    MISSION_DONE    
} MissionState_t;

volatile MissionState_t g_mission_state = MISSION_WAIT_FOR_COMMAND;

// Helper function to convert mission state enum to a string for printing
const char* mission_state_to_string(MissionState_t state) {
    switch (state) {
        case MISSION_WAIT_FOR_LOITER:       return "WAIT_FOR_LOITER";
        case MISSION_WAIT_FOR_COMMAND:      return "WAIT_FOR_COMMAND"; 
        case MISSION_ARM:                   return "ARM_VEHICLE";
        case MISSION_TAKEOFF:               return "COMMAND_TAKEOFF";
        case MISSION_HOVER:                 return "HOVER";
        case MISSION_PATH_EAST:             return "PATH_EAST";
        case MISSION_PATH_NORTH:            return "PATH_NORTH";
        case MISSION_PATH_WEST:             return "PATH_WEST";
        case MISSION_PATH_SOUTH:            return "PATH_SOUTH";
        case MISSION_COMMAND_LAND:          return "COMMAND_LAND";
        case MISSION_WAIT_FOR_LAND:         return "WAIT_FOR_LAND";
        case MISSION_DISARM:                return "DISARM";
        case MISSION_DONE:                  return "DONE";
        default:                            return "UNKNOWN_STATE";
    }
}

// Some minimal embedded libcs omit strncpy.
// Provide a small implementation to satisfy MAVLink helpers.
char *strncpy(char *dst, const char *src, size_t n)
{
    size_t i = 0;
    for (; i < n && src[i] != '\0'; i++) {
        dst[i] = src[i];
    }
    // Pad the rest with '\0' like standard strncpy
    for (; i < n; i++) {
        dst[i] = '\0';
    }
    return dst;
}

/**
 * @brief Configures the ESP32 to start its own WiFi Access Point.
 */
void setupWiFi(void) {
    static char ssid[] = "AideckDebugAP";
    printf("Setting up WiFi AP with SSID: %s\n", ssid);
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

    printf("WiFi AP setup commands sent.\n");
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
            printf("DEBUG: Waited for ACK %d but got %d\n", command_id, g_last_ack_command);
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
    // printf("-> Sent HEARTBEAT to host.\n");
}

/**
 * @brief NEW: Creates and sends a request to read a parameter from the drone.
 */
void request_parameter(const char *param_id) {
    mavlink_message_t msg;
    
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
    printf("-> Sent stream request for MSG ID %ld at %.1f Hz\n", message_id, frequency_hz);
    cpxPrintToConsole(LOG_TO_WIFI, "-> Sent stream request for MSG ID %ld at %.1f Hz\n", message_id, frequency_hz);
}

/**
 * @brief Requests the essential data streams from the flight controller at startup.
 */
 void mavlink_init_data_streams(void) {
    printf("-> Requesting essential data streams...\n");

    // Request LOCAL_POSITION_NED at 10 Hz
    // This provides the x, y, z position needed to calculate altitude.
    mavlink_request_data_stream(MAVLINK_MSG_ID_LOCAL_POSITION_NED, 3.0f);

    // You can add requests for other streams here if needed, for example:
    //mavlink_request_data_stream(MAVLINK_MSG_ID_ATTITUDE, 10.0f);

    // Request DISTANCE_SENSOR at 10 Hz for altitude
    mavlink_request_data_stream(MAVLINK_MSG_ID_RANGEFINDER, 3.0f);

    // Request AHRS2 at 10 Hz for attitude data
    mavlink_request_data_stream(MAVLINK_MSG_ID_AHRS2, 3.0f);    
}


/**
 * @brief Task to listen for and parse incoming MAVLink messages.
 */
void uart_receive_task(void *parameters) {
    uint8_t rx_byte;
    mavlink_message_t received_msg;
    mavlink_status_t status = {0};

    // --- Buffer for re-serializing the validated MAVLink message ---
    uint8_t mavlink_tx_buffer[MAVLINK_MAX_PACKET_LEN];

    while(1) {
        pi_uart_read(&uart_device, &rx_byte, 1);

        if (mavlink_parse_char(MAVLINK_COMM_0, rx_byte, &received_msg, &status)) {
                // --- 1. Validation Passed: A complete message was received ---
                // Your existing logic to handle messages for the AI-deck's own use can go here
                // switch(received_msg.msgid) { ... }

                // --- 2. Re-serialize the message into a raw byte buffer ---
                uint16_t len = mavlink_msg_to_send_buffer(mavlink_tx_buffer, &received_msg);

                // --- 3. Forward the raw buffer over WiFi using our safe function ---
                // Using CPX_F_APP as the "uplink" channel
                cpxSendRawData(CPX_T_WIFI_HOST, CPX_F_CONSOLE, mavlink_tx_buffer, len);

                // Use a switch to handle different message types
                switch (received_msg.msgid) {
                    case MAVLINK_MSG_ID_HEARTBEAT: {
                        mavlink_heartbeat_t hb;
                        mavlink_msg_heartbeat_decode(&received_msg, &hb);
                        g_vehicle_state.is_armed = (hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED);
                        g_vehicle_state.mode = (hb.custom_mode);

                        
                        if (!g_fc_connected) {
                            g_fc_connected = true;
                            printf("<- First HEARTBEAT received. FC is connected.\n");
                            //cpxPrintToConsole(LOG_TO_WIFI, "<- First HEARTBEAT received. FC is connected.\n");
                        }    

                        //cpxPrintToConsole(LOG_TO_WIFI, "<- HEARTBEAT from SysID: %d\n", received_msg.sysid);
                        break;
                    }
                    case MAVLINK_MSG_ID_STATUSTEXT: {
                        mavlink_statustext_t status;
                        mavlink_msg_statustext_decode(&received_msg, &status);
                        // Print the error message over WiFi
                        printf("FC Status: %s\n", status.text);
                        //cpxPrintToConsole(LOG_TO_WIFI, "FC Status: %s\n", status.text);
                        break;
                    }                
                    case MAVLINK_MSG_ID_ATTITUDE: {
                        if (!g_ahrs_stream_active) {
                            g_ahrs_stream_active = true;
                        }

                        static int att_msg_count = 0;
                        att_msg_count++;

                        mavlink_attitude_t att;
                        mavlink_msg_ahrs2_decode(&received_msg, &att);
                        // Update the global state with new attitude data
                        g_vehicle_state.roll = att.roll;
                        g_vehicle_state.pitch = att.pitch;
                        g_vehicle_state.yaw = att.yaw;

                        // Every 10 messages, print the data
                        if (att_msg_count % 10 == 0) {
                            // Print the attitude data to the WiFi console
                            // printf("<- AHRS: Roll=%.2f Pitch=%.2f Yaw=%.2f\n", ahrs2.roll, ahrs2.pitch, ahrs2.yaw);
                            //cpxPrintToConsole(LOG_TO_WIFI, "<- AHRS: Roll=%.2f Pitch=%.2f Yaw=%.2f\n", ahrs2.roll, ahrs2.pitch, ahrs2.yaw);
                        }                    
                        break;
                    }
                    case MAVLINK_MSG_ID_PARAM_VALUE: {
                        // We received a parameter value, decode it
                        mavlink_param_value_t param;
                        mavlink_msg_param_value_decode(&received_msg, &param);
                        
                        // Print the received parameter name and its value over WiFi
                        // printf("<- PARAM_VALUE: %s = %f\n", param.param_id, param.param_value);
                        //cpxPrintToConsole(LOG_TO_WIFI, "<- PARAM_VALUE: %s = %f\n", param.param_id, param.param_value);
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
                            // printf("<- POS: X=%.2f Y=%.2f Z=%.2f\n", pos.x, pos.y, pos.z);
                            //cpxPrintToConsole(LOG_TO_WIFI, "<- POS: X=%.2f Y=%.2f Z=%.2f\n", pos.x, pos.y, pos.z);                        
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
                            // printf("<- ALTITUDE: %.2f m\n", range.distance);
                            //cpxPrintToConsole(LOG_TO_WIFI, "<- ALTITUDE: %.2f m\n", range.distance);                        
                        }                    
                        break;
                    }      
                    case MAVLINK_MSG_ID_OPTICAL_FLOW: {
                        mavlink_optical_flow_t opt;
                        mavlink_msg_optical_flow_decode(&received_msg, &opt);

                        g_vehicle_state.flow_comp_m_x = opt.flow_comp_m_x;
                        g_vehicle_state.flow_comp_m_y = opt.flow_comp_m_y;
                        g_vehicle_state.quality = opt.quality;

                        break;
                    }                              
                    case MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN:
                        is_origin_set = true;
                        
                        break;
                    case MAVLINK_MSG_ID_COMMAND_ACK: {
                        mavlink_command_ack_t ack;
                        mavlink_msg_command_ack_decode(&received_msg, &ack);

                        // Use a harmless command as your "GO" trigger
                        if (ack.command == 42428) { // MAV_CMD_DO_SEND_BANNER
                            if (ack.result == MAV_RESULT_ACCEPTED) {
                                printf("MISSION GO!\n");
                                is_mission_start = true;
                            } else {
                                printf("GO command rejected/result=%u\n", ack.result);
                            }
                        }
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
 * @brief Task to listen for and process MAV data received over WiFi.
 */
void wifi_mav_receive_task(void *parameters) {
    CPXPacket_t rxp;
    // This task doesn't need to initialize a route, it only receives.

    while(1) {
        // Block until a packet is received on the WIFI_CTRL function channel
        cpxReceivePacketBlocking(CPX_F_MAVLINK_DOWNLINK, &rxp);

        // Check if the packet came from the WiFi Host
        if (rxp.route.source == CPX_T_WIFI_HOST && rxp.dataLength > 0) {
            WiFiCTRLPacket_t *wifiCtrl = (WiFiCTRLPacket_t*) rxp.data;

            // Check if it's a MAV data
            if (wifiCtrl->cmd == WIFI_MAV_DATA) {
                // Directly write the raw MAVLink payload to the UART for the flight controller
                pi_uart_write(&uart_device, rxp.data, rxp.dataLength);
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
 * @brief Sets the Home position. Crucial for non-GPS flight.
 * @param lat Latitude in degrees * 1e7
 * @param lon Longitude in degrees * 1e7
 * @param alt Altitude in millimeters
 */
void mavlink_set_home(int32_t lat, int32_t lon, int32_t alt) {
    printf("-> Setting EKF origin.\n");
    //cpxPrintToConsole(LOG_TO_WIFI, "-> Setting EKF origin.\n");
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

/**
 * @brief Sets the origin position. Crucial for non-GPS flight.
 * @param lat Latitude in degrees * 1e7
 * @param lon Longitude in degrees * 1e7
 * @param alt Altitude in millimeters
 */
void mavlink_set_origin(int32_t lat, int32_t lon, int32_t alt) {
    printf("-> Setting EKF origin.\n");
    //cpxPrintToConsole(LOG_TO_WIFI, "-> Setting EKF origin.\n");

    mavlink_message_t msg;
    mavlink_msg_set_gps_global_origin_pack_chan(
        GAP8_SYSTEM_ID, 
        GAP8_COMPONENT_ID, 
        MAVLINK_COMM_0, 
        &msg, 
        1, 
        lat, 
        lon, 
        alt,
        NAN
    );

    send_mavlink_message(&msg);

}

// Wrapper for MAV_CMD_NAV_TAKEOFF
void mavlink_command_takeoff(float altitude_m) {
    printf("-> Commanding takeoff to %.1f meters.\n", altitude_m);
    //cpxPrintToConsole(LOG_TO_WIFI, "-> Commanding takeoff to %.1f meters.\n", altitude_m);
    mavlink_send_command_long(MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, altitude_m);
}

// Wrapper for MAV_CMD_DO_SET_MODE
void mavlink_set_mode(uint8_t custom_mode) {
    printf("-> Setting mode to %d.\n", custom_mode);
    //cpxPrintToConsole(LOG_TO_WIFI, "-> Setting mode to %d.\n", custom_mode);
    mavlink_send_command_long(MAV_CMD_DO_SET_MODE, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, (float)custom_mode, 0, 0, 0, 0, 0);
}

// Wrapper for component arm
void mavlink_arm_vehicle() {
    printf("-> Sending ARM command.\n");
    //cpxPrintToConsole(LOG_TO_WIFI, "-> Sending ARM command.\n");
    mavlink_send_command_long(MAV_CMD_COMPONENT_ARM_DISARM, 1.0f, 0, 0, 0, 0, 0, 0);
}

void mavlink_disarm_vehicle() {
    printf("-> Sending DISARM command.\n");
    //cpxPrintToConsole(LOG_TO_WIFI, "-> Sending DISARM command.\n");
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
    printf("-> Setting local NED target to N:%.1f, E:%.1f, D:%.1f\n", x_north, y_east, z_down);
    //cpxPrintToConsole(LOG_TO_WIFI, "-> Setting local NED target to N:%.1f, E:%.1f, D:%.1f\n", x_north, y_east, z_down);

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
    printf("-> Commanding RTL.\n");
    //cpxPrintToConsole(LOG_TO_WIFI, "-> Commanding RTL.\n");
    // ArduCopter's custom mode number for RTL is 6.
    mavlink_set_mode(COPTER_MODE_RTL);
}

void mavlink_command_land() {
    printf("-> Commanding LAND.\n");
    //cpxPrintToConsole(LOG_TO_WIFI, "-> Commanding LAND.\n");
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

void mavlink_send_rc_override_4ch(uint8_t target_sys, uint8_t target_comp, uint16_t ch1_roll, uint16_t ch2_pitch, uint16_t ch3_throttle, uint16_t ch4_yaw)
{
    mavlink_message_t msg;

    // Ignore everything except ch1-4
    const uint16_t IGN = UINT16_MAX;

    mavlink_msg_rc_channels_override_pack(
        GAP8_SYSTEM_ID,
        GAP8_COMPONENT_ID,
        &msg,
        target_sys,
        target_comp,
        ch1_roll, ch2_pitch, ch3_throttle, ch4_yaw,  // ch1-4
        IGN, IGN, IGN, IGN,                          // ch5-8
        IGN, IGN, IGN, IGN, IGN, IGN, IGN, IGN, IGN, IGN  // ch9-18
    );

    send_mavlink_message(&msg);
}

static void mav_send_named_value_float(const char *name, float v)
{
    uint8_t mavlink_tx_buffer[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t msg;
    // time_boot_ms can be 0 if you don’t have a clock; QGC will still log it.
    mavlink_msg_named_value_float_pack(
        GAP8_SYSTEM_ID, GAP8_COMPONENT_ID,
        &msg,
        0,
        name,
        v
    );
    uint16_t len = mavlink_msg_to_send_buffer(mavlink_tx_buffer, &msg);
    cpxSendRawData(CPX_T_WIFI_HOST, CPX_F_CONSOLE, mavlink_tx_buffer, len);
    //send_mavlink_message(&msg);
}

static void mav_send_debug_vect(const char *name, float x, float y, float z)
{
    uint8_t mavlink_tx_buffer[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t msg;
    mavlink_msg_debug_vect_pack(
        GAP8_SYSTEM_ID, GAP8_COMPONENT_ID,
        &msg,
        name,
        0,   // time_boot_ms
        x, y, z
    );
    uint16_t len = mavlink_msg_to_send_buffer(mavlink_tx_buffer, &msg);
    cpxSendRawData(CPX_T_WIFI_HOST, CPX_F_CONSOLE, mavlink_tx_buffer, len);
    //send_mavlink_message(&msg);
}

static void mav_send_named_value_int(const char *name, int32_t v)
{
    uint8_t mavlink_tx_buffer[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t msg;
    mavlink_msg_named_value_int_pack(
        GAP8_SYSTEM_ID, GAP8_COMPONENT_ID,
        &msg,
        0,
        name,
        v
    );
    uint16_t len = mavlink_msg_to_send_buffer(mavlink_tx_buffer, &msg);
    cpxSendRawData(CPX_T_WIFI_HOST, CPX_F_CONSOLE, mavlink_tx_buffer, len);
    //send_mavlink_message(&msg);
}

static inline float wrap_pi(float a) {
    while (a >  M_PI) a -= 2.0f * M_PI;
    while (a < -M_PI) a += 2.0f * M_PI;
    return a;
}

static inline float clamp_f(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static inline int16_t clamp_i16(int32_t v, int16_t lo, int16_t hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return (int16_t)v;
}

static inline int16_t apply_deadband_i16(int16_t v, int16_t db) {
    if (v > -db && v < db) return 0;
    return v;
}

static inline int16_t slew_i16(int16_t prev, int16_t cmd, int16_t step)
{
    int16_t d = cmd - prev;
    if (d > step)  d = step;
    if (d < -step) d = -step;
    return (int16_t)(prev + d);
}

// =========================
// FRAME TRANSFORM (CRITICAL FIX)
// =========================
//
// Your position error is computed in world frame (N/E).
// Your RC roll/pitch commands act in body frame (Right/Forward).
//
// This function rotates the world error into the drone's body axes using yaw.
//
// ArduPilot yaw convention (for Copter): yaw=0 means facing North.
// Positive yaw rotates toward East (clockwise looking down).
//
// Output:
//   err_fwd   = how far "forward" the target is in the drone's body frame
//   err_right = how far "right"   the target is in the drone's body frame
//
static inline void ned_err_to_body(float yaw,
                                  float err_n, float err_e,
                                  float *err_fwd, float *err_right)
{
    float cy = cosf(yaw);
    float sy = sinf(yaw);

    // Body-forward is aligned with heading
    *err_fwd   =  cy * err_n + sy * err_e;

    // Body-right is +90deg clockwise from forward
    *err_right = -sy * err_n + cy * err_e;
}

static void get_square_corner(float origin_n, float origin_e, int idx, float *cn, float *ce)
{
    switch (idx & 3) {
    default:
    case 0: *cn = origin_n;                *ce = origin_e;                 break;
    case 1: *cn = origin_n;                *ce = origin_e + SQUARE_SIZE_M; break;
    case 2: *cn = origin_n + SQUARE_SIZE_M;*ce = origin_e + SQUARE_SIZE_M; break;
    case 3: *cn = origin_n + SQUARE_SIZE_M;*ce = origin_e;                 break;
    }
}

static int16_t g_prev_d_roll  = 0;
static int16_t g_prev_d_pitch = 0;
static int16_t g_prev_d_yaw   = 0;    

// =========================
// MAIN TRACKING STEP (CALL AT ~20Hz)
// =========================
//
// What it does each tick:
//  1) Ensure origin and yaw0 are captured once at start.
//  2) Choose current segment A->B (edge of the square).
//  3) Project current position onto the segment (get along-track progress s).
//  4) Pick a lookahead target point on the segment (smooth guidance).
//  5) Add some cross-track correction to pull onto the line.
//  6) Convert N/E guidance error into body forward/right error using yaw.
//  7) Convert body error into RC roll/pitch overrides (P control).
//  8) Optionally hold yaw to yaw0.
//  9) Switch to next segment when near B.
//
// Returns true when square is completed.
static bool path_track_square_bodyframe(void)
{
    // --- Current state from MAVLink ---
    float cur_n = g_vehicle_state.position_ned[0];  // LOCAL_POSITION_NED.x (North)
    float cur_e = g_vehicle_state.position_ned[1];  // LOCAL_POSITION_NED.y (East)
    float yaw   = g_vehicle_state.yaw;              // AHRS2.yaw (radians)

    // ------------------------------------------------------------
    // 1) CAPTURE ORIGIN + HEADING ONCE
    // ------------------------------------------------------------
    //
    // We define the square relative to where the drone is when tracking starts.
    // This makes the path repeatable and independent of global origin weirdness.
    if (!g_path.origin_set) {
        g_path.origin_n  = cur_n;
        g_path.origin_e  = cur_e;
        g_path.yaw0      = yaw;
        g_path.origin_set = true;
        g_path.seg        = 0;   // start on edge 0: corner0->corner1
        g_prev_d_roll  = 0;
        g_prev_d_pitch = 0;
        g_prev_d_yaw   = 0;        
    }

    // ------------------------------------------------------------
    // 2) SELECT CURRENT SEGMENT A->B
    // ------------------------------------------------------------
    float an, ae, bn, be;
    get_square_corner(g_path.origin_n, g_path.origin_e, g_path.seg,     &an, &ae);
    get_square_corner(g_path.origin_n, g_path.origin_e, g_path.seg + 1, &bn, &be);

    // Segment vector and length
    float dx = bn - an;
    float dy = be - ae;
    float L  = sqrtf(dx*dx + dy*dy);
    if (L < 1e-3f) {
        // Should never happen; fail safe to neutral sticks.
        mavlink_send_rc_override_4ch(STM32_SYSTEM_ID, STM32_COMPONENT_ID, RC_CENTER, RC_CENTER, RC_CENTER, RC_CENTER);
        return true;
    }

    // Unit direction along the segment
    float dir_n = dx / L;
    float dir_e = dy / L;

    // ------------------------------------------------------------
    // 3) PROJECT CURRENT POSITION ONTO SEGMENT (ALONG-TRACK "s")
    // ------------------------------------------------------------
    //
    // P = current position. A = segment start.
    // s = dot(P-A, dir) gives how far along the segment you are.
    //
    // Clamp to [0, L] so projection stays on the segment.
    float px = cur_n - an;
    float py = cur_e - ae;
    float s  = px * dir_n + py * dir_e;
    s = clamp_f(s, 0.0f, L);

    // Closest point C on the segment (used for cross-track correction)
    float c_n = an + dir_n * s;
    float c_e = ae + dir_e * s;

    // ------------------------------------------------------------
    // 4) LOOKAHEAD TARGET POINT (PURE PURSUIT IDEA)
    // ------------------------------------------------------------
    //
    // Instead of chasing the end corner directly, chase a point slightly ahead.
    // This reduces oscillations and improves smoothness.
    float s_look = clamp_f(s + LOOKAHEAD_M, 0.0f, L);
    float tgt_n  = an + dir_n * s_look;
    float tgt_e  = ae + dir_e * s_look;

    // ------------------------------------------------------------
    // 5) BUILD WORLD-FRAME GUIDANCE ERROR (N/E)
    // ------------------------------------------------------------
    //
    // Two pieces:
    //   (tgt - P) pushes you forward along the segment
    //   (C - P) pulls you back onto the line if you drift off it
    //
    // Cross-track gain of ~0.6 is a reasonable start. You can tune it.
    // Vector from current position to lookahead target:
    float v_tgt_n = tgt_n - cur_n;
    float v_tgt_e = tgt_e - cur_e;

    // Along error = projection onto dir
    float err_along = v_tgt_n*dir_n + v_tgt_e*dir_e;

    // Cross error = signed perpendicular distance to the line
    // Perp unit: (-dir_e, dir_n)
    //float err_cross = px*(-dir_e) + py*(dir_n);   // px,py = (cur - A)

    float cx = cur_n - c_n;
    float cy = cur_e - c_e;
    float err_cross = cx*(-dir_e) + cy*(dir_n);    

    float err_n = (err_along * dir_n) + (XTRACK_BLEND * err_cross * (-dir_e));
    float err_e = (err_along * dir_e) + (XTRACK_BLEND * err_cross * ( dir_n));

    // ------------------------------------------------------------
    // 6) SEGMENT SWITCHING (WHEN NEAR END CORNER)
    // ------------------------------------------------------------
    //
    // We want to switch to next edge only when:
    //   - we've progressed most of the way (s > 0.85L), and
    //   - we're close to the end corner B (distance < END_RADIUS)
    //
    float end_dn = bn - cur_n;
    float end_de = be - cur_e;
    float end_dist2 = end_dn*end_dn + end_de*end_de;

    static uint8_t end_ok_cnt = 0;
    bool end_ok = (s > 0.85f*L) && (end_dist2 < END_RADIUS_M*END_RADIUS_M);

    if (end_ok) end_ok_cnt++;
    else end_ok_cnt = 0;

    if (end_ok_cnt >= 6) {
        end_ok_cnt = 0;
        mavlink_send_rc_override_4ch(STM32_SYSTEM_ID, STM32_COMPONENT_ID, RC_CENTER, RC_CENTER, RC_CENTER, RC_CENTER);
        return true;
    }    

    //if ((s > (0.85f * L)) && (end_dist2 < (END_RADIUS_M * END_RADIUS_M))) {

        // Briefly neutralize during the transition (reduces corner kick)
    //    mavlink_send_rc_override_4ch(STM32_SYSTEM_ID, STM32_COMPONENT_ID, RC_CENTER, RC_CENTER, RC_CENTER, RC_CENTER);
        
    //    return true;

    //}

    // ------------------------------------------------------------
    // 7) WORLD ERROR -> BODY ERROR (THE FIX FOR YAW DRIFT)
    // ------------------------------------------------------------
    //
    // If yaw rotates, body axes rotate, so we rotate err_n/err_e into body axes.
    // Now "forward error" always maps to pitch, "right error" always maps to roll.
    float err_fwd, err_right;
    ned_err_to_body(yaw, err_n, err_e, &err_fwd, &err_right);

    // ------------------------------------------------------------
    // 8) BODY ERROR -> RC OVERRIDES (P CONTROL)
    // ------------------------------------------------------------
    //
    // d_pitch corresponds to forward/back "stick".
    // d_roll  corresponds to right/left "stick".
    int16_t d_pitch = (int16_t)(KP * err_fwd);
    int16_t d_roll  = (int16_t)(KP * err_right);

    // Deadband to prevent jitter from noise
    d_pitch = apply_deadband_i16(d_pitch, RC_DEADBAND_XY);
    d_roll  = apply_deadband_i16(d_roll,  RC_DEADBAND_XY);

    // Clamp for safety and to avoid over-commanding LOITER
    d_pitch = clamp_i16(d_pitch, -RC_MAX, RC_MAX);
    d_roll  = clamp_i16(d_roll,  -RC_MAX, RC_MAX);

    d_pitch = slew_i16(g_prev_d_pitch, d_pitch, SLEW_STEP_PITCH);
    d_roll  = slew_i16(g_prev_d_roll,  d_roll,  SLEW_STEP_ROLL);

    g_prev_d_pitch = d_pitch;
    g_prev_d_roll  = d_roll;    

    uint16_t ch1_roll  = (uint16_t)(RC_CENTER + d_roll);    // +roll -> East
    uint16_t ch2_pitch = (uint16_t)(RC_CENTER - d_pitch);   // +pitch demand -> smaller PWM -> North

    // ------------------------------------------------------------
    // 9) OPTIONAL YAW HOLD (HELPS A LOT INDOORS)
    // ------------------------------------------------------------
    //
    // We do a simple P controller on yaw: hold yaw near yaw0.
    // This reduces drift in heading and keeps control more consistent.
    uint16_t ch4_yaw = RC_CENTER;

#if YAW_HOLD_ENABLE
    float yaw_err = wrap_pi(g_path.yaw0 - yaw);

    // small deadband so you don't constantly "buzz" yaw
    if (fabsf(yaw_err) < YAW_ERR_DEADBAND) yaw_err = 0.0f;

    int16_t d_yaw = (int16_t)(YAW_KP_RAD2RC * yaw_err);
    d_yaw = clamp_i16(d_yaw, -RC_MAX_DELTA_YAW, RC_MAX_DELTA_YAW);

    d_yaw = slew_i16(g_prev_d_yaw, d_yaw, SLEW_STEP_YAW);
    g_prev_d_yaw = d_yaw;

    ch4_yaw = (uint16_t)(RC_CENTER + d_yaw);
#endif

    // ------------------------------------------------------------
    // 10) SEND RC OVERRIDES
    // ------------------------------------------------------------
    //
    // In LOITER, throttle is typically managed by the autopilot.
    // Your setup uses RC_CENTER for throttle to "not interfere".
    //
    // If you find it climbs/descends, you can manage throttle separately,
    // but for now we leave it centered.
    mavlink_send_rc_override_4ch(STM32_SYSTEM_ID, STM32_COMPONENT_ID, ch1_roll, ch2_pitch, RC_CENTER, ch4_yaw);

    // --- LOGGING --- //
    static uint32_t dbg_decim = 0;
    dbg_decim++;
    if ((dbg_decim % 4) == 0) { // 20 Hz loop -> 5 Hz logging
        mav_send_named_value_int("seg", g_path.seg);

        mav_send_debug_vect("pos_ne", cur_n, cur_e, 0.0f);
        mav_send_debug_vect("tgt_ne", tgt_n, tgt_e, 0.0f);

        mav_send_debug_vect("err_ne", err_n, err_e, 0.0f);
        mav_send_debug_vect("err_bf", err_fwd, err_right, 0.0f);

        mav_send_named_value_float("yaw", yaw);

        mav_send_debug_vect("prog", s, L, sqrtf(end_dist2));
#if YAW_HOLD_ENABLE
        mav_send_named_value_float("yaw_err", yaw_err);
#endif
        mav_send_named_value_float("q", (float)g_vehicle_state.quality);

        // If you want to see what you’re commanding:
        mav_send_debug_vect("rc_xy", (float)d_roll, (float)d_pitch, 0.0f);
    }    
    // --- LOGGING --- //    

    return false;
}


// The mission task function
void autonomous_mission_task(void *parameters) {
    VehicleState_t *state = (VehicleState_t *)parameters;

    // --- State-specific counters ---
    const int MISSION_DT_MS = (1000 / MISSION_HZ);
    const int ARM_TIME_MS = 3000;
    const int ARM_TICKS = (ARM_TIME_MS / MISSION_DT_MS);
    const int TAKEOFF_TIME_MS = 1200;
    const int TAKEOFF_TICKS = (TAKEOFF_TIME_MS / MISSION_DT_MS);
    const int LOITER_TIME_MS = 1000;
    const int LOITER_TICKS = (LOITER_TIME_MS / MISSION_DT_MS);
    const int HOVER_TIME_MS = 1500;
    const int HOVER_TICKS = (HOVER_TIME_MS / MISSION_DT_MS);
    const int MOVE_TIME_MS = 10000;
    const int MOVE_TICKS = (MOVE_TIME_MS / MISSION_DT_MS);
    const int BRAKE_TIME_MS = 1500;
    const int BRAKE_TICKS = (BRAKE_TIME_MS / MISSION_DT_MS);
    const int LAND_TIME_MS = 2000;
    const int LAND_TICKS = (LAND_TIME_MS / MISSION_DT_MS);    
    const int FLOW_BAD_TIME_MS = 500;
    const int FLOW_BAD_TICKS = (FLOW_BAD_TIME_MS / MISSION_DT_MS);     
    const int THR_MIN = 1000;
    const int THR_MAX = 1600;
    const int THR_STEP = 50;
    const int THR_HOLD = 1550;
    const float V_BAD_MPS = 1.5f;
    const float V_BAD2 = (V_BAD_MPS * V_BAD_MPS);
    const uint8_t Q_BAD = 10;      
    static int arm_ticks = 0;   
    static int takeoff_ticks = 0;
    static int loiter_ticks = 0;
    static int hover_ticks = 0;
    static int move_ticks = 0;
    static int brake_ticks = 0;
    static int land_ticks = 0;
    static int flow_bad_ticks = 0;
    static int qual_bad_ticks = 0;    

    static float alt_start = 0.0f;
    static uint16_t thr_pwm = 0;

    static int16_t s_takeoff_throttle = 500; 

    printf("Mission task started. Waiting for start command via WiFi...\n");
    //cpxPrintToConsole(LOG_TO_WIFI, "Mission task started. Waiting for start command via WiFi...\n");

    while (1) {
        if (is_mission_terminate) {
            cpxPrintToConsole(LOG_TO_WIFI, "Mission terminated by user. Disarming...\n");
            mavlink_disarm_vehicle();
            is_mission_terminate = false; // Clear the flag
            g_mission_state = MISSION_DONE;
            // Continue to the switch to immediately enter the DONE state
        }     
        
        // --- SAFETY --- //
        float v2 = (g_vehicle_state.flow_comp_m_x * g_vehicle_state.flow_comp_m_x) + (g_vehicle_state.flow_comp_m_y * g_vehicle_state.flow_comp_m_y);
        if (v2 > V_BAD2) {
            flow_bad_ticks++;
        } else {
            flow_bad_ticks = 0;
        }         
        if (g_vehicle_state.quality <= Q_BAD) {
            qual_bad_ticks++;
        } else {
            qual_bad_ticks = 0;
        }        
        bool flow_lost = (flow_bad_ticks >= FLOW_BAD_TICKS) || (qual_bad_ticks >= FLOW_BAD_TICKS);

        bool in_path =
            (g_mission_state == MISSION_PATH_EAST)  ||
            (g_mission_state == MISSION_PATH_NORTH) ||
            (g_mission_state == MISSION_PATH_WEST)  ||
            (g_mission_state == MISSION_PATH_SOUTH);

        if (flow_lost && /* only during maneuver */ 
            (in_path || (g_mission_state == MISSION_HOVER))) {

            printf("FLOW LOST: v2=%.2f (m/s)^2 qual=%u -> LAND\n", v2, g_vehicle_state.quality);
            g_mission_state = MISSION_COMMAND_LAND;
        }
        // --- SAFETY --- //

        switch (g_mission_state) {   
            case MISSION_WAIT_FOR_COMMAND:
                if (is_mission_start) {
                    is_mission_start = false; // Clear the flag
                    printf("Mission: Starting Mission!\n");
                    mavlink_arm_vehicle();
                    mavlink_send_rc_override_4ch(STM32_SYSTEM_ID, STM32_COMPONENT_ID, 1500, 1500, THR_MIN, 1500);
                    g_mission_state = MISSION_ARM;
                }         
                break;
            case MISSION_ARM:
                if (arm_ticks >= ARM_TICKS) {
                    arm_ticks = 0;
                    printf("Mission: Drone Armed. Taking off...\n");

                    mavlink_send_rc_override_4ch(STM32_SYSTEM_ID, STM32_COMPONENT_ID, 1500, 1500, THR_MIN, 1500);

                    takeoff_ticks = 0;
                    alt_start = g_vehicle_state.altitude_m;
                    thr_pwm = 1000;            

                    g_mission_state = MISSION_TAKEOFF;
                }
                else {
                    mavlink_send_rc_override_4ch(STM32_SYSTEM_ID, STM32_COMPONENT_ID, 1500, 1500, THR_MIN, 1500);
                    arm_ticks++;
                }
                break;
            case MISSION_TAKEOFF: {
                // 1) Timeout
                if (takeoff_ticks++ > TAKEOFF_TICKS) {
                    printf("Mission: Takeoff timeout -> LAND\n");

                    mavlink_send_rc_override_4ch(STM32_SYSTEM_ID, STM32_COMPONENT_ID, 1500, 1500, THR_HOLD, 1500);

                    mavlink_set_mode(COPTER_MODE_LOITER);

                    g_mission_state = MISSION_WAIT_FOR_LOITER;
                    break;
                }

                // 5) Ramp throttle
                if (thr_pwm < THR_MAX) {
                    thr_pwm = (uint16_t)((thr_pwm + THR_STEP > THR_MAX) ? THR_MAX : thr_pwm + THR_STEP);
                }

                // Keep roll/pitch centered during takeoff
                mavlink_send_rc_override_4ch(STM32_SYSTEM_ID, STM32_COMPONENT_ID, 1500, 1500, thr_pwm, 1500);

                break;
            }
            case MISSION_WAIT_FOR_LOITER:
                if (g_vehicle_state.mode == COPTER_MODE_LOITER) {
                    printf("Mission: Loiter Good.\n");
                    loiter_ticks = 0;
                    g_mission_state = MISSION_HOVER;
                    break;
                }
                if (loiter_ticks < LOITER_TICKS) {
                    loiter_ticks++;
                }
                else {
                    printf("Mission: Loiter Failed. Landing...\n");
                    loiter_ticks = 0;
                    g_mission_state = MISSION_COMMAND_LAND;
                }
                break;
            case MISSION_HOVER:
                if (hover_ticks >= HOVER_TICKS) {
                    hover_ticks = 0;
                    move_ticks = 0;
                    printf("Mission: Hover complete. Moving left...\n");

                    g_path.origin_set = false;

                    g_mission_state = MISSION_PATH_EAST;
                } else {
                    hover_ticks++;
                }
                break;
            case MISSION_PATH_EAST: {
                bool done = path_track_square_bodyframe();
                if (done) {
                    move_ticks = 0;
                    g_path.seg++;        
                    printf("East move complete -> North\n");
                    g_mission_state = MISSION_PATH_NORTH;
                }
                else if (move_ticks >= MOVE_TICKS) {
                    move_ticks = 0;
                    printf("East move failed -> Landing...\n");
                    g_mission_state = MISSION_COMMAND_LAND;                    
                }
                else {
                    move_ticks++;
                }
                break;                
            }
            case MISSION_PATH_NORTH: {
                bool done = path_track_square_bodyframe();
                if (done) {
                    move_ticks = 0;
                    g_path.seg++;                
                    printf("North move complete -> West\n");
                    g_mission_state = MISSION_PATH_WEST;
                }
                else if (move_ticks >= MOVE_TICKS) {
                    move_ticks = 0;
                    printf("North move failed -> Landing...\n");
                    g_mission_state = MISSION_COMMAND_LAND;                    
                }
                else {
                    move_ticks++;
                }                
                break;                
            }
            case MISSION_PATH_WEST: {
                bool done = path_track_square_bodyframe();
                if (done) {
                    move_ticks = 0;
                    g_path.seg++;              
                    printf("West move complete -> South\n");
                    g_mission_state = MISSION_PATH_SOUTH;
                }
                else if (move_ticks >= MOVE_TICKS) {
                    move_ticks = 0;
                    printf("West move failed -> Landing...\n");
                    g_mission_state = MISSION_COMMAND_LAND;                    
                }
                else {
                    move_ticks++;
                }                
                break;                
            }
            case MISSION_PATH_SOUTH: {
                bool done = path_track_square_bodyframe();
                if (done) {
                    move_ticks = 0;
                    g_path.seg++;                 
                    printf("South move complete -> Land\n");
                    g_mission_state = MISSION_COMMAND_LAND;
                }
                else if (move_ticks >= MOVE_TICKS) {
                    move_ticks = 0;
                    printf("South move failed -> Landing...\n");
                    g_mission_state = MISSION_COMMAND_LAND;                    
                }
                else {
                    move_ticks++;
                }                
                break;                
            }                                    
            case MISSION_COMMAND_LAND: {
                mavlink_command_land();

                g_mission_state = MISSION_WAIT_FOR_LAND;

                break;
            }
            case MISSION_WAIT_FOR_LAND:
                if (land_ticks >= LAND_TICKS) {
                    land_ticks = 0;
                    printf("Mission: Land Done. Disarming...\n");
                    //cpxPrintToConsole(LOG_TO_WIFI, "Mission: Land Done. Disarming...\n");
                    g_mission_state = MISSION_DISARM;
                } else {
                    //printf("Landing...\n");
                    //cpxPrintToConsole(LOG_TO_WIFI, "Landing...\n");
                    land_ticks++;
                }
                break;
            case MISSION_DISARM:
                if (land_ticks >= LAND_TICKS) {
                    land_ticks = 0;
                    
                    mavlink_disarm_vehicle();

                    mavlink_set_mode(COPTER_MODE_STABILIZE);

                    printf("Mission: Done!\n");
                    //cpxPrintToConsole(LOG_TO_WIFI, "Mission: Done!\n");
                    g_mission_state = MISSION_DONE;
                } else {
                    // mavlink_send_manual_control(0, 0, 0, 0);
                    mavlink_send_rc_override_4ch(STM32_SYSTEM_ID, STM32_COMPONENT_ID, 1500, 1500, THR_MIN, 1500);     

                    //printf("Disarming...\n");
                    //cpxPrintToConsole(LOG_TO_WIFI, "Disarming...\n");
                    land_ticks++;
                }
                break;                               
            case MISSION_DONE:
                if (g_vehicle_state.mode == COPTER_MODE_STABILIZE) {
                    printf("Mission state is being reset.\n");
                    mavlink_send_rc_override_4ch(STM32_SYSTEM_ID, STM32_COMPONENT_ID, 1500, 1500, THR_MIN, 1500);     

                    is_mission_reset = false; // <-- IMPORTANT: Clear the reset flag
                    // Go back to the very first state
                    g_mission_state = MISSION_WAIT_FOR_COMMAND;
                }
                break;                            
        }
        vTaskDelay(MISSION_DT_MS / portTICK_PERIOD_MS); // Loop 20Hz
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
        printf("FATAL: UART open failed!\n");
        cpxPrintToConsole(LOG_TO_WIFI, "FATAL: UART open failed!\n");
        pmsis_exit(-1);
    }
    printf("UART opened successfully.\n");
    cpxPrintToConsole(LOG_TO_WIFI, "UART opened successfully.\n");

    printf("-- GAP8 MAVLink WiFi Test --\n");
    cpxPrintToConsole(LOG_TO_WIFI, "-- GAP8 MAVLink WiFi Test --\n");

    // Enable the CPX function for WiFi control commands
    cpxEnableFunction(CPX_F_MAVLINK_DOWNLINK);

    BaseType_t xTask;
    xTask = xTaskCreate(uart_receive_task, "uart_receive_task", configMINIMAL_STACK_SIZE * 8, NULL, tskIDLE_PRIORITY + 1, NULL);
    if (xTask != pdPASS) {
        printf("FATAL: UART receive task creation failed!\n");
        cpxPrintToConsole(LOG_TO_WIFI, "FATAL: UART receive task creation failed!\n");
    }

    xTask = xTaskCreate(wifi_mav_receive_task, "wifi_mav_receive_task", configMINIMAL_STACK_SIZE * 8, NULL, tskIDLE_PRIORITY + 1, NULL);
    if (xTask != pdPASS) {
        printf("FATAL: WiFi Mav task creation failed!\n");
        cpxPrintToConsole(LOG_TO_WIFI, "FATAL: WiFi Mav task creation failed!\n");
    }    

    xTask = xTaskCreate(autonomous_mission_task, "mission_task", configMINIMAL_STACK_SIZE * 8, &g_vehicle_state, tskIDLE_PRIORITY + 1, NULL);
    if (xTask != pdPASS) {
        printf("FATAL: Mission task creation failed!\n");
        cpxPrintToConsole(LOG_TO_WIFI, "FATAL: Mission task creation failed!\n");
    }      

    xTask = xTaskCreate(led_blinky_task, "led_debug_task", configMINIMAL_STACK_SIZE * 2, NULL, tskIDLE_PRIORITY + 1, NULL);
    if (xTask != pdPASS) {
        printf("FATAL: LED debug task creation failed!\n");
        cpxPrintToConsole(LOG_TO_WIFI, "FATAL: LED debug task creation failed!\n");
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