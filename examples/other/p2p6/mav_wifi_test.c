#include "pmsis.h"
#include "bsp/bsp.h"
#include "cpx.h"
#include "ardupilotmega/mavlink.h"
#include "wifi.h" // Include the WiFi control header
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>

#define LED_PIN (2)
#define P2P_STX_ATT_V1 0xA7
#define P2P_ATT_V1_LEN 20
#define PEER_SYSTEM_ID 20
#define PEER2_SYSTEM_ID 21
#define P2P_MAX_CHAN   1   // adjust to however many logical streams
#define GAP8_SYSTEM_ID 255
#define GAP8_COMPONENT_ID 1
#define STM32_SYSTEM_ID 1
#define STM32_COMPONENT_ID 1

// --- Global Variables ---
static struct pi_device uart_device;
static pi_device_t led_device;
static CPXPacket_t txp; // Packet for sending CPX commands
static CPXPacket_t rxp; // Packet for receiving CPX commands
uint8_t send_buffer[MAVLINK_MAX_PACKET_LEN];

static volatile bool g_p2p_data_received_once = false; // Becomes true once we get the first p2p ATTITUDE message

// Define the states for your mission
typedef enum {
    MISSION_WAIT_FOR_COMMAND,
    MISSION_WAIT_FOR_LOITER,
    MISSION_ARM,
    MISSION_TAKEOFF,
    MISSION_HOVER,
    VOODOO_CONTROL,
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
        case MISSION_ARM:                   return "ARM";
        case MISSION_TAKEOFF:               return "TAKEOFF";
        case MISSION_HOVER:                 return "HOVER";
        case VOODOO_CONTROL:                return "VOODOO";
        case MISSION_COMMAND_LAND:          return "COMMAND_LAND";
        case MISSION_WAIT_FOR_LAND:         return "WAIT_FOR_LAND";
        case MISSION_DISARM:                return "DISARM";
        case MISSION_DONE:                  return "DONE";
        default:                            return "UNKNOWN_STATE";
    }
}

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

typedef struct {
    float roll;
    float pitch;
    float yaw;
    TickType_t last_rx_tick;
} PeerState_t;

PeerState_t g_peer_state = {0}; // Holds the last known attitude of the peer

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

#pragma pack(push, 1)
typedef struct {
    uint8_t  stx;           // 0xA7
    uint8_t  peer_id;
    uint32_t time_boot_ms;
    int16_t  roll_cd;       // centi-deg
    int16_t  pitch_cd;      // centi-deg
    int16_t  yaw_cd;        // centi-deg
    int16_t  rollrate_cds;  // centi-deg/s
    int16_t  pitchrate_cds; // centi-deg/s
    int16_t  yawrate_cds;   // centi-deg/s
    uint8_t  c0;            // Fletcher-8
    uint8_t  c1;            // Fletcher-8
} p2p_att_v1_t;
#pragma pack(pop)

_Static_assert(sizeof(p2p_att_v1_t) == P2P_ATT_V1_LEN, "p2p_att_v1_t must be 20 bytes");

static inline void p2p_fletcher8(const uint8_t *buf, uint8_t len, uint8_t *c0, uint8_t *c1)
{
    uint8_t a = 0, b = 0;
    for (uint8_t i = 0; i < len; i++) {
        a += buf[i];
        b += a;
    }
    *c0 = a;
    *c1 = b;
}

static inline bool p2p_fletcher8_ok(const uint8_t *buf, uint8_t len_with_crc)
{
    if (len_with_crc < 3) return false;
    uint8_t c0, c1;
    p2p_fletcher8(buf, (uint8_t)(len_with_crc - 2), &c0, &c1);
    return (c0 == buf[len_with_crc - 2]) && (c1 == buf[len_with_crc - 1]);
}

typedef struct {
    uint8_t  buf[P2P_ATT_V1_LEN];
    uint8_t  idx;
    uint32_t parse_errors;
    uint32_t packets_ok;
} p2p_parser_state_t;

static p2p_parser_state_t g_p2p_state[P2P_MAX_CHAN];

static inline void p2p_parser_reset(uint8_t chan)
{
    g_p2p_state[chan].idx = 0;
}

/**
 * Parse one byte of the P2P attitude stream.
 *
 * Returns 1 if a full valid packet was decoded into *out.
 * Returns 0 otherwise.
 *
 * - Resyncs on STX.
 * - Validates Fletcher checksum.
 * - Copies the packet into *out only on success.
 */
uint8_t p2p_att_parse_char(uint8_t chan, uint8_t c, p2p_att_v1_t *out)
{
    if (chan >= P2P_MAX_CHAN || out == NULL) {
        return 0;
    }

    p2p_parser_state_t *st = &g_p2p_state[chan];

    // If we're idle, wait for STX
    if (st->idx == 0) {
        if (c != P2P_STX_ATT_V1) {
            return 0;
        }
        st->buf[st->idx++] = c;
        return 0;
    }

    // If we see STX unexpectedly mid-packet, treat as resync (start new packet)
    if (c == P2P_STX_ATT_V1) {
        st->buf[0] = c;
        st->idx = 1;
        return 0;
    }

    st->buf[st->idx++] = c;

    if (st->idx < P2P_ATT_V1_LEN) {
        return 0; // still accumulating
    }

    // We have 20 bytes: validate checksum
    bool ok = p2p_fletcher8_ok(st->buf, P2P_ATT_V1_LEN);

    if (!ok) {
        st->parse_errors++;

        // Try to resync: if last byte could be STX, restart; else reset.
        // (We already handled "c == STX" above, so just reset.)
        st->idx = 0;
        return 0;
    }

    // Success
    memcpy(out, st->buf, P2P_ATT_V1_LEN);
    st->packets_ok++;

    // Ready for next packet
    st->idx = 0;
    return 1;
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
 * @brief Task to listen for and parse incoming MAVLink messages.
 */
void uart_receive_task(void *parameters) {
    uint8_t rx_byte;
    mavlink_message_t received_msg;
    mavlink_status_t status = {0};
    p2p_att_v1_t pkt;

    // --- Buffer for re-serializing the validated MAVLink message ---
    uint8_t mavlink_tx_buffer[MAVLINK_MAX_PACKET_LEN];

    while(1) {
        pi_uart_read(&uart_device, &rx_byte, 1);

        if (mavlink_parse_char(MAVLINK_COMM_0, rx_byte, &received_msg, &status)) {
                // --- 1. Validation Passed: A complete message was received ---

                // --- 2. Re-serialize the message into a raw byte buffer ---
                uint16_t len = mavlink_msg_to_send_buffer(mavlink_tx_buffer, &received_msg);

                // --- 3. Forward the raw buffer over WiFi using our safe function ---
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
                        // printf("FC Status: %s\n", status.text);
                        //cpxPrintToConsole(LOG_TO_WIFI, "FC Status: %s\n", status.text);
                        break;
                    }                
                    case MAVLINK_MSG_ID_PARAM_VALUE: {
                        // We received a parameter value, decode it
                        mavlink_param_value_t param;
                        mavlink_msg_param_value_decode(&received_msg, &param);
                        
                        // Print the received parameter name and its value over WiFi
                        //printf("<- PARAM_VALUE: %s = %f\n", param.param_id, param.param_value);
                        //cpxPrintToConsole(LOG_TO_WIFI, "<- PARAM_VALUE: %s = %f\n", param.param_id, param.param_value);
                        break;
                    }          
                    case MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN:
                        is_origin_set = true;
                        
                        break;
                    case MAVLINK_MSG_ID_OPTICAL_FLOW: {
                        mavlink_optical_flow_t opt;
                        mavlink_msg_optical_flow_decode(&received_msg, &opt);

                        g_vehicle_state.flow_comp_m_x = opt.flow_comp_m_x;
                        g_vehicle_state.flow_comp_m_y = opt.flow_comp_m_y;
                        g_vehicle_state.quality = opt.quality;

                        break;
                    }
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
        if (p2p_att_parse_char(0, rx_byte, &pkt)) {
            // printf("Received P2P [%d]", pkt.peer_id);

            if (pkt.peer_id == PEER_SYSTEM_ID) {
                if (!g_p2p_data_received_once) {
                    g_p2p_data_received_once = true;
                    printf("Peer stream active [%d]", pkt.peer_id);
                }              
                
                // Keep track of how many attitude messages we've received
                static int attitude_msg_count = 0;
                attitude_msg_count++;                

                // Update the global peer state
                g_peer_state.roll = pkt.roll_cd / 100.0f;
                g_peer_state.pitch = pkt.pitch_cd / 100.0f;
                g_peer_state.yaw = pkt.yaw_cd / 100.0f;  
                g_peer_state.last_rx_tick = xTaskGetTickCount();
                
                // Every 10 messages, print the data
                if (attitude_msg_count % 10 == 0) {
                    // Print the attitude data to the WiFi console
                    //printf("<- P2P ATTITUDE: Roll=%.2f Pitch=%.2f Yaw=%.2f\n", g_peer_state.roll, g_peer_state.pitch, g_peer_state.yaw);
                    // cpxPrintToConsole(LOG_TO_WIFI, "<- P2P ATTITUDE: Roll=%.2f Pitch=%.2f Yaw=%.2f\n", g_peer_state.roll, g_peer_state.pitch, g_peer_state.yaw);
                }                                 
            }
        }
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

// --- HELPERS --- //
/**
 * @brief Sends a MAVLink message structure over the UART.
 */
void send_mavlink_message(const mavlink_message_t *msg) {
    uint16_t len = mavlink_msg_to_send_buffer(send_buffer, msg);
    pi_uart_write(&uart_device, send_buffer, len);
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

// --- HELPERS --- //

// The mission task function
void voodoo_task(void *parameters) {
    VehicleState_t *state = (VehicleState_t *)parameters;

    // --- State-specific counters ---

    const int VOODOO_HZ = 10;
    const int VOODOO_DT_MS = (1000 / VOODOO_HZ);    
    const int VOODOO_MOVE_STRENGTH = 100;       // Command strength for manual_control (10% stick input)
    const int VOODOO_PITCH_THRESHOLD = 20; // ~20 degrees
    const int VOODOO_ROLL_THRESHOLD = 20;  // ~20 degrees
    const int VOODOO_TIMEOUT_MS = 3000;        // 5-second timeout 
    const int VOODOO_TICKS = (VOODOO_TIMEOUT_MS / VOODOO_DT_MS);    
    const int ARM_TIME_MS = 3000;
    const int ARM_TICKS = (ARM_TIME_MS / VOODOO_DT_MS);
    const int TAKEOFF_TIME_MS = 1200;
    const int TAKEOFF_TICKS = (TAKEOFF_TIME_MS / VOODOO_DT_MS);
    const int LOITER_TIME_MS = 1000;
    const int LOITER_TICKS = (LOITER_TIME_MS / VOODOO_DT_MS);      
    const int HOVER_TIME_MS = 3000;
    const int HOVER_TICKS = (HOVER_TIME_MS / VOODOO_DT_MS);    
    const int LAND_TIME_MS = 2000;
    const int LAND_TICKS = (LAND_TIME_MS / VOODOO_DT_MS);    
    const int FLOW_BAD_TIME_MS = 500;
    const int FLOW_BAD_TICKS = (FLOW_BAD_TIME_MS / VOODOO_DT_MS);
    const int THR_MIN = 1000;
    const int THR_MAX = 1600;
    const int THR_STEP = 100;
    const int THR_HOLD = 1550;
    const float V_BAD_MPS = 1.5f;
    const float V_BAD2 = (V_BAD_MPS * V_BAD_MPS);
    const uint8_t Q_BAD = 10;
    static int arm_ticks = 0;   
    static int takeoff_ticks = 0;
    static int loiter_ticks = 0;
    static int hover_ticks = 0;
    static int voodoo_ticks = 0;
    static int land_ticks = 0;
    static int flow_bad_ticks = 0;
    static int qual_bad_ticks = 0;

    static float alt_start = 0.0f;
    static uint16_t thr_pwm = 0;       

    //cpxPrintToConsole(LOG_TO_WIFI, "Mission task started. Waiting for start command via WiFi...\n");

    while (1) {
        if (is_mission_terminate) {
            printf("Mission terminated by user. Disarming...\n");
            //cpxPrintToConsole(LOG_TO_WIFI, "Mission terminated by user. Disarming...\n");
            mavlink_disarm_vehicle();
            is_mission_terminate = false; // Clear the flag
            g_mission_state = MISSION_DONE;
            // Continue to the switch to immediately enter the DONE state
        }
        switch (g_mission_state) {
            case MISSION_WAIT_FOR_COMMAND:
                if (is_mission_start && g_p2p_data_received_once) {
                    is_mission_start = false; // Clear the flag
                    g_p2p_data_received_once = false;
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
                    printf("Mission: Takeoff complete. Hovering...\n");

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
                    voodoo_ticks = 0;
                    flow_bad_ticks = 0;
                    qual_bad_ticks = 0;                    
                    printf("Mission: Hover complete. Moving left...\n");

                    g_mission_state = VOODOO_CONTROL;
                } else {
                    hover_ticks++;
                }
                break;
            case VOODOO_CONTROL: {
                if (voodoo_ticks >= VOODOO_TICKS) {
                    voodoo_ticks = 0;
                    printf("Voodoo: Voodoo Complete! Landing.\n");
                    //cpxPrintToConsole(LOG_TO_WIFI, "Voodoo: Voodoo Complete! Landing.\n");
                    g_mission_state = MISSION_COMMAND_LAND;
                    break; // Exit the state immediately
                }
                // --- SAFETY --- //
                if (xTaskGetTickCount() - g_peer_state.last_rx_tick > pdMS_TO_TICKS(VOODOO_TIMEOUT_MS)) {
                    printf("Voodoo: Peer Timeout! Landing.\n");
                    //cpxPrintToConsole(LOG_TO_WIFI, "Voodoo: Peer data timeout! Landing.\n");
                    g_mission_state = MISSION_COMMAND_LAND;
                    break; // Exit the state immediately                    
                }      
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
                if (flow_bad_ticks >= FLOW_BAD_TICKS || qual_bad_ticks >= FLOW_BAD_TICKS) {
                    printf("FLOW LOST: v=%.2f m/s qual=%u -> LAND\n", v2, g_vehicle_state.quality);
                    g_mission_state = MISSION_COMMAND_LAND;
                    break;
                }                
                // --- SAFETY --- //                

                // Local copies of peer attitude for this loop
                float peer_roll = g_peer_state.roll;
                float peer_pitch = g_peer_state.pitch;

                // --- Single-Action-Priority Logic ---
                if (peer_pitch < -VOODOO_PITCH_THRESHOLD) {
                    // Move Forward (Pitch stick forward is negative X)
                    voodoo_ticks = 0;
                    mavlink_send_rc_override_4ch(STM32_SYSTEM_ID, STM32_COMPONENT_ID, 1500, 1300, 1500, 1500);
                }
                else if (peer_pitch > VOODOO_PITCH_THRESHOLD) {
                    // Move Backward (Pitch stick back is positive X)
                    voodoo_ticks = 0;
                    mavlink_send_rc_override_4ch(STM32_SYSTEM_ID, STM32_COMPONENT_ID, 1500, 1700, 1500, 1500);
                }
                else if (peer_roll > VOODOO_ROLL_THRESHOLD) {
                    // Move Right (Roll stick right is positive Y)
                    voodoo_ticks = 0;
                    mavlink_send_rc_override_4ch(STM32_SYSTEM_ID, STM32_COMPONENT_ID, 1700, 1500, 1500, 1500);
                }
                else if (peer_roll < -VOODOO_ROLL_THRESHOLD) {
                    // Move Left (Roll stick left is negative Y)
                    voodoo_ticks = 0;
                    mavlink_send_rc_override_4ch(STM32_SYSTEM_ID, STM32_COMPONENT_ID, 1300, 1500, 1500, 1500);
                }
                else {
                    // No thresholds met, default to a stable hover
                    voodoo_ticks++;
                    mavlink_send_rc_override_4ch(STM32_SYSTEM_ID, STM32_COMPONENT_ID, 1500, 1500, 1500, 1500);
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
        vTaskDelay(VOODOO_DT_MS / portTICK_PERIOD_MS); // Loop 10Hz
    }
}

/**
 * @brief The main entry point for the application.
 */
void start_main_application(void *parameters) {

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
    cpxEnableFunction(CPX_F_MAVLINK_DOWNLINK);

    BaseType_t xTask;

    xTask = xTaskCreate(uart_receive_task, "uart_receive_task", configMINIMAL_STACK_SIZE * 8, NULL, tskIDLE_PRIORITY + 1, NULL);
    if (xTask != pdPASS) {
        printf("FATAL: UART receive task creation failed!\n");
        //cpxPrintToConsole(LOG_TO_WIFI, "FATAL: UART receive task creation failed!\n");
    }

    xTask = xTaskCreate(wifi_command_receive_task, "wifi_cmd_rx_task", configMINIMAL_STACK_SIZE * 8, NULL, tskIDLE_PRIORITY + 1, NULL);
    if (xTask != pdPASS) {
        printf("FATAL: WiFi command receive task creation failed!\n");
        //cpxPrintToConsole(LOG_TO_WIFI, "FATAL: WiFi command receive task creation failed!\n");
    }  

    xTask = xTaskCreate(voodoo_task, "voodoo_task", configMINIMAL_STACK_SIZE * 8, NULL, tskIDLE_PRIORITY + 1, NULL);
    if (xTask != pdPASS) {
        printf("FATAL: VOODOO task creation failed!\n");
        //cpxPrintToConsole(LOG_TO_WIFI, "FATAL: VOODOO task creation failed!\n");
    }    

    xTask = xTaskCreate(led_blinky_task, "led_debug_task", configMINIMAL_STACK_SIZE * 2, NULL, tskIDLE_PRIORITY + 1, NULL);
    if (xTask != pdPASS) {
        printf("FATAL: LED debug task creation failed!\n");
        //cpxPrintToConsole(LOG_TO_WIFI, "FATAL: LED debug task creation failed!\n");
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