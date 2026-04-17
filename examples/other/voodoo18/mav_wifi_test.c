#include "pmsis.h"
#include "bsp/bsp.h"
#include "cpx.h"
#include "ardupilotmega/mavlink.h"
#include "wifi.h" // Include the WiFi control header
#include <math.h>
#include <string.h>
#include <stddef.h>
#include <stdarg.h>

#define P2P_STX_ATT_V1 0xA7
#define P2P_STX_MS_V1 0xA8
#define P2P_STX_POS_V1 0xA9
#define P2P_ATT_V1_LEN 24
#define P2P_MS_V1_LEN 17
#define P2P_POS_V1_LEN 24
#define LEADER_SYSTEM_ID 20
#define PEER1_SYSTEM_ID 21
#define PEER2_SYSTEM_ID 22
#define P2P_MAX_CHAN   1   // adjust to however many logical streams
#define SECONDARY_SYSTEM_ID 42
#define SECONDARY_COMPONENT_ID 191
#define GAP8_SYSTEM_ID 255
#define GAP8_COMPONENT_ID 1
#define STM32_SYSTEM_ID 1
#define STM32_COMPONENT_ID 1
#define LED_PIN (2)

#define LAND_ALT 0.02f
#define PI_F 3.14159265358979323846f
#define DEG2RAD_F(x) ((x) * (PI_F / 180.0f))

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
static volatile bool g_param_hov_time = false;
static volatile bool g_param_tkoff_alt = false;
static volatile bool g_param_loops = false;
static volatile bool g_params_received = false;
static volatile bool g_p2p_data_received_once = false;
static volatile bool g_peer_land_requested = false;
static volatile int loop_counter = 0;

// ACK declarations
static SemaphoreHandle_t g_ack_semaphore;
static volatile uint16_t g_last_ack_command = 0;
static volatile uint8_t g_last_ack_result = 0;

typedef struct {
    bool is_armed;
    float position_ned[3]; // North, East, Down in meters
    float velocity_ned[3]; // North, East, Down in meters/second
    float altitude_m;      // Calculated from z position
    float roll;
    float pitch;
    float yaw;
    int mode;
    float flow_comp_m_x;
    float flow_comp_m_y;
    uint8_t quality;    
    uint8_t mission_state;
    uint16_t value;
    // Add other state variables here as needed (e.g., attitude, battery)
} VehicleState_t;

VehicleState_t g_vehicle_state = {0}; // A single, shared structure to hold the vehicle's state

typedef struct {
    float roll;
    float pitch;
    float yaw;
    float x_pos;
    float y_pos;
    float z_pos;
    bool peer_begin;
    TickType_t last_rx_tick;
} PeerState_t;

PeerState_t g_peer_state = {0}; // Holds the last known attitude of the peer

typedef struct {
    float hov_time;
    float tkoff_alt;
    float loops;
} ControllerState_t;

ControllerState_t g_controller_state = {0}; // A single, shared structure to hold the controller's state

// Define the states for your mission
typedef enum {
    MISSION_WAIT_FOR_LOITER,
    MISSION_WAIT_FOR_COMMAND,
    MISSION_WAIT_FOR_ALTHOLD,
    MISSION_ARM,
    MISSION_TAKEOFF,
    MISSION_HOVER,
    MISSION_MOVE_LEFT,
    MISSION_BRAKE_1,
    MISSION_MOVE_BACKWARDS,
    MISSION_BRAKE_2,
    MISSION_MOVE_RIGHT,
    MISSION_BRAKE_3,
    MISSION_MOVE_FORWARD,
    MISSION_BRAKE_4,
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
        case MISSION_WAIT_FOR_ALTHOLD:      return "WAIT_FOR_ALTHOLD"; 
        case MISSION_ARM:                   return "ARM_VEHICLE";
        case MISSION_TAKEOFF:               return "COMMAND_TAKEOFF";
        case MISSION_HOVER:                 return "HOVER";
        case MISSION_MOVE_LEFT:             return "MOVE_LEFT";
        case MISSION_BRAKE_1:               return "BRAKE";
        case MISSION_MOVE_BACKWARDS:        return "MOVE_BACKWARDS";
        case MISSION_BRAKE_2:               return "BRAKE";
        case MISSION_MOVE_RIGHT:            return "MOVE_RIGHT";
        case MISSION_BRAKE_3:               return "BRAKE";
        case MISSION_MOVE_FORWARD:          return "MOVE_FORWARD";
        case MISSION_BRAKE_4:               return "BRAKE";
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
    int16_t  res_0;         // reserved
    int16_t  res_1;         // reserved
    uint8_t  c0;            // Fletcher-8
    uint8_t  c1;            // Fletcher-8
} p2p_att_v1_t;
#pragma pack(pop)

_Static_assert(sizeof(p2p_att_v1_t) == P2P_ATT_V1_LEN, "p2p_att_v1_t must be 24 bytes");

#pragma pack(push, 1)
typedef struct {
    uint8_t  stx;          // 0xA8 for mission-state packet
    uint8_t  peer_id;      // source id
    uint16_t seq;          // sequence number for dedupe
    uint8_t  st;           // mission state/event
    uint16_t val;          // optional value (wp idx, substate, etc.)
    uint32_t time_ms;      // timestamp
    int16_t  res_0;        // reserved
    int16_t  res_1;        // reserved
    uint8_t  c0;           // Fletcher-8
    uint8_t  c1;           // Fletcher-8
} p2p_mstate_v1_t;
#pragma pack(pop)

_Static_assert(sizeof(p2p_mstate_v1_t) == P2P_MS_V1_LEN, "p2p_ms_v1_t must be 17 bytes");

#pragma pack(push, 1)
typedef struct {
    uint8_t  stx;           // 0xA9
    uint8_t  peer_id;
    uint32_t time_boot_ms;
    int16_t  x_pos;         // m
    int16_t  y_pos;         // m
    int16_t  z_pos;         // m
    int16_t  x_vel;         // m/s
    int16_t  y_vel;         // m/s
    int16_t  z_vel;         // m/s
    int16_t  res_0;         // reserved
    int16_t  res_1;         // reserved
    uint8_t  c0;            // Fletcher-8
    uint8_t  c1;            // Fletcher-8
} p2p_pos_v1_t;
#pragma pack(pop)

_Static_assert(sizeof(p2p_pos_v1_t) == P2P_POS_V1_LEN, "p2p_pos_v1_t must be 24 bytes");

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

static p2p_parser_state_t g_p2p_att_state[P2P_MAX_CHAN];
static p2p_parser_state_t g_p2p_ms_state[P2P_MAX_CHAN];
static p2p_parser_state_t g_p2p_pos_state[P2P_MAX_CHAN];

static inline void p2p_parser_reset(uint8_t chan)
{
    g_p2p_att_state[chan].idx = 0;
    g_p2p_ms_state[chan].idx = 0;
    g_p2p_pos_state[chan].idx = 0;
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

    p2p_parser_state_t *st = &g_p2p_att_state[chan];

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
 * Parse one byte of the P2P mission state stream.
 *
 * Returns 1 if a full valid packet was decoded into *out.
 * Returns 0 otherwise.
 *
 * - Resyncs on STX.
 * - Validates Fletcher checksum.
 * - Copies the packet into *out only on success.
 */
uint8_t p2p_ms_parse_char(uint8_t chan, uint8_t c, p2p_mstate_v1_t *out)
{
    if (chan >= P2P_MAX_CHAN || out == NULL) {
        return 0;
    }

    p2p_parser_state_t *st = &g_p2p_ms_state[chan];

    // If we're idle, wait for STX
    if (st->idx == 0) {
        if (c != P2P_STX_MS_V1) {
            return 0;
        }
        st->buf[st->idx++] = c;
        return 0;
    }

    // If we see STX unexpectedly mid-packet, treat as resync (start new packet)
    if (c == P2P_STX_MS_V1) {
        st->buf[0] = c;
        st->idx = 1;
        return 0;
    }

    st->buf[st->idx++] = c;

    if (st->idx < P2P_MS_V1_LEN) {
        return 0; // still accumulating
    }

    // We have 20 bytes: validate checksum
    bool ok = p2p_fletcher8_ok(st->buf, P2P_MS_V1_LEN);

    if (!ok) {
        st->parse_errors++;

        // Try to resync: if last byte could be STX, restart; else reset.
        // (We already handled "c == STX" above, so just reset.)
        st->idx = 0;
        return 0;
    }

    // Success
    memcpy(out, st->buf, P2P_MS_V1_LEN);
    st->packets_ok++;

    // Ready for next packet
    st->idx = 0;
    return 1;
}

/**
 * Parse one byte of the P2P position stream.
 *
 * Returns 1 if a full valid packet was decoded into *out.
 * Returns 0 otherwise.
 */
uint8_t p2p_pos_parse_char(uint8_t chan, uint8_t c, p2p_pos_v1_t *out)
{
    if (chan >= P2P_MAX_CHAN || out == NULL) {
        return 0;
    }

    p2p_parser_state_t *st = &g_p2p_pos_state[chan];

    if (st->idx == 0) {
        if (c != P2P_STX_POS_V1) {
            return 0;
        }
        st->buf[st->idx++] = c;
        return 0;
    }

    if (c == P2P_STX_POS_V1) {
        st->buf[0] = c;
        st->idx = 1;
        return 0;
    }

    st->buf[st->idx++] = c;

    if (st->idx < P2P_POS_V1_LEN) {
        return 0;
    }

    bool ok = p2p_fletcher8_ok(st->buf, P2P_POS_V1_LEN);

    if (!ok) {
        st->parse_errors++;
        st->idx = 0;
        return 0;
    }

    memcpy(out, st->buf, P2P_POS_V1_LEN);
    st->packets_ok++;
    st->idx = 0;
    return 1;
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

    hb.type = MAV_TYPE_GCS;
    hb.autopilot = MAV_AUTOPILOT_ARDUPILOTMEGA;
    hb.base_mode = 0;
    hb.custom_mode = 0;
    hb.system_status = MAV_STATE_ACTIVE;

    mavlink_msg_heartbeat_encode(SECONDARY_SYSTEM_ID, SECONDARY_COMPONENT_ID, &msg, &hb);
    send_mavlink_message(&msg);
    // printf("-> Sent HEARTBEAT to host.\n");
}

/**
 * @brief Creates and sends a request to read a parameter from the drone.
 */
void request_parameter(const char *param_id) {
    mavlink_message_t msg;
    char id[MAVLINK_MSG_PARAM_REQUEST_READ_FIELD_PARAM_ID_LEN] = {0};
    strncpy(id, param_id, MAVLINK_MSG_PARAM_REQUEST_READ_FIELD_PARAM_ID_LEN);    
    
    // Target the drone's system and component IDs. Using 1, 1 is standard for the first flight controller.
    uint8_t target_system = 1;
    uint8_t target_component = 1;

    mavlink_msg_param_request_read_pack(SECONDARY_SYSTEM_ID, SECONDARY_COMPONENT_ID, &msg, target_system, target_component, id, -1);
    //mavlink_msg_param_request_read_pack_chan(GAP8_SYSTEM_ID, GAP8_COMPONENT_ID, MAVLINK_COMM_0, &msg, target_system, target_component, id, -1);
    send_mavlink_message(&msg);
    printf("-> Sent PARAM_REQUEST_READ for %s.\n", param_id);
}

/**
 * @brief Creates and sends a request for the entire parameter list from the drone.
 */
void request_parameter_list() {
    mavlink_message_t msg;
    
    // Target the drone's system and component IDs. Using 1, 1 is standard for the first flight controller.
    uint8_t target_system = 1;
    uint8_t target_component = 1;

    mavlink_msg_param_request_list_pack(SECONDARY_SYSTEM_ID, SECONDARY_COMPONENT_ID, &msg, target_system, target_component);
    send_mavlink_message(&msg);
    printf("-> Sent PARAM_REQUEST_LIST\n");
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
    mavlink_request_data_stream(MAVLINK_MSG_ID_LOCAL_POSITION_NED, 10.0f);

    // You can add requests for other streams here if needed, for example:
    mavlink_request_data_stream(MAVLINK_MSG_ID_ATTITUDE, 10.0f);

    // Request DISTANCE_SENSOR at 10 Hz for altitude
    mavlink_request_data_stream(MAVLINK_MSG_ID_RANGEFINDER, 10.0f);

    // Request AHRS2 at 10 Hz for attitude data
    //mavlink_request_data_stream(MAVLINK_MSG_ID_AHRS2, 3.0f);    
}

void send_statustextf(uint8_t severity, const char *fmt, ...)
{
    mavlink_message_t msg;
    char text[50];

    va_list ap;
    va_start(ap, fmt);
    vsnprintf(text, sizeof(text), fmt, ap);
    va_end(ap);

    mavlink_msg_statustext_pack(
        SECONDARY_SYSTEM_ID,
        SECONDARY_COMPONENT_ID,
        &msg,
        severity,
        text,
        0, 0
    );

    send_mavlink_message(&msg);
}

static inline bool all_params_ready(void)
{
    return g_param_hov_time && g_param_tkoff_alt && g_param_loops;
}


// Tasks to periodically request updates from ArduPilot
void request_task(void *parameters) {
    static uint32_t param_ticks = 0;
    
    // “sent notification already” latches
    static bool n_hov = false, n_tkoff = false, n_loops = false;
    static bool requested_list_once = false;
    static uint16_t sequence = 0;
    static uint16_t value = 0;
    static uint16_t res0 = 0;
    static uint16_t res1 = 0;
    
    while(1) {
        // Send Heartbeats so FC knows you're awake
        send_heartbeat();

        // Send Mission State Updates
        send_statustextf(MAV_SEVERITY_ALERT, "MS1,st=%u,val=%u,seq=%u,res0=%u,res1=%u", g_vehicle_state.mission_state, value, sequence, res0, res1);

        // Make sure FC is connected first
        if (g_fc_connected) {
            if (!requested_list_once) {
                request_parameter_list();
                requested_list_once = true;
            }
            
            // 2) If not all ready, request missing ones
            //    Do this every ~1–2 seconds, but ONE request per tick to avoid bursts.
            if (!g_params_received && (param_ticks % 2 == 0)) {

                if (!g_param_loops) {
                    request_parameter("CF_LOOPS");
                } else if (!g_param_hov_time) {
                    request_parameter("CF_HOV_TIME");
                } else if (!g_param_tkoff_alt) {
                    request_parameter("PILOT_TKOFF_ALT");
                }
            }            

            // 3) Send “got param” STATUSTEXT only once per param
            if (g_param_hov_time && !n_hov) {
                send_statustextf(MAV_SEVERITY_ALERT, "AI: got HOV TIME=%.3f", (double)g_controller_state.hov_time);
                n_hov = true;
            }  
            if (g_param_tkoff_alt && !n_tkoff) {
                send_statustextf(MAV_SEVERITY_ALERT, "AI: got TKOFF ALT=%.3f", (double)g_controller_state.tkoff_alt);
                n_tkoff = true;
            }  
            if (g_param_loops && !n_loops) {
                send_statustextf(MAV_SEVERITY_ALERT, "AI: got LOOPS=%.3f", (double)g_controller_state.loops);
                n_loops = true;
            }                                         
            
            // 4) When all ready, latch
            if (!g_params_received && all_params_ready()) {
                g_params_received = true;
                send_statustextf(MAV_SEVERITY_ALERT, "AI: All Params Received.");
                printf("All params received\n");
            }            
        }
        param_ticks++;

        sequence++;

        vTaskDelay(1000 / portTICK_PERIOD_MS); // Loop 1Hz
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
    p2p_mstate_v1_t pkt2;
    p2p_pos_v1_t pkt3;

    // --- Buffer for re-serializing the validated MAVLink message ---
    uint8_t mavlink_tx_buffer[MAVLINK_MAX_PACKET_LEN];

    while(1) {
        pi_uart_read(&uart_device, &rx_byte, 1);

        if (mavlink_parse_char(MAVLINK_COMM_0, rx_byte, &received_msg, &status)) {
                // --- 1. Validation Passed: A complete message was received ---

                bool forward_to_wifi = true;    // Assume we will forward the message to WiFi by default

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
                        }    
                        break;
                    }
                    case MAVLINK_MSG_ID_STATUSTEXT: {
                        mavlink_statustext_t status;
                        mavlink_msg_statustext_decode(&received_msg, &status);
                        break;
                    }                
                    case MAVLINK_MSG_ID_ATTITUDE: {
                        if (!g_ahrs_stream_active) {
                            g_ahrs_stream_active = true;
                        }

                        static int att_msg_count = 0;
                        att_msg_count++;

                        mavlink_attitude_t att;
                        mavlink_msg_attitude_decode(&received_msg, &att);
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
    
                        char name[17];
                        memcpy(name, param.param_id, 16);
                        name[16] = '\0';                    
                        
                        // Do not print here to avoid missing params

                        if (strcmp(name, "CF_HOV_TIME") == 0) {
                            g_controller_state.hov_time = param.param_value;
                            g_param_hov_time = true;
                        }
                        else if (strcmp(name, "PILOT_TKOFF_ALT") == 0) {
                            g_controller_state.tkoff_alt = (param.param_value / 100.0f);    // convert to m
                            g_param_tkoff_alt = true;
                        }
                        else if (strcmp(name, "CF_LOOPS") == 0) {
                            g_controller_state.loops = param.param_value;
                            g_param_loops = true;
                        }                                                      
                        break;
                    }
                    case MAVLINK_MSG_ID_LOCAL_POSITION_NED: {
                        if (!g_pos_stream_active) {
                            g_pos_stream_active = true;
                        }                    
                        
                        // Keep track of how many position messages we've received
                        static int pos_msg_count = 0;
                        pos_msg_count++;
                        if (pos_msg_count % 5 != 0) {
                            forward_to_wifi = false; // Drop 4 out of 5 messages for WiFi
                        }

                        // We received a position, decode it
                        mavlink_local_position_ned_t pos;
                        mavlink_msg_local_position_ned_decode(&received_msg, &pos);
                        g_vehicle_state.position_ned[0] = pos.x; // North
                        g_vehicle_state.position_ned[1] = pos.y; // East
                        g_vehicle_state.position_ned[2] = pos.z; // Down
                        g_vehicle_state.velocity_ned[0] = pos.vx; // North
                        g_vehicle_state.velocity_ned[1] = pos.vy; // East
                        g_vehicle_state.velocity_ned[2] = pos.vz; // Down
                        
                        break;
                    }
                    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
                        static int g_pos_msg_count = 0;
                        g_pos_msg_count++;
                        if (g_pos_msg_count % 5 != 0) {
                            forward_to_wifi = false;    // Drop 4 out of 5 messages for WiFi
                        }
                        break;
                    }
                    case MAVLINK_MSG_ID_RANGEFINDER: {
                        if (!g_range_stream_active) {
                            g_range_stream_active = true;
                        }

                        static int range_msg_count = 0;
                        range_msg_count++;  
                        if (range_msg_count % 2 != 0) {
                            forward_to_wifi = false; // Drop 1 out of 2 messages for WiFi
                        }

                        mavlink_rangefinder_t range;
                        mavlink_msg_rangefinder_decode(&received_msg, &range);
                        // Update altitude from the rangefinder
                        g_vehicle_state.altitude_m = range.distance;
   
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
                        break;
                    case MAVLINK_MSG_ID_COMMAND_ACK: {
                        mavlink_command_ack_t ack;
                        mavlink_msg_command_ack_decode(&received_msg, &ack);

                        // Use a harmless command as "GO" trigger
                        if (ack.command == 42428) { // MAV_CMD_DO_SEND_BANNER
                            if (ack.result == MAV_RESULT_ACCEPTED) {
                                printf("MISSION GO! (COMMAND RECEIVED)\n");
                                send_statustextf(MAV_SEVERITY_ALERT, "MISSION GO! (COMMAND RECEIVED)");
                                is_mission_start = true;
                                if (loop_counter >= 1) {
                                    is_mission_reset = true;
                                }
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

            // Only send over WiFi if the flag is still true
            if (forward_to_wifi) {
                // Re-serialize the message into a raw byte buffer ---
                uint16_t len = mavlink_msg_to_send_buffer(mavlink_tx_buffer, &received_msg);
                // Forward the raw buffer over WiFi using our safe function ---            
                cpxSendRawData(CPX_T_WIFI_HOST, CPX_F_CONSOLE, mavlink_tx_buffer, len);
            }                
        }         
        if (p2p_att_parse_char(0, rx_byte, &pkt)) {
            if (pkt.peer_id == LEADER_SYSTEM_ID) {
                if (!g_p2p_data_received_once) {
                    g_p2p_data_received_once = true;
                    printf("Peer stream active [%d]", pkt.peer_id);
                    send_statustextf(MAV_SEVERITY_ALERT, "Peer Stream Active.");
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
                    // printf("<- P2P ATTITUDE: Roll=%.2f Pitch=%.2f Yaw=%.2f\n", g_peer_state.roll, g_peer_state.pitch, g_peer_state.yaw);
                    // cpxPrintToConsole(LOG_TO_WIFI, "<- P2P ATTITUDE: Roll=%.2f Pitch=%.2f Yaw=%.2f\n", g_peer_state.roll, g_peer_state.pitch, g_peer_state.yaw);
                }                                 
            }
        }  
        if (p2p_ms_parse_char(0, rx_byte, &pkt2)) {
            if ((pkt2.peer_id == PEER1_SYSTEM_ID) || (pkt2.peer_id == PEER2_SYSTEM_ID)) {                
                if ((pkt2.st >= 1) && !is_mission_start) {
                    printf("MISSION GO! (FOLLOWING)\n");
                    send_statustextf(MAV_SEVERITY_ALERT, "MISSION GO! (FOLLOWING)");                    
                    is_mission_start = true;
                }
                if (pkt2.st == 6) {
                    printf("Starting Voodoo!\n");
                    send_statustextf(MAV_SEVERITY_ALERT, "Starting Voodoo!");
                }
                if (pkt2.st >= 7) {
                    g_peer_land_requested = true;
                }
            }
        }
        if (p2p_pos_parse_char(0, rx_byte, &pkt3)) {
            if (pkt3.peer_id == LEADER_SYSTEM_ID) {
                g_peer_state.x_pos = pkt3.x_pos;
                g_peer_state.y_pos = pkt3.y_pos;
                g_peer_state.z_pos = pkt3.z_pos;
                g_peer_state.last_rx_tick = xTaskGetTickCount();
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
    // printf("-> Commanding takeoff to %.1f meters.\n", altitude_m);
    //cpxPrintToConsole(LOG_TO_WIFI, "-> Commanding takeoff to %.1f meters.\n", altitude_m);
    mavlink_send_command_long(MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, altitude_m);
}

// Wrapper for MAV_CMD_DO_SET_MODE
void mavlink_set_mode(uint8_t custom_mode) {
    // printf("-> Setting mode to %d.\n", custom_mode);
    //cpxPrintToConsole(LOG_TO_WIFI, "-> Setting mode to %d.\n", custom_mode);
    mavlink_send_command_long(MAV_CMD_DO_SET_MODE, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, (float)custom_mode, 0, 0, 0, 0, 0);
}

// Wrapper for component arm
void mavlink_arm_vehicle() {
    // printf("-> Sending ARM command.\n");
    //cpxPrintToConsole(LOG_TO_WIFI, "-> Sending ARM command.\n");
    mavlink_send_command_long(MAV_CMD_COMPONENT_ARM_DISARM, 1.0f, 0, 0, 0, 0, 0, 0);
}

void mavlink_disarm_vehicle() {
    // printf("-> Sending DISARM command.\n");
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
    // printf("-> Setting local NED target to N:%.1f, E:%.1f, D:%.1f\n", x_north, y_east, z_down);
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
 * @param yaw       The desired yaw angle in radians.
 * @param thrust    The collective thrust from 0.0 (idle) to 1.0 (full). 0.5 is hover.
 */
void mavlink_set_attitude_target(float roll, float pitch, float yaw, float thrust) {
    mavlink_message_t msg;

    // This bitmask tells the flight controller to use the quaternion for attitude ignoring rates
    uint8_t type_mask = 0x07; // ignore body roll/pitch/yaw rates

    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);

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
        0,                 // body_yaw_rate (ignored by mask)
        thrust,            // thrust (ignored by mask)
        thrust_body        // The missing argument
    );

    send_mavlink_message(&msg);
}

/**
 * @brief Commands the vehicle to enter Return-To-Launch (RTL) mode.
 */
void mavlink_rtl(void) {
    //printf("-> Commanding RTL.\n");
    //cpxPrintToConsole(LOG_TO_WIFI, "-> Commanding RTL.\n");
    mavlink_set_mode(COPTER_MODE_RTL);
}

void mavlink_command_land() {
    //printf("-> Commanding LAND.\n");
    //cpxPrintToConsole(LOG_TO_WIFI, "-> Commanding LAND.\n");
    mavlink_send_command_long(MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0);
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

/**
 * @brief Send float to ArduPilot to log.
 * @param name      Variable name
 * @param v         Value
 */
void mav_send_named_value_float(const char *name, float v)
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
    send_mavlink_message(&msg);
}

// ArduPilot doesn't log in dataflash -- Dont use
void mav_send_debug_vect(const char *name, float x, float y, float z)
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
    send_mavlink_message(&msg);
}

// The mission task function
void autonomous_mission_task(void *parameters) {
    VehicleState_t *state = (VehicleState_t *)parameters;

    // --- State-specific counters ---
    const int MISSION_HZ = 50;
    const int MISSION_DT_MS = (1000 / MISSION_HZ);
    const int ARM_TIME_MS = 3000;
    const int ARM_TICKS = (ARM_TIME_MS / MISSION_DT_MS);
    const int TAKEOFF_TIME_MS = 5000;
    const int TAKEOFF_TICKS = (TAKEOFF_TIME_MS / MISSION_DT_MS);
    const int LOITER_TIME_MS = 2000;
    const int LOITER_TICKS = (LOITER_TIME_MS / MISSION_DT_MS);
    const int MOVE_TIME_MS = 1000;
    const int MOVE_TICKS = (MOVE_TIME_MS / MISSION_DT_MS);
    const int BRAKE_TIME_MS = 1500;
    const int BRAKE_TICKS = (BRAKE_TIME_MS / MISSION_DT_MS);
    const int LAND_TIME_MS = 4000;
    const int LAND_TICKS = (LAND_TIME_MS / MISSION_DT_MS);       
    const int FLOW_BAD_TIME_MS = 500;
    const int FLOW_BAD_TICKS = (FLOW_BAD_TIME_MS / MISSION_DT_MS);       
    const int THR_MIN = 1000;
    const int THR_HOLD = 1550;
    const int THR_TKOFF = 1700;
    const float V_BAD_MPS = 1.5f;
    const float V_BAD2 = (V_BAD_MPS * V_BAD_MPS);
    const uint8_t Q_BAD = 10; 
    static int althold_ticks = 0;   
    static int arm_ticks = 0;   
    static int takeoff_ticks = 0;
    static int loiter_ticks = 0;
    static int hover_ticks = 0;
    static int move_ticks = 0;
    static int brake_ticks = 0;
    static int land_ticks = 0;
    static int flow_bad_ticks = 0;
    static int qual_bad_ticks = 0;    
    static float tkoff_alt = 1.0f;
    static float hov_time = 6.0f;   // Default
    static int guid_loops = 1;      // Default
    static int disarm_ticks = 0;
    static int move_counter = 0;

    g_vehicle_state.mission_state = 0;

    printf("Mission task started. Waiting for start command via WiFi...\n");
    send_statustextf(MAV_SEVERITY_ALERT, "Mission task started. Waiting for start command via WiFi...");
    //cpxPrintToConsole(LOG_TO_WIFI, "Mission task started. Waiting for start command via WiFi...\n");

    while (1) {
        if (is_mission_terminate) {
            //cpxPrintToConsole(LOG_TO_WIFI, "Mission terminated by user. Disarming...\n");
            mavlink_disarm_vehicle();
            is_mission_terminate = false;
            g_mission_state = MISSION_DONE;
        }
        if (g_peer_land_requested) {
            g_peer_land_requested = false;
            g_vehicle_state.mission_state = 14;
            g_mission_state = MISSION_COMMAND_LAND;
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

        if (flow_lost && /* only during maneuver */ 
            (g_mission_state == MISSION_MOVE_LEFT ||
            g_mission_state == MISSION_MOVE_BACKWARDS ||
            g_mission_state == MISSION_MOVE_RIGHT ||
            g_mission_state == MISSION_MOVE_FORWARD ||
            g_mission_state == MISSION_BRAKE_1 ||
            g_mission_state == MISSION_BRAKE_2 ||
            g_mission_state == MISSION_BRAKE_3 ||
            g_mission_state == MISSION_BRAKE_4 ||
            g_mission_state == MISSION_HOVER)) {

            //printf("FLOW LOST: v2=%.2f (m/s)^2 qual=%u -> LAND\n", v2, g_vehicle_state.quality);

            g_vehicle_state.mission_state = 14;

            g_mission_state = MISSION_COMMAND_LAND;
        }
        // --- SAFETY --- //

        switch (g_mission_state) {   
            case MISSION_WAIT_FOR_COMMAND:
                if (is_mission_start && g_params_received) {
                    g_vehicle_state.mission_state = 1;
                    tkoff_alt = g_controller_state.tkoff_alt;
                    hov_time = g_controller_state.hov_time;    
                    guid_loops = g_controller_state.loops;    
                    is_mission_start = false; // Clear the flag
                    //printf("Mission: Starting Mission!\n");
                    send_statustextf(MAV_SEVERITY_ALERT, "Mission: Starting Mission!");
                    mavlink_arm_vehicle();
                    mavlink_send_rc_override_4ch(STM32_SYSTEM_ID, STM32_COMPONENT_ID, 1500, 1500, THR_MIN, 1500);     
                    g_mission_state = MISSION_ARM;
                }         
                break;         
            case MISSION_ARM:
                if (arm_ticks >= ARM_TICKS) {
                    g_vehicle_state.mission_state = 2;

                    arm_ticks = 0;
                    //printf("Mission: Drone Armed. Taking off...\n");

                    mavlink_send_rc_override_4ch(STM32_SYSTEM_ID, STM32_COMPONENT_ID, 1500, 1500, THR_MIN, 1500);

                    takeoff_ticks = 0;
                    
                    mavlink_set_mode(COPTER_MODE_ALT_HOLD);

                    g_mission_state = MISSION_WAIT_FOR_ALTHOLD;
                }
                else {
                    mavlink_send_rc_override_4ch(STM32_SYSTEM_ID, STM32_COMPONENT_ID, 1500, 1500, THR_MIN, 1500);
                    arm_ticks++;
                }
                break;
            case MISSION_WAIT_FOR_ALTHOLD:
                if (g_vehicle_state.mode == COPTER_MODE_ALT_HOLD) {
                    //printf("Mission: ALTHOLD Good.\n");
                    althold_ticks = 0;

                    g_vehicle_state.mission_state = 3;

                    mavlink_send_rc_override_4ch(STM32_SYSTEM_ID, STM32_COMPONENT_ID, 1500, 1500, THR_MIN, 1500);                    
                    g_mission_state = MISSION_TAKEOFF;
                    break;
                }
                if (althold_ticks < LOITER_TICKS) {
                    althold_ticks++;
                }
                else {
                    //printf("Mission: Althold Failed.\n");
                    althold_ticks = 0;

                    g_vehicle_state.mission_state = 10;

                    g_mission_state = MISSION_DONE;
                }
                break;                            
            case MISSION_TAKEOFF: {
                // 1) Timeout
                if ((g_vehicle_state.altitude_m > 1.1f * tkoff_alt) || (takeoff_ticks++ > TAKEOFF_TICKS)) {
                    //printf("Mission: Takeoff complete -> Try LOITER.\n");

                    g_vehicle_state.mission_state = 4;

                    mavlink_send_rc_override_4ch(STM32_SYSTEM_ID, STM32_COMPONENT_ID, 1500, 1500, THR_HOLD, 1500);

                    mavlink_set_mode(COPTER_MODE_LOITER);

                    g_mission_state = MISSION_WAIT_FOR_LOITER;
                    break;                    
                }

                // Keep roll/pitch/yaw centered during takeoff
                mavlink_send_rc_override_4ch(STM32_SYSTEM_ID, STM32_COMPONENT_ID, 1500, 1500, THR_TKOFF, 1500);

                break;
            }
            case MISSION_WAIT_FOR_LOITER:
                if (g_vehicle_state.mode == COPTER_MODE_LOITER) {
                    //printf("Mission: LOITER Good.\n");
                    loiter_ticks = 0;

                    g_vehicle_state.mission_state = 5;

                    g_mission_state = MISSION_HOVER;
                    break;
                }
                if (loiter_ticks < LOITER_TICKS) {
                    loiter_ticks++;
                    mavlink_set_mode(COPTER_MODE_LOITER);   // keep sending so ArduPilot accepts
                }
                else {
                    //printf("Mission: LOITER Failed. Landing...\n");
                    loiter_ticks = 0;

                    g_vehicle_state.mission_state = 7;

                    g_mission_state = MISSION_COMMAND_LAND;
                }
                break;
            case MISSION_HOVER:
                if (hover_ticks >= (hov_time * 1000.0f / MISSION_DT_MS)) {
                    hover_ticks = 0;
                    move_ticks = 0;
                    land_ticks = 0;
                    flow_bad_ticks = 0;
                    qual_bad_ticks = 0;  
                    
                    g_vehicle_state.mission_state = 6;

                    g_mission_state = MISSION_MOVE_LEFT;
                } else {
                    hover_ticks++;
                }
                break;
            case MISSION_MOVE_LEFT:
                // Strafe left slowly: y negative
                mavlink_send_rc_override_4ch(STM32_SYSTEM_ID, STM32_COMPONENT_ID, 1300, 1500, 1500, 1500);

                if (move_ticks >= MOVE_TICKS) {
                    move_ticks = 0;
                    brake_ticks = 0;
                    g_vehicle_state.mission_state = 7;
                    g_mission_state = MISSION_BRAKE_1;
                }
                else {
                    move_ticks++;
                }
                break;           
            case MISSION_BRAKE_1:
                if (brake_ticks >= BRAKE_TICKS) {
                    brake_ticks = 0;
                    move_ticks = 0;
                    g_vehicle_state.mission_state = 8;
                    g_mission_state = MISSION_MOVE_BACKWARDS;
                } else {
                    mavlink_send_rc_override_4ch(STM32_SYSTEM_ID, STM32_COMPONENT_ID, 1500, 1500, 1500, 1500); // reset
                    brake_ticks++;
                }
                break;              
            case MISSION_MOVE_BACKWARDS:
                // Strafe backwards slowly: x positive
                mavlink_send_rc_override_4ch(STM32_SYSTEM_ID, STM32_COMPONENT_ID, 1500, 1700, 1500, 1500);

                if (move_ticks >= MOVE_TICKS) {
                    move_ticks = 0;
                    brake_ticks = 0;
                    g_vehicle_state.mission_state = 9;
                    g_mission_state = MISSION_BRAKE_2;
                }
                else {
                    move_ticks++;
                }
                break;                  
            case MISSION_BRAKE_2:
                if (brake_ticks >= BRAKE_TICKS) {
                    brake_ticks = 0;
                    move_ticks = 0;
                    g_vehicle_state.mission_state = 10;
                    g_mission_state = MISSION_MOVE_RIGHT;
                } else {
                    mavlink_send_rc_override_4ch(STM32_SYSTEM_ID, STM32_COMPONENT_ID, 1500, 1500, 1500, 1500); // reset
                    brake_ticks++;
                }
                break;        
            case MISSION_MOVE_RIGHT:
                // Strafe right slowly: y positive
                mavlink_send_rc_override_4ch(STM32_SYSTEM_ID, STM32_COMPONENT_ID, 1700, 1500, 1500, 1500);

                if (move_ticks >= MOVE_TICKS) {
                    move_ticks = 0;
                    brake_ticks = 0;
                    g_vehicle_state.mission_state = 11;
                    g_mission_state = MISSION_BRAKE_3;
                }
                else {
                    move_ticks++;
                }
                break;          
            case MISSION_BRAKE_3:
                if (brake_ticks >= BRAKE_TICKS) {
                    brake_ticks = 0;
                    move_ticks = 0;
                    g_vehicle_state.mission_state = 12;
                    g_mission_state = MISSION_MOVE_FORWARD;
                } else {
                    mavlink_send_rc_override_4ch(STM32_SYSTEM_ID, STM32_COMPONENT_ID, 1500, 1500, 1500, 1500); // reset
                    brake_ticks++;
                }
                break;       
            case MISSION_MOVE_FORWARD:
                // Strafe right slowly: x negative
                mavlink_send_rc_override_4ch(STM32_SYSTEM_ID, STM32_COMPONENT_ID, 1500, 1300, 1500, 1500);

                if (move_ticks >= MOVE_TICKS) {
                    move_ticks = 0;
                    brake_ticks = 0;
                    g_vehicle_state.mission_state = 13;
                    g_mission_state = MISSION_BRAKE_4;
                }
                else {
                    move_ticks++;
                }
                break;    
            case MISSION_BRAKE_4:
                if (brake_ticks >= BRAKE_TICKS) {
                    brake_ticks = 0;
                    move_ticks = 0;
                    if (move_counter >= guid_loops) {
                        move_counter = 0;
                        g_vehicle_state.mission_state = 14;
                        g_mission_state = MISSION_COMMAND_LAND;
                    }
                    else {
                        move_counter++;
                        g_vehicle_state.mission_state = 6;
                        g_mission_state = MISSION_MOVE_LEFT;
                    }                    
                } else {
                    mavlink_send_rc_override_4ch(STM32_SYSTEM_ID, STM32_COMPONENT_ID, 1500, 1500, 1500, 1500); // reset
                    brake_ticks++;
                }
                break;                                                                                                              
            case MISSION_COMMAND_LAND: {
                mavlink_command_land();

                land_ticks = 0;

                g_vehicle_state.mission_state = 15;

                loop_counter++;     // Count loops here as this case does not repeat

                g_mission_state = MISSION_WAIT_FOR_LAND;

                break;
            }
            case MISSION_WAIT_FOR_LAND:
                if ((g_vehicle_state.altitude_m < 2.0f * LAND_ALT) || (land_ticks >= LAND_TICKS)) {
                    land_ticks = 0;
                    disarm_ticks = 0;

                    g_vehicle_state.mission_state = 16;

                    //printf("Mission: Land Done. Disarming...\n");
                    g_mission_state = MISSION_DISARM;
                } else {
                    land_ticks++;
                }
                break;
            case MISSION_DISARM:
                if (!g_vehicle_state.is_armed) {

                    g_vehicle_state.mission_state = 10;

                    disarm_ticks = 0;

                    send_statustextf(MAV_SEVERITY_ALERT, "Mission: Done!");

                    g_mission_state = MISSION_DONE;
                } 
                if (disarm_ticks >= ARM_TICKS) {
                    mavlink_disarm_vehicle();
                } else {
                    disarm_ticks++;
                }
                break;                              
            case MISSION_DONE:
                if (g_vehicle_state.mode == COPTER_MODE_STABILIZE) {
                    //printf("Mission state is being reset.\n");
                    send_statustextf(MAV_SEVERITY_ALERT, "Mission state is being reset."); 

                    if (is_mission_reset) {
                        is_mission_reset = false; // <-- IMPORTANT: Clear the reset flag
                        g_vehicle_state.mission_state = 0;
                        // Go back to the very first state
                        g_mission_state = MISSION_WAIT_FOR_COMMAND;                        
                    } else {
                        is_mission_start = false; // <-- IMPORTANT: Clear the start flag
                    }
                }
                break;                            
        }
        vTaskDelay(MISSION_DT_MS / portTICK_PERIOD_MS); // Loop 50Hz
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
    xTask = xTaskCreate(autonomous_mission_task, "mission_task", configMINIMAL_STACK_SIZE * 12, &g_vehicle_state, tskIDLE_PRIORITY + 1, NULL);
    if (xTask != pdPASS) {
        printf("FATAL: Mission task creation failed!\n");
        cpxPrintToConsole(LOG_TO_WIFI, "FATAL: Mission task creation failed!\n");
    }      

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

    xTask = xTaskCreate(request_task, "request_task", configMINIMAL_STACK_SIZE * 4, NULL, tskIDLE_PRIORITY + 1, NULL);
    if (xTask != pdPASS) {
        printf("FATAL: Request task creation failed!\n");
        cpxPrintToConsole(LOG_TO_WIFI, "FATAL: Request task creation failed!\n");
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