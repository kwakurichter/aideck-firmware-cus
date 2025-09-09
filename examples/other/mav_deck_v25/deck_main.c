#include "pmsis.h"
#include "bsp/bsp.h"
#include "cpx.h"
#include "wifi.h"
#include "ardupilotmega/mavlink.h"
#include <stdbool.h>

#define GAP8_SYSTEM_ID 1
#define GAP8_COMPONENT_ID 1
#define LED_PIN (2)

#define COPTER_MODE_GUIDED 4
#define COPTER_MODE_RTL 6
#define TAKEOFF_ALT 1.0f
#define FORWARD_DIST 3.0f

static volatile bool g_start_mission_commanded = false;

typedef struct {
    bool is_armed;
    float position_ned[3]; // North, East, Down in meters
    float altitude_m;      // Calculated from z position
    // Add other state variables here as needed (e.g., attitude, battery)
} VehicleState_t;

// --- Global Variables ---
static pi_device_t led_device;
static CPXPacket_t txp; // Packet for sending CPX commands
static CPXPacket_t rxp; // Packet for receiving CPX commands
struct pi_device uart_device; // UART device handle

uint8_t send_buffer[MAVLINK_MAX_PACKET_LEN];
VehicleState_t g_vehicle_state = {0}; // A single, shared structure to hold the vehicle's state

// Forward declarations for tasks that live in this file
void led_blinky_task(void *parameters);
void wifi_command_receive_task(void *parameters);
void setupWiFi(void);
void wifi_debug_task(void *parameters);
void uart_receive_task(void *parameters);
void uart_transmit_task(void *parameters);
void mavlink_init_data_streams(void);

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

void autonomous_mission_task(void *parameters);

bool mission_is_start_commanded(void);

void mission_signal_start(void);


/**
 * @brief A simple task to periodically send a heartbeat.
 */
void heartbeat_task(void *parameters) {
    while (1) {
        send_heartbeat();
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Send every 1 second
    }
}

void start_main_application(void *parameters) {
    // 1. Initialize core hardware and protocols
    cpxInit();

    #ifdef SETUP_WIFI_AP
        setupWiFi();
    #endif

    // 1. Configure and open the UART
    struct pi_uart_conf conf;
    pi_uart_conf_init(&conf);
    conf.enable_tx = 1;
    conf.enable_rx = 1;
    conf.baudrate_bps = 115200;
    pi_open_from_conf(&uart_device, &conf);
    if (pi_uart_open(&uart_device)) {
        printf("FATAL: UART open failed!\n");
        pmsis_exit(-1);
    }
    printf("UART opened successfully.\n");    

    cpxEnableFunction(CPX_F_WIFI_CTRL); // Enable the CPX function for WiFi control commands

    // 3. Start the application-specific tasks
    BaseType_t xTask;

    // MAVLink Tasks
    xTask = xTaskCreate(uart_receive_task, "mav_rx_task", configMINIMAL_STACK_SIZE * 8, NULL, tskIDLE_PRIORITY + 1, NULL);
    if (xTask != pdPASS) {
        cpxPrintToConsole(LOG_TO_WIFI, "FATAL: MAVLink UART receive task creation failed!\n");
    }

    // Application-specific Tasks 
    xTask = xTaskCreate(led_blinky_task, "led_task", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
    if (xTask != pdPASS) {
        cpxPrintToConsole(LOG_TO_WIFI, "FATAL: LED task creation failed!\n");
    }
    //xTask = xTaskCreate(wifi_command_receive_task, "wifi_cmd_task", configMINIMAL_STACK_SIZE * 2, NULL, tskIDLE_PRIORITY + 1, NULL);
    //if (xTask != pdPASS) {
    //    cpxPrintToConsole(LOG_TO_WIFI, "FATAL: WiFi Command task creation failed!\n");
    //}
    xTask = xTaskCreate(heartbeat_task, "heartbeat_task", configMINIMAL_STACK_SIZE * 2, NULL, tskIDLE_PRIORITY + 1, NULL);
    if (xTask != pdPASS) {
        cpxPrintToConsole(LOG_TO_WIFI, "FATAL: Heartbeat task creation failed!\n");
    }
    xTask = xTaskCreate(wifi_debug_task, "wifi_debug_task", configMINIMAL_STACK_SIZE * 2, NULL, tskIDLE_PRIORITY + 1, NULL);
    if (xTask != pdPASS) {
        cpxPrintToConsole(LOG_TO_WIFI, "FATAL: WiFi Debug task creation failed!\n");
    }   
    
    // Mission Task
    //xTask = xTaskCreate(autonomous_mission_task, "mission_task", configMINIMAL_STACK_SIZE * 4, &g_vehicle_state, tskIDLE_PRIORITY + 1, NULL);
    //if (xTask != pdPASS) {
    //    cpxPrintToConsole(LOG_TO_WIFI, "FATAL: Mission task creation failed!\n");
    //}

    // 4. Request MAVLink streams
    //mavlink_init_data_streams();
    
    // 4. Yield forever, allowing the created tasks to run.
    while(1) {
        pi_yield();
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
 * @brief Task to send periodic debug messages over WiFi.
 */
void wifi_debug_task(void *parameters) {
    while(1) {
        cpxPrintToConsole(LOG_TO_WIFI, "WiFi task alive...\n");
        vTaskDelay(2000 / portTICK_PERIOD_MS);
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
                int cmd_len = strlen(start_cmd);
                int received_len = rxp.dataLength - sizeof(wifiCtrl->cmd);
                
                // Check if the received command matches "start_mission"
                if (cmd_len == received_len && strncmp(start_cmd, (const char*)wifiCtrl->data, cmd_len) == 0) {
                    cpxPrintToConsole(LOG_TO_WIFI, "<- Received START MISSION command!\n");
                    mission_signal_start(); // Set the flag!
                } else {
                    // Print any other commands for debugging
                    cpxPrintToConsole(LOG_TO_WIFI, "<- Received command: %.*s\n", received_len, wifiCtrl->data);
                }
            }
        }
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
                    cpxPrintToConsole(LOG_TO_WIFI, "<- HEARTBEAT from SysID: %d\n", received_msg.sysid);
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
                    // Keep track of how many attitude messages we've received
                    static int attitude_msg_count = 0;
                    attitude_msg_count++;

                    // Every 5 messages, print the data
                    if (attitude_msg_count % 5 == 0) {
                        // Decode the message
                        mavlink_attitude_t attitude;
                        mavlink_msg_attitude_decode(&received_msg, &attitude);

                        // Print the attitude data to the WiFi console
                        cpxPrintToConsole(LOG_TO_WIFI, "<- ATTITUDE: Roll=%.2f Pitch=%.2f Yaw=%.2f\n",
                            attitude.roll, attitude.pitch, attitude.yaw);
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
 * @brief Requests the essential data streams from the flight controller at startup.
 */
void mavlink_init_data_streams(void) {
    printf("-> Requesting essential data streams...\n");

    // Request LOCAL_POSITION_NED at 10 Hz
    // This provides the x, y, z position needed to calculate altitude.
    mavlink_request_data_stream(MAVLINK_MSG_ID_LOCAL_POSITION_NED, 10.0f);

    // You can add requests for other streams here if needed, for example:
    mavlink_request_data_stream(MAVLINK_MSG_ID_ATTITUDE, 10.0f);
}

int main(void) {
    pi_bsp_init();
    pi_freq_set(PI_FREQ_DOMAIN_FC, 250000000);
    __pi_pmu_voltage_set(PI_PMU_DOMAIN_FC, 1200);
    return pmsis_kickoff((void *)start_main_application);
}

// ---------------------------------------------------------- //

/**
 * @brief Public function to send a MAVLink message.
 * This function is thread-safe.
 */
 void send_mavlink_message(const mavlink_message_t *msg) {
    // This now mimics the mav_wifi_test behavior
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

// ------------------------------------------- //

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