#include "pmsis.h"
#include "bsp/bsp.h"
#include "cpx.h"
#include "ardupilotmega/mavlink.h"
#include "wifi.h" // Include the WiFi control header

#define GAP8_SYSTEM_ID 1
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

static volatile bool is_mission_start = false;

typedef struct {
    bool is_armed;
    float position_ned[3]; // North, East, Down in meters
    float altitude_m;      // Calculated from z position
    // Add other state variables here as needed (e.g., attitude, battery)
} VehicleState_t;

VehicleState_t g_vehicle_state = {0}; // A single, shared structure to hold the vehicle's state

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
    mavlink_request_data_stream(MAVLINK_MSG_ID_ATTITUDE, 10.0f);
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
            request_parameter();
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
                case MAVLINK_MSG_ID_LOCAL_POSITION_NED: {
                    // Keep track of how many attitude messages we've received
                    static int pos_msg_count = 0;
                    pos_msg_count++;

                    // We received a position, decode it
                    mavlink_local_position_ned_t pos;
                    mavlink_msg_local_position_ned_decode(&received_msg, &pos);
                    g_vehicle_state.position_ned[0] = pos.x; // North
                    g_vehicle_state.position_ned[1] = pos.y; // East
                    g_vehicle_state.position_ned[2] = pos.z; // Down
                    g_vehicle_state.altitude_m = -pos.z; // Convert 'down' to altitude
                    
                    if (pos_msg_count % 5 == 0) {
                        // Print the received position over WiFi
                        cpxPrintToConsole(LOG_TO_WIFI, "<- POS: X=%.2f Y=%.2f Z=%.2f\n",
                            pos.x, pos.y, pos.z);                        
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
                    is_mission_start = true;
                } else {
                    // Print any other commands for debugging
                    cpxPrintToConsole(LOG_TO_WIFI, "<- Received command: %.*s\n", received_len, wifiCtrl->data);
                }
            }
        }
    }
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
    mavlink_set_mode(COPTER_MODE_RTL);
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
                if (is_mission_start) {
                    cpxPrintToConsole(LOG_TO_WIFI, "Mission: Start command received! Waiting for arm...\n");
                    current_state = MISSION_WAIT_FOR_ARM;
                }
                break;            
            case MISSION_WAIT_FOR_ARM:
                if (g_vehicle_state.is_armed) {
                    cpxPrintToConsole(LOG_TO_WIFI, "Mission: Vehicle is armed.\n");
                    current_state = MISSION_SET_ORIGIN;
                }
                else {
                    mavlink_arm_vehicle();
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
    cpxEnableFunction(CPX_F_WIFI_CTRL);

    BaseType_t xTask;
    xTask = xTaskCreate(uart_transmit_task, "uart_transmit_task", configMINIMAL_STACK_SIZE * 8, NULL, tskIDLE_PRIORITY + 1, NULL);
    if (xTask != pdPASS) {
        cpxPrintToConsole(LOG_TO_WIFI, "FATAL: UART transmit task creation failed!\n");
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

    xTask = xTaskCreate(autonomous_mission_task, "mission_task", configMINIMAL_STACK_SIZE * 4, &g_vehicle_state, tskIDLE_PRIORITY + 1, NULL);
    if (xTask != pdPASS) {
        cpxPrintToConsole(LOG_TO_WIFI, "FATAL: Mission task creation failed!\n");
    }    

    // 4. Request MAVLink streams
    mavlink_init_data_streams();    

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