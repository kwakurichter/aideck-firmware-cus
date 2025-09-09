#include "pmsis.h"
#include "bsp/bsp.h"
#include "cpx.h"
#include "wifi.h"
#include "mavlink_interface.h"
#include "mission_control.h"


#define LED_PIN (2)

// --- Global Variables ---
static pi_device_t led_device;
static CPXPacket_t txp; // Packet for sending CPX commands
static CPXPacket_t rxp; // Packet for receiving CPX commands
static struct pi_device uart_device; // UART device handle

xQueueHandle mavlink_tx_queue; // Queue for outgoing MAVLink messages
uint8_t send_buffer[MAVLINK_MAX_PACKET_LEN];
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
 * @brief Task to listen for and parse incoming MAVLink messages.
 */
void uart_receive_task(void *parameters) {
    mavlink_message_t received_msg;
    mavlink_status_t status = {0};
    uint8_t rx_byte;

    while(1) {
        pi_uart_read(&uart_device, &rx_byte, 1);
        if (mavlink_parse_char(MAVLINK_COMM_0, rx_byte, &received_msg, &status)) {
            // A complete message has been received, update our state
            switch (received_msg.msgid) {
                case MAVLINK_MSG_ID_HEARTBEAT: {
                    mavlink_heartbeat_t hb;
                    mavlink_msg_heartbeat_decode(&received_msg, &hb);
                    g_vehicle_state.is_armed = (hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED);
                    break;
                }
                case MAVLINK_MSG_ID_LOCAL_POSITION_NED: {
                    mavlink_local_position_ned_t pos;
                    mavlink_msg_local_position_ned_decode(&received_msg, &pos);
                    g_vehicle_state.position_ned[0] = pos.x; // North
                    g_vehicle_state.position_ned[1] = pos.y; // East
                    g_vehicle_state.position_ned[2] = pos.z; // Down
                    g_vehicle_state.altitude_m = -pos.z; // Convert 'down' to altitude
                    break;
                }
                case MAVLINK_MSG_ID_PARAM_VALUE: {
                    mavlink_param_value_t param;
                    mavlink_msg_param_value_decode(&received_msg, &param);
                    cpxPrintToConsole(LOG_TO_WIFI, "<- PARAM_VALUE: %s = %f\n", param.param_id, param.param_value);
                    break;
                }
                // Add more cases here for other messages you're interested in
            }
        }
    }
}

/**
 * @brief Task to send MAVLink messages over UART.
 */
void uart_transmit_task(void *parameters) {
    mavlink_message_t msg_to_send;

    while (1) {
        // Block until a message is available in the queue
        if (xQueueReceive(mavlink_tx_queue, &msg_to_send, portMAX_DELAY) == pdPASS) {
            uint16_t len = mavlink_msg_to_send_buffer(send_buffer, &msg_to_send);
            pi_uart_write(&uart_device, send_buffer, len);
        }
    }
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
 * @brief A simple task to periodically send a heartbeat.
 */
void heartbeat_task(void *parameters) {
    while (1) {
        send_heartbeat();
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Send every 1 second
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
        cpxPrintToConsole(LOG_TO_WIFI, "FATAL: UART open failed!\n");
        pmsis_exit(-1);
    }
    cpxPrintToConsole(LOG_TO_WIFI, "UART opened successfully.\n");    

    cpxEnableFunction(CPX_F_WIFI_CTRL); // Enable the CPX function for WiFi control commands

    // 2. Create the transmit queue
    mavlink_tx_queue = xQueueCreate(10, sizeof(mavlink_message_t)); // Queue can hold 10 messages
    if (mavlink_tx_queue == NULL) {
        cpxPrintToConsole(LOG_TO_WIFI, "FATAL: MAVLink TX queue creation failed!\n");   
        pmsis_exit(-1);
    }    

    cpxPrintToConsole(LOG_TO_WIFI, "MAVLink TX queue creation successful.\n");

    // 3. Start the application-specific tasks
    BaseType_t xTask;

    // MAVLink Tasks
    xTask = xTaskCreate(uart_receive_task, "mav_rx_task", configMINIMAL_STACK_SIZE * 4, NULL, tskIDLE_PRIORITY + 2, NULL);
    if (xTask != pdPASS) {
        cpxPrintToConsole(LOG_TO_WIFI, "FATAL: MAVLink UART receive task creation failed!\n");
    }
    xTask = xTaskCreate(uart_transmit_task, "mav_tx_task", configMINIMAL_STACK_SIZE * 4, NULL, tskIDLE_PRIORITY + 2, NULL);
    if (xTask != pdPASS) {
        cpxPrintToConsole(LOG_TO_WIFI, "FATAL: MAVLink UART transmit task creation failed!\n");
    }

    // Application-specific Tasks 
    xTask = xTaskCreate(led_blinky_task, "led_task", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
    if (xTask != pdPASS) {
        cpxPrintToConsole(LOG_TO_WIFI, "FATAL: LED task creation failed!\n");
    }
    xTask = xTaskCreate(wifi_command_receive_task, "wifi_cmd_task", configMINIMAL_STACK_SIZE * 2, NULL, tskIDLE_PRIORITY + 1, NULL);
    if (xTask != pdPASS) {
        cpxPrintToConsole(LOG_TO_WIFI, "FATAL: WiFi Command task creation failed!\n");
    }
    xTask = xTaskCreate(heartbeat_task, "heartbeat_task", configMINIMAL_STACK_SIZE * 2, NULL, tskIDLE_PRIORITY + 2, NULL);
    if (xTask != pdPASS) {
        cpxPrintToConsole(LOG_TO_WIFI, "FATAL: Heartbeat task creation failed!\n");
    }
    xTask = xTaskCreate(wifi_debug_task, "wifi_debug_task", configMINIMAL_STACK_SIZE * 2, NULL, tskIDLE_PRIORITY + 1, NULL);
    if (xTask != pdPASS) {
        cpxPrintToConsole(LOG_TO_WIFI, "FATAL: WiFi Debug task creation failed!\n");
    }   
    
    // Mission Task
    xTask = xTaskCreate(autonomous_mission_task, "mission_task", configMINIMAL_STACK_SIZE * 4, &g_vehicle_state, tskIDLE_PRIORITY + 1, NULL);
    if (xTask != pdPASS) {
        cpxPrintToConsole(LOG_TO_WIFI, "FATAL: Mission task creation failed!\n");
    }

    // 4. Request MAVLink streams
    mavlink_init_data_streams();
    
    // 4. Yield forever, allowing the created tasks to run.
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