#include "pmsis.h"
#include "bsp/bsp.h"
#include "cpx.h"
#include "ardupilotmega/mavlink.h"
#include "wifi.h" // Include the WiFi control header

#define GAP8_SYSTEM_ID 1
#define GAP8_COMPONENT_ID 1
#define LED_PIN (2)

// --- Global Variables ---
static struct pi_device uart_device;
static pi_device_t led_device;
static CPXPacket_t txp; // Packet for sending CPX commands
uint8_t send_buffer[MAVLINK_MAX_PACKET_LEN];

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
                default:
                    // Optional: Log other messages if needed for debugging
                    // cpxPrintToConsole(LOG_TO_WIFI, "<- RX MSG with ID: %d\n", received_msg.msgid);
                    break;
            }
        }
    }
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
 * @brief Task to send periodic debug messages over WiFi.
 */
void wifi_debug_task(void *parameters) {
    while(1) {
        cpxPrintToConsole(LOG_TO_WIFI, "WiFi task alive...\n");
        vTaskDelay(2000 / portTICK_PERIOD_MS);
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