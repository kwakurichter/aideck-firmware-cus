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
 * @brief NEW TASK: Listens for and parses incoming MAVLink messages.
 */
void uart_receive_task(void *parameters) {
    // MAVLink parser state variables
    mavlink_message_t received_msg;
    mavlink_status_t status = {0}; // IMPORTANT: Zero-initialize status
    uint8_t rx_byte;

    while(1) {
        // Read one byte at a time (this will block the task until a byte is available)
        pi_uart_read(&uart_device, &rx_byte, 1);

        // Feed the byte into the MAVLink parser
        if (mavlink_parse_char(MAVLINK_COMM_0, rx_byte, &received_msg, &status)) {
            // A complete and valid message has been received
            cpxPrintToConsole(LOG_TO_WIFI, "<- MAVLink MSG RX OK! (SysID: %d, CompID: %d, MsgID: %d)\n",
                   received_msg.sysid, received_msg.compid, received_msg.msgid);

            // Example: You could handle specific messages here
            // if (received_msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
            //     // Now we know the host is alive
            // }
        }
        // This part is commented out but shows how you could detect parsing errors
        /* else if (status.parse_error > 0) {
            // A parse error occurred (e.g., bad checksum)
            cpxPrintToConsole(LOG_TO_WIFI, "<- MAVLink Parse Error! Count: %d\n", status.parse_error);
            // Reset the error count after logging it
            status.parse_error = 0;
        } */
    }
}


/**
 * @brief Task to send periodic HEARTBEAT messages over UART.
 */
void uart_transmit_task(void *parameters) {
    while (1) {
        // This print goes to the serial console, not the STM32
        // It's useful for seeing if the task is running.
        printf("TX task alive...\n");
        send_heartbeat();
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Send heartbeat every second
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

    // Call the function to set up the WiFi AP
    #ifdef SETUP_WIFI_AP
        setupWiFi();
    #endif

    // --- Centralized UART Initialization ---
    struct pi_uart_conf conf;
    pi_uart_conf_init(&conf);
    conf.enable_tx = 1;
    conf.enable_rx = 1; // Enable both TX and RX
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
    // Renamed task to be more descriptive
    xTask = xTaskCreate(uart_transmit_task, "uart_transmit_task", configMINIMAL_STACK_SIZE * 8, NULL, tskIDLE_PRIORITY + 1, NULL);
    if (xTask != pdPASS) {
        cpxPrintToConsole(LOG_TO_WIFI, "FATAL: UART transmit task creation failed!\n");
    }

    // Create the new UART receive task
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
        // The pi_yield() is important in the main task loop if it's doing work,
        // but here the FreeRTOS scheduler has taken over, so this loop is effectively idle.
        pi_yield();
    }
}

int main(void) {
    pi_bsp_init();
    pi_freq_set(PI_FREQ_DOMAIN_FC, 250000000);
    __pi_pmu_voltage_set(PI_PMU_DOMAIN_FC, 1200);
    return pmsis_kickoff((void *)start_main_application);
}