#include "pmsis.h"
#include "bsp/bsp.h"
#include "cpx.h"
#include "ardupilotmega/mavlink.h"
#include "wifi.h" // Include the WiFi control header

#define LED_PIN (2)

// --- Global Variables ---
static struct pi_device uart_device;
static pi_device_t led_device;
static CPXPacket_t txp; // Packet for sending CPX commands
static CPXPacket_t rxp; // Packet for receiving CPX commands

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

        // Check if the packet is from the WiFi Host and has data
        if (rxp.route.source == CPX_T_WIFI_HOST && rxp.dataLength > 0) {
            // Directly write the raw MAVLink payload to the UART for the flight controller
            pi_uart_write(&uart_device, rxp.data, rxp.dataLength);
        }
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
        cpxPrintToConsole(LOG_TO_WIFI, "FATAL: UART receive task creation failed!\n");
    }

    xTask = xTaskCreate(wifi_command_receive_task, "wifi_cmd_rx_task", configMINIMAL_STACK_SIZE * 8, NULL, tskIDLE_PRIORITY + 1, NULL);
    if (xTask != pdPASS) {
        cpxPrintToConsole(LOG_TO_WIFI, "FATAL: WiFi command receive task creation failed!\n");
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