#include "pmsis.h"
#include "bsp/bsp.h"
#include "cpx.h"
#include "ardupilotmega/mavlink.h"
#include "wifi.h" // Include the WiFi control header
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#define LED_PIN (2)
#define P2P_STX_ATT_V1 0xA7
#define P2P_ATT_V1_LEN 20
#define PEER_SYSTEM_ID 20
#define PEER2_SYSTEM_ID 21
#define P2P_MAX_CHAN   1   // adjust to however many logical streams

// --- Global Variables ---
static struct pi_device uart_device;
static pi_device_t led_device;
static CPXPacket_t txp; // Packet for sending CPX commands
static CPXPacket_t rxp; // Packet for receiving CPX commands


static volatile bool g_p2p_data_received_once = false; // Becomes true once we get the first p2p ATTITUDE message


typedef struct {
    float roll;
    float pitch;
    float yaw;
} PeerState_t;

PeerState_t g_peer_state = {0}; // Holds the last known attitude of the peer

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
                // Your existing logic to handle messages for the AI-deck's own use can go here
                // switch(received_msg.msgid) { ... }

                // --- 2. Re-serialize the message into a raw byte buffer ---
                uint16_t len = mavlink_msg_to_send_buffer(mavlink_tx_buffer, &received_msg);

                // --- 3. Forward the raw buffer over WiFi using our safe function ---
                // Using CPX_F_APP as the "uplink" channel
                cpxSendRawData(CPX_T_WIFI_HOST, CPX_F_CONSOLE, mavlink_tx_buffer, len);
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

// The mission task function
void voodoo_task(void *parameters) {
    // --- State-specific counters ---
    int stream_retries = 0;
    const int MAX_STREAM_RETRIES = 6; // 3-second timeout
    cpxPrintToConsole(LOG_TO_WIFI, "Mission task started. Waiting for start command via WiFi...\n");

    const int VOODOO_DT_MS = 50;
    const int VOODOO_PITCH_THRESHOLD = 20; // ~20 degrees
    const int VOODOO_ROLL_THRESHOLD = 20;  // ~20 degrees
    const int VOODOO_TIMEOUT_MS = 5000;        // 5-second timeout 
    const int VOODOO_TICKS = (VOODOO_TIMEOUT_MS / VOODOO_DT_MS);         

    static int voodoo_ticks = 0;

    while (1) {
        // SAFETY: Check for data timeout first
        if (voodoo_ticks >= VOODOO_TICKS) {
            printf("Voodoo: Peer data timeout! Landing.\n");
            //cpxPrintToConsole(LOG_TO_WIFI, "Voodoo: Peer data timeout! Landing.\n");
            break; // Exit the state immediately
        }

        // Local copies of peer attitude for this loop
        float peer_roll = g_peer_state.roll;
        float peer_pitch = g_peer_state.pitch;

        // --- Single-Action-Priority Logic ---
        if (peer_pitch < -VOODOO_PITCH_THRESHOLD) {
            voodoo_ticks = 0;
            // Move Forward (Pitch stick forward is negative X)
            printf("Voodoo: FORWARD\n");
            //cpxPrintToConsole(LOG_TO_WIFI, "Voodoo: FORWARD\n");
        }
        else if (peer_pitch > VOODOO_PITCH_THRESHOLD) {
            voodoo_ticks = 0;
            // Move Backward (Pitch stick back is positive X)
            printf("Voodoo: BACKWARD\n");
            //cpxPrintToConsole(LOG_TO_WIFI, "Voodoo: BACKWARD\n");
        }
        else if (peer_roll > VOODOO_ROLL_THRESHOLD) {
            voodoo_ticks = 0;
            // Move Right (Roll stick right is positive Y)
            printf("Voodoo: RIGHT\n");
            //cpxPrintToConsole(LOG_TO_WIFI, "Voodoo: RIGHT\n");
        }
        else if (peer_roll < -VOODOO_ROLL_THRESHOLD) {
            voodoo_ticks = 0;
            // Move Left (Roll stick left is negative Y)
            printf("Voodoo: LEFT\n");
            //cpxPrintToConsole(LOG_TO_WIFI, "Voodoo: LEFT\n");
        }
        else {
            // No thresholds met, default to a stable hover
            voodoo_ticks++;

            printf("Voodoo: HOVER\n");
            //cpxPrintToConsole(LOG_TO_WIFI, "Voodoo: HOVER\n");
        }

        vTaskDelay(50 / portTICK_PERIOD_MS); // Loop 20Hz
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