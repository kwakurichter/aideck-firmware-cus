#include "pmsis.h"
#include "stdio.h"
#include "ardupilotmega/mavlink.h"

// --- Global Variables ---
// UART device handle
struct pi_device uart_device;
// Buffer for serializing messages before sending
uint8_t send_buffer[MAVLINK_MAX_PACKET_LEN];
// Buffer for receiving one byte at a time
uint8_t rx_byte;

// --- MAVLink System and Component IDs for the GAP8 ---
#define GAP8_SYSTEM_ID 1
#define GAP8_COMPONENT_ID 1

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

void test_mavlink_comm(void) {
    printf("Entering main controller (MAVLink Comm Test)...\n");

    struct pi_uart_conf conf;
    pi_uart_conf_init(&conf);
    conf.enable_tx = 1;
    conf.enable_rx = 1;
    conf.baudrate_bps = 115200;

    pi_open_from_conf(&uart_device, &conf);
    if (pi_uart_open(&uart_device)) {
        printf("Uart open failed!\n");
        pmsis_exit(-1);
    }
    printf("UART opened successfully.\n");

    // MAVLink parser state variables
    mavlink_message_t received_msg;
    // THE FIX: Zero-initialize the status struct before use.
    mavlink_status_t status = {0};

    // Send the first heartbeat to let the host know we are here
    pi_time_wait_us(1000 * 1000); // Wait a second for host to be ready
    send_heartbeat();

    while (1) {
        // Read one byte at a time from the UART (blocking)
        pi_uart_read(&uart_device, &rx_byte, 1);

        // Feed the byte into the MAVLink parser
        if (mavlink_parse_char(MAVLINK_COMM_0, rx_byte, &received_msg, &status)) {
            // A complete and valid message has been received
            if (received_msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                printf("<- Received HEARTBEAT from host (SysID: %d, CompID: %d)\n",
                       received_msg.sysid, received_msg.compid);

                // Respond with our own heartbeat
                send_heartbeat();
            }
        }
    }
    pmsis_exit(0);
}

int main(void) {
    printf("\n\n\t *** PMSIS Full MAVLink Test ***\n\n");
    return pmsis_kickoff((void *) test_mavlink_comm);
}
