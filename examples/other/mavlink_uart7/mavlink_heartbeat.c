#include "pmsis.h"
#include "stdio.h"

// 1. Include the MAVLink header
#include "ardupilotmega/mavlink.h"

// --- Global variables to avoid stack issues ---

// 2. Buffer to hold the serialized MAVLink message before sending
uint8_t send_buffer[MAVLINK_MAX_PACKET_LEN];

// 3. Keep track of the MAVLink packet sequence number
uint8_t mavlink_sequence = 0;

// UART device handle
struct pi_device uart_device;


/**
 * @brief Sends a MAVLink message over the UART port.
 * @param msg A pointer to the MAVLink message to be sent.
 */
void send_mavlink_message(const mavlink_message_t *msg)
{
    // Serialize the MAVLink message into the global send_buffer
    uint16_t len = mavlink_msg_to_send_buffer(send_buffer, msg);

    // Send the serialized bytes over UART
    pi_uart_write(&uart_device, send_buffer, len);
}


void test_mavlink_api(void)
{
    printf("Entering main controller (MAVLink API Test)...\n");

    struct pi_uart_conf conf;

    pi_uart_conf_init(&conf);
    conf.enable_tx = 1;
    conf.enable_rx = 0;
    conf.baudrate_bps = 115200;

    pi_open_from_conf(&uart_device, &conf);
    if (pi_uart_open(&uart_device))
    {
        printf("Uart open failed !\n");
        pmsis_exit(-1);
    }

    printf("UART opened successfully. Starting MAVLink heartbeats...\n");

    while(1)
    {
        // --- Create and send the HEARTBEAT message ---
        mavlink_message_t msg;
        mavlink_heartbeat_t heartbeat;

        // Set heartbeat properties
        heartbeat.type = MAV_TYPE_QUADROTOR;
        heartbeat.autopilot = MAV_AUTOPILOT_GENERIC;
        heartbeat.base_mode = 0;
        heartbeat.custom_mode = 0;
        heartbeat.system_status = MAV_STATE_ACTIVE;

        // Pack the heartbeat message
        mavlink_msg_heartbeat_encode(
            1, // System ID
            1, // Component ID
            &msg,
            &heartbeat
        );

        // Send the message over UART
        send_mavlink_message(&msg);

        // Delay for 1 second
        pi_time_wait_us(1000 * 1000);
    }

    pmsis_exit(0);
}


int main(void)
{
    printf("\n\n\t *** PMSIS MAVLink API UART Test ***\n\n");
    return pmsis_kickoff((void *) test_mavlink_api);
}