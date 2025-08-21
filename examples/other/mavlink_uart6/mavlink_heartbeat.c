#include "pmsis.h"
#include "stdio.h"

/*
 * The FIX: Declare the MAVLink packet array as a global variable.
 * This puts it in permanent, static memory, preventing the stack-related
 * issues we saw in the previous tests.
*/
uint8_t mavlink_heartbeat[] = {
    0xFE, // Start of frame
    0x09, // Payload length
    0x00, // Packet sequence
    0x01, // System ID
    0x01, // Component ID
    0x00, // Message ID (HEARTBEAT)
    0x00, 0x00, 0x00, 0x00, // custom_mode
    0x13, // type (MAV_TYPE_QUADROTOR)
    0x08, // autopilot (MAV_AUTOPILOT_ARDUPILOTMEGA)
    0x03, // base_mode
    0x04, // system_status
    0x03, // mavlink_version
    0x4B, // Checksum A
    0x17  // Checksum B
};

void test_mavlink_uart(void)
{
    printf("Entering main controller (MAVLink test)...\n");

    struct pi_device uart_device;
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

    printf("UART opened. Sending MAVLink heartbeats...\n");

    while(1)
    {
        // Write the entire global mavlink_heartbeat array.
        pi_uart_write(&uart_device, mavlink_heartbeat, sizeof(mavlink_heartbeat));

        // Increment the sequence number for the next packet.
        mavlink_heartbeat[2]++;

        // Delay for 1 second.
        pi_time_wait_us(1000 * 1000);
    }

    pmsis_exit(0);
}

int main(void)
{
    printf("\n\n\t *** PMSIS MAVLink UART Test ***\n\n");
    return pmsis_kickoff((void *) test_mavlink_uart);
}