#include "pmsis.h"
#include "stdio.h"

// A MAVLink v1.0 heartbeat message is 17 bytes long
// (6 header + 9 payload + 2 checksum)
#define MAVLINK_PACKET_SIZE 17

// --- Global buffer to avoid stack issues ---
uint8_t echo_buffer[MAVLINK_PACKET_SIZE];

void test_raw_echo(void)
{
    printf("Entering main controller (Raw Echo Test)...\n");

    struct pi_device uart_device;
    struct pi_uart_conf conf;

    pi_uart_conf_init(&conf);
    conf.enable_tx = 1;
    conf.enable_rx = 1;
    conf.baudrate_bps = 115200;

    pi_open_from_conf(&uart_device, &conf);
    if (pi_uart_open(&uart_device))
    {
        printf("Uart open failed!\n");
        pmsis_exit(-1);
    }

    printf("UART opened. Waiting for %d-byte packet to echo...\n", MAVLINK_PACKET_SIZE);

    while(1)
    {
        // 1. Block and wait until a full packet is received
        pi_uart_read(&uart_device, echo_buffer, MAVLINK_PACKET_SIZE);

        // 2. Immediately write the entire buffer back
        pi_uart_write(&uart_device, echo_buffer, MAVLINK_PACKET_SIZE);
    }

    pmsis_exit(0);
}

int main(void)
{
    printf("\n\n\t *** PMSIS Raw Echo Test ***\n\n");
    return pmsis_kickoff((void *) test_raw_echo);
}
