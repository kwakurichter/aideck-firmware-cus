#include "pmsis.h"
#include "stdio.h"

// A MAVLink v1.0 heartbeat message is 17 bytes long
#define MAVLINK_PACKET_SIZE 17
// The special byte that marks the beginning of a MAVLink v1 packet
#define MAVLINK_START_OF_FRAME 0xFE

// --- Global buffer to avoid stack issues ---
uint8_t echo_buffer[MAVLINK_PACKET_SIZE];

void test_raw_echo(void)
{
    printf("Entering main controller (Robust Echo Test)...\n");

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

    printf("UART opened. Synchronizing to MAVLink stream...\n");

    while(1)
    {
        // 1. Read one byte at a time until we find the start of a packet.
        //    This allows us to synchronize with the stream at any time.
        do {
            pi_uart_read(&uart_device, &echo_buffer[0], 1);
        } while (echo_buffer[0] != MAVLINK_START_OF_FRAME);

        // 2. We found the start byte! Now read the rest of the packet.
        //    (MAVLINK_PACKET_SIZE - 1) more bytes.
        if (MAVLINK_PACKET_SIZE > 1) {
            pi_uart_read(&uart_device, &echo_buffer[1], MAVLINK_PACKET_SIZE - 1);
        }

        // 3. We have a complete, aligned packet. Echo it back.
        pi_uart_write(&uart_device, echo_buffer, MAVLINK_PACKET_SIZE);
    }

    pmsis_exit(0);
}

int main(void)
{
    printf("\n\n\t *** PMSIS Robust Raw Echo Test ***\n\n");
    return pmsis_kickoff((void *) test_raw_echo);
}
