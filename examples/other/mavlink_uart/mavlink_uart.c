#include "pmsis.h"
#include "bsp/bsp.h"

// Your MAVLink header would go here
// #include "mavlink.h"

void start_example(void)
{
    // --- 1. Initialize the board hardware ---
    pi_bsp_init();

    // --- 2. Initialize and open the UART device ---
    struct pi_uart_conf conf;
    struct pi_device uart_device;
    pi_uart_conf_init(&conf);
    conf.baudrate_bps = 115200; // Or your desired MAVLink baud rate

    pi_open_from_conf(&uart_device, &conf);
    if (pi_uart_open(&uart_device))
    {
        // A printf here will go to the JTAG console
        printf("UART open failed!\n");
        pmsis_exit(-1);
    }

    // --- 3. Main Loop: Send your MAVLink Heartbeat ---
    while (1)
    {
        // Example: sending a single character 'U' (like in ArduPilot)
        uint8_t mavlink_byte = 'U';

        // Use pi_uart_write to send data
        pi_uart_write(&uart_device, &mavlink_byte, 1);

        // Your logic for constructing and sending a full heartbeat packet would go here.

        // Delay for 1 second
        pi_time_wait_us(1000 * 1000);
    }
}

// The main function remains the same
int main(void)
{
    return pmsis_kickoff((void *)start_example);
}