#include "pmsis.h"
#include "stdio.h"

// --- Global variable for the receive buffer ---
// Using a global buffer is crucial to avoid stack memory issues.
uint8_t rx_buffer;

void test_uart_echo(void)
{
    printf("Entering main controller (UART Echo Test)...\n");

    struct pi_device uart_device;
    struct pi_uart_conf conf;

    // 1. Initialize UART configuration
    pi_uart_conf_init(&conf);
    conf.enable_tx = 1; // Enable transmitter to send data back
    conf.enable_rx = 1; // Enable receiver to listen for data
    conf.baudrate_bps = 115200;

    // 2. Open the UART device
    pi_open_from_conf(&uart_device, &conf);
    if (pi_uart_open(&uart_device))
    {
        printf("Uart open failed !\n");
        pmsis_exit(-1);
    }

    printf("UART opened successfully. Waiting for data to echo...\n");

    // 3. Main echo loop
    while(1)
    {
        // pi_uart_read is a "blocking" function. It will pause here
        // and wait until it receives exactly 1 byte.
        pi_uart_read(&uart_device, &rx_buffer, 1);

        // As soon as a byte is received, write it right back.
        pi_uart_write(&uart_device, &rx_buffer, 1);
    }

    // This code is unreachable but good practice
    pi_uart_close(&uart_device);
    pmsis_exit(0);
}

int main(void)
{
    printf("\n\n\t *** PMSIS UART Echo Test ***\n\n");
    return pmsis_kickoff((void *) test_uart_echo);
}
