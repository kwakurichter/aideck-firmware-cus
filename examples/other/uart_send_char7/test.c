#include "pmsis.h"
#include "stdio.h"

// The main application function, modeled directly after the SDK example.
void test_uart_simple(void)
{
    printf("Entering main controller (SDK simple method)...\n");

    struct pi_device uart_device;
    struct pi_uart_conf conf;

    // 1. Initialize the UART configuration structure with defaults.
    pi_uart_conf_init(&conf);
    conf.enable_tx = 1;
    conf.enable_rx = 0; // We are only sending data
    conf.baudrate_bps = 115200;

    // 2. Open the UART device using the configuration.
    // The SDK automatically uses the correct default UART for the AI-deck.
    pi_open_from_conf(&uart_device, &conf);
    if (pi_uart_open(&uart_device))
    {
        printf("Uart open failed !\n");
        pmsis_exit(-1);
    }

    printf("UART opened successfully. Starting transmission...\n");

    uint8_t test_char = 'U';

    // 3. Main loop to send the character.
    while(1)
    {
        // The pi_uart_write function is the standard way to send data.
        pi_uart_write(&uart_device, &test_char, 1);

        // Simple blocking delay for 1 second.
        pi_time_wait_us(1000 * 1000);
    }

    // This part of the code is not reachable in this example,
    // but it's good practice to include.
    pi_uart_close(&uart_device);
    pmsis_exit(0);
}

// The main entry point is identical to the SDK example.
int main(void)
{
    printf("\n\t *** AI-Deck UART Test ***\n\n");
    return pmsis_kickoff((void *) test_uart_simple);
}