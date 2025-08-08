#include "pmsis.h" // Main header for all PULP-SDK drivers

// --- Configuration ---
// The AI-deck hardware connects the STM32 to the GAP8's UART peripheral number 1.
#define UART_INTERFACE      (1)
#define UART_BAUDRATE       (57600)

// --- Global Variables ---
static struct pi_device uart_device; // Device struct to hold UART configuration

/**
 * @brief The main task for our application.
 *
 * This function initializes the UART and then enters an infinite loop
 * to echo data.
 */
void uart_echo_task(void)
{
    printf("Entering main task, initializing UART...\n");

    // 1. Configure the UART peripheral
    struct pi_uart_conf conf;
    pi_uart_conf_init(&conf);
    conf.uart_itf = UART_INTERFACE;
    conf.baudrate_bps = UART_BAUDRATE;
    conf.enable_rx = 1;
    conf.enable_tx = 1;

    // 2. Open the UART device with the configuration
    pi_open_from_conf(&uart_device, &conf);
    if (pi_uart_open(&uart_device))
    {
        printf("Error: UART open failed!\n");
        pmsis_exit(-1); // Exit if we can't open the UART
    }

    printf("UART opened successfully. Ready to echo data.\n");

    uint8_t received_char;

    // 3. Enter the infinite echo loop
    while (1)
    {
        // Read 1 byte from the UART. This is a blocking call, so the code
        // will wait here until a character is received.
        pi_uart_read(&uart_device, &received_char, 1);

        // Write the same byte back to the UART.
        pi_uart_write(&uart_device, &received_char, 1);
    }
}

/**
 * @brief The main entry point for the application.
 *
 * This function is called first. It simply starts the PMSIS scheduler
 * and kicks off our main task.
 */
int main(void)
{
    // pmsis_kickoff is the standard way to start a GAP8 application.
    // It will hand control over to the uart_echo_task function.
    return pmsis_kickoff((void *)uart_echo_task);
}