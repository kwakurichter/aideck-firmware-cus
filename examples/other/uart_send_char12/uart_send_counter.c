/* PMSIS includes */
#include "pmsis.h"

/*
 * The FIX: Declare the variable globally, outside the function.
 * This places it in static memory, protecting it from stack optimizations.
*/
uint8_t test_byte = 85;

void test_uart_helloworld(void)
{
    printf("Entering main controller\n");

    uint32_t errors = 0;
    struct pi_device uart;
    struct pi_uart_conf conf;

    /* Init & open uart. */
    pi_uart_conf_init(&conf);
    conf.enable_tx = 1;
    conf.enable_rx = 0;
    conf.baudrate_bps = 115200;
    pi_open_from_conf(&uart, &conf);
    if (pi_uart_open(&uart))
    {
        printf("Uart open failed !\n");
        pmsis_exit(-1);
    }

    // Now, the while loop will work correctly because 'test_byte' is global.
    while(1)
    {
        pi_uart_write(&uart, &test_byte, 1);
        pi_time_wait_us(500000);
    }

    // This code is unreachable, but that's okay for this test.
    pi_uart_close(&uart);
    pmsis_exit(errors);
}

/* Program Entry. */
int main(void)
{
    printf("\n\n\t *** PMSIS Uart HelloWorld ***\n\n");
    return pmsis_kickoff((void *) test_uart_helloworld);
}