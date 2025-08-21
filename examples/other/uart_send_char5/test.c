#include "pmsis.h"
#include "stdio.h"
#include "bsp/bsp.h"

static void test_gap8(void)
{
    printf("Entering main controller with BSP init...\n");

    struct pi_device uart_device;
    struct pi_uart_conf conf;

    pi_uart_conf_init(&conf);
    conf.uart_id = 1;
    conf.baudrate_bps = 115200;
    conf.enable_tx = 1;
    conf.enable_rx = 0;
    conf.parity_mode = PI_UART_PARITY_DISABLE;
    conf.stop_bit_count = 1;

    pi_open_from_conf(&uart_device, &conf);

    if (pi_uart_open(&uart_device))
    {
        printf("[UART] open failed !\n");
        pmsis_exit(-1);
    }

    printf("[UART] Successfully opened UART ID 1.\n");

    uint8_t test_char = 'U';

    printf("Sending 'U' (0x55) every second...\n");

    while(1)
    {
        pi_uart_write(&uart_device, &test_char, 1);
        pi_time_wait_us(1000 * 1000);
    }

    pmsis_exit(0);
}

int main(void)
{
    pi_bsp_init();
    pi_freq_set(PI_FREQ_DOMAIN_FC, 250000000);
    return pmsis_kickoff((void *)test_gap8);
}