#include "pmsis.h"
#include "stdio.h"

// No longer need the task-based variables
// static pi_task_t led_task;
static int led_val = 0;
static struct pi_device gpio_device;
static PI_L2 uint32_t value;


static void test_gap8(void)
{
    printf("Entering main controller...\n");

    // --- UART Configuration ---
    struct pi_uart_conf conf;
    struct pi_device device;
    pi_uart_conf_init(&conf);
    conf.baudrate_bps = 115200;

    // --- GPIO (LED) Configuration ---
    pi_gpio_pin_configure(&gpio_device, 2, PI_GPIO_OUTPUT);

    // --- Open UART ---
    pi_open_from_conf(&device, &conf);
    if (pi_uart_open(&device))
    {
        printf("[UART] open failed !\n");
        pmsis_exit(-1);
    }

    value = 0xff;

    while(1)
    {
        // Toggle the LED
        pi_gpio_pin_write(&gpio_device, 2, led_val);
        led_val ^= 1;

        // Write the value to uart
        pi_uart_write(&device, &value, 1);

        // Use the modern, simple delay function for 1 second
        pi_time_wait_us(1000000);
    }

    pmsis_exit(0);
}

int main(void)
{
    return pmsis_kickoff((void *)test_gap8);
}
