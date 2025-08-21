#include "pmsis.h"
#include "stdio.h"

static int led_val = 0;
static struct pi_device gpio_device;

static void test_gap8(void)
{
    printf("Entering main controller...\n");

    // --- UART Configuration ---
    struct pi_uart_conf conf;
    struct pi_device uart_device;
    pi_uart_conf_init(&conf);
    conf.baudrate_bps = 115200; // Standard MAVLink baud rate

    // --- GPIO (LED) Configuration ---
    pi_gpio_pin_configure(&gpio_device, 2, PI_GPIO_OUTPUT);

    // --- Open UART ---
    pi_open_from_conf(&uart_device, &conf);
    if (pi_uart_open(&uart_device))
    {
        printf("[UART] open failed !\n");
        pmsis_exit(-1);
    }

    // --- Manually packed MAVLink HEARTBEAT message ---
    // This is a sample packet. In a real application, you would generate
    // the sequence number and checksum dynamically.
    // Packet structure:
    // [Start Byte, Payload Length, Sequence, System ID, Component ID, Msg ID, Payload..., Checksum A, Checksum B]
    uint8_t mavlink_heartbeat[] = {
        0xFE, // Start of frame
        0x09, // Payload length (9 bytes for heartbeat)
        0x00, // Packet sequence
        0x01, // System ID (1 for a typical GCS/MAV)
        0x01, // Component ID
        0x00, // Message ID (0 for HEARTBEAT)
        // --- Payload (9 bytes) ---
        0x00, 0x00, 0x00, 0x00, // custom_mode
        0x13, // type (MAV_TYPE_QUADROTOR)
        0x08, // autopilot (MAV_AUTOPILOT_ARDUPILOTMEGA)
        0x03, // base_mode
        0x04, // system_status
        0x03, // mavlink_version
        // --- Checksum (2 bytes) ---
        0x4B, // Checksum A
        0x17  // Checksum B
    };


    uint8_t test_char = 'U'; // 0x55

    while(1)
    {
        // Toggle the LED to show activity
        pi_gpio_pin_write(&gpio_device, 2, led_val);
        led_val ^= 1;

        // Write the test character
        pi_uart_write(&uart_device, &test_char, 1);

        // Wait for 1 second
        pi_time_wait_us(1000000);
    }

    pmsis_exit(0);
}

int main(void)
{
    pi_freq_set(PI_FREQ_DOMAIN_FC, 250000000);
    
    return pmsis_kickoff((void *)test_gap8);
}