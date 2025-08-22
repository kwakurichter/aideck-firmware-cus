#include "pmsis.h"
#include "stdio.h"
#include "bsp/bsp.h" // Needed for pi_bsp_init()
#include "ardupilotmega/mavlink.h" // Assuming ardupilotmega/mavlink.h is in your include path

// --- Configuration ---
#define GAP8_SYSTEM_ID 1
#define GAP8_COMPONENT_ID 1
#define LED_PIN (2)

// --- FreeRTOS Task Configuration ---
#define RX_TASK_STACK_SIZE      (configMINIMAL_STACK_SIZE * 2)
#define MAIN_TASK_STACK_SIZE    (configMINIMAL_STACK_SIZE * 2)
#define LED_TASK_STACK_SIZE     (configMINIMAL_STACK_SIZE)
#define RX_TASK_PRIORITY        (tskIDLE_PRIORITY + 2)
#define MAIN_TASK_PRIORITY      (tskIDLE_PRIORITY + 1)
#define LED_TASK_PRIORITY       (tskIDLE_PRIORITY + 1)

// --- Global Variables ---
struct pi_device uart_device;
struct pi_device led_gpio_dev;
uint8_t send_buffer[MAVLINK_MAX_PACKET_LEN];
QueueHandle_t mavlink_message_queue;

// --- Function Prototypes ---
void send_heartbeat();
void send_mavlink_message(const mavlink_message_t *msg);
void rx_task(void *pvParameters);
void main_task(void *pvParameters);
void led_task(void *pvParameters);

/**
 * @brief Blinks the on-board LED to show the system is running.
 */
void led_task(void *pvParameters) {
    while(1) {
        pi_gpio_pin_write(&led_gpio_dev, LED_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(500));
        pi_gpio_pin_write(&led_gpio_dev, LED_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/**
 * @brief RECEIVER TASK: Reads bytes from UART and queues complete MAVLink messages.
 */
void rx_task(void *pvParameters) {
    mavlink_message_t received_msg;
    mavlink_status_t status = {0};
    uint8_t rx_byte;
    while (1) {
        pi_uart_read(&uart_device, &rx_byte, 1);
        if (mavlink_parse_char(MAVLINK_COMM_0, rx_byte, &received_msg, &status)) {
            xQueueSend(mavlink_message_queue, &received_msg, (TickType_t)0);
        }
    }
}

/**
 * @brief MAIN LOGIC TASK: Processes incoming messages and sends periodic heartbeats.
 */
void main_task(void *pvParameters) {
    printf("Main task started.\n");
    mavlink_message_t incoming_msg;
    TickType_t last_heartbeat_sent = xTaskGetTickCount();

    while (1) {
        // Check for an incoming message (non-blocking, wait up to 10ms)
        if (xQueueReceive(mavlink_message_queue, &incoming_msg, pdMS_TO_TICKS(10)) == pdPASS) {
            if (incoming_msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                printf("<- Received HEARTBEAT from host (SysID: %d)\n", incoming_msg.sysid);
            }
        }

        // Send a heartbeat every 1 second, regardless of received messages
        if ((xTaskGetTickCount() - last_heartbeat_sent) > pdMS_TO_TICKS(1000)) {
             send_heartbeat();
             last_heartbeat_sent = xTaskGetTickCount();
        }
    }
}

/**
 * @brief Main application setup function.
 */
void start_mavlink_system(void) {
    printf("Entering main controller (FreeRTOS Multi-Task MAVLink)...\n");

    // --- Peripheral and Queue Setup ---
    struct pi_uart_conf conf;
    pi_uart_conf_init(&conf);
    conf.enable_tx = 1;
    conf.enable_rx = 1;
    conf.baudrate_bps = 115200;
    pi_open_from_conf(&uart_device, &conf);
    if (pi_uart_open(&uart_device)) {
        printf("Uart open failed!\n"); pmsis_exit(-1);
    }
    pi_gpio_pin_configure(&led_gpio_dev, LED_PIN, PI_GPIO_OUTPUT);
    mavlink_message_queue = xQueueCreate(5, sizeof(mavlink_message_t));
    if (mavlink_message_queue == NULL) {
        printf("Failed to create message queue!\n"); pmsis_exit(-1);
    }

    printf("Creating FreeRTOS tasks...\n");

    // --- Task Creation ---
    BaseType_t xTask;
    xTask = xTaskCreate(led_task, "LedTask", LED_TASK_STACK_SIZE, NULL, LED_TASK_PRIORITY, NULL);
    if (xTask != pdPASS) { printf("Failed to create LED task!\n"); pmsis_exit(-1); }

    xTask = xTaskCreate(rx_task, "RxTask", RX_TASK_STACK_SIZE, NULL, RX_TASK_PRIORITY, NULL);
    if (xTask != pdPASS) { printf("Failed to create RX task!\n"); pmsis_exit(-1); }

    xTask = xTaskCreate(main_task, "MainTask", MAIN_TASK_STACK_SIZE, NULL, MAIN_TASK_PRIORITY, NULL);
    if (xTask != pdPASS) { printf("Failed to create Main task!\n"); pmsis_exit(-1); }

    printf("Tasks created. Scheduler will now start.\n");
    // This function now returns, allowing pmsis_kickoff to start the scheduler.
}

int main(void) {
    printf("\n\t *** PMSIS FreeRTOS MAVLink Test ***\n\n");

    // --- CRITICAL: Full hardware initialization before starting the system ---
    pi_freq_set(PI_FREQ_DOMAIN_FC, 250000000);
    pi_bsp_init();

    // Kick off the application. This will call start_mavlink_system,
    // and once it returns, it will start the FreeRTOS scheduler.
    return pmsis_kickoff((void *) start_mavlink_system);
}

// --- Helper Functions ---
void send_mavlink_message(const mavlink_message_t *msg) {
    uint16_t len = mavlink_msg_to_send_buffer(send_buffer, msg);
    pi_uart_write(&uart_device, send_buffer, len);
}

void send_heartbeat() {
    mavlink_message_t msg;
    mavlink_heartbeat_t hb = {0};
    hb.type = MAV_TYPE_ONBOARD_CONTROLLER;
    hb.autopilot = MAV_AUTOPILOT_ARDUPILOTMEGA;
    hb.system_status = MAV_STATE_ACTIVE;
    mavlink_msg_heartbeat_encode(GAP8_SYSTEM_ID, GAP8_COMPONENT_ID, &msg, &hb);
    send_mavlink_message(&msg);
    printf("-> Sent HEARTBEAT to host.\n");
}