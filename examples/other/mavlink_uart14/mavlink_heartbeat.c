#include "pmsis.h"
#include "stdio.h"
#include "ardupilotmega/mavlink.h"
#include "bsp/bsp.h"

#define LED_PIN (2)
static pi_device_t led_gpio_dev;

// --- Configuration ---
#define GAP8_SYSTEM_ID 1
#define GAP8_COMPONENT_ID 1

// --- FreeRTOS Task Configuration ---
#define RX_TASK_STACK_SIZE      (configMINIMAL_STACK_SIZE * 2)
#define LED_TASK_STACK_SIZE     (configMINIMAL_STACK_SIZE * 2)
#define MAIN_TASK_STACK_SIZE    (configMINIMAL_STACK_SIZE * 2)
#define RX_TASK_PRIORITY        (tskIDLE_PRIORITY + 2)
#define LED_TASK_PRIORITY       (tskIDLE_PRIORITY + 2)
#define MAIN_TASK_PRIORITY      (tskIDLE_PRIORITY + 1)


// --- Global Variables ---
struct pi_device uart_device;
uint8_t send_buffer[MAVLINK_MAX_PACKET_LEN];
QueueHandle_t mavlink_message_queue;

// --- Function Prototypes ---
void send_heartbeat();
void send_mavlink_message(const mavlink_message_t *msg);
void rx_task(void *pvParameters);
void main_task(void *pvParameters);
void led_debug_task(void *parameters);


/**
 * @brief RECEIVER TASK (FreeRTOS)
 * Reads bytes from UART and places complete MAVLink messages on the queue.
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
 * @brief Flash LED.
 */
void led_debug_task(void *parameters) {
    while(1) {
        pi_gpio_pin_write(&led_gpio_dev, LED_PIN, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        pi_gpio_pin_write(&led_gpio_dev, LED_PIN, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

/**
 * @brief MAIN LOGIC AND TRANSMITTER TASK (FreeRTOS)
 * Sends periodic heartbeats and processes incoming messages from the queue.
 */
void main_task(void *pvParameters) {
    printf("Main task started. Sending initial heartbeat...\n");
    vTaskDelay(pdMS_TO_TICKS(1000)); // FreeRTOS delay for 1 second
    send_heartbeat();

    mavlink_message_t incoming_msg;
    TickType_t last_heartbeat_sent = xTaskGetTickCount();

    while (1) {
        // Check for an incoming message from the queue (non-blocking)
        if (xQueueReceive(mavlink_message_queue, &incoming_msg, (TickType_t)0) == pdPASS) {
            if (incoming_msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                printf("<- Received HEARTBEAT from host (SysID: %d)\n", incoming_msg.sysid);
                send_heartbeat(); // Respond immediately
                last_heartbeat_sent = xTaskGetTickCount(); // Reset timer after responding
            }
        }

        // Also send a heartbeat every 1 second, regardless of received messages
        if ((xTaskGetTickCount() - last_heartbeat_sent) > pdMS_TO_TICKS(1000)) {
             send_heartbeat();
             last_heartbeat_sent = xTaskGetTickCount();
        }

        // Yield for a short time to allow other tasks to run
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


void start_mavlink_system(void) {
    printf("Entering main controller (FreeRTOS Multi-Task MAVLink)...\n");

    struct pi_uart_conf conf;
    pi_uart_conf_init(&conf);
    conf.enable_tx = 1;
    conf.enable_rx = 1;
    conf.baudrate_bps = 115200;

    pi_open_from_conf(&uart_device, &conf);
    if (pi_uart_open(&uart_device)) {
        printf("Uart open failed!\n");
        pmsis_exit(-1);
    }
    printf("UART opened successfully.\n");

    mavlink_message_queue = xQueueCreate(5, sizeof(mavlink_message_t));
    if (mavlink_message_queue == NULL) {
        printf("Failed to create message queue!\n");
        pmsis_exit(-1);
    }

    printf("Creating FreeRTOS tasks...\n");

    // Use xTaskCreate for FreeRTOS
    BaseType_t xTask;
    xTask = xTaskCreate(led_debug_task, "led_debug_task", LED_TASK_STACK_SIZE, NULL, LED_TASK_PRIORITY, NULL);
    if (xTask != pdPASS) {
        printf("Failed to create LED task!\n");
        pmsis_exit(-1);
    }

    xTask = xTaskCreate(rx_task, "RxTask", RX_TASK_STACK_SIZE, NULL, RX_TASK_PRIORITY, NULL);
    if (xTask != pdPASS) {
        printf("Failed to create RX task!\n");
        pmsis_exit(-1);
    }

    xTask = xTaskCreate(main_task, "MainTask", MAIN_TASK_STACK_SIZE, NULL, MAIN_TASK_PRIORITY, NULL);
    if (xTask != pdPASS) {
        printf("Failed to create Main task!\n");
        pmsis_exit(-1);
    }

    while(1) {
        pi_yield();
    }   
}

int main(void) {
    printf("\n\n\t *** PMSIS FreeRTOS MAVLink Test ***\n\n");

    pi_bsp_init();
    pi_gpio_pin_configure(&led_gpio_dev, LED_PIN, PI_GPIO_OUTPUT);
    pi_freq_set(PI_FREQ_DOMAIN_FC, 250000000);
    __pi_pmu_voltage_set(PI_PMU_DOMAIN_FC, 1200);

    // 3. Kick off the application, which will start the FreeRTOS scheduler.
    return pmsis_kickoff((void *) start_mavlink_system);
}

// --- Helper Functions (unchanged) ---
void send_mavlink_message(const mavlink_message_t *msg) {
    uint16_t len = mavlink_msg_to_send_buffer(send_buffer, msg);
    pi_uart_write(&uart_device, send_buffer, len);
}

void send_heartbeat() {
    mavlink_message_t msg;
    mavlink_heartbeat_t hb = {0};
    hb.type = MAV_TYPE_ONBOARD_CONTROLLER;
    hb.autopilot = MAV_AUTOPILOT_GENERIC;
    hb.system_status = MAV_STATE_ACTIVE;
    mavlink_msg_heartbeat_encode(GAP8_SYSTEM_ID, GAP8_COMPONENT_ID, &msg, &hb);
    send_mavlink_message(&msg);
    printf("-> Sent HEARTBEAT to host.\n");
}
