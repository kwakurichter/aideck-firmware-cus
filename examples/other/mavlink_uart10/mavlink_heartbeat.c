#include "pmsis.h"
#include "stdio.h"
#include "ardupilotmega/mavlink.h"

// --- Configuration ---
#define GAP8_SYSTEM_ID 1
#define GAP8_COMPONENT_ID 1
#define RX_BUFFER_SIZE 128 // A small ring buffer for incoming bytes

// --- Global Variables ---
struct pi_device uart_device;
uint8_t send_buffer[MAVLINK_MAX_PACKET_LEN];

// --- Task Handles & Queues ---
// A queue to pass fully parsed MAVLink messages from the RX task to the main task.
QueueHandle_t mavlink_message_queue;

// --- Function Prototypes ---
void send_heartbeat();
void send_mavlink_message(const mavlink_message_t *msg);

/**
 * @brief RECEIVER TASK
 * This task runs continuously in the background. Its only job is to read bytes
 * from the UART, parse them for MAVLink messages, and put any valid messages
 * onto the queue for the main task to process.
 */
void rx_task(void *pvParameters) {
    mavlink_message_t received_msg;
    mavlink_status_t status = {0};
    uint8_t rx_byte;

    while (1) {
        // Block and wait for a single byte to arrive
        pi_uart_read(&uart_device, &rx_byte, 1);

        // Feed the byte into the MAVLink parser
        if (mavlink_parse_char(MAVLINK_COMM_0, rx_byte, &received_msg, &status)) {
            // A complete and valid message was received.
            // Send a copy of it to the message queue for the other task.
            xQueueSend(mavlink_message_queue, &received_msg, (TickType_t)0);
        }
    }
}

/**
 * @brief MAIN LOGIC AND TRANSMITTER TASK
 * This task handles the main application logic. It sends a heartbeat periodically
 * and checks the queue for any incoming messages that need a response.
 */
void main_task(void *pvParameters) {
    printf("Main task started. Sending initial heartbeat...\n");

    // Give the system a moment to stabilize before starting
    pi_time_wait_us(1000 * 1000);
    send_heartbeat();

    mavlink_message_t incoming_msg;

    while (1) {
        // Check the queue for a new message.
        // This is a non-blocking check. It returns immediately if the queue is empty.
        if (xQueueReceive(mavlink_message_queue, &incoming_msg, (TickType_t)0) == pdPASS) {
            // A message was received from the rx_task
            if (incoming_msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                printf("<- Received HEARTBEAT from host (SysID: %d)\n", incoming_msg.sysid);
                // Respond with our own heartbeat
                send_heartbeat();
            }
        }

        // We can add logic here to send our own heartbeats periodically,
        // independent of what we receive. For this test, we'll just respond.

        // Let other tasks run
        pi_time_wait_us(10 * 1000); // 10ms delay
    }
}


void start_mavlink_system(void) {
    printf("Entering main controller (Multi-Task MAVLink)...\n");

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

    // Create the message queue. It can hold up to 5 MAVLink messages.
    mavlink_message_queue = xQueueCreate(5, sizeof(mavlink_message_t));
    if (mavlink_message_queue == NULL) {
        printf("Failed to create message queue!\n");
        pmsis_exit(-1);
    }

    printf("Creating tasks...\n");

    // Create the receiver and transmitter tasks.
    // NOTE: We are using PULP-OS tasks here. If using FreeRTOS, you would use xTaskCreate.
    // The function names and parameters might differ slightly.
    pi_task_t rx_task_handle;
    pi_task_block(&rx_task_handle);
    pi_task_callback(&rx_task_handle, rx_task, NULL);
    pi_task_push(&rx_task_handle);


    pi_task_t main_task_handle;
    pi_task_block(&main_task_handle);
    pi_task_callback(&main_task_handle, main_task, NULL);
    pi_task_push(&main_task_handle);


    // The scheduler will now run the tasks. We can loop forever here.
    while(1) {
       pi_yield();
    }

    pmsis_exit(0);
}

int main(void) {
    // NOTE: This example is structured for PULP-OS.
    // If you are using FreeRTOS, the task creation and scheduler start
    // would be slightly different (using xTaskCreate and vTaskStartScheduler).
    printf("\n\n\t *** PMSIS Multi-Task MAVLink Test ***\n\n");
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
    hb.autopilot = MAV_AUTOPILOT_GENERIC;
    hb.system_status = MAV_STATE_ACTIVE;
    mavlink_msg_heartbeat_encode(GAP8_SYSTEM_ID, GAP8_COMPONENT_ID, &msg, &hb);
    send_mavlink_message(&msg);
    printf("-> Sent HEARTBEAT to host.\n");
}
