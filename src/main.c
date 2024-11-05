#include <stdio.h>               // For basic input/output
#include <stdlib.h>              // For general utilities
#include "freertos/FreeRTOS.h"   // For FreeRTOS functionality
#include "freertos/task.h"       // For tasks in FreeRTOS
#include "esp_system.h"          // For ESP32 system functionalities
#include "esp_timer.h"           // For timing functionalities
#include "esp_event.h"           // For event handling
#include "esp_log.h"             // For logging
#include "nvs_flash.h"           // For non-volatile storage
#include "cbor.h"                // For CBOR encoding/decoding
#include "driver/gpio.h"         // For GPIO control
#include "driver/uart.h"         // For UART control

// Custom Libraries
#include "eztcp.h"               // For TCP functionalities
#include "ezudp.h"               // For UDP functionalities
#include "ezwifi.h"              // For WiFi functionalities
#include "robotat_util.h"        // For common functionalities (e.g., init functions)
#include "kalman.h"              // For Kalman filter implementation
#include "robotat.h"             // For Robotat-specific functionalities
#include "config.h"              // For static configuration parameters

#define DEBUG_PRINT

#define TXD_PIN             (GPIO_NUM_17)
#define RXD_PIN             (GPIO_NUM_16)
#define RX_BUF_SIZE         (1024)
#define CBOR_BUF_SIZE       (32)
#define CBOR_MSG_MIN_SIZE   (16)

// Variable for turning on/off the PID controller
bool b_is_pid = true;

// Time based data obtention
static const unsigned int control_time_ms = 100;
static const unsigned int odoread_time_ms = 100;

uint8_t cbor_buf_tx[CBOR_BUF_SIZE] = {0};

float xi_odo[3];  // Robot's odometry pose
float phi_ell = 0; // Left wheel speed in rpm
float phi_r = 0;   // Right wheel speed in rpm

// PID controller parameters
volatile double xpos = 0, ypos = 0, phi = 0;
volatile double eO = 0, eO_1 = 0, EO = 0;
const double alpha = 50, kpO = 2, kiO = 0.0001, kdO = -1, vMax = 0.5;
//volatile float xg = 1.54, yg = 1.38;  // Desired initial coordinates
//volatile float xg = -0.47, yg = 0.90;  // Desired initial coordinates first route
//volatile float xg = 0.51, yg = -0.09;  // Desired initial coordinates second route
volatile float xg = -0.49, yg = 2.28;  // Desired initial coordinates third route
float uRef = 0.0; 
float wRef = 0.0; 

// Robotat pose
float X_rob = 0.0;
float Y_rob = 0.0;
float Theta_rob = 0.0;
float X_rob_prev = 0.0, Y_rob_prev = 0.0, Theta_rob_prev = 0.0;

CborEncoder encoder, array_encoder;

// EKF variables
volatile double xi_prior[3] = {0};
volatile double xi_post[3] = {0};
volatile double P_prior[9] = {0};
volatile double P_post[9] = {0};

static const float sigma_e = 1;

volatile float drho, dtheta;
static const float sigma_drho = 0.0189375770;
static const float sigma_dtheta = 0.0000105891;

volatile float x_ot, y_ot, theta_ot, theta;
static const float sigma_x = 0.0000000098;
static const float sigma_y = 0.0000000026;
static const float sigma_theta = 0.0162853000;

volatile bool b_did_correction = false;

void 
init(void) 
{
    // Initialize WiFi with static IP
    wifi_init_static(ssid, password, static_ip, static_gw, static_netmask);

    // Initialize UART
    uart_init(115200, TXD_PIN, RXD_PIN, RX_BUF_SIZE);

    // Kalman filter initial state
    P_prior[0] = sigma_e;
    P_prior[3] = sigma_e;
    P_prior[6] = sigma_e;
}

void 
uart_tx_task(void * p_params) 
{
    TickType_t last_control_time;
    const TickType_t control_freq_ticks = pdMS_TO_TICKS(control_time_ms);
    size_t numbytes;

    last_control_time = xTaskGetTickCount();

    while(1) 
    {
        vTaskDelayUntil(&last_control_time, control_freq_ticks);
        // Encode wheel speeds CBOR
        cbor_encoder_init(&encoder, cbor_buf_tx, sizeof(cbor_buf_tx), 0);
        cbor_encoder_create_array(&encoder, &array_encoder, 2);
        cbor_encode_float(&array_encoder, phi_ell);
        cbor_encode_float(&array_encoder, phi_r);
        cbor_encoder_close_container(&encoder, &array_encoder);
        numbytes = cbor_encoder_get_buffer_size(&encoder, cbor_buf_tx);
        uart_write_bytes(UART_NUM_2, cbor_buf_tx, numbytes);
    }
}

void 
robotat_task(void *p_params) 
{
    int robotat_sock = *((int *)p_params);
    TickType_t last_wake_time;
    const TickType_t frequency = pdMS_TO_TICKS(100); // 100 ms

    last_wake_time = xTaskGetTickCount();

    while (1) 
    {
        vTaskDelayUntil(&last_wake_time, frequency);
        float mocap_data[6];
        if (robotat_get_pose(robotat_sock, 125, mocap_data) == 0) 
        {
            #ifdef DEBUG_PRINT
            printf("Pose Data: X=%.4f, Y=%.4f, Z=%.4f, Eul_X=%.4f, Eul_Y=%.4f, Eul_Z=%.4f\n",
            mocap_data[0], mocap_data[1], mocap_data[2], mocap_data[3], mocap_data[4], mocap_data[5]);
            #endif
            x_ot = mocap_data[0];
            y_ot = mocap_data[1];
            theta = mocap_data[5];
            theta_ot = atan2(sin(theta*(M_PI/180.0)),cos(theta*(M_PI/180.0)));
            theta_ot = theta_ot-3.2;
            // Theta odometry [-π, π]
            theta_ot = fmod(theta_ot, 2 * M_PI);
            if (theta_ot > M_PI) 
            {
                theta_ot -= 2 * M_PI;
            }
            else if (theta_ot < -M_PI) 
            {
                theta_ot += 2 * M_PI;
            }

            // EKF correction with MOCAP data
            state_correction(P_prior[0], P_prior[1], P_prior[2], P_prior[4], P_prior[6], P_prior[8], sigma_theta, sigma_x, sigma_y,
                            theta_ot, xi_prior[2], x_ot, xi_prior[0], y_ot, xi_prior[1], &xi_post[0]);
            var_correction(P_prior[0], P_prior[1], P_prior[2], P_prior[4], P_prior[6], P_prior[8], sigma_theta, sigma_x, sigma_y, &P_post[0]);
            if(!b_did_correction)
            {
                b_did_correction = true;
            }

        } 
        else 
        {
            #ifdef DEBUG_PRINT
            printf("Failed to get pose data from Robotat server\n");
            #endif
        }
    }
}   

void 
udp_send_odometry_task(void *p_params) 
{
    TickType_t last_send_time;
    const TickType_t send_freq_ticks = pdMS_TO_TICKS(control_time_ms);

    last_send_time = xTaskGetTickCount();

    // Set up UDP destination
    ezudp_set_destination(udp_server_ip, udp_server_port);

    while (1) 
    {
        vTaskDelayUntil(&last_send_time, send_freq_ticks);
        
        // Prepare the JSON string (custom made for monitoring data in the server)
        char json_string[256];
        snprintf(json_string, sizeof(json_string), 
                 "{\"x\":%.4f,\"y\":%.4f,\"thetaK\":%.4f,\"xg\":%.4f,\"yg\":%.4f}", 
                 xi_post[0], xi_post[1], xi_post[2], xg, yg);

        // Send odometry data using the ezudp function
        if (ezudp_send_json(json_string) < 0) 
        {
            #ifdef DEBUG_PRINT
            printf("Failed to send odometry data over UDP\n");
            #endif
        }
    }
}

void 
uart_rx_task(void * p_params) 
{
    while (1) 
    {
        float xi_prior_copy[3];
        char json_string[200];
        int len = uart_read_bytes(UART_NUM_2, json_string, sizeof(json_string) - 1, odoread_time_ms / portTICK_PERIOD_MS);
        if (len > 0) 
        {
            json_string[len] = 0;
            cJSON* root = cJSON_Parse(json_string);
            if (root) 
            {
                xi_odo[0] = cJSON_GetObjectItem(root, "x")->valuedouble;
                xi_odo[1] = cJSON_GetObjectItem(root, "y")->valuedouble;
                xi_odo[2] = cJSON_GetObjectItem(root, "theta")->valuedouble;
                drho = cJSON_GetObjectItem(root, "drho")->valuedouble;
                dtheta = cJSON_GetObjectItem(root, "dtheta")->valuedouble;
                cJSON_Delete(root);

                xi_odo[2] = atan2(sin(xi_odo[2]),cos(xi_odo[2]));

                // Theta odometry [-pi, pi]
                xi_odo[2] = fmod(xi_odo[2], 2 * M_PI);
                if (xi_odo[2] > M_PI) 
                {
                    xi_odo[2] -= 2 * M_PI;
                } 
                else if (xi_odo[2] < -M_PI) 
                {
                    xi_odo[2] += 2 * M_PI;
                }

                xi_prior[0] = xi_odo[0]/1000.0f;    // X
                xi_prior[1] = xi_odo[1]/1000.0f;    // Y
                xi_prior[2] = xi_odo[2];            // Theta

                // Check if the correction of the EKF was made to chose with which data to do the prediction
                if(b_did_correction)
                {
                    state_prediction(drho, dtheta, xi_post[2], xi_post[0], xi_post[1], &xi_prior[0]);
                    var_prediction(drho, P_post[0], P_post[1], P_post[2], P_post[4], P_post[6], P_post[8], sigma_drho, sigma_dtheta, xi_post[2], &P_prior[0]);
                    b_did_correction = false;
                }
                else
                {
                    xi_prior_copy[0] = xi_prior[0];
                    xi_prior_copy[1] = xi_prior[1];
                    xi_prior_copy[2] = xi_prior[2];
                    state_prediction(drho, dtheta, xi_prior_copy[2], xi_prior_copy[0], xi_prior_copy[1], &xi_prior[0]);
                    var_prediction(drho, P_post[0], P_post[1], P_post[2], P_post[4], P_post[6], P_post[8], sigma_drho, sigma_dtheta, xi_post[2], &P_prior[0]);
                }

                // Run the PID controller if selected
                if (b_is_pid)
                {
                    pid_control(xg, yg, xi_post, &eO, &eO_1, &EO, &wRef, &uRef, &phi_ell, &phi_r);
                }
            }
        }
    }
}

void 
tcp_receive_task(void *p_params) 
{
    char rx_buffer[128];
    int server_sock = eztcp_begin(5005);  // Initialize TCP server on port 5005
    if (server_sock < 0) 
    {
        vTaskDelete(NULL);  // Stop the task if server initialization fails
    }

    int client_sock = eztcp_accept(server_sock);  // Wait for a client connection
    if (client_sock < 0) 
    {
        eztcp_close(-1, server_sock);  // Close server socket if accepting fails
        vTaskDelete(NULL);
    }

    while (1) 
    {
        ssize_t len = eztcp_receive(client_sock, rx_buffer, sizeof(rx_buffer));  // Receive data
        if (len < 0) 
        {
            break;  // Exit on receive error
        } else if (len == sizeof(float) * 2) // Check for expected data size 
        {  
            float *data_received = (float *)rx_buffer;
            xg = (double)data_received[0];
            yg = (double)data_received[1];
        } 
        else 
        {
            #ifdef DEBUG_PRINT
            printf("Received data length mismatch: expected %d bytes, but got %d bytes.\n", (int)(sizeof(float) * 2), (int)len);
            #endif
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);  // Delay to control task frequency
    }

    eztcp_close(client_sock, server_sock);  // Close sockets
    vTaskDelete(NULL);
}

void 
app_main() 
{
    init();

    // Small delay to ensure network is stable
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    // Connect to Robotat server
    int robotat_sock = robotat_connect();
    if (robotat_sock < 0) 
    {
        #ifdef DEBUG_PRINT
        printf("Failed to connect to Robotat server at startup\n");
        #endif
        return; // Stop execution if connection fails
    }

    // Start tasks
    xTaskCreate(uart_tx_task, "uart_tx_task", 2048, NULL, 20, NULL);
    xTaskCreate(uart_rx_task, "uart_rx_task", 4096, &robotat_sock, 19, NULL); 
    xTaskCreate(robotat_task, "robotat_task", 4096, &robotat_sock, 18, NULL);
    xTaskCreate(tcp_receive_task, "tcp_receive_task", 2048, NULL, 17, NULL);
    xTaskCreate(udp_send_odometry_task, "udp_send_odometry_task", 2048, NULL, 16, NULL);
}