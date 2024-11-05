/**
 * @file functionalities.c
 * @author Javier Monzón (mon20054@uvg.edu.gt)
 * @brief Library for extra needeed functionalities. 
 * @version 0.1
 * @date 2024
 *
 * @copyright Copyright (c) 2024
 *
 */

// ================================================================================
// Dependencies
// ================================================================================
#include "robotat_util.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <esp_netif.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include <assert.h>

// ================================================================================
// Public function definitions
// ================================================================================

double
radians_to_degrees(double radians)
{
    return radians * (180.0 / M_PI);
}

void
quaternion_to_euler_xyz(const float q[4], float euler[3])
{
    const float unity_norm_tol = 0.1f;
    const float angcomp_tol = 0.1f * (M_PI / 180.0f);

    float norm_q = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (fabs(norm_q - 1.0f) > unity_norm_tol)
    {
        #ifdef DEBUG_PRINT
        printf("ERROR: Input is NOT a unit quaternion\n");
        #endif
        return;
    }

    float t0 = q[3] - q[1];
    float t1 = q[0] - q[2];
    float t2 = q[1] + q[3];
    float t3 = q[2] + q[0];

    euler[2] = atan2f(t0, t1) + atan2f(t2, t3);
    euler[1] = acosf(t1 * t1 + t0 * t0 - 1.0f) - M_PI / 2.0f;
    euler[0] = -(atan2f(t0, t1) - atan2f(t2, t3));

    euler[0] = radians_to_degrees(euler[0]);
    euler[1] = radians_to_degrees(euler[1]);
    euler[2] = radians_to_degrees(euler[2]);

    if (fabs(fabs(euler[1]) - M_PI / 2.0f) < angcomp_tol)
    {
        #ifdef DEBUG_PRINT
        printf("WARNING: Euler angle sequence is in a singularity.\n");
        #endif
        euler[2] = euler[2] + copysignf(1.0f, euler[1]) * euler[0];
        euler[0] = 0.0f;
    }
}

void
clamp_rpm(float *rpm)
{
    if (*rpm > 850.0f)
    {
        *rpm = 850.0f;
    }
    else if (*rpm < -850.0f)
    {
        *rpm = -850.0f;
    }
}

void 
uart_init(int baud_rate, int tx_pin, int rx_pin, int rx_buffer_size) 
{
    const uart_config_t uart_config = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    uart_driver_install(UART_NUM_2, rx_buffer_size, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_2, &uart_config);
    uart_set_pin(UART_NUM_2, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

void 
pid_control(double xg, double yg, const volatile double xi_post[3], 
                 volatile double *eO, volatile double *eO_1, volatile double *EO, 
                 volatile float *wRef, volatile float *uRef, 
                 float *phi_ell, float *phi_r) 
{
    extern const double alpha, kpO, kiO, kdO, vMax;  // Constants defined in main.c

    // Compute the position error
    double e[2] = {xg - xi_post[0], yg - xi_post[1]};
    double thetag = atan2(e[1], e[0]);  // Calculate target angle
    double eP = sqrt(e[0] * e[0] + e[1] * e[1]);

    // Compute orientation error
    *eO = thetag - xi_post[2];
    *eO = atan2(sin(*eO), cos(*eO));  // Normalize to [-pi, pi]

    // Compute proportional gain for linear velocity
    double kP = vMax * (1 - exp(-alpha * eP * eP)) / eP;
    double v = kP * eP;

    // Compute PID terms for angular velocity
    double eO_D = *eO - *eO_1;
    *EO += *eO;
    double w = kpO * (*eO) + kiO * (*EO) + kdO * eO_D;
    *eO_1 = *eO;

    // Set output variables for reference velocities
    *wRef = (float)w;
    *uRef = (float)v;

    // Compute wheel speeds in RPM
    *phi_ell = (*uRef - 39.5f / 1000.0f * *wRef) / (16.0f / 1000.0f) * 60.0f / (2.0f * M_PI);
    *phi_r = (*uRef + 39.5f / 1000.0f * *wRef) / (16.0f / 1000.0f) * 60.0f / (2.0f * M_PI);

    // Clamp the wheel speeds
    clamp_rpm(phi_ell);
    clamp_rpm(phi_r);

    // Stop wheels if the position error is small
    if (eP < 0.1) 
    {
        *phi_ell = 0;
        *phi_r = 0;
    }
}