/**
 * @file functionalities.h
 * @author Javier Monzón (mon20054@uvg.edu.gt)
 * @brief Conversion and protection functionalities
 * @version 0.1
 * @date 2024
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef FUNCTIONALITIES_H
#define FUNCTIONALITIES_H

#include <math.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ================================================================================
// Function declarations
// ================================================================================

/**
 * @brief Converts radians to degrees.
 *
 * @param radians   Angle in radians.
 * @return          Angle in degrees.
 */
double 
radians_to_degrees(double radians);

/**
 * @brief Converts a quaternion to Euler angles using XYZ sequence.
 *
 * @param[in] q         Quaternion input as an array of size 4.
 * @param[out] euler    Euler angles output as an array of size 3.
 */
void 
quaternion_to_euler_xyz(const float q[4], float euler[3]);

/**
 * @brief Clamps RPM to a specified range.
 *
 * @param[in, out] rpm  Pointer to the RPM value to clamp.
 */
void 
clamp_rpm(float *rpm);

/**
 * @brief Initializes UART with the specified configuration.
 * 
 * @param baud_rate         UART baud rate.
 * @param tx_pin            Transmit (TX) pin number.
 * @param rx_pin            Receive (RX) pin number.
 * @param rx_buffer_size    Size of the UART RX buffer.
 */
void 
uart_init(int baud_rate, int tx_pin, int rx_pin, int rx_buffer_size);

/**
 * @brief Executes the PID control logic for the robot's movement.
 * 
 * @param xg        Target X coordinate.
 * @param yg        Target Y coordinate.
 * @param xi_post   Array of size 3 representing the current position and orientation (X, Y, Theta output form EKF).
 * @param eO        Pointer to error in orientation.
 * @param eO_1      Pointer to previous orientation error.
 * @param EO        Pointer to accumulated orientation error (integral).
 * @param wRef      Pointer to angular velocity reference (output).
 * @param uRef      Pointer to linear velocity reference (output).
 * @param phi_ell   Pointer to left wheel speed (output).
 * @param phi_r     Pointer to right wheel speed (output).
 */
void 
pid_control(double xg, double yg, const volatile double xi_post[3], 
                 volatile double *eO, volatile double *eO_1, volatile double *EO, 
                 volatile float *wRef, volatile float *uRef, 
                 float *phi_ell, float *phi_r);

#ifdef __cplusplus
}
#endif

#endif /* FUNCTIONALITIES_H */
