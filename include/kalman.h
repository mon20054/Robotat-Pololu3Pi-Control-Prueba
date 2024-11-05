/**
 * @file kalman.h
 * @author Javier Monzón (mon20054@uvg.edu.gt)
 * @brief This functions were created using Matlab and Matlab Coder Toolbox
 * @version 0.1
 * @date 2024
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#ifndef KALMAN_H
#define KALMAN_H

// ================================================================================
// Dependencies
// ================================================================================
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif
// ================================================================================
// Public function declarations
// ================================================================================
/**
 * @brief Makes the state prediction of the Kalman Filter
 * 
 * @param[in] drho          Delta Rho
 * @param[in] dtheta        Delta Theta
 * @param[in] theta_post    Post value of Theta for the prediction
 * @param[in] x_post        Post value of X for the prediction
 * @param[in] y_post        Post value of Y for the prediction
 * @param[in] prediction    Array of size 3 to store the state variables of the prediction
 * 
 * @returns Nothing, but the prediction is stored in the corresponding array
 */
extern void 
state_prediction(double drho, double dtheta, double theta_post,
                             double x_post, double y_post,
                             volatile double * prediction);

/**
 * @brief Makes the variance prediction of the Kalman Filter
 * 
 * @param[in] drho          Delta Rho
 * @param[in] p11_post      11 position of P matrix
 * @param[in] p12_post      12 position of P matrix
 * @param[in] p13_post      13 position of P matrix
 * @param[in] p22_post      22 position of P matrix
 * @param[in] p23_post      23 position of P matrix
 * @param[in] p33_post      33 position of P matrix
 * @param[in] sigma_drho    Delta Rho odometry variance
 * @param[in] sigma_dtheta  Delta Theta odometry variance
 * @param[in] theta_post    Post value of Theta
 * @param[in] P_prediction  Array of size 9 to store the P matriz
 * 
 * @returns Nothing, but the prediction is stored in the corresponding array
 */
extern void 
var_prediction(double drho, double p11_post, double p12_post,
                           double p13_post, double p22_post, double p23_post,
                           double p33_post, double sigma_drho,
                           double sigma_dtheta, double theta_post,
                           volatile double * P_prediction);

/**
 * @brief Makes the state correction of the Kalman Filter
 * 
 * @param[in] p11_prior         Prior value of 11 position of P matrix
 * @param[in] p12_prior         Prior value of 12 position of P matrix
 * @param[in] p13_prior         Prior value of 13 position of P matrix
 * @param[in] p22_prior         Prior value of 22 position of P matrix
 * @param[in] p23_prior         Prior value of 23 position of P matrix
 * @param[in] p33_prior         Prior value of 33 position of P matrix
 * @param[in] sigma_theta       Theta OptiTrack variance
 * @param[in] sigma_x           X OptiTriack variance
 * @param[in] sigma_y           Y OptiTriack variance
 * @param[in] thetaOT           Theta OptiTrack value
 * @param[in] theta_prior       Prior value of Theta
 * @param[in] xOT               X OptiTrack value
 * @param[in] x_prior           Prior value of X
 * @param[in] yOT               Y OptiTrack value
 * @param[in] y_prior           Prior value of Y
 * @param[in] correction        Array of size 3 to store the state variables of the correction 
 * 
 * @returns Nothing, but the correction is stored in the corresponding array
 */
extern void 
state_correction(double p11_prior, double p12_prior,
                             double p13_prior, double p22_prior,
                             double p23_prior, double p33_prior,
                             double sigma_theta, double sigma_x, double sigma_y,
                             double thetaOT, double theta_prior, double xOT,
                             double x_prior, double yOT, double y_prior,
                             volatile double * correction);

/**
 * @brief Makes the variance correction of the Kalman Filter
 * 
 * @param[in] p11_prior         Prior value of 11 position of P matrix
 * @param[in] p12_prior         Prior value of 12 position of P matrix
 * @param[in] p13_prior         Prior value of 13 position of P matrix
 * @param[in] p22_prior         Prior value of 22 position of P matrix
 * @param[in] p23_prior         Prior value of 23 position of P matrix
 * @param[in] p33_prior         Prior value of 33 position of P matrix
 * @param[in] sigma_theta       Theta variance
 * @param[in] sigma_x           X variance
 * @param[in] sigma_y           Y variance
 * @param[in] P_correction      Array of size 9 to store the values of the variance correction 
 * 
 * @returns Nothing, but the prediction is stored in the corresponding array
 */
extern void 
var_correction(double p11_prior, double p12_prior, double p13_prior,
                           double p22_prior, double p23_prior, double p33_prior,
                           double sigma_theta, double sigma_x, double sigma_y,
                           volatile double * P_correction);

#ifdef __cplusplus
}
#endif

#endif/* KALMAN_H */
