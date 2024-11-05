/**
 * @file robotat.h
 * @author Javier Monzón (mon20054@uvg.edu.gt)
 * @brief Robotat server functionalities
 * @version 0.1
 * @date 2024
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef ROBOTAT_H
#define ROBOTAT_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <cJSON.h>
#include "robotat_util.h"

#ifdef __cplusplus
extern "C" {
#endif

// ================================================================================
// External variables
// ================================================================================

extern const char *robotat_server_ip;
extern const int robotat_server_port;

// ================================================================================
// Function declarations
// ================================================================================

/**
 * @brief Connects to the Robotat server.
 *
 * @return  The socket descriptor on success, -1 on failure.
 */
int 
robotat_connect(void);

/**
 * @brief Retrieves the pose of an agent from the Robotat server.
 *
 * @param sock          Socket descriptor connected to the Robotat server.
 * @param agent_id      ID of the agent whose pose is requested.
 * @param mocap_data    Array to store the retrieved pose and orientation data.
 * @return              0 on success, -1 on failure.
 */
int 
robotat_get_pose(int sock, int agent_id, float *mocap_data);

#ifdef __cplusplus
}
#endif

#endif /* ROBOTAT_H */
