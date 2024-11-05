/**
 * @file functionalities.c
 * @author Javier Monzón (mon20054@uvg.edu.gt)
 * @brief Implementation of functions to connect and retrieve pose data from the Robotat server.
 * @version 0.1
 * @date 2024
 *
 * @copyright Copyright (c) 2024
 *
 */

// ================================================================================
// Dependencies
// ================================================================================
#include "robotat.h"


// ================================================================================
// Public function definitions
// ================================================================================

const char *robotat_server_ip = "192.168.50.200";
const int robotat_server_port = 1883;

int
robotat_connect(void)
{
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0)
    {
        #ifdef DEBUG_PRINT
        printf("ERROR: Unable to create socket: %d\n", errno);
        #endif
        return -1;
    }

    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = inet_addr(robotat_server_ip);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(robotat_server_port);

    int err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err != 0)
    {
        #ifdef DEBUG_PRINT
        printf("ERROR: Unable to connect to Robotat server: %d\n", errno);
        #endif
        close(sock);
        return -1;
    }
    #ifdef DEBUG_PRINT
    printf("Connected to Robotat server\n");
    #endif
    return sock;
}

int
robotat_get_pose(int sock, int agent_id, float *mocap_data)
{
    char buffer[256];
    int bytes_read;
    fd_set read_fds;
    struct timeval timeout;

    if (sock < 0)
    {
        #ifdef DEBUG_PRINT
        printf("ERROR: Invalid socket\n");
        #endif
        return -1;
    }

    sprintf(buffer, "{\"dst\":1,\"cmd\":1,\"pld\":%d}", agent_id);

    int send_result = send(sock, buffer, strlen(buffer), 0);
    if (send_result < 0)
    {
        #ifdef DEBUG_PRINT
        printf("ERROR: Failed to send request to Robotat server: %d\n", errno);
        #endif
        return -1;
    }

    timeout.tv_sec = 1;
    timeout.tv_usec = 0;

    FD_ZERO(&read_fds);
    FD_SET(sock, &read_fds);
    int ret = select(sock + 1, &read_fds, NULL, NULL, &timeout);
    if (ret <= 0)
    {
        #ifdef DEBUG_PRINT
        printf("ERROR: Timeout or error waiting for data from Robotat server: %d\n", errno);
        #endif
        return -1;
    }

    bytes_read = recv(sock, buffer, sizeof(buffer) - 1, 0);
    if (bytes_read <= 0)
    {
        #ifdef DEBUG_PRINT
        printf("ERROR: Failed to receive data from Robotat server: %d\n", errno);
        #endif
        return -1;
    }

    buffer[bytes_read] = 0;

    if (buffer[0] != '{')
    {
        #ifdef DEBUG_PRINT
        printf("ERROR: Response is not a valid JSON string\n");
        #endif
        return -1;
    }

    cJSON *response = cJSON_Parse(buffer);
    if (response == NULL)
    {
        #ifdef DEBUG_PRINT
        printf("ERROR: Failed to parse JSON response from Robotat server\n");
        #endif
        return -1;
    }

    cJSON *data = cJSON_GetObjectItem(response, "data");
    if (data == NULL || !cJSON_IsArray(data) || cJSON_GetArraySize(data) < 7)
    {
        #ifdef DEBUG_PRINT
        printf("ERROR: Invalid data in response from Robotat server\n");
        #endif
        cJSON_Delete(response);
        return -1;
    }

    mocap_data[0] = cJSON_GetArrayItem(data, 0)->valuedouble; // X
    mocap_data[1] = cJSON_GetArrayItem(data, 1)->valuedouble; // Y
    mocap_data[2] = cJSON_GetArrayItem(data, 2)->valuedouble; // Z
    float q[4];
    q[0] = cJSON_GetArrayItem(data, 3)->valuedouble; // qw
    q[1] = cJSON_GetArrayItem(data, 4)->valuedouble; // qx
    q[2] = cJSON_GetArrayItem(data, 5)->valuedouble; // qy
    q[3] = cJSON_GetArrayItem(data, 6)->valuedouble; // qz

    float euler[3];
    quaternion_to_euler_xyz(q, euler);

    mocap_data[3] = euler[0]; // Roll
    mocap_data[4] = euler[1]; // Pitch
    mocap_data[5] = euler[2]; // Yaw

    cJSON_Delete(response);
    return 0;
}
