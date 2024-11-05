/**
 * @file eztcp.h
 * @author Javier Monzón (mon20054@uvg.edu.gt)
 * @brief TCP communication for the ESP32.
 * @version 0.1
 * @date 2024
 *
 * @copyright Copyright (c) 2024
 *
 */

 #ifndef EZTCP_H
#define EZTCP_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <sys/socket.h>

#ifdef __cplusplus
extern "C" {
#endif

// ================================================================================
// Public function declarations
// ================================================================================

/**
 * @brief Initializes and binds a TCP socket to a specified port.
 *
 * @param port  The port to bind the server socket.
 * @return int  The socket descriptor, or -1 on error.
 */
int 
eztcp_begin(uint16_t port);

/**
 * @brief Waits for and accepts an incoming client connection.
 *
 * @param server_sock   The server socket descriptor.
 * @return int          The client socket descriptor, or -1 on error.
 */
int 
eztcp_accept(int server_sock);

/**
 * @brief Receives data from the client socket.
 *
 * @param client_sock   The client socket descriptor.
 * @param buffer        The buffer to store received data.
 * @param buffer_size   The size of the buffer.
 * @return ssize_t      Number of bytes received, or -1 on error.
 */
ssize_t 
eztcp_receive(int client_sock, char *buffer, size_t buffer_size);

/**
 * @brief Closes the client and server sockets.
 *
 * @param client_sock The client socket descriptor.
 * @param server_sock The server socket descriptor.
 */
void 
eztcp_close(int client_sock, int server_sock);

#ifdef __cplusplus
}
#endif

#endif /* EZTCP_H */
