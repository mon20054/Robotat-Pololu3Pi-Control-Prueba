/**
 * @file functionalities.c
 * @author Javier Monzón (mon20054@uvg.edu.gt)
 * @brief TCP communication for the ESP32.
 * @version 0.1
 * @date 2024
 *
 * @copyright Copyright (c) 2024
 *
 */


// ================================================================================
// Dependencies
// ================================================================================
#include "eztcp.h"
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>

// ================================================================================
// Public function definitions
// ================================================================================

int 
eztcp_begin(uint16_t port) 
{
    int sock;
    struct sockaddr_in addr;

    sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (sock < 0) 
    {
        #ifdef DEBUG_PRINT
        printf("Unable to create TCP socket: %d\n", errno);
        #endif
        return -1;
    }

    int opt = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port);

    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) 
    {
        #ifdef DEBUG_PRINT
        printf("Unable to bind TCP socket: %d\n", errno);
        #endif
        close(sock);
        return -1;
    }

    if (listen(sock, 1) < 0) 
    {
        #ifdef DEBUG_PRINT
        printf("Error during listen: %d\n", errno);
        #endif
        close(sock);
        return -1;
    }

    #ifdef DEBUG_PRINT
    printf("TCP socket bound, listening on port %d\n", port);
    #endif
    return sock;
}

int 
eztcp_accept(int server_sock) 
{
    struct sockaddr_in addr;
    socklen_t addr_len = sizeof(addr);
    int client_sock = accept(server_sock, (struct sockaddr *)&addr, &addr_len);

    if (client_sock < 0) 
    {
        #ifdef DEBUG_PRINT
        printf("Unable to accept connection: %d\n", errno);
        #endif
    } 
    else 
    {
        #ifdef DEBUG_PRINT
        printf("Accepted TCP client connection.\n");
        #endif
    }
    return client_sock;
}

ssize_t 
eztcp_receive(int client_sock, char *buffer, size_t buffer_size) 
{
    ssize_t len = recv(client_sock, buffer, buffer_size, 0);
    if (len < 0) 
    {
        #ifdef DEBUG_PRINT
        printf("Error receiving TCP data: %d\n", errno);
        #endif
    }
    return len;
}

void 
eztcp_close(int client_sock, int server_sock) 
{
    close(client_sock);
    close(server_sock);
    #ifdef DEBUG_PRINT
    printf("TCP client and server sockets closed.\n");
    #endif
}
