#include "ezudp.h"
#include <stdio.h>
#include <string.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>

// ================================================================================
// Private variables
// ================================================================================
static const char tag[6] = "EzUDP";
static char remote_ip[16] = {0};

static int sock_rx = 0; // Socket to receive
static struct sockaddr_in6 addr_rx; // Receive address
static socklen_t socklen_rx = sizeof(addr_rx);

static int sock_tx = 0; // Socket to send
static struct sockaddr_in6 addr_tx; // Send address


// ================================================================================
// Public function definitions
// ================================================================================
void 
ezudp_begin(uint16_t port)
{
    memset(&addr_rx, 0, sizeof(addr_rx));
    struct sockaddr_in * p_to_addr_ip4 = (struct sockaddr_in *)&addr_rx;
    p_to_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
    p_to_addr_ip4->sin_family = AF_INET;
    p_to_addr_ip4->sin_port = htons(port);

    if (!sock_rx)
    {
        sock_rx = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
        if (sock_rx < 0) 
        {
            ESP_LOGE(tag, "Unable to create RX socket: errno %d", errno);
            return;
        }
        ESP_LOGI(tag, "RX socket created.");
    }

    int err = bind(sock_rx, (struct sockaddr *)&addr_rx, sizeof(addr_rx));
    if (err < 0) 
    {
        ESP_LOGE(tag, "RX socket unable to bind: errno %d", errno);
        close(sock_rx);
        return;
    }
    ESP_LOGI(tag, "RX socket bound, port %d", port);
    fcntl(sock_rx, F_SETFL, O_NONBLOCK); // Set the receive socket to be non-blocking
}


void 
ezudp_begin_packet(const char *ip, uint16_t port)
{
    memset(&addr_tx, 0, sizeof(addr_tx));
    struct sockaddr_in * p_to_addr_ip4 = (struct sockaddr_in *)&addr_tx;
    p_to_addr_ip4->sin_addr.s_addr = inet_addr(ip);
    p_to_addr_ip4->sin_family = AF_INET;
    p_to_addr_ip4->sin_port = htons(port);

    if(!sock_tx)
    {
        sock_tx = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
        if (sock_tx < 0) 
        {
            ESP_LOGE(tag, "Unable to create TX socket: errno %d", errno);
            return;
        }
        ESP_LOGI(tag, "TX socket created."); 
    }
}


void 
ezudp_stop(void)
{
    ESP_LOGE(tag, "Shutting down RX and TX sockets...");
    shutdown(sock_rx, 0);
    close(sock_rx);
    shutdown(sock_tx, 0);
    close(sock_tx);
}


void 
ezudp_flush(void)
{
    uint8_t bff = 0;
    int avl = ezudp_available();
    for (int16_t i = 0; i < avl; i++)
    {
        recvfrom(sock_rx, &bff, 1, MSG_DONTWAIT, (struct sockaddr*)&sock_rx, &socklen_rx);
    }
}


ssize_t 
ezudp_available(void)
{
    // Uncomment this if the given solution acts unexpectedly
    // uint8_t buf[UDP_MTU];
    // return recvfrom(sock_rx, &buf, UDP_MTU, MSG_PEEK, (struct sockaddr*)&addr_rx, &socklen_rx);

    uint8_t temp;
    return recvfrom(sock_rx, &temp, UDP_MTU, MSG_PEEK, (struct sockaddr*)&addr_rx, &socklen_rx);

    // This supposedly had to work but didn't
    // int16_t avl = 0;
    // #if LWIP_POSIX_SOCKETS_IO_NAMES
    //     ioctl(sock_rx, FIONREAD, &avl);
    // #else
    //     lwip_ioctl(sock_rx, FIONREAD, &avl);
    // #endif
    // return avl;
}


const char*
ezudp_get_remote_ip(void)
{
    struct sockaddr_in * p_to_addr_ip4 = (struct sockaddr_in *)&addr_rx;
    inet_ntop(AF_INET, &p_to_addr_ip4->sin_addr, remote_ip, sizeof(remote_ip));
    return remote_ip;
}


uint16_t 
ezudp_get_remote_port(void)
{
    struct sockaddr_in * p_to_addr_ip4 = (struct sockaddr_in *)&addr_rx;
    return ntohs(p_to_addr_ip4->sin_port); 
}


ssize_t 
ezudp_write(uint8_t *data, size_t datasize)
{
    int err = sendto(sock_tx, data, datasize, 0, (struct sockaddr *)&addr_tx, sizeof(addr_tx));
    if (err < 0) 
    {
        ESP_LOGE(tag, "Error occurred during sending: errno %d", errno);
        return -1;
    }
    return datasize;
}


ssize_t 
ezudp_printf(const char *format, ...)
{
    va_list vl;

    va_start(vl, format);
    int size = vsnprintf(NULL, 0, format, vl) + 1;
    va_end(vl);

    char * p_to_buf = (char*) malloc(size);
    vsprintf(p_to_buf, format, vl);
    p_to_buf[size] = 0;

    ssize_t ret = ezudp_write((uint8_t*)p_to_buf, size);
    free(p_to_buf);
    
    return ret;
}


uint8_t 
ezudp_read(void)
{
    uint8_t ret = 0;
    // The MSG_DONTWAIT flag did not work, the port had to be set to non-blocking in the setup via fctrl
    //recvfrom(sock_rx, &ret, 1, MSG_DONTWAIT, (struct sockaddr*)&addr_rx, &socklen_rx);
    recvfrom(sock_rx, &ret, 1, 0, (struct sockaddr*)&addr_rx, &socklen_rx);
    return ret;
}


void 
ezudp_read_bytes(uint8_t *buf, size_t bufsize)
{
    //int len = recvfrom(sock_rx, buf, bufsize, MSG_DONTWAIT, (struct sockaddr *)&addr_rx, &socklen_rx);
    int len = recvfrom(sock_rx, buf, bufsize, 0, (struct sockaddr *)&addr_rx, &socklen_rx);
    // Error occurred during receiving
    if (len < 0) 
    {
        ESP_LOGE(tag, "Error occured during receiving: errno %d", errno);
        return;
    }
}

static struct sockaddr_in dest_addr;

void 
ezudp_set_destination(const char *ip, uint16_t port) 
{
    memset(&dest_addr, 0, sizeof(dest_addr));
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_addr.s_addr = inet_addr(ip);
    dest_addr.sin_port = htons(port);
}

int 
ezudp_send_json(const char *json_string) 
{
    if (sock_tx == 0) 
    {
        sock_tx = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
        if (sock_tx < 0) {
            #ifdef DEBUG_PRINT
            printf("Unable to create UDP socket: %d\n", errno);
            #endif
            return -1;
        }
    }

    int err = sendto(sock_tx, json_string, strlen(json_string), 0, 
                     (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err < 0) 
    {
        #ifdef DEBUG_PRINT
        printf("Error occurred during sending: %d\n", errno);
        #endif
        return -1;
    }

    return err;  // Number of bytes sent
}