/**
 * @file ezudp.h
 * @author Miguel Zea (mezea@uvg.edu.gt) & Javier Monzón (mon20054@uvg.edu.gt)
 * @brief This was originally a C port of Jose Morais Arduino stlye UDP library 
 * (https://github.com/urbanze/esp32-udp) BUT ended up being a mix between it and
 * the provided udp_server example provided by Espressif, modified to have a 
 * non-blocking data read. Added two funuctions to send data to a specific server
 * for monitoring data while not having a serial terminal available.
 * @version 0.1
 * @date 2022-11-23
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef EZUDP_H
#define EZUDP_H

#ifdef __cplusplus
extern "C" {
#endif

// ================================================================================
// Dependencies
// ================================================================================
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "ezwifi.h"

// ================================================================================
// Public macro definitions
// ================================================================================
/**
 * Maximum Transmission Unit for the UDP protocol
 */
#define UDP_MTU (1454)

// ================================================================================
// Public function declarations
// ================================================================================
/**
 * @brief Binds the RX socket to the specified port, and listens for UDP packets.
 * NOTE: the TX socket is created on the first call to the function. 
 * 
 * @param[in] port Port to bind.
 * 
 * @returns Nothing, but logs every error found.
 */
void 
ezudp_begin(uint16_t port);


/**
 * @brief Sets the IP address and port to write to. 
 * NOTE: the RX socket is created on the first call to the function. 
 * 
 * @param ip    Destination IP address.
 * @param port  Destination port.
 * 
 * @returns Nothing, but logs every error found.
 */
void 
ezudp_begin_packet(const char *ip, uint16_t port);


/**
 * @brief Shuts down and closes both TX and RX sockets.
 * 
 * @returns Nothing, but logs every error found.
 */
void 
ezudp_stop(void);


/**
 * @brief Clears the RX socket's receive buffer.
 * 
 * @returns Nothing, but logs every error found.
 */
void 
ezudp_flush(void);


/**
 * @brief Gets the number of available bytes to read from the RX socket.
 * 
 * @return ssize_t Number of bytes available to read. 
 */
ssize_t 
ezudp_available(void);


/**
 * @brief Gets the IP address of the last received data's source.
 * 
 * @return const char* Las source's IP address as string.
 */
const char*
ezudp_get_remote_ip(void);


/**
 * @brief Gets the port of the last received data's source.
 * 
 * @return uint16_t Last source's port. 
 */
uint16_t 
ezudp_get_remote_port(void);


/**
 * @brief Uses the TX socket to write the given data buffer to the IP address 
 * and port specified in ezudp_begin_packet.
 * 
 * @param data     Data buffer. 
 * @param datasize Size of data buffer.
 * @return ssize_t Number of succesfully written bytes. 
 */
ssize_t 
ezudp_write(uint8_t *data, size_t datasize);


/**
 * @brief Aux function to send formatted strings via UDP.
 * 
 * @return ssize_t Number of succesfully written bytes.
 */
ssize_t 
ezudp_printf(const char *format, ...);


/**
 * @brief Gets a single byte from the RX's socket receive buffer.
 * 
 * @return uint8_t Byte read. 
 */
uint8_t 
ezudp_read(void);


/**
 * @brief Takes and stores the specified number of bytes from the 
 * RX's socket receive buffer.
 * 
 * @param buf Buffer to store the data read. 
 * @param len Number of bytes to read.
 * 
 * @returns Nothing, but logs every error found.
 */
void 
ezudp_read_bytes(uint8_t *buf, size_t len);

/**
 * @brief Sets the IP address and port of the destination UDP server for monitoring data.
 * 
 * @param ip Destination IP address.
 * @param port Destination port number.
 */
void 
ezudp_set_destination(const char *ip, uint16_t port);

/**
 * @brief Sends a JSON string to the configured UDP server including the data for monitoring
 * 
 * @param json_string The JSON string to send.
 * @return int Number of bytes sent, or -1 on error.
 */
int 
ezudp_send_json(const char *json_string);


#ifdef __cplusplus
}
#endif

#endif /* EZUDP_H */