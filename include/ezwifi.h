/**
 * @file ezwifi.h
 * @author Miguel Zea (mezea@uvg.edu.gt)
 * @brief Simple library wrapper for the getting started WiFi station example by Espressif.
 * @version 0.1
 * @date 2022-11-22
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef EZWIFI_H
#define EZWIFI_H

#ifdef __cplusplus
extern "C" {
#endif

// ================================================================================
// Dependencies
// ================================================================================
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"

// ================================================================================
// WiFi network security (uncomment and comment as needed)
// ================================================================================
// #define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN
// #define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WEP
// #define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
// #define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_WPA2_PSK
// #define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA3_PSK
// #define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_WPA3_PSK
// #define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WAPI_PSK

// ================================================================================
// Public function declarations
// ================================================================================
/**
 * @brief Initializes the WiFi module as station and connects it to a 
 * specified network. 
 * 
 * @param[in] ssid      Network's SSID (must be a char array of size 32).
 * @param[in] password  Network's password (must be a char array of size 64).
 * @param[in] max_retry Maximum number of tries before giving up on connection.   
 * 
 * @returns Nothing, but logs every error found.
 */
void 
ezwifi_sta_connect(const char *ssid, const char *password, int32_t max_retry);


/**
 * @brief Disconnects from the current network.
 * 
 * @returns Nothing, but logs every error found.
 */
void 
ezwifi_sta_disconnect(void);


/**
 * @brief Tries to reconnect to the current network, fails if the module has yet
 * to be initialized as station.
 * 
 * @returns Nothing, but logs every error found.
 */
void 
ezwifi_sta_reconnect(void);


/**
 * @brief Gets the IP assigned to the station by the access point.
 * 
 * @returns char*    Pointer to assigned IP.
 */
char *
ezwifi_sta_get_ip(void);


/**
 * @brief Gets the current operation mode of the WiFi module.
 * 
 * @return wifi_mode_t  Current mode of operation. 
 */
wifi_mode_t
ezwifi_get_mode(void);


/**
 * @brief Gets the last recorded WiFi event as the module's status.
 * 
 * @return wifi_event_t Last WiFi event id.    
 */
wifi_event_t
ezwifi_get_status(void);


/**
 * @brief Gets the last recorded IP event as the module's status.
 * 
 * @return ip_event_t   Last IP event id. 
 */
ip_event_t
ezwifi_get_ip_status(void);


/**
 * @brief Gets the reason for the last connection failure.
 * 
 * @return wifi_err_reason_t Disconnection reason.
 */
wifi_err_reason_t
ezwifi_sta_get_disconnection_reason(void);

/**
 * @brief Waits for the ESP32 to obtain a static IP address.
 */
void 
wait_for_ip();

/**
 * @brief Initializes WiFi in station mode with a static IP configuration.
 * 
 * @param ssid              WiFi SSID to connect to.
 * @param password          WiFi password.
 * @param static_ip         Static IP address.
 * @param static_gw         Gateway address.
 * @param static_netmask    Network mask.
 */
void 
wifi_init_static(const char *ssid, const char *password, const char *static_ip, const char *static_gw, const char *static_netmask);


// To Be Implemented
// void ezwifi_sta_static_ip(const char *ip, const char *gateway, const char *mask);
// void ezwifi_ap_start(const char *ssid, const char *password, uint8_t channel, uint8_t max_connection);	
// void ezwifi_ap_stop(void);

#ifdef __cplusplus
}
#endif

#endif /* EZWIFI_H */