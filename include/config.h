/**
 * @file config.h
 * @author Javier Monzón (mon20054@uvg.edu.gt)
 * @brief Definitions for static configuration parameters
 * @version 0.1
 * @date 2024
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef CONFIG_H
#define CONFIG_H

// WiFi Credentials for CIT-116
static const char ssid[32] = "Robotat";
static const char password[64] = "iemtbmcit116";

// Static IP Configuration
static const char *static_ip = "192.168.50.100";
static const char *static_gw = "192.168.50.1";
static const char *static_netmask = "255.255.255.0";

// UDP Server Information
static const char *udp_server_ip = "192.168.50.116";
static const int udp_server_port = 8888;

// TCP Port
static const int tcp_port = 5005;

#endif // CONFIG_H
