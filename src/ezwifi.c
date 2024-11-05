/**
 * @file ezwifi.c
 * @author Miguel Zea (mezea@uvg.edu.gt
 * @brief 
 * @version 0.1
 * @date 2022-11-22
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "ezwifi.h"

// ================================================================================
// Private macro definitions
// ================================================================================
// The event group allows multiple bits for each event, but we only care about two events:
// - we are connected to the AP with an IP
// - we failed to connect after the maximum amount of retries
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1


// ================================================================================
// Private variables
// ================================================================================
static const char tag[] = "EzWiFi";     // TAG to ESP_LOGx()
static bool b_ezwifi_started = false;   // Start WiFi driver one time
static EventGroupHandle_t s_wifi_event_group;   // FreeRTOS event group to signal when we are connected
static int32_t maximum_retry = 0;
static int s_retry_num = 0;
static char sta_ip[16] = {0};   // STA IP Address
static wifi_event_t wifi_status = 0;    // Last WiFi status
static ip_event_t ip_status = 0;    // Last IP status
static wifi_err_reason_t sta_dscrsn = 0;  // Last STA disconnect reason


// ================================================================================
// Public function declarations
// ================================================================================
void 
ezwifi_init(void);

static void 
event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);


// ================================================================================
// Public function definitions
// ================================================================================
void 
ezwifi_init(void)
{
    // Do nothing if WiFi already started
    if(b_ezwifi_started)
        return;
    
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) 
    {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    b_ezwifi_started = true;
}


wifi_mode_t
ezwifi_get_mode(void)
{
    wifi_mode_t mode;
    esp_wifi_get_mode(&mode);
    return mode;
}


wifi_event_t
ezwifi_get_status(void)
{
    return wifi_status;
}


ip_event_t
ezwifi_get_ip_status(void)
{
    return ip_status;
}


void 
ezwifi_sta_connect(const char *ssid, const char *password, int32_t max_retry)
{
    ezwifi_init();
    ESP_LOGI(tag, "ESP_WIFI_MODE_STA");

    maximum_retry = max_retry;
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip));

    wifi_config_t wifi_config = {0};
    memcpy(&wifi_config.sta.ssid[0], ssid, 32);
    memcpy(&wifi_config.sta.password[0], password, 64);
    wifi_config.sta.threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(tag, "wifi_init_sta finished.");

    // Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
    // number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler()
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, portMAX_DELAY);

    // xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually happened 
    if (bits & WIFI_CONNECTED_BIT)
    {
        ESP_LOGI(tag, "Connected to ap SSID:%s password:%s", ssid, password);
    }
    else if(bits & WIFI_FAIL_BIT)
    {
        ESP_LOGI(tag, "Failed to connect to SSID:%s, password:%s", ssid, password);
    }
    else 
    {
        ESP_LOGE(tag, "Unexpected event");
    }
}   

	
void 
ezwifi_sta_disconnect(void)
{
    ESP_ERROR_CHECK(esp_wifi_disconnect());
    b_ezwifi_started = false;
}


void 
ezwifi_sta_reconnect(void)
{
    if (b_ezwifi_started) 
        return;
    ESP_ERROR_CHECK(esp_wifi_connect());
}


char*
ezwifi_sta_get_ip(void)
{
    return sta_ip;
}


wifi_err_reason_t
ezwifi_sta_get_disconnection_reason(void)
{
    return sta_dscrsn;
}


static void 
event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT) 
    {
        wifi_status = event_id;
        switch(event_id)
        {
            case WIFI_EVENT_STA_START:
            {
                esp_wifi_connect();
                break;
            }
            case WIFI_EVENT_STA_DISCONNECTED:
            {
                if (s_retry_num < maximum_retry) 
                {
                    esp_wifi_connect();
                    s_retry_num++;
                    ESP_LOGI(tag, "Retrying to connect to the AP");
                } 
                else 
                {
                    xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
                }
                ESP_LOGI(tag, "Failed to connect to the AP");
                wifi_event_sta_disconnected_t* sta_event = (wifi_event_sta_disconnected_t*) event_data;
                sta_dscrsn = sta_event->reason;
                memset(sta_ip, 0, sizeof(sta_ip));
                ESP_LOGW(tag, "STA_DISCONNECTED_REASON: %d", sta_event->reason);
                break;
            }
            case WIFI_EVENT_AP_STACONNECTED:
            {
                break;
            }
            case WIFI_EVENT_AP_STADISCONNECTED:
            {
                break;
            }
            default:
                break;
        }
    } 
    else if(event_base == IP_EVENT)
    {
        ip_status = event_id;
        switch(event_id)
        {
            case IP_EVENT_STA_GOT_IP:
            {
                ip_event_got_ip_t* ip_event = (ip_event_got_ip_t*) event_data;
                ESP_LOGI(tag, "got IP:" IPSTR, IP2STR(&ip_event->ip_info.ip));
                esp_ip4addr_ntoa(&ip_event->ip_info.ip, sta_ip, 16);
                s_retry_num = 0;
                xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
                break;
            }
            case IP_EVENT_STA_LOST_IP:
            {
                ESP_LOGI(tag, "lost IP");
                memset(sta_ip, 0, sizeof(sta_ip));
                break;
            }
            default:
                break;
        }
    }
}

void 
wifi_init_static(const char *ssid, const char *password, const char *static_ip, const char *static_gw, const char *static_netmask) 
{
    nvs_flash_init();
    esp_netif_init();
    esp_event_loop_create_default();
    
    esp_netif_t *netif = esp_netif_create_default_wifi_sta();
    assert(netif);

    // Static IP Configuration
    esp_netif_ip_info_t ip_info;
    ip_info.ip.addr = ipaddr_addr(static_ip);
    ip_info.gw.addr = ipaddr_addr(static_gw);
    ip_info.netmask.addr = ipaddr_addr(static_netmask);
    esp_netif_dhcpc_stop(netif); // Stop DHCP client
    esp_netif_set_ip_info(netif, &ip_info); // Set static IP

    // Initialize WiFi with default config
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_mode(WIFI_MODE_STA);

    // Configure WiFi connection
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "",
            .password = "",
        },
    };
    strcpy((char *)wifi_config.sta.ssid, ssid);
    strcpy((char *)wifi_config.sta.password, password);

    esp_err_t ret = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    if (ret != ESP_OK) 
    {
        #ifdef DEBUG_PRINT
        printf("Failed to set WiFi configuration: %s\n", esp_err_to_name(ret));
        #endif
        return;
    }

    ret = esp_wifi_start();
    if (ret != ESP_OK) 
    {
        #ifdef DEBUG_PRINT
        printf("Failed to start WiFi: %s\n", esp_err_to_name(ret));
        #endif
        return;
    }

    ret = esp_wifi_connect();
    if (ret != ESP_OK) 
    {
        #ifdef DEBUG_PRINT
        printf("Failed to connect to WiFi: %s\n", esp_err_to_name(ret));
        #endif
        return;
    }

    wait_for_ip();
}

void
wait_for_ip()
{
    esp_netif_ip_info_t ip_info;
    esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    while (true)
    {
        if (netif)
        {
            esp_netif_get_ip_info(netif, &ip_info);
            if (ip_info.ip.addr)
            {
                #ifdef DEBUG_PRINT
                printf("ESP32 IP Address: " IPSTR "\n", IP2STR(&ip_info.ip));
                #endif
                break;
            }
        }
        #ifdef DEBUG_PRINT
        printf("Waiting for IP address...\n");
        #endif
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    
}