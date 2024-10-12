// web_server.cpp
#include <string.h>
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_http_server.h"
#include "esp_netif.h"  // Include this for esp_ip4addr_ntoa()

#define WIFI_SSID      "K.G.F-Extension"
#define WIFI_PASS      "ashfaque92786"

static const char *TAG = "esp32-webserver";

extern int throt;
extern float Rin,Pin,Yin;

// Structure to hold joystick data
typedef struct {
    int x;
    int y;
    int x2;
    int y2;
} joystick_data_t;

joystick_data_t joystick_data = {0, 0, 0, 0};
bool isPowerOn = false;  // Declare isPowerOn as a global variable


esp_err_t js_data_handler(httpd_req_t *req) {
    char buf[100];
    httpd_req_get_url_query_str(req, buf, sizeof(buf));

    char param[8];
    if (httpd_query_key_value(buf, "x", param, sizeof(param)) == ESP_OK) {
        joystick_data.x = atoi(param);
    }
    if (httpd_query_key_value(buf, "y", param, sizeof(param)) == ESP_OK) {
        joystick_data.y = atoi(param);
    }
    if (httpd_query_key_value(buf, "x2", param, sizeof(param)) == ESP_OK) {
        joystick_data.x2 = atoi(param);
    }
    if (httpd_query_key_value(buf, "y2", param, sizeof(param)) == ESP_OK) {
        joystick_data.y2 = atoi(param);
    }
    
    char power_param[4];
    if (httpd_query_key_value(buf, "power", power_param, sizeof(power_param)) == ESP_OK) {
        isPowerOn = (strcmp(power_param, "ON") == 0);
    }
    
    Rin=joystick_data.x2;
    Pin=joystick_data.y2;
    throt=joystick_data.x;
    Yin=joystick_data.y;

    // // Log the joystick values and power state
    ESP_LOGI(TAG, "Joystick Values -> X: %d, Y: %d, X2: %d, Y2: %d, Power: %s", 
             joystick_data.x, joystick_data.y, joystick_data.x2, joystick_data.y2, isPowerOn ? "ON" : "OFF");

    httpd_resp_send(req, "OK", 2);
    return ESP_OK;
}

esp_err_t index_get_handler(httpd_req_t *req) {
    extern const unsigned char index_html_start[] asm("_binary_index_html_start");
    extern const unsigned char index_html_end[] asm("_binary_index_html_end");
    const size_t index_html_len = index_html_end - index_html_start;
    httpd_resp_send(req, (const char *)index_html_start, index_html_len);
    return ESP_OK;
}

esp_err_t js_get_handler(httpd_req_t *req) {
    extern const unsigned char virtualjoystick_js_start[] asm("_binary_virtualjoystick_js_start");
    extern const unsigned char virtualjoystick_js_end[] asm("_binary_virtualjoystick_js_end");
    const size_t js_length = virtualjoystick_js_end - virtualjoystick_js_start;
    httpd_resp_send(req, (const char *)virtualjoystick_js_start, js_length);
    return ESP_OK;
}

httpd_uri_t js_data_uri = {
    .uri = "/jsData.html",
    .method = HTTP_PUT,
    .handler = js_data_handler,
    .user_ctx = NULL
};

httpd_uri_t index_uri = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = index_get_handler,
    .user_ctx = NULL
};

httpd_uri_t js_uri = {
    .uri = "/virtualjoystick.js",
    .method = HTTP_GET,
    .handler = js_get_handler,
    .user_ctx = NULL
};

void start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &index_uri);
        httpd_register_uri_handler(server, &js_uri);
        httpd_register_uri_handler(server, &js_data_uri);
    }
}

static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "Disconnected. Reconnecting...");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        char ip[16];
        esp_ip4addr_ntoa(&event->ip_info.ip, ip, sizeof(ip));
        ESP_LOGI(TAG, "Got IP: %s", ip);
    }
}

void wifi_init(void) {
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip);

    wifi_config_t wifi_config;
    memset(&wifi_config, 0, sizeof(wifi_config));

    strncpy((char*)wifi_config.sta.ssid, WIFI_SSID, sizeof(wifi_config.sta.ssid));
    strncpy((char*)wifi_config.sta.password, WIFI_PASS, sizeof(wifi_config.sta.password));

    wifi_config.sta.scan_method = WIFI_FAST_SCAN;
    wifi_config.sta.bssid_set = 0;
    wifi_config.sta.channel = 0;
    wifi_config.sta.listen_interval = 0;
    wifi_config.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
    wifi_config.sta.threshold.rssi = 0;
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();
}

extern "C" void web_server_task(void *pvParameters) {
    wifi_init();
    start_webserver();
    vTaskDelete(NULL);
}
