#include <stdio.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "MadgwickAHRS.h"

// External variables
extern Madgwick madgwick;
extern float alpha;
extern float pKp, pKi, pKd;
extern float rKp, rKi, rKd;
extern float yKp, yKi, yKd;
extern float iP, iR, iY;

extern int throt;
extern float tKf;

#define WIFI_SSID      "K.G.F-Extension"
#define WIFI_PASS      "ashfaque92786"
#define TAG            "example"

// Slider values
float slider1_value = alpha;
float slider2_value = pKp;
float slider3_value = pKi;
float slider4_value = pKd;
float slider5_value = throt;

bool extern motrState;  // Boolean flag for start/stop

// HTML content for the webpage with input fields, buttons, and start/stop control
const char* html_content = "<!DOCTYPE html>\
<html>\
<head>\
<title>ESP32 Float Input with Start/Stop</title>\
</head>\
<body>\
<h2>Set Float Values</h2>\
<div>\
  <label for='input1'>Throttle:</label>\
  <input type='number' id='input1' value='10.0' step='1.0' />\
  <button onclick='sendValue(1)'>Send</button><br><br>\
</div>\
<div>\
  <label for='input2'>Alpha:</label>\
  <input type='number' id='input2' value='0.015' step='0.001' />\
  <button onclick='sendValue(2)'>Send</button><br><br>\
</div>\
<div>\
  <label for='input3'>Value P:</label>\
  <input type='number' id='input3' value='0.01' step='0.01' />\
  <button onclick='sendValue(3)'>Send</button><br><br>\
</div>\
<div>\
  <label for='input4'>Value I:</label>\
  <input type='number' id='input4' value='0.0001' step='0.0001' />\
  <button onclick='sendValue(4)'>Send</button><br><br>\
</div>\
<div>\
  <label for='input5'>Value D:</label>\
  <input type='number' id='input5' value='0.001' step='0.001' />\
  <button onclick='sendValue(5)'>Send</button><br><br>\
</div>\
<div>\
  <button onclick='toggleStartStop()' id='startStopBtn'>Start</button><br><br>\
</div>\
<script>\
function sendValue(slider) {\
    var value = document.getElementById('input' + slider).value;\
    var xhr = new XMLHttpRequest();\
    xhr.open('GET', '/slider?slider=' + slider + '&value=' + parseFloat(value).toFixed(3), true);\
    xhr.send();\
}\
function toggleStartStop() {\
    var xhr = new XMLHttpRequest();\
    xhr.open('GET', '/toggleStartStop', true);\
    xhr.send();\
    var btn = document.getElementById('startStopBtn');\
    if (btn.innerHTML === 'Start') {\
        btn.innerHTML = 'Stop';\
    } else {\
        btn.innerHTML = 'Start';\
    }\
}\
function fetchValues() {\
    var xhr = new XMLHttpRequest();\
    xhr.open('GET', '/getValues', true);\
    xhr.onload = function() {\
        if (xhr.status === 200) {\
            var data = JSON.parse(xhr.responseText);\
            document.getElementById('input1').value = data.throt;\
            document.getElementById('input2').value = data.alpha;\
            document.getElementById('input3').value = data.pKp;\
            document.getElementById('input4').value = data.pKi;\
            document.getElementById('input5').value = data.pKd;\
        }\
    };\
    xhr.send();\
}\
window.onload = fetchValues;\
</script>\
</body>\
</html>";

// HTTP GET handler for the root page
esp_err_t get_handler(httpd_req_t *req) {
    httpd_resp_send(req, html_content, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// HTTP GET handler to fetch current values for sliders
esp_err_t get_values_handler(httpd_req_t *req) {
    char response[256];
    snprintf(response, sizeof(response), 
             "{\"throt\": %d, \"alpha\": %.3f, \"pKp\": %.3f, \"pKi\": %.4f, \"pKd\": %.3f}", 
             throt, alpha, pKp, pKi, pKd);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// HTTP GET handler for the slider updates
esp_err_t slider_handler(httpd_req_t *req) {
    char* buf;
    size_t buf_len;
    int slider;
    float value;

    buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1) {
        buf = (char*) malloc(buf_len);
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
            sscanf(buf, "slider=%d&value=%f", &slider, &value);
            ESP_LOGI(TAG, "Slider %d set to %.3f", slider, value);
            iP = iR = iY = 0;
            // Update slider values based on the slider number
            switch (slider) {
                case 1: slider1_value = value; throt = value; break;
                case 2: slider2_value = value; alpha = value; break;
                case 3: slider3_value = value; pKp = rKp = value; break;
                case 4: slider4_value = value; pKi = rKi = value; break;
                case 5: slider5_value = value; pKd = rKd = value; break;
                default: break;
            }
        }
        free(buf);
    }
    httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// HTTP GET handler for toggling start/stop
esp_err_t toggle_start_stop_handler(httpd_req_t *req) {
    motrState = !motrState;  // Toggle the boolean value
    ESP_LOGI(TAG, "System %s", motrState ? "Started" : "Stopped");
    httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// URIs for the webserver
httpd_uri_t uri_root = {
    .uri      = "/",
    .method   = HTTP_GET,
    .handler  = get_handler,
    .user_ctx = NULL
};

httpd_uri_t uri_slider = {
    .uri      = "/slider",
    .method   = HTTP_GET,
    .handler  = slider_handler,
    .user_ctx = NULL
};

httpd_uri_t uri_toggle_start_stop = {
    .uri      = "/toggleStartStop",
    .method   = HTTP_GET,
    .handler  = toggle_start_stop_handler,
    .user_ctx = NULL
};

httpd_uri_t uri_get_values = {
    .uri      = "/getValues",
    .method   = HTTP_GET,
    .handler  = get_values_handler,
    .user_ctx = NULL
};

// Start the web server
static httpd_handle_t start_webserver(void) {
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &uri_root);
        httpd_register_uri_handler(server, &uri_slider);
        httpd_register_uri_handler(server, &uri_toggle_start_stop);
        httpd_register_uri_handler(server, &uri_get_values);  // Register new handler
    }
    return server;
}

// Event handler for Wi-Fi events
static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
    }
}

// Wi-Fi initialization
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

    // Set Wi-Fi configuration
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

// Main application task for the Wi-Fi webserver
extern "C" void wifi_webserver_task(void *pvParameters) {
    wifi_init();
    start_webserver();
    vTaskDelete(NULL);
}
