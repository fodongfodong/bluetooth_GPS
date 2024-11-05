#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_bt.h"
#include "esp_gap_bt_api.h"
#include "esp_gattc_api.h"
#include "esp_bt_main.h"
#include "esp_spp_api.h"
#include "esp_bt_device.h"
#include "nvs_flash.h"
#include "esp_heap_caps.h"
#include "esp_event.h"
#include "esp_netif.h"
#include <lwip/sockets.h>

// Wi-Fi 연결 포함
#include "wificonnect.h"

// NTRIP 관련 매크로 정의
#define NTRIP_HOST "RTS2.ngii.go.kr"
#define NTRIP_PORT 2101
#define NTRIP_MOUNTPOINT "VRS-RTCM31"
#define NTRIP_USERNAME "whdtjr2529"
#define NTRIP_PASSWORD "qkrwhdtjr12"
#define BASE64_CREDENTIALS "d2hkdGpyMjUyOTpxa3J3aGR0anIxMg=="

// UART 및 GPS 설정
#define TXD 17
#define RXD 16
#define UART_NUM UART_NUM_2
#define GPS_RESET_PIN 5
#define NMEA_MAX_LENGTH 128

static const char *TAG = "BT_GPS";
static uint32_t spp_client_handle = 0;

// UART 초기화
void init_uart() {
    const uart_config_t uart_config = {
        .baud_rate  = 460800,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE
    };
    uart_driver_install(UART_NUM, 1024, 0, 0, NULL, 0);
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, TXD, RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

// GPS 리셋
void gps_reset() {
    gpio_set_direction(GPS_RESET_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(GPS_RESET_PIN, 1);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    gpio_set_level(GPS_RESET_PIN, 0);
}

// Bluetooth SPP 이벤트 처리기
static void spp_event_handler(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
    switch (event) {
        case ESP_SPP_INIT_EVT:
            ESP_LOGI(TAG, "SPP initialized");
            esp_bt_dev_set_device_name("ESP-32 GPS 송신");
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
            esp_spp_start_srv(ESP_SPP_SEC_NONE, ESP_SPP_ROLE_SLAVE, 0, "SPP_SERVER");
            break;
        case ESP_SPP_SRV_OPEN_EVT:
            ESP_LOGI(TAG, "SPP server connection opened");
            spp_client_handle = param->srv_open.handle;
            break;
        case ESP_SPP_DATA_IND_EVT:
            ESP_LOGI(TAG, "SPP data received");
            break;
        case ESP_SPP_CLOSE_EVT:
            ESP_LOGI(TAG, "SPP server connection closed");
            spp_client_handle = 0;
            break;
        default:
            ESP_LOGI(TAG, "Unhandled SPP event: %d", event);
            break;
    }
}

// Bluetooth 초기화
void init_bluetooth() {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BTDM));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());
    ESP_ERROR_CHECK(esp_spp_register_callback(spp_event_handler));
    esp_spp_cfg_t bt_spp_cfg = {.mode = ESP_SPP_MODE_CB};
    ESP_ERROR_CHECK(esp_spp_enhanced_init(&bt_spp_cfg));
}

// GPS 데이터 처리 (NMEA $GNGGA 메시지만 출력)
void gps_task(void *arg) {
    uint8_t data[NMEA_MAX_LENGTH];
    while (1) {
        if (spp_client_handle != 0) {
            int length = uart_read_bytes(UART_NUM, data, sizeof(data), 100 / portTICK_PERIOD_MS);
            if (length > 0) {
                data[length < NMEA_MAX_LENGTH ? length : NMEA_MAX_LENGTH - 1] = '\0';
                if (data[0] == '$' && strstr((char *)data, "\r\n") && strncmp((char *)data, "$GNGGA", 6) == 0) {
                    ESP_LOGI(TAG, "Valid NMEA GNGGA message: %s", (char *)data);
                    esp_spp_write(spp_client_handle, length, data);
                }
            }
        } else {
            vTaskDelay(500 / portTICK_PERIOD_MS);
        }
    }
}

// NTRIP 클라이언트 작업
void ntrip_client_task(void *pvParameters) {
    char ntrip_request[512];
    snprintf(ntrip_request, sizeof(ntrip_request),
             "GET /%s HTTP/1.0\r\n"
             "User-Agent: NTRIP ESP32Client\r\n"
             "Authorization: Basic %s\r\n"
             "\r\n",
             NTRIP_MOUNTPOINT, BASE64_CREDENTIALS);

    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        ESP_LOGE(TAG, "Failed to create socket");
        return;
    }

    struct sockaddr_in dest_addr = {
        .sin_addr.s_addr = inet_addr(NTRIP_HOST),
        .sin_family = AF_INET,
        .sin_port = htons(NTRIP_PORT),
    };

    if (connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr)) != 0) {
        ESP_LOGE(TAG, "Socket connection failed");
        close(sock);
        return;
    }

    send(sock, ntrip_request, strlen(ntrip_request), 0);
    char response[1024];
    int len;

    while ((len = recv(sock, response, sizeof(response) - 1, 0)) > 0) {
        response[len] = 0;
        ESP_LOGI(TAG, "Received: %s", response);
        // RTK 보정 데이터 처리
    }

    close(sock);
    vTaskDelete(NULL);
}

void app_main() {
    wifi_init_sta();
    vTaskDelay(500 / portTICK_PERIOD_MS);
    init_bluetooth();
    gps_reset();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    init_uart();

    xTaskCreate(ntrip_client_task, "ntrip_client_task", 4096, NULL, 5, NULL);
    xTaskCreate(gps_task, "gps_task", 8192, NULL, 10, NULL);
}
