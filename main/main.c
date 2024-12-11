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
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "esp_spp_api.h"
#include "esp_bt_device.h"
#include "nvs_flash.h"
#include "esp_heap_caps.h"
#include <sys/time.h> // Time stamp

#define TXD 17
#define RXD 16
#define UART_NUM UART_NUM_2
#define GPS_RESET_PIN 5
#define NMEA_MAX_LENGTH 128

static const char *TAG = "BT_GPS";
static uint32_t spp_client_handle = 0;

// 함수 선언
void print_hex(const uint8_t *data, int length);
void log_with_real_time(const char *tag, const char *fmt, ...);

void init_uart() {
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    int rx_buffer_size = 4096;
    uart_driver_install(UART_NUM, rx_buffer_size, 0, 0, NULL, 0);
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, TXD, RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

void gps_reset() {
    gpio_set_direction(GPS_RESET_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(GPS_RESET_PIN, 1);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    gpio_set_level(GPS_RESET_PIN, 0);
}

void handle_received_data(const uint8_t *data, int length) {
    // 데이터가 NMEA 메시지인지 확인
    if (data[0] == '$') {
        // NMEA 메시지 로그 출력
        ESP_LOGI(TAG, "NMEA Message: %.*s", length, (char *)data);
    } else {
        // RTCM 데이터 처리
        ESP_LOGI(TAG, "RTCM Data (formatted):");
        print_hex(data, length);

        // GNSS 모듈로 데이터 전달 (UART 전송)
        int bytes_written = uart_write_bytes(UART_NUM, (const char *)data, length);
        if (bytes_written > 0) {
            ESP_LOGI(TAG, "Forwarded %d bytes to GNSS receiver via UART", bytes_written);
        } else {
            ESP_LOGE(TAG, "Failed to forward data to GNSS receiver");
        }

        // 로그에 타임스탬프 포함
        log_with_real_time(TAG, "Processed %d bytes", length);
    }
}

static void spp_event_handler(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
    switch (event) {
        case ESP_SPP_INIT_EVT:
            ESP_LOGI(TAG, "SPP initialized");
            esp_bt_gap_set_device_name("ESP-32 GPS 송신");
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
            esp_spp_start_srv(ESP_SPP_SEC_NONE, ESP_SPP_ROLE_SLAVE, 0, "SPP_SERVER");
            break;
        case ESP_SPP_SRV_OPEN_EVT:
            ESP_LOGI(TAG, "SPP server connection opened");
            spp_client_handle = param->srv_open.handle;
            break;
        case ESP_SPP_DATA_IND_EVT:
            ESP_LOGI(TAG, "SPP data received, length: %d", param->data_ind.len);
            handle_received_data(param->data_ind.data, param->data_ind.len);
            break;
        case ESP_SPP_CLOSE_EVT:
            ESP_LOGI(TAG, "SPP server connection closed");
            spp_client_handle = 0;
            break;
        default:
            ESP_LOGW(TAG, "Unhandled SPP event: %d", event); // 경고 수준으로 변경
            break;
    }
}

void init_bluetooth() {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    } else if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize NVS flash: %s", esp_err_to_name(ret));
        return;
    }

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bluetooth controller initialization failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BTDM);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bluetooth enable failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bluedroid initialization failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bluedroid enable failed: %s", esp_err_to_name(ret));
        return;
    }

    ESP_ERROR_CHECK(esp_spp_register_callback(spp_event_handler));
    esp_spp_cfg_t bt_spp_cfg = {
        .mode = ESP_SPP_MODE_CB
    };
    ESP_ERROR_CHECK(esp_spp_enhanced_init(&bt_spp_cfg));
}

void print_hex(const uint8_t *data, int length) {
    for (int i = 0; i < length; i++) {
        if (i < length - 1) {
            printf("%02X,", data[i]);
        } else {
            printf("%02X", data[i]);
        }
    }
    printf("\n"); // 마지막 줄바꿈
}

void log_with_real_time(const char *tag, const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);

    struct timeval tv;
    gettimeofday(&tv, NULL);
    struct tm timeinfo;
    localtime_r(&tv.tv_sec, &timeinfo);

    printf("[%02d:%02d:%02d.%03ld] %s: ",
        timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec, tv.tv_usec / 1000,
        tag);
    vprintf(fmt, args);
    printf("\n");

    va_end(args);
}

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

void app_main() {
    init_bluetooth();
    gps_reset();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    init_uart();
    xTaskCreate(gps_task, "gps_task", 8192, NULL, 10, NULL);
}
