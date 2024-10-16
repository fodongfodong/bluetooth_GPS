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

#define TXD 17
#define RXD 16
#define UART_NUM UART_NUM_2
#define GPS_RESET_PIN 5
#define NMEA_MAX_LENGTH 128

static const char *TAG = "BT_GPS";
static uint32_t spp_client_handle = 0; // SPP 연결 핸들 저장


//uart 연결
void init_uart() {
    const uart_config_t uart_config = {
        .baud_rate  = 460800,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE
    };
    int rx_buffer_size = 1024;
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


//Serial 통신 연결 
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
            spp_client_handle = param->srv_open.handle; // 핸들 저장

            break;
        case ESP_SPP_DATA_IND_EVT:
            ESP_LOGI(TAG, "SPP data received");
            break;
        case ESP_SPP_CLOSE_EVT:
            ESP_LOGI(TAG, "SPP server connection closed");
            spp_client_handle = 0; // 클라이언트 핸들 리셋
            break;
        default:
            ESP_LOGI(TAG, "Unhandled SPP event: %d", event);
            break;
    }
}


//bluetooth 연결
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

    // Bluetooth Classic + BLE (Dual Mode) 모드를 활성화
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





/* GPS 전체 내용 출력
void gps_task(void *arg) {
    uint8_t data[NMEA_MAX_LENGTH];
    while (1) {
        int length = uart_read_bytes(UART_NUM, data, sizeof(data), 100 / portTICK_PERIOD_MS);
        if (length > 0) {
            if (length < NMEA_MAX_LENGTH) {
                data[length] = '\0';
            } else {
                data[NMEA_MAX_LENGTH - 1] = '\0';
            }
            
            if (data[0] == '$' && strstr((char *)data, "\r\n")) {
                ESP_LOGI(TAG, "Valid NMEA message: %s", (char *)data);
                
                if (spp_client_handle != 0) { // 연결된 클라이언트가 있을 때만 전송
                    esp_spp_write(spp_client_handle, length, data); // GPS 데이터 전송
                }
            }
        }
        size_t freeHeap = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
        
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

*/


// GPS %GNGGA 값만 출력
void gps_task(void *arg) {
    uint8_t data[NMEA_MAX_LENGTH];
    while (1) {
        // Bluetooth 연결이 되어 있는 경우에만 GPS 데이터를 읽음
        if (spp_client_handle != 0) {
            int length = uart_read_bytes(UART_NUM, data, sizeof(data), 100 / portTICK_PERIOD_MS);
            if (length > 0) {
                if (length < NMEA_MAX_LENGTH) {
                    data[length] = '\0';
                } else {
                    data[NMEA_MAX_LENGTH - 1] = '\0';
                }

                // NMEA 메시지가 $GNGGA로 시작하는 경우에만 로그에 기록
                if (data[0] == '$' && strstr((char *)data, "\r\n") && strncmp((char *)data, "$GNGGA", 6) == 0) {
                    ESP_LOGI(TAG, "Valid NMEA GNGGA message: %s", (char *)data);

                    // 연결된 클라이언트가 있을 때만 전송
                    esp_spp_write(spp_client_handle, length, data); // GPS 데이터 전송
                }
            }
            //GPS 전송 속도 ms
            //vTaskDelay(50 / portTICK_PERIOD_MS);
        } else {
            // Bluetooth 연결이 없을 경우, GPS 데이터를 읽지 않고 대기
            vTaskDelay(500 / portTICK_PERIOD_MS);
        }
    }
}


void app_main() {
    init_bluetooth();
    

    gps_reset();
    vTaskDelay(1000 / portTICK_PERIOD_MS); // GPS 재부팅 시간 지연
    init_uart();

    
    xTaskCreate(gps_task, "gps_task", 8192, NULL, 10, NULL);
}
