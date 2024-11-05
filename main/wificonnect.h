#ifndef _WIFICONNECT_H_
#define _WIFICONNECT_H_

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "freertos/event_groups.h"

extern EventGroupHandle_t s_wifi_event_group;


// Wi-Fi 연결을 위한 함수 선언
void wifi_init_sta(void);

#endif // _WIFICONNECT_H_
