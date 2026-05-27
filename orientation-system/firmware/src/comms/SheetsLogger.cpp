#include "SheetsLogger.h"
#include "Config.h"

#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

using namespace core::config;

namespace sheets {

static QueueHandle_t s_queue   = nullptr;
static TaskHandle_t  s_task    = nullptr;
static volatile bool s_wifi_up = false;

static void wifiConnectIfNeeded() {
    if (WiFi.status() == WL_CONNECTED) { s_wifi_up = true; return; }
    s_wifi_up = false;
    WiFi.mode(WIFI_STA);
    WiFi.begin(kWifiSsid, kWifiPassword);
    const uint32_t end = millis() + 15000U;
    while (WiFi.status() != WL_CONNECTED && millis() < end) {
        vTaskDelay(pdMS_TO_TICKS(250));
    }
    s_wifi_up = (WiFi.status() == WL_CONNECTED);
}

static bool postOne(const SheetsSample& s) {
    if (WiFi.status() != WL_CONNECTED) return false;

    WiFiClientSecure client;
    client.setInsecure();

    HTTPClient http;
    http.setConnectTimeout(5000);
    http.setTimeout(5000);
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);

    char url[512];
    snprintf(url, sizeof(url),
             "%s?t=%lu&y=%.3f&p=%.3f&r=%.3f",
             kSheetsUrl,
             static_cast<unsigned long>(s.t_ms),
             static_cast<double>(s.yaw),
             static_cast<double>(s.pitch),
             static_cast<double>(s.roll));

    if (!http.begin(client, url)) return false;
    const int code = http.GET();
    http.end();
    return code == 200;
}

static void taskFn(void* /*arg*/) {
    wifiConnectIfNeeded();
    SheetsSample s{};
    uint32_t last_retry_ms = 0;
    for (;;) {
        if (WiFi.status() != WL_CONNECTED) {
            if (millis() - last_retry_ms > 15000U) {
                last_retry_ms = millis();
                wifiConnectIfNeeded();
            }
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        if (xQueueReceive(s_queue, &s, pdMS_TO_TICKS(500)) == pdTRUE) {
            (void)postOne(s);
        }
    }
}

void begin() {
    s_queue = xQueueCreate(kSheetsQueueDepth, sizeof(SheetsSample));
    if (s_queue == nullptr) return;
    xTaskCreatePinnedToCore(taskFn, "sheets", 8192, nullptr, 1, &s_task, 0);
}

bool enqueue(const SheetsSample& s) {
    if (s_queue == nullptr) return false;
    if (xQueueSend(s_queue, &s, 0) == pdTRUE) return true;
    SheetsSample dump;
    (void)xQueueReceive(s_queue, &dump, 0);
    return xQueueSend(s_queue, &s, 0) == pdTRUE;
}

bool wifiUp() { return s_wifi_up; }

}  // namespace sheets
