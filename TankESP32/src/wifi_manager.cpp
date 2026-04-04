#include "wifi_manager.h"
#include "esp_bt.h"

// WiFi AP credentials (open network — no password, any device can connect instantly)
static const char* WIFI_SSID = "TankBot-AP";

static const IPAddress AP_IP(192, 168, 4, 1);
static const IPAddress AP_GW(192, 168, 4, 1);
static const IPAddress AP_MASK(255, 255, 255, 0);

// Single server instance — shared with web_api.cpp via the extern in wifi_manager.h
AsyncWebServer server(80);

// Captive-portal DNS server: responds to every DNS query with AP_IP so phones
// automatically pop up the dashboard and don't route browser traffic to cellular.
static DNSServer dnsServer;
static bool dnsRunning = false;

// Shared guard flag — true only while the server is operational
volatile bool serverActive = false;

// ── WiFi event diagnostics ─────────────────────────────────────────────────
static void onWiFiEvent(WiFiEvent_t event) {
    switch (event) {
        case ARDUINO_EVENT_WIFI_AP_START:
            Serial.println("[WiFi] AP started");
            break;
        case ARDUINO_EVENT_WIFI_AP_STOP:
            Serial.println("[WiFi] AP stopped");
            break;
        case ARDUINO_EVENT_WIFI_AP_STACONNECTED:
            Serial.printf("[WiFi] Client connected — %d connected\n",
                          WiFi.softAPgetStationNum());
            break;
        case ARDUINO_EVENT_WIFI_AP_STADISCONNECTED:
            Serial.printf("[WiFi] Client disconnected — %d remaining\n",
                          WiFi.softAPgetStationNum());
            break;
        case ARDUINO_EVENT_WIFI_AP_STAIPASSIGNED:
            Serial.println("[WiFi] Client got DHCP IP");
            break;
        default: break;
    }
}

void startWiFi() {
    Serial.printf("[WiFi] Starting AP... (heap: %u)\n", ESP.getFreeHeap());

    // ── Kill Bluetooth completely ───────────────────────────────────────────
    // Bluepad32 auto-starts BT during initArduino().
    // BT and WiFi share the ESP32 radio — BT must be fully released for AP to work.
    btStop();
    esp_bt_controller_disable();
    esp_bt_controller_deinit();   // Frees ~60 KB of BT RAM
    Serial.printf("[WiFi] Bluetooth released (heap: %u)\n", ESP.getFreeHeap());
    delay(100);

    // ── Register event diagnostics ─────────────────────────────────────────
    WiFi.onEvent(onWiFiEvent);

    // ── Full radio reset ───────────────────────────────────────────────────
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(300);

    // ── Configure and start Access Point ──────────────────────────────────
    WiFi.mode(WIFI_AP);
    delay(200);
    WiFi.softAPConfig(AP_IP, AP_GW, AP_MASK);

    // 13 dBm: good range for a nearby phone/laptop while still reducing
    // peak current draw vs the 20 dBm default.
    WiFi.setTxPower(WIFI_POWER_13dBm);

    // Channel 1, open network, visible SSID, max 4 clients
    // Channel 1 is the most universally supported; avoids DFS issues on ch 6/11.
    bool apOK = WiFi.softAP(WIFI_SSID, "", 1, 0, 4);
    if (!apOK) {
        Serial.println("[WiFi] First AP start failed — retrying on ch 6...");
        WiFi.mode(WIFI_OFF);
        delay(1500);
        WiFi.mode(WIFI_AP);
        delay(300);
        WiFi.softAPConfig(AP_IP, AP_GW, AP_MASK);
        WiFi.setTxPower(WIFI_POWER_13dBm);
        apOK = WiFi.softAP(WIFI_SSID, "", 6, 0, 4);
    }

    // Wait for AP + DHCP server to fully initialise before clients try to connect.
    // 2 s is enough for most phones; going shorter causes "connected, no IP" symptoms.
    delay(2000);

    IPAddress myIP = WiFi.softAPIP();
    Serial.printf("[WiFi] AP %s — IP: %s  MAC: %s\n",
        apOK ? "OK" : "FAILED",
        myIP.toString().c_str(),
        WiFi.softAPmacAddress().c_str());

    if (myIP.toString() == "0.0.0.0") {
        Serial.println("[WiFi] WARNING: IP 0.0.0.0 — DHCP not ready, waiting extra 2 s");
        delay(2000);
        myIP = WiFi.softAPIP();
        Serial.printf("[WiFi] Retry IP: %s\n", myIP.toString().c_str());
    }

    // ── Captive-portal DNS server ──────────────────────────────────────────
    // Responds to ALL domain lookups with AP_IP — this is what makes iOS/Android
    // auto-pop the dashboard and makes the browser work even if the user doesn't
    // know to type 192.168.4.1 manually.
    dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
    dnsRunning = dnsServer.start(53, "*", AP_IP);
    Serial.printf("[WiFi] DNS captive-portal: %s\n", dnsRunning ? "OK" : "FAILED");

    server.begin();
    serverActive = true;
    Serial.printf("[WiFi] Dashboard: http://%s/  (or any URL — captive portal)\n",
                  myIP.toString().c_str());
}

void loopWiFi() {
    if (dnsRunning) dnsServer.processNextRequest();
}

void stopWiFi() {
    serverActive = false;  // Tell all handlers to bail immediately
    delay(100);            // Let any in-flight response finish
    if (dnsRunning) {
        dnsServer.stop();
        dnsRunning = false;
    }
    server.end();
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_OFF);
    Serial.println("[WiFi] Radio off");
}
