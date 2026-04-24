#pragma once
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <DNSServer.h>

// The single AsyncWebServer instance — defined in wifi_manager.cpp,
// used by web_api.cpp via this extern declaration.
extern AsyncWebServer server;

// Guards all HTTP handlers against use-after-teardown crashes.
extern volatile bool serverActive;

// Requested values
extern volatile int sok_left;
extern volatile int sok_right;

// Start the WiFi AP, DNS captive-portal server, and async web server.
// Call once — after this, the dashboard is reachable at http://192.168.4.1/
// or by any URL (captive portal DNS redirects everything to the ESP32).
void startWiFi();

// Tear down the web server, DNS server, and turn off the radio.
// Safe to call from button handler or RC command.
void stopWiFi();

// Must be called every loop iteration while WiFi is active.
// Pumps the DNS server so captive-portal responses are sent immediately.
void loopWiFi();
