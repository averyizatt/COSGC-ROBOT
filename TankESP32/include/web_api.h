#pragma once

// Register all HTTP routes with the AsyncWebServer and start the WiFi AP.
// Call this once from setup() — it calls startWiFi() internally.
void setupWebServer();
