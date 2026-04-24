#pragma once // Editor's note: This still feels so weird
#include <ESPAsyncWebServer.h>

// Serves the canvas-based pre-map editor page at GET /control
void handleSokCtrl(AsyncWebServerRequest *request);
