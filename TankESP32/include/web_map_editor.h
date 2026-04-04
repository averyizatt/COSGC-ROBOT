#pragma once
#include <ESPAsyncWebServer.h>

// Serves the canvas-based pre-map editor page at GET /map/draw
void handleMapEditor(AsyncWebServerRequest *request);
