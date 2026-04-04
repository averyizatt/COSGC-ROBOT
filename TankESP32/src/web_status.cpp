// web_status.cpp — implementation moved to wifi_manager.cpp and web_api.cpp
// This stub keeps the file present so nothing in the build system breaks.
#include "web_status.h"

// (no definitions here — see wifi_manager.cpp and web_api.cpp)
#if 0  // BEGIN dead code block — preserved for reference only
void setupWebServer_DEAD() {
    // ---- Register routes (once) ----

    // Tiny test endpoint
    server.on("/ping", HTTP_GET, [](AsyncWebServerRequest *request){
        if (!serverActive) { request->send(503); return; }
        Serial.printf("[HTTP] GET /ping from %s\n", request->client()->remoteIP().toString().c_str());
        request->send(200, "text/plain", "OK");
    });

    // Status JSON API
    server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest *request){
        if (!serverActive) { request->send(503); return; }
        Serial.printf("[HTTP] GET /api/status from %s\n", request->client()->remoteIP().toString().c_str());

        const char* modeStr =
            currentMode == MODE_RC_CONTROL   ? "RC Control" :
            currentMode == MODE_UART_CONTROL ? "UART Control" :
            currentMode == MODE_AUTONOMOUS   ? "Autonomous" : "Simple Auto";

        const char* navStr = autoNav.getStateString();

        // Sensor health strings
        const char* hlNames[] = {"OK","DEGRADED","FAILED"};
        const char* usHealthL = hlNames[ultrasonicLeft.getHealth()];
        const char* usHealthR = hlNames[ultrasonicRight.getHealth()];

        char buf[1024];
        snprintf(buf, sizeof(buf),
            "{"
            "\"mode\":\"%s\","
            "\"navState\":\"%s\","
            "\"odomX\":%.1f,\"odomY\":%.1f,\"odomTheta\":%.2f,"
            "\"speed\":%.1f,"
            "\"explored\":%.1f,"
            "\"motorCurrentL\":%.2f,\"motorCurrentR\":%.2f,"
            "\"leftPulses\":%lu,\"rightPulses\":%lu,"
            "\"leftDist\":%.1f,\"rightDist\":%.1f,"
            "\"leftRPM\":%.1f,\"rightRPM\":%.1f,"
            "\"stallL\":%s,\"stallR\":%s,"
            "\"usLeft\":%.1f,\"usRight\":%.1f,\"usFwd\":%.1f,"
            "\"wallAngle\":%.1f,\"gapWidth\":%.1f,"
            "\"usHealthL\":\"%s\",\"usHealthR\":\"%s\","
            "\"plannerMode\":\"%s\",\"plannerPath\":%s,\"plannerDist\":%.1f,"
            "\"terrainBoost\":%s,"
            "\"accelX\":%.2f,\"accelY\":%.2f,\"accelZ\":%.2f,"
            "\"gyroX\":%.1f,\"gyroY\":%.1f,\"gyroZ\":%.1f,"
            "\"imuTemp\":%.1f,\"espTemp\":%.1f,"
            "\"upsideDown\":%s,"
            "\"mapShifts\":%d,"
            "\"uptime\":%lu,"
            "\"freeHeap\":%u,"
            "\"clients\":%d"
            "}",
            modeStr, navStr,
            odomX, odomY, odomTheta,
            encoders.getSpeed(),
            envMap.getExplorationPercent(),
            motors.getEstimatedCurrentA(), motors.getEstimatedCurrentB(),
            encoders.getLeftPulses(), encoders.getRightPulses(),
            encoders.getLeftDistance(), encoders.getRightDistance(),
            encoders.getLeftRPM(), encoders.getRightRPM(),
            encoders.isLeftStalled() ? "true" : "false",
            encoders.isRightStalled() ? "true" : "false",
            sensorData.distanceLeft, sensorData.distanceRight, sensorData.distance,
            sensorData.wallAngle, sensorData.gapWidth,
            usHealthL, usHealthR,
            pathPlanner.getModeString(),
            pathPlanner.hasValidPath() ? "true" : "false",
            pathPlanner.getDistanceToGoal(),
            autoNav.isTerrainBoostActive() ? "true" : "false",
            sensorData.accelX, sensorData.accelY, sensorData.accelZ,
            sensorData.gyroX, sensorData.gyroY, sensorData.gyroZ,
            sensorData.temperature, motors.getESPTemperature(),
            motors.isUpsideDown() ? "true" : "false",
            envMap.getMapShiftCount(),
            millis() / 1000,
            ESP.getFreeHeap(),
            WiFi.softAPgetStationNum()
        );

        AsyncWebServerResponse *response = request->beginResponse(200, "application/json", buf);
        response->addHeader("Access-Control-Allow-Origin", "*");
        request->send(response);
    });

    // Map JSON API — uses response stream to avoid one giant allocation
    server.on("/api/map", HTTP_GET, [](AsyncWebServerRequest *request){
        if (!serverActive) { request->send(503); return; }

        // If the main loop is mid-write on the grid, skip to avoid crash
        if (envMap.busy) {
            request->send(503, "application/json", "{\"error\":\"map busy\"}");
            return;
        }

        Serial.printf("[HTTP] GET /api/map from %s\n", request->client()->remoteIP().toString().c_str());

        AsyncResponseStream *response = request->beginResponseStream("application/json");
        response->addHeader("Access-Control-Allow-Origin", "*");

        response->printf("{\"w\":%d,\"h\":%d,\"rx\":%d,\"ry\":%d,\"d\":\"",
            MAP_WIDTH, MAP_HEIGHT, envMap.getRobotX(), envMap.getRobotY());

        char row[MAP_WIDTH + 1];
        for (int y = 0; y < MAP_HEIGHT; y++) {
            for (int x = 0; x < MAP_WIDTH; x++) {
                uint8_t cell = envMap.getCell(x, y);
                if (cell == CELL_PIT)              { row[x] = 'v'; continue; }
                if (cell == CELL_HILL)             { row[x] = '^'; continue; }
                if (cell == CELL_HAZARD)           { row[x] = '!'; continue; }
                if (cell > THRESH_OCCUPIED)        { row[x] = '#'; continue; }
                if (cell > THRESH_LIKELY_OCC)      { row[x] = 'x'; continue; }
                if (cell >= THRESH_LIKELY_FREE)    { row[x] = '.'; continue; }  // unknown

                // Free or likely-free cell — check if it's a frontier
                // (borders at least one unknown cell = edge of explored territory)
                bool frontier = false;
                for (int ox = -1; ox <= 1 && !frontier; ox++) {
                    for (int oy = -1; oy <= 1 && !frontier; oy++) {
                        if (!ox && !oy) continue;
                        int nx = x + ox, ny = y + oy;
                        if (nx >= 0 && nx < MAP_WIDTH && ny >= 0 && ny < MAP_HEIGHT) {
                            uint8_t nc = envMap.getCell(nx, ny);
                            if (nc >= THRESH_LIKELY_FREE && nc <= THRESH_LIKELY_OCC) frontier = true;
                        }
                    }
                }
                row[x] = frontier ? 'f' : (cell < THRESH_FREE ? ' ' : '-');
            }
            row[MAP_WIDTH] = '\0';
            response->print(row);
        }
        response->print("\"}");

        request->send(response);
    });

    // Dashboard
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        if (!serverActive) { request->send(503); return; }
        Serial.printf("[HTTP] GET / from %s\n", request->client()->remoteIP().toString().c_str());
        handleDashboard(request);
    });

    // All unknown → redirect to dashboard
    server.onNotFound([](AsyncWebServerRequest *request){
        Serial.printf("[HTTP] 404 %s from %s\n", request->url().c_str(), request->client()->remoteIP().toString().c_str());
        request->redirect("http://192.168.4.1/");
    });

    // Start WiFi AP + web server
    startWiFi();
}

void startWiFi() {
    Serial.printf("[WiFi] Starting AP... (heap free: %u)\n", ESP.getFreeHeap());

    // ── Kill Bluetooth completely ──
    // Bluepad32's framework auto-starts BT during initArduino().
    // BT and WiFi share the ESP32's radio — BT must be fully off for AP to work.
    btStop();
    esp_bt_controller_disable();
    esp_bt_controller_deinit();   // Release all BT memory (~60KB)
    Serial.printf("[WiFi] Bluetooth fully released (heap free: %u)\n", ESP.getFreeHeap());

    // ── Clean WiFi state ──
    WiFi.disconnect(true);   // Drop any lingering STA connection
    WiFi.mode(WIFI_OFF);     // Full radio reset
    delay(200);

    // ── Start Access Point ──
    WiFi.mode(WIFI_AP);
    delay(200);

    IPAddress apIP(192, 168, 4, 1);
    IPAddress gateway(192, 168, 4, 1);
    IPAddress netMsk(255, 255, 255, 0);
    WiFi.softAPConfig(apIP, gateway, netMsk);

    // Reduce TX power to prevent current-spike brownouts (default 20dBm draws ~300mA peaks).
    // WIFI_POWER_8_5dBm is plenty for a nearby phone/laptop and cuts peak draw significantly.
    WiFi.setTxPower(WIFI_POWER_8_5dBm);

    // Open network, channel 6 (less crowded than 1), visible, max 4 clients
    bool apOK = WiFi.softAP(ssid, "", 6, 0, 4);
    if (!apOK) {
        Serial.println("[WiFi] First AP attempt failed, retrying...");
        WiFi.mode(WIFI_OFF);
        delay(1000);
        WiFi.mode(WIFI_AP);
        delay(100);
        WiFi.softAPConfig(apIP, gateway, netMsk);
        apOK = WiFi.softAP(ssid, "", 6, 0, 4);
    }

    delay(1000);  // Let AP + DHCP stabilize

    IPAddress myIP = WiFi.softAPIP();
    Serial.printf("[WiFi] AP %s: %s (MAC: %s)\n",
        apOK ? "OK" : "FAILED",
        myIP.toString().c_str(),
        WiFi.softAPmacAddress().c_str());

    if (myIP.toString() == "0.0.0.0") {
        Serial.println("[WiFi] WARNING: AP IP is 0.0.0.0 — AP did not start properly!");
    }

    server.begin();
    serverActive = true;
    Serial.printf("[WiFi] Dashboard ready at http://%s/\n", myIP.toString().c_str());
}

void stopWiFi() {
    serverActive = false;      // Tell handlers to bail out immediately
    delay(100);                // Let any in-flight handler finish
    server.end();
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_OFF);
    Serial.println("[WiFi] WiFi radio disabled");
}
#endif  // END dead code block
