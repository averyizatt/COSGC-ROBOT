#include "web_api.h"
#include "wifi_manager.h"
#include "web_dashboard.h"
#include "web_map_editor.h"
#include "config.h"
#include "globals.h"
#include "autonomous_nav.h"
#include "occupancy_map.h"
#include "hall_encoder.h"
#include "motor_control.h"
#include "mpu6050_sensor.h"
#include "uart_comm.h"
#include "ultrasonic_sensor.h"
#include "path_planner.h"
#include "sok_ctrl.h"

extern AutonomousNav autoNav;
extern SensorData sensorData;
extern MPU6050Sensor imu;
extern UltrasonicSensor ultrasonicLeft;
extern UltrasonicSensor ultrasonicRight;
extern PathPlanner pathPlanner;

// ---------------------------------------------------------------------------
// Route handlers
// ---------------------------------------------------------------------------

static void handlePing(AsyncWebServerRequest *request) {
    if (!serverActive) { request->send(503); return; }
    Serial.printf("[HTTP] GET /ping from %s\n",
                  request->client()->remoteIP().toString().c_str());
    request->send(200, "text/plain", "OK");
}

static void handleStatus(AsyncWebServerRequest *request) {
    if (!serverActive) { request->send(503); return; }
    Serial.printf("[HTTP] GET /api/status from %s\n",
                  request->client()->remoteIP().toString().c_str());

    // Human-readable mode name — mirrors the 5-mode button cycle in main.cpp
    const char* modeStr =
        currentMode == MODE_RC_CONTROL   ? "RC Control"  :
        currentMode == MODE_UART_CONTROL ? "Straight Control":
        currentMode == MODE_SOK_CONTROL   ? "SOK Control"  :
        currentMode == MODE_AUTONOMOUS   ? "Autonomous"  :
        currentMode == MODE_SIMPLE_AUTO  ? "Simple Auto" :
        currentMode == MODE_WALL_FOLLOW  ? "Wall Follow" :
        currentMode == MODE_PREMAP_NAV  ? "PREMAP" : "Unknown";

    const char* navStr = autoNav.getStateString();

    const char* hlNames[] = {"OK", "DEGRADED", "FAILED"};
    const char* usHealthL = hlNames[ultrasonicLeft.getHealth()];
    const char* usHealthR = hlNames[ultrasonicRight.getHealth()];

    char buf[1100];
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

    AsyncWebServerResponse *response =
        request->beginResponse(200, "application/json", buf);
    response->addHeader("Access-Control-Allow-Origin", "*");
    request->send(response);
}

static void handleMap(AsyncWebServerRequest *request) {
    if (!serverActive) { request->send(503); return; }

    if (envMap.busy) {
        request->send(503, "application/json", "{\"error\":\"map busy\"}");
        return;
    }

    Serial.printf("[HTTP] GET /api/map from %s\n",
                  request->client()->remoteIP().toString().c_str());

    AsyncResponseStream *response =
        request->beginResponseStream("application/json");
    response->addHeader("Access-Control-Allow-Origin", "*");

    response->printf("{\"w\":%d,\"h\":%d,\"rx\":%d,\"ry\":%d,\"d\":\"",
        MAP_WIDTH, MAP_HEIGHT, envMap.getRobotX(), envMap.getRobotY());

    char row[MAP_WIDTH + 1];
    for (int y = 0; y < MAP_HEIGHT; y++) {
        for (int x = 0; x < MAP_WIDTH; x++) {
            uint8_t cell = envMap.getCell(x, y);
            if      (cell == CELL_PIT)           { row[x] = 'v'; continue; }
            if      (cell == CELL_HILL)          { row[x] = '^'; continue; }
            if      (cell == CELL_HAZARD)        { row[x] = '!'; continue; }
            if      (cell > THRESH_OCCUPIED)     { row[x] = '#'; continue; }
            if      (cell > THRESH_LIKELY_OCC)   { row[x] = 'x'; continue; }
            if      (cell >= THRESH_LIKELY_FREE) { row[x] = '.'; continue; }

            // Free cell — mark as frontier if it borders any unknown cell
            bool frontier = false;
            for (int ox = -1; ox <= 1 && !frontier; ox++) {
                for (int oy = -1; oy <= 1 && !frontier; oy++) {
                    if (!ox && !oy) continue;
                    int nx = x + ox, ny = y + oy;
                    if (nx >= 0 && nx < MAP_WIDTH && ny >= 0 && ny < MAP_HEIGHT) {
                        uint8_t nc = envMap.getCell(nx, ny);
                        if (nc >= THRESH_LIKELY_FREE && nc <= THRESH_LIKELY_OCC)
                            frontier = true;
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
}

// ---------------------------------------------------------------------------
// POST /api/premap?rx=N&ry=N
// Body: exactly MAP_WIDTH*MAP_HEIGHT bytes, row-major flat string of cell chars.
//   '#' = wall/obstacle  ' ' = free  '!' = hazard
//   'v' = pit            '^' = hill  anything else = unknown (127)
// Seeds the occupancy map; the robot uses it when switched to Premap Nav mode.
// ---------------------------------------------------------------------------
static void handlePremapBody(AsyncWebServerRequest *request, uint8_t *data,
                             size_t len, size_t index, size_t total) {
    // Accumulate body into a heap buffer stored on the request object.
    // AsyncWebServer calls this handler potentially in multiple chunks.
    if (index == 0) {
        // First chunk — allocate accumulation buffer
        char *buf = (char*)malloc(MAP_WIDTH * MAP_HEIGHT + 1);
        if (!buf) { request->send(500, "text/plain", "OOM"); return; }
        memset(buf, '?', MAP_WIDTH * MAP_HEIGHT);
        buf[MAP_WIDTH * MAP_HEIGHT] = '\0';
        request->_tempObject = (void*)buf;
    }
    // Copy this chunk into our buffer (bounds-checked)
    char *buf = (char*)request->_tempObject;
    if (buf) {
        size_t copyStart = index < (size_t)(MAP_WIDTH * MAP_HEIGHT) ? index : (size_t)(MAP_WIDTH * MAP_HEIGHT);
        size_t copyLen   = (copyStart + len > (size_t)(MAP_WIDTH * MAP_HEIGHT))
                           ? (size_t)(MAP_WIDTH * MAP_HEIGHT) - copyStart : len;
        memcpy(buf + copyStart, data, copyLen);
    }
}

static void handlePremapComplete(AsyncWebServerRequest *request) {
    if (!serverActive) { request->send(503); return; }

    char *body = (char*)request->_tempObject;
    if (!body) { request->send(400, "text/plain", "No body"); return; }

    // Parse robot start position from query params (default = map center)
    int rx = request->hasParam("rx") ? request->getParam("rx")->value().toInt() : MAP_CENTER_X;
    int ry = request->hasParam("ry") ? request->getParam("ry")->value().toInt() : MAP_CENTER_Y;
    rx = constrain(rx, 0, MAP_WIDTH  - 1);
    ry = constrain(ry, 0, MAP_HEIGHT - 1);

    Serial.printf("[Premap] Seeding map (robot start: %d,%d)\n", rx, ry);

    // Lock map against async readers, reset to unknown, then seed
    envMap.busy = true;
    envMap.reset();

    for (int y = 0; y < MAP_HEIGHT; y++) {
        for (int x = 0; x < MAP_WIDTH; x++) {
            char ch = body[y * MAP_WIDTH + x];
            uint8_t val;
            switch (ch) {
                case '#': val = CELL_OCCUPIED; break;
                case ' ': val = CELL_FREE;     break;
                case '!': val = CELL_HAZARD;   break;
                case 'v': val = CELL_PIT;      break;
                case '^': val = CELL_HILL;     break;
                default:  val = CELL_UNKNOWN;  break;
            }
            if (val != CELL_UNKNOWN) envMap.setCell(x, y, val);
        }
    }

    // Position the robot at the user-specified start cell.
    // setOdometryPosition expects world-cm relative to map origin (center cell).
    float worldX = (rx - MAP_CENTER_X) * CELL_SIZE_CM;
    float worldY = (ry - MAP_CENTER_Y) * CELL_SIZE_CM;
    envMap.setOdometryPosition(worldX, worldY, 0.0f);

    envMap.busy = false;

    free(body);
    request->_tempObject = nullptr;

    Serial.printf("[Premap] Done — switch robot to Premap Nav (mode 5) to use it\n");
    AsyncWebServerResponse *resp = request->beginResponse(200, "application/json",
        "{\"ok\":true,\"msg\":\"Map seeded. Switch to Premap Nav mode.\"}");
    resp->addHeader("Access-Control-Allow-Origin", "*");
    request->send(resp);
}

// ---------------------------------------------------------------------------
// Route registration — call once from setup()
// ---------------------------------------------------------------------------

void setupWebServer() {
    server.on("/ping", HTTP_GET, handlePing);

    server.on("/api/status", HTTP_GET, handleStatus);

    server.on("/api/map", HTTP_GET, handleMap);

    // Pre-map editor UI
    server.on("/map/draw", HTTP_GET, [](AsyncWebServerRequest *request){
        if (!serverActive) { request->send(503); return; }
        Serial.printf("[HTTP] GET /map/draw from %s\n",
                      request->client()->remoteIP().toString().c_str());
        handleMapEditor(request);
    });

    // Pre-map data submission (POST body = flat cell string, rx/ry as params)
    server.on("/api/premap", HTTP_POST,
        handlePremapComplete,   // called when all body chunks received
        nullptr,                // upload handler (files) — not needed
        handlePremapBody        // body chunk handler
    );

    server.on("/control", HTTP_GET, [](AsyncWebServerRequest *request){
        if (!serverActive) { request->send(503); return; }
        Serial.printf("[HTTP] GET /control from %s\n",
                      request->client()->remoteIP().toString().c_str());
        handleSokCtrl(request);
    });

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        if (!serverActive) { request->send(503); return; }
        Serial.printf("[HTTP] GET / from %s\n",
                      request->client()->remoteIP().toString().c_str());
        handleDashboard(request);
    });

    // ── Captive-portal detection endpoints ─────────────────────────────────
    // These are the URLs that iOS, Android, and Windows probe instantly after
    // connecting to a WiFi network to detect whether it has internet access.
    // By redirecting them to our dashboard, the OS pops up the page automatically
    // instead of leaving the user to manually type 192.168.4.1.

    // Android: expects HTTP 204 or redirects for captive portal detection
    server.on("/generate_204", HTTP_GET, [](AsyncWebServerRequest *r){
        r->redirect("http://192.168.4.1/");
    });
    server.on("/gen_204", HTTP_GET, [](AsyncWebServerRequest *r){
        r->redirect("http://192.168.4.1/");
    });

    // iOS / macOS: probes this URL and looks for a redirect
    server.on("/hotspot-detect.html", HTTP_GET, [](AsyncWebServerRequest *r){
        r->redirect("http://192.168.4.1/");
    });
    server.on("/library/test/success.html", HTTP_GET, [](AsyncWebServerRequest *r){
        r->redirect("http://192.168.4.1/");
    });

    // Windows: NCSI (Network Connectivity Status Indicator) probe
    server.on("/ncsi.txt", HTTP_GET, [](AsyncWebServerRequest *r){
        r->redirect("http://192.168.4.1/");
    });
    server.on("/connecttest.txt", HTTP_GET, [](AsyncWebServerRequest *r){
        r->redirect("http://192.168.4.1/");
    });

    // Catch-all: any unrecognised URL → dashboard (keeps captive portal working)
    server.onNotFound([](AsyncWebServerRequest *request){
        Serial.printf("[HTTP] 404→redirect %s from %s\n",
                      request->url().c_str(),
                      request->client()->remoteIP().toString().c_str());
        request->redirect("http://192.168.4.1/");
    });

    // Start the WiFi AP and server after all routes are registered
    startWiFi();
}
