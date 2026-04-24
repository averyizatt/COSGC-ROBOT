#include "web_map_editor.h"

// ─────────────────────────────────────────────────────────────────────────────
// Pre-map editor page — stored in flash (PROGMEM) to save heap.
// Grid is MAP_WIDTH × MAP_HEIGHT (10 × 10), each cell = 50 cm.
// Cell encoding matches /api/map: '#'=wall ' '=free '!'=hazard 'v'=pit '^'=hill
// ─────────────────────────────────────────────────────────────────────────────
static const char SOK_CTRL_HTML[] PROGMEM = R"rawliteral(<!doctype html>
<html lang="en">
    <head>
        <meta charset="UTF-8" />
        <meta name="viewport" content="width=device-width,initial-scale=1.0" />
        <title>Schmetterling — Control</title>

        <style>
            :root {
                --bg: #080c10;
                --card: #0f1419;
                --border: #1e252e;
                --text: #cdd6e0;
                --dim: #7a8899;
                --accent: #4da6ff;
            }

            /* reset */
            * {
                box-sizing: border-box;
                margin: 0;
                padding: 0;
            }

            body {
                background: var(--bg);
                color: var(--text);
                font-family: -apple-system, "Segoe UI", system-ui, sans-serif;
            }

            /* layout */
            .wrap {
                max-width: 700px;
                margin: 0 auto;
                padding: 12px;
            }

            /* header */
            .hdr {
                display: flex;
                justify-content: space-between;
                align-items: center;
                background: var(--card);
                border: 1px solid var(--border);
                border-radius: 12px;
                padding: 12px 16px;
                margin-bottom: 12px;
            }

            .hdr h1 {
                font-size: 1.1em;
                color: var(--accent);
            }

            a.back {
                text-decoration: none;
                color: var(--accent);
                font-size: 0.85em;
            }

            /* joystick area */
            .joystick-area {
                width: 100%;
                aspect-ratio: 1 / 1;
                background: radial-gradient(
                    circle at center,
                    #0f1419 40%,
                    #0a1a1a 100%
                );
                border: 1px solid var(--border);
                border-radius: 16px;
                position: relative;
                overflow: hidden;
                touch-action: none;
                margin-bottom: 12px;
            }

            /* joystick base ring */
            .joystick-base {
                position: absolute;
                inset: 0;
                display: flex;
                align-items: center;
                justify-content: center;
            }

            .joystick-ring {
                width: 70%;
                height: 70%;
                border: 2px dashed rgba(77, 166, 255, 0.2);
                border-radius: 50%;
            }

            /* joystick knob */
            .joystick-knob {
                position: absolute;
                width: 80px;
                height: 80px;
                background: radial-gradient(
                    circle at 30% 30%,
                    #4da6ff,
                    #1b3c5a
                );
                border-radius: 50%;
                border: 2px solid rgba(255, 255, 255, 0.2);
                transform: translate(-50%, -50%);
                left: 50%;
                top: 50%;
                transition: box-shadow 0.1s;
            }

            .joystick-knob.active {
                box-shadow: 0 0 20px rgba(77, 166, 255, 0.5);
            }

            /* ghost knob */
            .joystick-knob.ghost {
                background: radial-gradient(
                    circle at 30% 30%,
                    #ffffff22,
                    #ffffff08
                );
                border: 2px dashed rgba(255, 255, 255, 0.25);
                box-shadow: none;
                opacity: 0.6;
                pointer-events: none;
                backdrop-filter: blur(2px);
            }

            /* buttons */
            .button-row {
                display: flex;
                gap: 10px;
                justify-content: center;
            }

            .ctrl-btn {
                flex: 1;
                height: 80px;
                background: var(--card);
                border: 1.5px solid var(--border);
                border-radius: 10px;
                cursor: pointer;
                position: relative;
                transition: all 0.15s;
            }

            .ctrl-btn:hover {
                border-color: var(--accent);
                background: rgba(77, 166, 255, 0.08);
            }

            /* CSS arrows */
            .ctrl-btn.left::before,
            .ctrl-btn.right::before {
                content: "";
                position: absolute;
                top: 50%;
                left: 50%;
                width: 0;
                height: 0;
                border-style: solid;
                transform: translate(-50%, -50%);
            }

            .ctrl-btn.left::before {
                border-width: 10px 14px 10px 0;
                border-color: transparent var(--text) transparent transparent;
            }

            .ctrl-btn.right::before {
                border-width: 10px 0 10px 14px;
                border-color: transparent transparent transparent var(--text);
            }

            /* footer */
            .foot {
                text-align: center;
                font-size: 0.7em;
                color: #2e3a48;
                margin-top: 14px;
            }
            .foot a {
                color: var(--accent);
                text-decoration: none;
                font-weight: 600;
            }
        </style>
    </head>

    <body>
        <div class="wrap">
            <div class="hdr">
                <h1>Control</h1>
                <div
                    id="wsStatus"
                    style="
                        font-size: 0.75em;
                        color: #7a8899;
                        display: flex;
                        align-items: center;
                        gap: 6px;
                    "
                >
                    <span
                        id="wsDot"
                        style="
                            width: 8px;
                            height: 8px;
                            border-radius: 50%;
                            background: #ff4d4d;
                            display: inline-block;
                        "
                    ></span>
                    <span>disconnected</span>
                </div>
                <a class="back" href="/">Dashboard</a>
            </div>

            <div class="joystick-area" id="joyArea">
                <div class="joystick-base">
                    <div class="joystick-ring" id="joystick-ring"></div>
                </div>

                <!-- Our desired state -->
                <div class="joystick-knob" id="knob"></div>

                <!-- ESP32's understanding -->
                <div class="joystick-knob ghost" id="ghost"></div>
            </div>

            <div class="button-row">
                <button class="ctrl-btn left"></button>
                <button class="ctrl-btn right"></button>
            </div>

            <div class="foot">
                Schmetterling &bull; Pre-Map Editor &bull;
                <a href="/">Dashboard</a>
            </div>
        </div>

        <script>
            const ghost = document.getElementById("ghost");
            const knob = document.getElementById("knob");
            const area = document.getElementById("joyArea");
            const ring = document.getElementById("joystick-ring");

            let ws;
            let wsReady = false;

            const wsDot = document.getElementById("wsDot");
            const wsStatusText = document.querySelector(
                "#wsStatus span:nth-child(2)",
            );

            function setWSState(state) {
                if (state === "open") {
                    wsDot.style.background = "#4dff88";
                    wsStatusText.textContent = "connected";
                } else if (state === "connecting") {
                    wsDot.style.background = "#ffcc00";
                    wsStatusText.textContent = "connecting";
                } else {
                    wsDot.style.background = "#ff4d4d";
                    wsStatusText.textContent = "disconnected";
                }
            }

            function connectWS() {
                setWSState("connecting");

                ws = new WebSocket(`ws://${location.host}/ws`);

                ws.onopen = () => {
                    wsReady = true;
                    setWSState("open");
                    console.log("WS connected");
                };

                ws.onclose = () => {
                    wsReady = false;
                    setWSState("closed");
                    console.log("WS disconnected, retrying...");
                    setTimeout(connectWS, 1000);
                };

                ws.onerror = () => {
                    wsReady = false;
                    ws.close();
                };

                ws.onmessage = (e) => {
                    handleWSMessage(e.data);
                };
            }

            connectWS();

            let dragging = false;
            area.addEventListener("pointerdown", (e) => {
                dragging = true;
                knob.classList.add("active");
                moveKnob(e);
            });

            window.addEventListener("pointermove", (e) => {
                if (dragging) moveKnob(e);
            });

            window.addEventListener("pointerup", () => {
                dragging = false;
                knob.classList.remove("active");
                knob.style.left = "50%";
                knob.style.top = "50%";

                sendPos(0, 0);
            });

            function moveKnob(e) {
                const radius = ring.offsetWidth / 2;

                const rect = area.getBoundingClientRect();

                // Midpoint
                const x_mid = rect.left + rect.width / 2;
                const y_mid = rect.top + rect.height / 2;

                // Get x relative to center
                let dx = e.clientX - x_mid;
                let dy = e.clientY - y_mid;

                // Clamp within radius
                const mag = Math.sqrt(dx * dx + dy * dy);
                if (mag > radius) {
                    dx *= radius / mag;
                    dy *= radius / mag;
                }

                // Get x/y from relative dx/dy
                const x = dx + rect.width / 2;
                const y = dy + rect.height / 2;

                knob.style.left = x + "px";
                knob.style.top = y + "px";

                sendPos(dx / radius, -dy / radius); // Negate dy to make forwards (up) positive
            }

            let tim = null;
            let qdx = null;
            let qdy = null;

            // Debounce
            function sendPos(dx, dy) {
                // Ignore if debouncing...
                if (tim !== null) {
                    qdx = dx;
                    qdy = dy;
                }

                tim = setTimeout(() => {
                    tim = null;

                    if (qdx === null) return; // No new data to send

                    // Send queued data
                    _sendPos(qdx, qdy);
                    qdx = null;
                    qdy = null;
                }, 50);

                _sendPos(dx, dy);
            }

            // Data format:
            // Comma separated list of power, in
            // the format `<left>,<right>`
            // wherere `left`,`right` are in the range [-255, 255]
            // Raw dx/dy given as normalized [-1, 1] vectors
            function _sendPos(dx, dy) {
                let right = dy + dx;
                let left = dy - dx;

                // Scale left/right to ensure neither value is ouside of [-255, 255]
                const mag = Math.max(Math.abs(left), Math.abs(right));
                if (mag > 1) {
                    left = left / mag;
                    right = right / mag;
                }

                // Unnormalize data
                left = Math.trunc(left * 255);
                right = Math.trunc(right * 255);

                const payload = `${left},${right}`;
                console.log("TX:", payload);

                if (wsReady) {
                    ws.send(payload);
                }

                // Echo test
                // setTimeout(() => {
                //     handleWSMessage(payload);
                // }, 500);
            }

            function handleWSMessage(msg) {
                console.log("RX:", msg);

                const [left, right] = msg.split(",").map(Number);

                // Extract dx/dy from left/right
                let dy = (left - right) / 2;
                let dx = (left + right) / 2;

                // Normalize dx/dy
                dx /= 255;
                dy /= 255;

                // Cap dx/dy
                const mag = Math.sqrt(dx * dx + dy * dy);
                if (mag > 1) {
                    dx /= mag;
                    dy /= mag;
                }

                updateGhost(dx, dy);
            }

            function updateGhost(dx, dy) {
                const radius = ring.offsetWidth / 2;

                const rect = area.getBoundingClientRect();

                const x_mid = rect.width / 2;
                const y_mid = rect.height / 2;

                const x = x_mid + dx * radius;
                const y = y_mid - dy * radius;

                ghost.style.left = x + "px";
                ghost.style.top = y + "px";
            }
        </script>
    </body>
</html>)rawliteral";

static const size_t SOK_CTRL_LEN = sizeof(SOK_CTRL_HTML) - 1;

void handleSokCtrl(AsyncWebServerRequest *request) {
    AsyncWebServerResponse *response = request->beginChunkedResponse("text/html",
        [](uint8_t *buffer, size_t maxLen, size_t index) -> size_t {
            if (index >= SOK_CTRL_LEN) return 0;
            size_t remaining = SOK_CTRL_LEN - index;
            size_t toSend = (remaining < maxLen) ? remaining : maxLen;
            memcpy_P(buffer, SOK_CTRL_HTML + index, toSend);
            return toSend;
        });
    request->send(response);
}
