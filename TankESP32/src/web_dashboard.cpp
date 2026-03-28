#include "web_dashboard.h"

// Full-featured rover dashboard — WiFi runs exclusively (no BT), so size is not a concern.
static const char DASHBOARD_HTML[] PROGMEM = R"rawliteral(<!DOCTYPE html>
<html lang="en"><head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1.0">
<title>ESP32 Rover Dashboard</title>
<style>
:root {
  --bg: #0d1117; --card: #161b22; --border: #21262d; --text: #c9d1d9;
  --dim: #8b949e; --accent: #58a6ff; --green: #3fb950; --red: #f85149;
  --orange: #d29922; --purple: #bc8cff;
}
* { box-sizing: border-box; margin: 0; padding: 0; }
body { background: var(--bg); color: var(--text); font-family: -apple-system, 'Segoe UI', system-ui, sans-serif; font-size: 14px; }
.wrap { max-width: 720px; margin: 0 auto; padding: 12px; }

/* Header */
.hdr { display: flex; align-items: center; justify-content: space-between; padding: 12px 0 8px; border-bottom: 1px solid var(--border); margin-bottom: 12px; }
.hdr h1 { color: var(--accent); font-size: 1.4em; letter-spacing: 0.5px; }
.hdr h1 span { font-size: 0.6em; color: var(--dim); font-weight: 400; display: block; }
.status-dot { width: 10px; height: 10px; border-radius: 50%; display: inline-block; margin-right: 6px; }
.dot-ok { background: var(--green); box-shadow: 0 0 6px var(--green); }
.dot-err { background: var(--red); box-shadow: 0 0 6px var(--red); }
#connbar { font-size: 0.8em; color: var(--dim); text-align: right; }

/* Cards */
.card { background: var(--card); border: 1px solid var(--border); border-radius: 10px; padding: 14px 16px; margin-bottom: 12px; }
.card h2 { color: var(--orange); font-size: 0.85em; text-transform: uppercase; letter-spacing: 1.2px; margin-bottom: 10px; display: flex; align-items: center; gap: 6px; }
.card h2::before { content: ''; display: inline-block; width: 3px; height: 14px; background: var(--orange); border-radius: 2px; }

/* Two-column grid */
.grid { display: grid; grid-template-columns: 1fr 1fr; gap: 4px 20px; }
.row { display: flex; justify-content: space-between; align-items: center; padding: 5px 0; border-bottom: 1px solid #1c2028; }
.row:last-child { border-bottom: none; }
.row .k { color: var(--dim); font-size: 0.85em; }
.row .v { color: #fff; font-weight: 600; font-size: 0.85em; text-align: right; }
.full { grid-column: 1 / -1; }

/* Badges */
.badge { display: inline-block; padding: 2px 10px; border-radius: 12px; font-size: 0.78em; font-weight: 700; }
.b-blue { background: #1a2a3a; color: var(--accent); }
.b-green { background: #1a3a2a; color: var(--green); }
.b-red { background: #3a1a1a; color: var(--red); }
.b-orange { background: #3a2a1a; color: var(--orange); }
.b-purple { background: #2a1a3a; color: var(--purple); }

/* Progress bar */
.pbar { height: 6px; background: #21262d; border-radius: 3px; overflow: hidden; margin-top: 4px; }
.pbar .fill { height: 100%; border-radius: 3px; transition: width 0.5s ease; }
.fill-green { background: var(--green); }
.fill-blue { background: var(--accent); }
.fill-red { background: var(--red); }

/* Map section */
.map-wrap { text-align: center; }
.map-wrap canvas { background: #0d1117; border-radius: 6px; border: 1px solid var(--border); max-width: 100%; image-rendering: pixelated; cursor: crosshair; }
.legend { display: flex; justify-content: center; gap: 14px; margin: 8px 0; font-size: 0.75em; color: var(--dim); }
.legend i { display: inline-block; width: 12px; height: 12px; border-radius: 3px; margin-right: 4px; vertical-align: middle; }
.map-controls { display: flex; justify-content: center; gap: 8px; margin-bottom: 8px; }
button { background: var(--card); color: var(--accent); border: 1px solid var(--accent); border-radius: 6px; padding: 6px 16px; cursor: pointer; font-size: 0.82em; transition: background 0.2s; }
button:hover { background: #1a2a3a; }
button:active { background: #0d1a2a; }
.btn-sm { padding: 4px 10px; font-size: 0.75em; }

/* Stall warning */
.stall-warn { color: var(--red); font-weight: 700; }
.stall-ok { color: var(--green); }

/* Footer */
.foot { text-align: center; color: #30363d; font-size: 0.7em; padding: 16px 0 8px; border-top: 1px solid var(--border); margin-top: 8px; }

/* Responsive: single column on narrow screens */
@media (max-width: 480px) {
  .grid { grid-template-columns: 1fr; }
  .wrap { padding: 8px; }
}
</style>
</head>
<body>
<div class="wrap">

<div class="hdr">
  <h1>ESP32 Rover<span>Dashboard</span></h1>
  <div id="connbar"><span class="status-dot dot-err" id="dot"></span><span id="conntext">Connecting...</span></div>
</div>

<!-- Mode & Navigation -->
<div class="card">
  <h2>Control</h2>
  <div class="grid">
    <div class="row full"><span class="k">Mode</span><span class="v" id="mode">--</span></div>
    <div class="row full"><span class="k">Navigation State</span><span class="v" id="nav">--</span></div>
  </div>
</div>

<!-- Odometry & Motion -->
<div class="card">
  <h2>Odometry &amp; Motion</h2>
  <div class="grid">
    <div class="row"><span class="k">Position X</span><span class="v" id="ox">--</span></div>
    <div class="row"><span class="k">Position Y</span><span class="v" id="oy">--</span></div>
    <div class="row"><span class="k">Heading</span><span class="v" id="ot">--</span></div>
    <div class="row"><span class="k">Speed</span><span class="v" id="spd">--</span></div>
  </div>
</div>

<!-- Encoders -->
<div class="card">
  <h2>Encoders</h2>
  <div class="grid">
    <div class="row"><span class="k">Left Pulses</span><span class="v" id="lp">--</span></div>
    <div class="row"><span class="k">Right Pulses</span><span class="v" id="rp">--</span></div>
    <div class="row"><span class="k">Left Distance</span><span class="v" id="ld">--</span></div>
    <div class="row"><span class="k">Right Distance</span><span class="v" id="rd">--</span></div>
    <div class="row"><span class="k">Left RPM</span><span class="v" id="lr">--</span></div>
    <div class="row"><span class="k">Right RPM</span><span class="v" id="rr">--</span></div>
    <div class="row"><span class="k">Left Stall</span><span class="v" id="sl">--</span></div>
    <div class="row"><span class="k">Right Stall</span><span class="v" id="sr">--</span></div>
  </div>
</div>

<!-- Motors -->
<div class="card">
  <h2>Motors</h2>
  <div class="grid">
    <div class="row"><span class="k">Left Current</span><span class="v" id="mcl">--</span></div>
    <div class="row"><span class="k">Right Current</span><span class="v" id="mcr">--</span></div>
    <div class="row full"><span class="k">Orientation</span><span class="v" id="flip">--</span></div>
  </div>
</div>

<!-- Ultrasonic Sensors -->
<div class="card">
  <h2>Ultrasonics</h2>
  <div class="grid">
    <div class="row"><span class="k">Left</span><span class="v" id="ul">--</span></div>
    <div class="row"><span class="k">Right</span><span class="v" id="ur">--</span></div>
    <div class="row"><span class="k">Forward (min)</span><span class="v" id="uf">--</span></div>
    <div class="row"><span class="k">Wall Angle</span><span class="v" id="wa">--</span></div>
    <div class="row"><span class="k">Gap Width</span><span class="v" id="gw">--</span></div>
    <div class="row"><span class="k">Left Health</span><span class="v" id="uhl">--</span></div>
    <div class="row"><span class="k">Right Health</span><span class="v" id="uhr">--</span></div>
  </div>
</div>

<!-- Path Planner & Terrain -->
<div class="card">
  <h2>Planner &amp; Terrain</h2>
  <div class="grid">
    <div class="row full"><span class="k">Planner Mode</span><span class="v" id="pm">--</span></div>
    <div class="row"><span class="k">Has Path</span><span class="v" id="pp">--</span></div>
    <div class="row"><span class="k">Dist to Goal</span><span class="v" id="pd">--</span></div>
    <div class="row full"><span class="k">Terrain Boost</span><span class="v" id="tb">--</span></div>
  </div>
</div>

<!-- IMU -->
<div class="card">
  <h2>IMU (MPU6050)</h2>
  <div class="grid">
    <div class="row"><span class="k">Accel X</span><span class="v" id="ax">--</span></div>
    <div class="row"><span class="k">Accel Y</span><span class="v" id="ay">--</span></div>
    <div class="row"><span class="k">Accel Z</span><span class="v" id="az">--</span></div>
    <div class="row"><span class="k">Gyro X</span><span class="v" id="gx">--</span></div>
    <div class="row"><span class="k">Gyro Y</span><span class="v" id="gy">--</span></div>
    <div class="row"><span class="k">Gyro Z</span><span class="v" id="gz">--</span></div>
    <div class="row full"><span class="k">IMU Temperature</span><span class="v" id="it">--</span></div>
  </div>
</div>

<!-- System -->
<div class="card">
  <h2>System</h2>
  <div class="grid">
    <div class="row"><span class="k">ESP32 Temperature</span><span class="v" id="et">--</span></div>
    <div class="row"><span class="k">Free Heap</span><span class="v" id="heap">--</span></div>
    <div class="row full">
      <span class="k">Exploration</span>
      <span class="v" id="expl">--</span>
    </div>
    <div class="full"><div class="pbar"><div class="fill fill-green" id="explbar" style="width:0%"></div></div></div>
    <div class="row"><span class="k">Map Shifts</span><span class="v" id="ms">--</span></div>
    <div class="row"><span class="k">WiFi Clients</span><span class="v" id="cli">--</span></div>
    <div class="row full"><span class="k">Uptime</span><span class="v" id="uptime">--</span></div>
  </div>
</div>

<!-- Occupancy Map -->
<div class="card">
  <h2>Occupancy Map</h2>
  <div class="legend">
    <span><i style="background:#c8c8c8"></i>Free</span>
    <span><i style="background:#6e7681"></i>Likely&nbsp;Free</span>
    <span><i style="background:#da6840"></i>Likely&nbsp;Obs</span>
    <span><i style="background:#f85149"></i>Obstacle</span>
    <span><i style="background:#d29922"></i>Hazard</span>
    <span><i style="background:#8b5cf6"></i>Pit</span>
    <span><i style="background:#2ea043"></i>Hill</span>
    <span><i style="background:#00bfff"></i>Frontier</span>
    <span><i style="background:#21262d"></i>Unexplored</span>
    <span><i style="background:#3fb950"></i>Robot</span>
  </div>
  <div class="map-controls">
    <button id="mapBtn" onclick="toggleMap()">Start Map</button>
    <button class="btn-sm" onclick="mapZoom(1)">Zoom +</button>
    <button class="btn-sm" onclick="mapZoom(-1)">Zoom -</button>
  </div>
  <div class="map-wrap"><canvas id="map" width="320" height="256"></canvas></div>
</div>

<div class="foot">ESP32 Rover &bull; WiFi Dashboard &bull; Auto-refresh 1.5s</div>
</div>

<script>
const $=id=>document.getElementById(id);
function fmt(v,u,d){return v!==undefined&&v!==null?(d!==undefined?v.toFixed(d):v)+(u||''):'--';}
function stall(v){return v?'<span class="stall-warn">STALLED</span>':'<span class="stall-ok">OK</span>';}
function modeBadge(m){
  const c={'RC Control':'b-blue','UART Control':'b-orange','Autonomous':'b-green','Simple Auto':'b-purple'};
  return '<span class="badge '+(c[m]||'b-blue')+'">'+m+'</span>';
}
function navBadge(n){return '<span class="badge b-orange">'+n+'</span>';}
function healthBadge(h){
  if(h==='FAILED')return '<span class="badge b-red">FAILED</span>';
  if(h==='DEGRADED')return '<span class="badge b-orange">DEGRADED</span>';
  return '<span class="badge b-green">OK</span>';
}
function fmtTime(sec){
  let s=Math.floor(sec),m=Math.floor(s/60),h=Math.floor(m/60),d=Math.floor(h/24);
  h%=24;m%=60;s%=60;
  if(d)return d+'d '+h+'h '+m+'m';
  if(h)return h+'h '+m+'m '+s+'s';
  if(m)return m+'m '+s+'s';
  return s+'s';
}
function fmtHeap(b){
  if(b>1024*1024)return (b/(1024*1024)).toFixed(1)+' MB';
  if(b>1024)return (b/1024).toFixed(1)+' KB';
  return b+' B';
}
function tempColor(t){return t>70?'var(--red)':t>55?'var(--orange)':'var(--green)';}

async function poll(){
  try{
    let r=await fetch('/api/status');
    let d=await r.json();

    $('mode').innerHTML=modeBadge(d.mode);
    $('nav').innerHTML=navBadge(d.navState);

    $('ox').textContent=fmt(d.odomX,' cm',1);
    $('oy').textContent=fmt(d.odomY,' cm',1);
    $('ot').textContent=fmt(d.odomTheta,' rad',3);
    $('spd').textContent=fmt(d.speed,' cm/s',1);

    $('lp').textContent=d.leftPulses;
    $('rp').textContent=d.rightPulses;
    $('ld').textContent=fmt(d.leftDist,' cm',1);
    $('rd').textContent=fmt(d.rightDist,' cm',1);
    $('lr').textContent=fmt(d.leftRPM,' RPM',1);
    $('rr').textContent=fmt(d.rightRPM,' RPM',1);
    $('sl').innerHTML=stall(d.stallL);
    $('sr').innerHTML=stall(d.stallR);

    $('mcl').textContent=fmt(d.motorCurrentL,' A',2);
    $('mcr').textContent=fmt(d.motorCurrentR,' A',2);
    $('flip').innerHTML=d.upsideDown?'<span class="badge b-red">FLIPPED</span>':'<span class="badge b-green">Normal</span>';

    $('ul').textContent=fmt(d.usLeft,' cm',1);
    $('ur').textContent=fmt(d.usRight,' cm',1);
    $('uf').textContent=fmt(d.usFwd,' cm',1);
    $('wa').textContent=fmt(d.wallAngle,'\u00b0',1);
    $('gw').textContent=fmt(d.gapWidth,' cm',1);
    $('uhl').innerHTML=healthBadge(d.usHealthL);
    $('uhr').innerHTML=healthBadge(d.usHealthR);

    $('pm').innerHTML=navBadge(d.plannerMode||'IDLE');
    $('pp').innerHTML=d.plannerPath?'<span class="badge b-green">YES</span>':'<span class="badge b-red">NO</span>';
    $('pd').textContent=fmt(d.plannerDist,' cm',1);
    $('tb').innerHTML=d.terrainBoost?'<span class="badge b-orange">ACTIVE</span>':'<span class="badge b-green">Off</span>';

    $('ax').textContent=fmt(d.accelX,' g',3);
    $('ay').textContent=fmt(d.accelY,' g',3);
    $('az').textContent=fmt(d.accelZ,' g',3);
    $('gx').textContent=fmt(d.gyroX,'\u00b0/s',1);
    $('gy').textContent=fmt(d.gyroY,'\u00b0/s',1);
    $('gz').textContent=fmt(d.gyroZ,'\u00b0/s',1);
    $('it').textContent=fmt(d.imuTemp,'\u00b0C',1);

    let et=d.espTemp;
    $('et').innerHTML='<span style="color:'+tempColor(et)+'">'+fmt(et,'\u00b0C',1)+'</span>';
    $('heap').textContent=fmtHeap(d.freeHeap);
    let ep=d.explored||0;
    $('expl').textContent=fmt(ep,'%',1);
    $('explbar').style.width=Math.min(ep,100)+'%';
    $('ms').textContent=d.mapShifts;
    $('cli').textContent=d.clients;
    $('uptime').textContent=fmtTime(d.uptime);

    $('dot').className='status-dot dot-ok';
    $('conntext').textContent='Updated '+new Date().toLocaleTimeString();
  }catch(e){
    $('dot').className='status-dot dot-err';
    $('conntext').textContent='Connection lost';
  }
}

/* Map rendering with zoom */
let mapCanvas=$('map'), ctx=mapCanvas.getContext('2d');
let mapRunning=false, mapScale=40, mapTimer=null;

function toggleMap(){
  mapRunning=!mapRunning;
  $('mapBtn').textContent=mapRunning?'Stop Map':'Start Map';
  if(mapRunning){pollMap();mapTimer=setInterval(pollMap,3000);}
  else{clearInterval(mapTimer);mapTimer=null;}
}
function mapZoom(dir){
  mapScale=Math.max(10,Math.min(80,mapScale+dir*5));
  if(mapRunning)pollMap();
}

async function pollMap(){
  try{
    let r=await fetch('/api/map');
    let m=await r.json();
    let w=m.w, h=m.h, s=mapScale;
    mapCanvas.width=w*s; mapCanvas.height=h*s;
    ctx.imageSmoothingEnabled=false;

    // Draw cells — 6 levels of classification
    for(let y=0;y<h;y++){
      for(let x=0;x<w;x++){
        let c=m.d[y*w+x];
        if(c==='#'){ctx.fillStyle='#f85149';}       // Confident obstacle (red)
        else if(c==='x'){ctx.fillStyle='#da6840';}   // Likely obstacle (orange)
        else if(c==='!'){ctx.fillStyle='#d29922';}   // Hazard zone (yellow)
        else if(c==='v'){ctx.fillStyle='#8b5cf6';}   // Pit / drop-off (purple)
        else if(c==='^'){ctx.fillStyle='#2ea043';}   // Hill / slope (green)
        else if(c===' '){ctx.fillStyle='#c8c8c8';}   // Confident free (light gray)
        else if(c==='-'){ctx.fillStyle='#6e7681';}   // Likely free (mid gray)
        else if(c==='f'){ctx.fillStyle='#00bfff';}   // Frontier edge (cyan) — fog-of-war boundary
        else{ctx.fillStyle='#21262d';}               // Unknown / fog (dark)
        ctx.fillRect(x*s,y*s,s,s);
      }
    }
    // Grid lines at higher zoom
    if(s>=3){
      ctx.strokeStyle='rgba(255,255,255,0.04)';
      ctx.lineWidth=0.5;
      for(let x=0;x<=w;x++){ctx.beginPath();ctx.moveTo(x*s,0);ctx.lineTo(x*s,h*s);ctx.stroke();}
      for(let y=0;y<=h;y++){ctx.beginPath();ctx.moveTo(0,y*s);ctx.lineTo(w*s,y*s);ctx.stroke();}
    }
    // Robot marker
    ctx.beginPath();
    ctx.arc(m.rx*s+s/2, m.ry*s+s/2, Math.max(s*1.2,3), 0, Math.PI*2);
    ctx.fillStyle='#3fb950';
    ctx.fill();
    ctx.strokeStyle='#0d1117';
    ctx.lineWidth=1;
    ctx.stroke();
  }catch(e){}
}

/* Start polling */
setInterval(poll,1500);
setTimeout(poll,300);
</script>
</body></html>)rawliteral";

static const size_t DASHBOARD_LEN = sizeof(DASHBOARD_HTML) - 1;

void handleDashboard(AsyncWebServerRequest *request) {
    // Stream the PROGMEM HTML in small chunks to avoid large heap allocation
    AsyncWebServerResponse *response = request->beginChunkedResponse("text/html",
        [](uint8_t *buffer, size_t maxLen, size_t index) -> size_t {
            if (index >= DASHBOARD_LEN) return 0;  // Done
            size_t remaining = DASHBOARD_LEN - index;
            size_t toSend = (remaining < maxLen) ? remaining : maxLen;
            memcpy_P(buffer, DASHBOARD_HTML + index, toSend);
            return toSend;
        });
    request->send(response);
}
