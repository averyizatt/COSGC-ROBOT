#include "web_map_editor.h"

// ─────────────────────────────────────────────────────────────────────────────
// Pre-map editor page — stored in flash (PROGMEM) to save heap.
// Grid is MAP_WIDTH × MAP_HEIGHT (10 × 10), each cell = 50 cm.
// Cell encoding matches /api/map: '#'=wall ' '=free '!'=hazard 'v'=pit '^'=hill
// ─────────────────────────────────────────────────────────────────────────────
static const char MAP_EDITOR_HTML[] PROGMEM = R"rawliteral(<!DOCTYPE html>
<html lang="en"><head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1.0">
<title>Pre-Map Editor — TankBot</title>
<style>
:root{
  --bg:#080c10;--card:#0f1419;--card2:#131920;--border:#1e252e;
  --text:#cdd6e0;--dim:#7a8899;--accent:#4da6ff;--green:#34c759;
  --red:#ff453a;--orange:#ff9f0a;--purple:#bf5af2;--cyan:#00e5cc;
  --yellow:#ffd60a;--teal:#30d158;
}
*{box-sizing:border-box;margin:0;padding:0;}
html{scroll-behavior:smooth;}
body{
  background:var(--bg);
  background-image:radial-gradient(ellipse at 80% 0%,rgba(0,229,204,0.05) 0%,transparent 55%);
  color:var(--text);font-family:-apple-system,'Segoe UI',system-ui,sans-serif;font-size:14px;
  min-height:100vh;
}
.wrap{max-width:680px;margin:0 auto;padding:14px 12px;}

/* ── Header ─────────────────────────────────────── */
.hdr{
  display:flex;align-items:center;justify-content:space-between;
  padding:14px 18px;margin-bottom:14px;
  background:linear-gradient(135deg,#0a1a1a 0%,#0f1419 100%);
  border:1px solid var(--border);border-radius:14px;
  box-shadow:0 2px 16px rgba(0,0,0,0.4);
}
.hdr-left{display:flex;align-items:center;gap:12px;}
.hdr-icon{font-size:1.7em;line-height:1;}
.hdr h1{color:var(--cyan);font-size:1.2em;font-weight:700;letter-spacing:.3px;line-height:1.15;}
.hdr h1 span{font-size:.62em;color:var(--dim);font-weight:400;display:block;}
a.back{
  color:var(--accent);text-decoration:none;font-size:.82em;font-weight:600;
  background:rgba(77,166,255,0.08);border:1px solid rgba(77,166,255,0.25);
  border-radius:20px;padding:5px 13px;transition:background .15s;
}
a.back:hover{background:rgba(77,166,255,0.18);}

/* ── Info banner ─────────────────────────────────── */
.info-banner{
  display:flex;align-items:center;gap:10px;
  background:var(--card);border:1px solid var(--border);border-radius:10px;
  padding:10px 14px;margin-bottom:12px;
  border-left:3px solid var(--cyan);
}
.info-banner .ico{font-size:1.2em;}
.info-banner p{font-size:.8em;color:var(--dim);line-height:1.5;}
.info-banner strong{color:var(--text);}

/* ── Tool bar ────────────────────────────────────── */
.toolbar-card{
  background:var(--card);border:1px solid var(--border);border-radius:12px;
  padding:12px 14px;margin-bottom:10px;
  box-shadow:0 1px 8px rgba(0,0,0,0.3);
}
.toolbar-label{
  font-size:.7em;text-transform:uppercase;letter-spacing:1.4px;
  color:var(--dim);margin-bottom:10px;display:flex;align-items:center;gap:6px;
}
.toolbar-label::after{content:'';flex:1;height:1px;background:var(--border);}
.tool-grid{display:flex;flex-wrap:wrap;gap:6px;}
button.tool{
  padding:6px 14px;border-radius:8px;border:1.5px solid var(--border);
  background:rgba(255,255,255,0.03);color:var(--text);
  cursor:pointer;font-size:.8em;font-weight:600;
  transition:all .15s;display:flex;align-items:center;gap:5px;
}
button.tool:hover{border-color:var(--accent);color:var(--accent);background:rgba(77,166,255,0.08);}
button.tool.active{border-color:currentColor;}
.t-wall.active   {color:#ff453a;border-color:#ff453a;background:rgba(255,69,58,0.1);}
.t-hazard.active {color:#ff9f0a;border-color:#ff9f0a;background:rgba(255,159,10,0.1);}
.t-pit.active    {color:#bf5af2;border-color:#bf5af2;background:rgba(191,90,242,0.1);}
.t-hill.active   {color:#30d158;border-color:#30d158;background:rgba(48,209,88,0.1);}
.t-free.active   {color:#c8c8c8;border-color:#c8c8c8;background:rgba(200,200,200,0.07);}
.t-unknown.active{color:var(--dim);border-color:var(--dim);background:rgba(122,136,153,0.1);}
.t-start.active  {color:#34c759;border-color:#34c759;background:rgba(52,199,89,0.1);}

/* tool color swatches */
.swatch{display:inline-block;width:9px;height:9px;border-radius:2px;flex-shrink:0;}

/* ── Canvas area ─────────────────────────────────── */
#coords{
  font-size:.76em;color:var(--dim);text-align:center;
  margin-bottom:6px;height:18px;
  font-variant-numeric:tabular-nums;letter-spacing:.3px;
}
.canvas-wrap{
  display:flex;justify-content:center;margin-bottom:12px;
  position:relative;
}
canvas{
  border:1.5px solid var(--border);border-radius:8px;
  cursor:crosshair;touch-action:none;max-width:100%;
  box-shadow:0 0 24px rgba(0,229,204,0.06),0 2px 12px rgba(0,0,0,0.4);
}

/* ── Legend ──────────────────────────────────────── */
.legend{
  display:flex;flex-wrap:wrap;justify-content:center;
  gap:7px 14px;margin-bottom:12px;font-size:.74em;color:var(--dim);
}
.legend span{display:flex;align-items:center;gap:4px;}
.legend i{display:inline-block;width:11px;height:11px;border-radius:3px;flex-shrink:0;}

/* ── Action buttons ──────────────────────────────── */
.actions{display:flex;flex-wrap:wrap;gap:8px;margin-bottom:12px;}
button.act{
  padding:9px 20px;border-radius:9px;border:1.5px solid;
  cursor:pointer;font-size:.85em;font-weight:700;
  transition:background .15s,box-shadow .15s;
  display:flex;align-items:center;gap:7px;
}
.act-send{
  background:rgba(48,209,88,0.1);color:var(--teal);border-color:var(--teal);
  box-shadow:0 0 0 0 rgba(48,209,88,0);
}
.act-send:hover{background:rgba(48,209,88,0.18);box-shadow:0 0 12px rgba(48,209,88,0.2);}
.act-load{background:rgba(77,166,255,0.1);color:var(--accent);border-color:var(--accent);}
.act-load:hover{background:rgba(77,166,255,0.18);}
.act-clear{background:rgba(255,69,58,0.08);color:var(--red);border-color:var(--red);}
.act-clear:hover{background:rgba(255,69,58,0.16);}

/* ── Toast ───────────────────────────────────────── */
#toast{
  padding:10px 16px;border-radius:9px;font-size:.85em;font-weight:600;
  display:none;margin-top:6px;
  display:none;align-items:center;gap:8px;
}
.toast-ok{background:rgba(48,209,88,0.12);color:var(--teal);border:1.5px solid var(--teal);}
.toast-err{background:rgba(255,69,58,0.1);color:var(--red);border:1.5px solid var(--red);}

/* ── Footer ──────────────────────────────────────── */
.foot{
  text-align:center;color:#2e3a48;font-size:.7em;
  padding:16px 0 8px;border-top:1px solid var(--border);margin-top:4px;
}
.foot a{color:var(--accent);text-decoration:none;font-weight:600;}
</style>
</head>
<body>
<div class="wrap">

<!-- Header -->
<div class="hdr">
  <div class="hdr-left">
    <div class="hdr-icon">&#9998;</div>
    <h1>Pre-Map Editor<span>Draw your arena before the run</span></h1>
  </div>
  <a class="back" href="/">&#8592; Dashboard</a>
</div>

<!-- Info banner -->
<div class="info-banner">
  <span class="ico">&#128207;</span>
  <p>Grid: <strong>10 &times; 10 cells</strong> &nbsp;|&nbsp; Each cell: <strong>50 cm &times; 50 cm</strong> &nbsp;|&nbsp; Total: <strong>5 m &times; 5 m</strong> &nbsp;|&nbsp; Robot starts at the <strong style="color:#34c759">&#9679; green cell</strong></p>
</div>

<!-- Tool selector -->
<div class="toolbar-card">
  <div class="toolbar-label">&#127912; Paint tool &mdash; click &amp; drag on the grid</div>
  <div class="tool-grid">
    <button class="tool t-wall  active" onclick="setTool('wall')"    id="btn-wall">   <span class="swatch" style="background:#ff453a"></span>Wall</button>
    <button class="tool t-hazard"       onclick="setTool('hazard')"  id="btn-hazard"> <span class="swatch" style="background:#ff9f0a"></span>Hazard</button>
    <button class="tool t-pit"          onclick="setTool('pit')"     id="btn-pit">    <span class="swatch" style="background:#bf5af2"></span>Pit</button>
    <button class="tool t-hill"         onclick="setTool('hill')"    id="btn-hill">   <span class="swatch" style="background:#30d158"></span>Hill</button>
    <button class="tool t-free"         onclick="setTool('free')"    id="btn-free">   <span class="swatch" style="background:#c8c8c8"></span>Free</button>
    <button class="tool t-unknown"      onclick="setTool('unknown')" id="btn-unknown"><span class="swatch" style="background:#3a4a5a"></span>Unknown</button>
    <button class="tool t-start"        onclick="setTool('start')"   id="btn-start">  <span class="swatch" style="background:#34c759"></span>Robot Start</button>
  </div>
</div>

<!-- Grid canvas -->
<div id="coords">&nbsp;</div>
<div class="canvas-wrap">
  <canvas id="grid" width="544" height="532"></canvas>
</div>

<div class="scale-note">
  &#128207; Grid: <strong>10 &times; 10 cells</strong> &nbsp;|&nbsp;
  Each cell = <strong>50 cm &times; 50 cm</strong> &nbsp;|&nbsp;
  Total: <strong>5 m &times; 5 m (16.4 ft &times; 16.4 ft)</strong> &nbsp;|&nbsp;
  Robot starts at the <span style="color:#3fb950">&#9679; green cell</span>
</div>

<!-- Tool selector -->
<div class="toolbar">
  <label>Paint tool (click &amp; drag on the grid):</label>
  <button class="tool t-wall  active" onclick="setTool('wall')"   id="btn-wall">&#9632; Wall</button>
  <button class="tool t-hazard"       onclick="setTool('hazard')" id="btn-hazard">&#9888; Hazard</button>
  <button class="tool t-pit"          onclick="setTool('pit')"    id="btn-pit">&#8711; Pit</button>
  <button class="tool t-hill"         onclick="setTool('hill')"   id="btn-hill">&#9651; Hill</button>
  <button class="tool t-free"         onclick="setTool('free')"   id="btn-free">&#9633; Free</button>
  <button class="tool t-unknown"      onclick="setTool('unknown')" id="btn-unknown">? Unknown</button>
  <button class="tool t-start"        onclick="setTool('start')"  id="btn-start">&#9654; Robot Start</button>
</div>

<!-- Grid canvas -->
<div id="coords">&nbsp;</div>
<div class="canvas-wrap">
  <canvas id="grid" width="544" height="532"></canvas>
</div>

<!-- Legend -->
<div class="legend">
  <span><i style="background:#ff453a"></i>Wall</span>
  <span><i style="background:#ff9f0a"></i>Hazard</span>
  <span><i style="background:#bf5af2"></i>Pit</span>
  <span><i style="background:#30d158"></i>Hill</span>
  <span><i style="background:#c8c8c8"></i>Free</span>
  <span><i style="background:#1e252e;border:1px solid #3a4a5a"></i>Unknown</span>
  <span><i style="background:#34c759"></i>Robot Start</span>
</div>

<!-- Action buttons -->
<div class="actions">
  <button class="act act-send"  onclick="sendMap()">&#9654; Send Map to Robot</button>
  <button class="act act-load"  onclick="loadMap()">&#8635; Load Current Map</button>
  <button class="act act-clear" onclick="clearMap()">&#10006; Clear All</button>
</div>

<div id="toast"></div>

<div class="foot">TankBot &bull; Pre-Map Editor &bull; <a href="/">&#8592; Dashboard</a></div>

</div><!-- .wrap -->

<script>
const W=10, H=10;
const CELL=50; // px per cell on canvas
const MARGIN=44; // left margin px — Y axis labels
const MBOT=32;   // bottom margin px — X axis labels
const canvas=document.getElementById('grid');
const ctx=canvas.getContext('2d');

// Map state — 2D array [y][x] of chars
let cells=[];
let robotX=5, robotY=5;  // 0-based grid coords of robot start
let currentTool='wall';
let painting=false;

const COLORS={
  '#':'#ff453a',  // wall
  '!':'#ff9f0a',  // hazard
  'v':'#bf5af2',  // pit
  '^':'#30d158',  // hill
  ' ':'#c8c8c8',  // free
  '-':'#8b949e',  // likely free (from loaded map)
  'x':'#da6840',  // likely obstacle (from loaded map)
  'f':'#00bfff',  // frontier (from loaded map, treated as unknown on save)
  '?':'#1e252e',  // unknown
};

const TOOL_CHARS={
  wall:'#', hazard:'!', pit:'v', hill:'^', free:' ', unknown:'?', start:'R'
};

function initCells(){
  cells=[];
  for(let y=0;y<H;y++){cells.push([]);for(let x=0;x<W;x++)cells[y].push('?');}
  robotX=5; robotY=5;
}

function fmtDist(cm){
  if(cm===0)return'0';
  const sg=cm<0?'-':'';
  const a=Math.abs(cm);
  return a>=100?sg+(a/100).toFixed(1)+'m':sg+a+'cm';
}

function draw(){
  canvas.width=W*CELL+MARGIN;
  canvas.height=H*CELL+MBOT;
  ctx.clearRect(0,0,canvas.width,canvas.height);

  // Fill margin areas dark
  ctx.fillStyle='#080c10';
  ctx.fillRect(0,0,MARGIN,canvas.height);
  ctx.fillRect(0,H*CELL,canvas.width,MBOT);

  // Draw cells (shifted right by MARGIN)
  for(let y=0;y<H;y++){
    for(let x=0;x<W;x++){
      let ch=cells[y][x];
      if(x===robotX&&y===robotY){
        ctx.fillStyle='#0a1e10';
        ctx.fillRect(x*CELL+MARGIN,y*CELL,CELL,CELL);
        ctx.fillStyle='#34c759';
        ctx.beginPath();
        ctx.arc(x*CELL+MARGIN+CELL/2,y*CELL+CELL/2,CELL*0.3,0,Math.PI*2);
        ctx.fill();
      } else {
        ctx.fillStyle=COLORS[ch]||'#1e252e';
        ctx.fillRect(x*CELL+MARGIN,y*CELL,CELL,CELL);
      }
    }
  }

  // Grid lines
  ctx.strokeStyle='rgba(255,255,255,0.08)';
  ctx.lineWidth=1;
  for(let i=0;i<=W;i++){ctx.beginPath();ctx.moveTo(i*CELL+MARGIN,0);ctx.lineTo(i*CELL+MARGIN,H*CELL);ctx.stroke();}
  for(let j=0;j<=H;j++){ctx.beginPath();ctx.moveTo(MARGIN,j*CELL);ctx.lineTo(W*CELL+MARGIN,j*CELL);ctx.stroke();}

  // Axis separator lines
  ctx.strokeStyle='#444';ctx.lineWidth=1;
  ctx.beginPath();ctx.moveTo(MARGIN,0);ctx.lineTo(MARGIN,H*CELL);ctx.stroke();
  ctx.beginPath();ctx.moveTo(MARGIN,H*CELL);ctx.lineTo(W*CELL+MARGIN,H*CELL);ctx.stroke();

  // X axis labels — distance East/West from robot start
  ctx.font='10px sans-serif';
  ctx.fillStyle='#8b949e';
  ctx.textAlign='center';
  ctx.textBaseline='middle';
  ctx.strokeStyle='#555';
  ctx.lineWidth=0.8;
  for(let x=0;x<=W;x++){
    const dm=(x-robotX)*50;
    ctx.beginPath();ctx.moveTo(x*CELL+MARGIN,H*CELL);ctx.lineTo(x*CELL+MARGIN,H*CELL+5);ctx.stroke();
    ctx.fillText(fmtDist(dm),x*CELL+MARGIN,H*CELL+MBOT/2);
  }
  // X axis direction label
  ctx.fillStyle='#58a6ff';ctx.font='bold 9px sans-serif';ctx.textAlign='right';
  ctx.textBaseline='bottom';
  ctx.fillText('\u2192 E',W*CELL+MARGIN,H*CELL+MBOT-2);

  // Y axis labels — distance North/South from robot start (positive = North)
  ctx.font='10px sans-serif';ctx.fillStyle='#8b949e';
  ctx.textAlign='right';ctx.textBaseline='middle';
  ctx.strokeStyle='#555';ctx.lineWidth=0.8;
  for(let y=0;y<=H;y++){
    const dm=(robotY-y)*50;
    ctx.beginPath();ctx.moveTo(MARGIN-4,y*CELL);ctx.lineTo(MARGIN,y*CELL);ctx.stroke();
    ctx.fillText(fmtDist(dm),MARGIN-6,y*CELL);
  }
  // Y axis direction label
  ctx.fillStyle='#58a6ff';ctx.font='bold 9px sans-serif';
  ctx.textAlign='center';ctx.textBaseline='top';
  ctx.fillText('N \u2191',MARGIN/2,2);

  // Robot start label
  ctx.fillStyle='#fff';
  ctx.font='bold 10px sans-serif';
  ctx.textAlign='center';
  ctx.textBaseline='bottom';
  ctx.fillText('START',robotX*CELL+MARGIN+CELL/2,robotY*CELL+CELL-2);
}

function cellFromEvent(e){
  const r=canvas.getBoundingClientRect();
  const scaleX=canvas.width/r.width;
  const scaleY=canvas.height/r.height;
  const cx=e.touches?e.touches[0].clientX:e.clientX;
  const cy=e.touches?e.touches[0].clientY:e.clientY;
  return{
    x:Math.floor(((cx-r.left)*scaleX-MARGIN)/CELL),
    y:Math.floor((cy-r.top)*scaleY/CELL)
  };
}

function paintCell(gx,gy){
  if(gx<0||gx>=W||gy<0||gy>=H)return;
  if(currentTool==='start'){
    robotX=gx; robotY=gy;
  } else {
    cells[gy][gx]=TOOL_CHARS[currentTool]||'?';
  }
  draw();
}

canvas.addEventListener('mousedown',e=>{painting=true;paintCell(...Object.values(cellFromEvent(e)));});
canvas.addEventListener('mousemove',e=>{
  const{x,y}=cellFromEvent(e);
  document.getElementById('coords').textContent=
    (x>=0&&x<W&&y>=0&&y<H)?`Cell (${x}, ${y}) — ${(x-robotX)*50} cm E, ${(robotY-y)*50} cm N`:'';
  if(painting)paintCell(x,y);
});
canvas.addEventListener('mouseup',()=>painting=false);
canvas.addEventListener('mouseleave',()=>painting=false);
// Touch support
canvas.addEventListener('touchstart',e=>{e.preventDefault();painting=true;paintCell(...Object.values(cellFromEvent(e)));},{passive:false});
canvas.addEventListener('touchmove', e=>{e.preventDefault();if(painting){const{x,y}=cellFromEvent(e);paintCell(x,y);}},{passive:false});
canvas.addEventListener('touchend', ()=>painting=false);

function setTool(t){
  currentTool=t;
  document.querySelectorAll('button.tool').forEach(b=>b.classList.remove('active'));
  document.getElementById('btn-'+t).classList.add('active');
}

function clearMap(){
  initCells(); draw();
  toast('Map cleared','ok');
}

async function loadMap(){
  try{
    const r=await fetch('/api/map');
    const m=await r.json();
    if(m.w!==W||m.h!==H){toast('Map size mismatch ('+m.w+'x'+m.h+', expected 10x10)','err');return;}
    initCells();
    robotX=m.rx; robotY=m.ry;
    for(let y=0;y<H;y++)
      for(let x=0;x<W;x++)
        cells[y][x]=m.d[y*W+x]||'?';
    draw();
    toast('Current map loaded','ok');
  }catch(e){toast('Load failed: '+e.message,'err');}
}

async function sendMap(){
  // Build 100-char cell string (row-major).
  // 'R' (robot start) and frontier 'f' both map to unknown '?' for sending.
  let body='';
  for(let y=0;y<H;y++)
    for(let x=0;x<W;x++){
      let ch=cells[y][x];
      if(ch==='R'||ch==='f'||ch==='-')ch='?';  // normalize
      body+=ch;
    }
  try{
    const r=await fetch('/api/premap?rx='+robotX+'&ry='+robotY,{
      method:'POST',
      headers:{'Content-Type':'text/plain'},
      body:body
    });
    if(r.ok) toast('Map sent! Switch robot to Premap Nav mode to use it.','ok');
    else      toast('Robot rejected map (HTTP '+r.status+')','err');
  }catch(e){toast('Send failed: '+e.message,'err');}
}

function toast(msg,type){
  const el=document.getElementById('toast');
  el.textContent=msg;
  el.className=(type==='ok'?'toast-ok':'toast-err');
  el.style.display='flex';
  clearTimeout(el._t);
  el._t=setTimeout(()=>{el.style.display='none';},4000);
}

// Init
initCells();
draw();
</script>
</body></html>)rawliteral";

static const size_t MAP_EDITOR_LEN = sizeof(MAP_EDITOR_HTML) - 1;

void handleMapEditor(AsyncWebServerRequest *request) {
    AsyncWebServerResponse *response = request->beginChunkedResponse("text/html",
        [](uint8_t *buffer, size_t maxLen, size_t index) -> size_t {
            if (index >= MAP_EDITOR_LEN) return 0;
            size_t remaining = MAP_EDITOR_LEN - index;
            size_t toSend = (remaining < maxLen) ? remaining : maxLen;
            memcpy_P(buffer, MAP_EDITOR_HTML + index, toSend);
            return toSend;
        });
    request->send(response);
}
