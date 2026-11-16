/* SPDX-License-Identifier: MIT */
/**
 * @file  car_web.cpp
 * @brief Hand-written HTTP + WebSocket server and the control page.
 *
 * Zero third-party libraries: WiFi core, SHA1 (RFC 3174) from base64 raw data,
 * core base64::encode for the handshake accept value.
 *
 * Protocol (WebSocket text frames)
 *   page -> board : DRV:<mm/s>,<turn>  the command the page uses: speed and
 *                                turn intent together. turn +1 = left (right
 *                                wheel drives), -1 = right, 0 = straight.
 *                                mm/s 0 with a turn = spin in place.
 *                   SPD:<mm/s>   signed linear speed, 0 = coast
 *                   PIVOT:1|0|-1 single-wheel pivot: 1 = left, -1 = right
 *                   TURN:<deg>   pivot by a fixed angle, + = counter-clockwise
 *                   HDG:1|0      gyro heading hold while driving straight
 *                                (the board also does it by default)
 *                   FWD          "set forward": current Euler yaw becomes 0
 *                   CAL          re-zero calibration: gyro bias + gravity seed,
 *                                yaw 0, heading target 0. Stops the car, then the
 *                                loop blocks ~1.2 s on purpose: the car must be still
 *                   STOP         emergency stop, coast
 * The page repeats the active command every 300 ms while a key is held or HOLD
 * is on, so a dead link stops the car through the board's watchdog. Pressed
 * keys are tracked as a set (one pointer per finger), so forward and turn can
 * be held at the same time and no release can cancel the other.
 *   board -> page : T:<mm>,<adc>,<tapMv>,<rpmL>,<rpmR>,<cal>   (5 Hz)
 *                   L:<text>   one line of the serial log
 *                   OK ... / ERR
 * Every serial line is mirrored into the page's log window, so the bench never
 * needs a USB cable and the phone sees exactly what the console sees.
 */

#include "src/net/car_web.h"
#include <WiFi.h>
#include <ESPmDNS.h>
/* lwIP's send() is the only non-blocking write this socket has - see sockWrite(). */
#include <lwip/sockets.h>
#include <errno.h>
#include "base64.h"
#include <string.h>
#include <stdio.h>

/* ── server state (one controller at a time) ── */
static WiFiServer s_server(80);
static WiFiClient s_ws;

/* Half-open sockets are parked here instead of being waited on: browsers open
 * speculative ones, and a slow phone sends its request late. */
#define WEB_PEND_MAX        4       /* a browser opens several sockets at once */
#define WEB_HDR_TIMEOUT_MS  1500
#define WEB_HDR_MAX         1024    /* Chrome's GET header, UA + cookies included,
                                     * runs past 600 B - 512 could never complete */
struct WebPending {
    WiFiClient c;
    char       buf[WEB_HDR_MAX];
    size_t     n;
    uint32_t   t0;
};
static WebPending s_pend[WEB_PEND_MAX];
static char     s_host[24];            /* mDNS name, to re-register after a drop */
static uint16_t s_port = 80;
static bool     s_linkUp  = false;     /* begin() only *asks* for the link; poll() reports it */
static unsigned long s_lastCmd = 0;
static uint16_t s_sendDrop = 0;        /* frames dropped because the peer stalled */
static uint32_t s_reqs = 0;            /* HTTP/WS requests that arrived intact */
static uint32_t s_pages = 0;           /* pages served to completion */
static uint32_t s_pageFail = 0;        /* pages dropped before the last byte */

/* receive buffer + parser state (persistent, so partial frames are fine) */
static uint8_t s_buf[192];
static size_t  s_have = 0;

/* ── SHA1 (RFC 3174) for the handshake ── */
static inline uint32_t rol32(uint32_t x, int n) { return (x << n) | (x >> (32 - n)); }

static void wsSha1(const uint8_t *data, size_t len, uint8_t out[20]) {
    uint32_t h0 = 0x67452301UL, h1 = 0xEFCDAB89UL, h2 = 0x98BADCFEUL,
             h3 = 0x10325476UL, h4 = 0xC3D2E1F0UL;
    uint8_t msg[192];                     /* inputs here are < 120 B */
    size_t plen = len + 1;
    while (plen % 64 != 56) plen++;
    memset(msg, 0, sizeof(msg));
    memcpy(msg, data, len);
    msg[len] = 0x80;
    uint64_t bits = (uint64_t)len * 8;
    for (int i = 0; i < 8; i++) msg[plen + 7 - i] = (uint8_t)((bits >> (i * 8)) & 0xFF);

    for (size_t off = 0; off < plen + 8; off += 64) {
        uint32_t w[80];
        for (int i = 0; i < 16; i++)
            w[i] = ((uint32_t)msg[off + i * 4] << 24) | ((uint32_t)msg[off + i * 4 + 1] << 16) |
                   ((uint32_t)msg[off + i * 4 + 2] << 8) | (uint32_t)msg[off + i * 4 + 3];
        for (int i = 16; i < 80; i++) w[i] = rol32(w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16], 1);

        uint32_t a = h0, b = h1, c = h2, d = h3, e = h4;
        for (int i = 0; i < 80; i++) {
            uint32_t f, k;
            if      (i < 20) { f = (b & c) | (~b & d);         k = 0x5A827999UL; }
            else if (i < 40) { f = b ^ c ^ d;                  k = 0x6ED9EBA1UL; }
            else if (i < 60) { f = (b & c) | (b & d) | (c & d); k = 0x8F1BBCDCUL; }
            else             { f = b ^ c ^ d;                  k = 0xCA62C1D6UL; }
            uint32_t t = rol32(a, 5) + f + e + k + w[i];
            e = d; d = c; c = rol32(b, 30); b = a; a = t;
        }
        h0 += a; h1 += b; h2 += c; h3 += d; h4 += e;
    }
    uint32_t h[5] = {h0, h1, h2, h3, h4};
    for (int i = 0; i < 5; i++) {
        out[i * 4 + 0] = (uint8_t)(h[i] >> 24);
        out[i * 4 + 1] = (uint8_t)(h[i] >> 16);
        out[i * 4 + 2] = (uint8_t)(h[i] >> 8);
        out[i * 4 + 3] = (uint8_t)(h[i]);
    }
}

/* ── the control page ── */
/* Control page. Montserrat is used under the SIL Open Font License 1.1, which
 * allows commercial use; the Google Fonts link is only a convenience, so the
 * page still renders with the system stack when the network has no internet
 * (in that case the page is NOT in Montserrat - that is the fallback working).
 * Keep prose selectable: buttons own user-select:none, text owns user-select:text,
 * otherwise both "select and copy" and long-press translation stop working. */
static const char PAGE_HTML[] PROGMEM = R"raw(<!DOCTYPE html>
<html lang="en"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<meta name="theme-color" content="#0e1116">
<title>XIAO Robot Remote</title>
<link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
<link href="https://fonts.googleapis.com/css2?family=Montserrat:wght@500;600;700&display=swap" rel="stylesheet">
<style>
  :root{
    --bg:#0e1116; --card:#161b23; --line:#242c38; --txt:#e7ecf3; --dim:#8b97a8;
    --go:#37a866; --stop:#d64550; --warn:#e0a33c;
    --key:clamp(62px,25vmin,112px);
  }
  *{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
  html,body{margin:0;min-height:100%;background:var(--bg);color:var(--txt)}
  body{font-family:'Montserrat',system-ui,-apple-system,'Segoe UI',Roboto,Arial,sans-serif;
       text-align:center}
  /* Selection is disabled ONLY on the controls: a long press on a key must not
     start a text selection or the iOS callout, because that eats the pointerup and
     the car keeps driving. Prose stays selectable so the page can be read with
     copy/paste and translated (browser translate or a long-press translate). */
  .k,.act,.chip{user-select:none;-webkit-user-select:none;-webkit-touch-callout:none}
  header,.hint,.log{-webkit-touch-callout:default}
  .hint,.log{user-select:text;-webkit-user-select:text}
  .wrap{max-width:460px;margin:0 auto;padding:16px 14px 24px}
  header h1{font-size:17px;font-weight:700;letter-spacing:.4px;margin:0}
  header p{margin:4px 0 0;font-size:11px;color:var(--dim);letter-spacing:.3px}
  .chips{display:flex;gap:8px;justify-content:center;flex-wrap:wrap;margin:12px 0 2px}
  .chip{display:inline-flex;align-items:center;gap:6px;font-size:11px;color:var(--dim);
        background:var(--card);border:1px solid var(--line);border-radius:999px;padding:6px 12px}
  .chip b{font-size:12px;font-weight:600;color:var(--txt);letter-spacing:.3px}
  .dot{width:8px;height:8px;border-radius:50%;background:var(--stop);transition:background .2s}
  .chip.on .dot{background:var(--go);box-shadow:0 0 10px var(--go)}
  .board{margin-top:14px}
  .pad{display:grid;grid-template-columns:repeat(3,var(--key));
       grid-template-rows:repeat(3,var(--key));gap:9px;justify-content:center;margin:0 auto}
  .k{border:1px solid var(--line);border-radius:20px;background:linear-gradient(180deg,#1b222c,#151a22);
     color:var(--txt);font-family:inherit;font-size:calc(var(--key) * .3);font-weight:600;
     display:flex;flex-direction:column;align-items:center;justify-content:center;gap:2px;
     touch-action:none;cursor:pointer;padding:0;transition:transform .06s,background .12s,border-color .12s}
  .k small{font-size:calc(var(--key) * .115);font-weight:600;letter-spacing:.4px;color:var(--dim)}
  .k:active,.k.hold{background:#22402f;border-color:var(--go);transform:scale(.96)}
  .k#stop{background:linear-gradient(180deg,#432026,#2a1418);border-color:#6d2a31;color:#ffd9dc}
  .k#stop small{color:#e79ba1}
  .k#stop:active,.k#stop.hit{background:#8c2b34;border-color:#ff5b66;color:#fff;transform:scale(.96)}
  #up{grid-area:1/2}#left{grid-area:2/1}#stop{grid-area:2/2}#right{grid-area:2/3}#down{grid-area:3/2}
  .acts{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-top:10px}
  .act{padding:13px 8px;border-radius:15px;border:1px solid var(--line);
       background:var(--card);color:var(--txt);font-family:inherit;font-size:10.5px;
       font-weight:600;letter-spacing:.7px;cursor:pointer;touch-action:none}
  .act:active,.act.hold{background:#20415c;border-color:#3d8fc4}
  .act.on{background:#1d3a2b;border-color:var(--go);color:#b8f2cd}
  .act.wide{grid-column:1/-1}
  .act.wait{background:#4a3410;border-color:#c98a25;color:#ffd79a}
  .hint{margin:14px 0 0;font-size:10px;line-height:1.75;color:var(--dim);letter-spacing:.2px}
  .hint.flag{color:var(--warn)}
  .log{display:none;margin-top:14px;text-align:left;background:#0a0d12;border:1px solid var(--line);
       border-radius:14px;padding:9px 11px;height:190px;overflow-y:auto;
       font-family:ui-monospace,SFMono-Regular,Menlo,Consolas,monospace;
       font-size:9.5px;line-height:1.55;color:#b6c3d4;white-space:pre-wrap;word-break:break-word;
       -webkit-overflow-scrolling:touch}
  .log.on{display:block}
  .log div{padding:1px 0}
  .log div.st{color:#68758a}
  .log div.ok{color:#7fd4a0}
  .log div.er{color:#ef8b93}
  @media (min-width:760px){
    .wrap{max-width:none;display:grid;grid-template-columns:minmax(280px,400px) minmax(320px,440px);
          align-items:start;justify-content:center;column-gap:34px;padding:30px 24px}
    header,.chips,.hint{grid-column:1}
    .log{grid-column:1 / -1;height:220px}
    .board{grid-column:2;grid-row:1 / span 3;margin-top:0}
    .hint{text-align:left}
  }
  @media (min-width:1100px){
    .wrap{grid-template-columns:minmax(320px,440px) minmax(360px,500px);column-gap:52px}
    :root{--key:clamp(84px,16vmin,124px)}
  }
</style>
</head>
<body>
<div class="wrap">
  <header>
    <h1>XIAO Robot Remote</h1>
    <p>Robot Kit &middot; encoder drive &middot; Euler reference</p>
  </header>
  <div class="chips">
    <div class="chip" id="conn"><span class="dot"></span><span id="connTxt">connecting&hellip;</span></div>
    <div class="chip"><b id="bat">&ndash;&ndash;</b></div>
  </div>

  <div class="board">
    <div class="pad">
      <button class="k" id="up">&#9650;<small>forward</small></button>
      <button class="k" id="left">&#9664;<small>turn left</small></button>
      <button class="k" id="stop">&#9632;<small>stop</small></button>
      <button class="k" id="right">&#9654;<small>turn right</small></button>
      <button class="k" id="down">&#9660;<small>reverse</small></button>
    </div>
    <div class="acts">
      <button class="act" id="fwd">SET FORWARD</button>
      <button class="act" id="hold">HOLD &middot; OFF</button>
      <button class="act wide" id="cal">CALIBRATE &middot; ZERO YAW</button>
    </div>
  </div>

  <p class="hint" id="hint">Hold an arrow key: forward / reverse drive, left / right turn.<br>
  The gyro holds the heading while driving, so the car goes straight instead of
  drifting. HOLD keeps repeating the last direction.<br>
  The red key always stops. Keys: WASD, Space stop, H hold.</p>

  <div class="log on" id="log"></div>
</div>
<script>
var DRIVE_MMPS = 150, REPEAT_MS = 300;
var ws = null, latched = false, lastCmd = null, sent = false;
var calUntil = 0;   /* until then the hint says "calibrating" instead of the help */

/* pressed keys, one pointer per finger: forward and turn can be held at once */
var keys = { fwd: 0, back: 0, left: 0, right: 0 };

function $(id){ return document.getElementById(id); }
function send(msg){ if (ws && ws.readyState === 1) { ws.send(msg); return true; } return false; }

/* A press the page cannot deliver is the most confusing failure there is: the page
   looks alive and the car ignores it. Say it out loud instead. */
function dead(){
  var h = $('hint');
  h.style.color = '#ff6b6b';
  h.textContent = 'NOT CONNECTED to the car - that press did nothing. Reload this page ' +
                  'and watch the dot in the top right corner.';
}

/* speed and turn intent travel in one command; turn +1 = left, -1 = right */
function intent(){
  var mmps = (keys.fwd ? DRIVE_MMPS : 0) + (keys.back ? -DRIVE_MMPS : 0);
  var turn = (keys.left ? 1 : 0) + (keys.right ? -1 : 0);
  if (!mmps && !turn) return null;
  return 'DRV:' + mmps + ',' + turn;
}

/* repeat the pressed keys, or the last direction while HOLD is on, so a dead
   link stops the car through the board's 1.5 s watchdog */
function push(){
  var m = intent();
  if (m) lastCmd = m;
  var out = m || (latched ? lastCmd : null);
  if (out) { if (!send(out)) dead(); sent = true; }
  else if (sent) { send('STOP'); sent = false; }
}

function setConn(on){
  $('conn').classList.toggle('on', on);
  $('connTxt').textContent = on ? 'connected' : 'reconnecting\u2026';
  if (!on) hardStop();
  else $('hint').style.color = '';
}

var elOf = { fwd: 'up', back: 'down', left: 'left', right: 'right' };

function setKey(name, on){
  keys[name] = on ? 1 : 0;
  $(elOf[name]).classList.toggle('hold', !!on);
  push();
}

function hardStop(){
  keys.fwd = keys.back = keys.left = keys.right = 0;
  for (var k in elOf) $(elOf[k]).classList.remove('hold');
  latched = false;
  $('hold').classList.remove('on');
  $('hold').innerHTML = 'HOLD &middot; OFF';
  lastCmd = null;
  sent = false;
  send('STOP');
}

/* log window: what the board prints on its serial port */
function logLine(t){
  var el = $('log');
  var d = document.createElement('div');
  if (t.indexOf('STATE') === 0) d.className = 'st';
  else if (t.indexOf('OK ') === 0) d.className = 'ok';
  else if (t.indexOf('ERR') >= 0) d.className = 'er';
  d.textContent = t;
  var stick = el.scrollHeight - el.scrollTop - el.clientHeight < 30;
  el.appendChild(d);
  while (el.childNodes.length > 240) el.removeChild(el.firstChild);
  if (stick) el.scrollTop = el.scrollHeight;
}

/* the hint line is owned by one function: telemetry rewrites it 5 times a second,
   so anything transient (the calibrating note) has to be a deadline, not a write
   that the next frame would erase */
function renderHint(calOn){
  var cal = $('hint'), waiting = Date.now() < calUntil;
  if (waiting) {
    cal.className = 'hint flag';
    cal.innerHTML = 'Calibrating: the car must stand STILL for about a second.<br>' +
                    'Gyro bias and yaw zero are being redone.';
  } else if (calOn) {
    cal.className = 'hint';
    cal.innerHTML = 'Hold an arrow key: forward / reverse drive, left / right turn.<br>' +
                    'Hold two of them together to drive a curve.<br>' +
                    'HOLD keeps repeating the last direction by itself.<br>' +
                    'CALIBRATE redoes the IMU zero: stand still, then press it.<br>' +
                    'The red key always stops. Keys: WASD, Space stop, H hold.';
  } else {
    cal.className = 'hint flag';
    cal.innerHTML = 'IMU not calibrated: heading hold is off until the board is<br>' +
                    'still and calibrated. Press CALIBRATE (or serial c) while it stands.';
  }
}

var calOn = 1;   /* last known calibration flag, so renderHint() can be called alone */

function telemetry(s){
  var a = s.split(',');
  if (a.length < 6) return;
  /* battery: raw ADC counts (a[1]) and the tap in mV (a[2]). The tap sits on the
     battery connector behind a 100K:30K divider, and the PMIC holds that rail near
     8.4 V (2S) whenever the board is powered - so 8.4 V on a 5 V input is right. */
  var volts = (+a[2]) * 130 / 30 / 1000;
  $('bat').textContent = 'ADC ' + a[1] + ' · ' + volts.toFixed(2) + ' V';
  calOn = a[5] === '1' ? 1 : 0;
  renderHint(calOn);
}

function connect(){
  ws = new WebSocket('ws://' + location.host + '/');
  ws.onopen = function(){ setConn(true); sent = false; push(); };
  ws.onclose = function(){ setConn(false); setTimeout(connect, 1500); };
  ws.onerror = function(){ setConn(false); };
  ws.onmessage = function(e){
    var d = e.data;
    if (d.charAt(0) === 'T' && d.charAt(1) === ':') telemetry(d.slice(2));
    else if (d.charAt(0) === 'L' && d.charAt(1) === ':') logLine(d.slice(2));
  };
}

setInterval(push, REPEAT_MS);

function holdKey(el, name){
  el.addEventListener('pointerdown', function(e){
    e.preventDefault();
    /* Capture the pointer: on a phone a finger slides a few pixels off a small
       button, and pointerleave used to release the key after ~100 ms - a "hold
       left" became a 100 ms twitch plus a STOP, which reads as the turn button
       doing nothing. With capture the key lives until the finger really lifts. */
    try { el.setPointerCapture(e.pointerId); } catch (err) {}
    setKey(name, 1);
  });
  ['pointerup', 'pointercancel', 'lostpointercapture'].forEach(function(ev){
    el.addEventListener(ev, function(){ setKey(name, 0); });
  });
}

holdKey($('up'),    'fwd');
holdKey($('down'),  'back');
holdKey($('left'),  'left');
holdKey($('right'), 'right');

$('stop').addEventListener('pointerdown', function(e){
  e.preventDefault();
  var self = this;
  self.classList.add('hit');
  hardStop();
  setTimeout(function(){ self.classList.remove('hit'); }, 160);
});

$('fwd').addEventListener('pointerdown', function(e){
  e.preventDefault();
  var self = this;
  send('FWD');
  self.classList.add('hold');
  setTimeout(function(){ self.classList.remove('hold'); }, 160);
});

$('cal').addEventListener('pointerdown', function(e){
  /* Re-zero calibration. It stops the car on the board and blocks there for about
   * a second, so nothing may be moving: the hint says so until it should be done.
   * Render is driven by telemetry (5 Hz), which is why the message is a deadline
   * and not a one-shot write that the next frame would erase. */
  e.preventDefault();
  hardStop();
  calUntil = Date.now() + 2600;
  var self = this;
  self.classList.add('wait');
  setTimeout(function(){ self.classList.remove('wait'); }, 2600);
  send('CAL');
  renderHint(calOn);
});

$('hold').addEventListener('pointerdown', function(e){
  e.preventDefault();
  latched = !latched;
  this.classList.toggle('on', latched);
  this.innerHTML = 'HOLD &middot; ' + (latched ? 'ON' : 'OFF');
  push();
});

document.addEventListener('contextmenu', function(e){ e.preventDefault(); });

var keyMap = { KeyW: 'fwd', ArrowUp: 'fwd',
               KeyS: 'back', ArrowDown: 'back',
               KeyA: 'left', ArrowLeft: 'left',
               KeyD: 'right', ArrowRight: 'right' };

window.addEventListener('keydown', function(e){
  if (keyMap[e.code]) { e.preventDefault(); if (!e.repeat) setKey(keyMap[e.code], 1); }
  else if (e.code === 'Space' || e.code === 'Escape'){ e.preventDefault(); hardStop(); }
  else if (e.code === 'KeyH' && !e.repeat) $('hold').dispatchEvent(new Event('pointerdown'));
});
window.addEventListener('keyup', function(e){
  if (keyMap[e.code]) setKey(keyMap[e.code], 0);
});
window.addEventListener('blur', hardStop);

connect();
</script>
</body></html>)raw";

/* ── HTTP helpers ── */
/* A request is complete as soon as the blank line arrives. Look for it ANYWHERE:
 * a browser may append a second pipelined request or body bytes right behind it,
 * so "the buffer ends with CRLFCRLF" is not the same question. */
static bool headerDone(const char *buf, size_t n) {
    for (size_t i = 3; i < n; i++)
        if (buf[i - 3] == '\r' && buf[i - 2] == '\n' &&
            buf[i - 1] == '\r' && buf[i] == '\n') return true;
    return false;
}

static bool headerVal(const char *buf, const char *name, char *val, size_t valMax) {
    const char *p = strstr(buf, name);
    if (!p) return false;
    p += strlen(name);
    while (*p == ' ' || *p == '\t') p++;
    size_t i = 0;
    while (*p && *p != '\r' && *p != '\n' && i < valMax - 1) val[i++] = *p++;
    val[i] = '\0';
    return i > 0;
}

static void wsHandshake(WiFiClient &c, const char *key) {
    char combined[160];   /* key up to 95 B + GUID 36 B */
    snprintf(combined, sizeof(combined), "%s258EAFA5-E914-47DA-95CA-C5AB0DC85B11", key);
    uint8_t digest[20];
    wsSha1((const uint8_t *)combined, strlen(combined), digest);
    String accept = base64::encode(digest, 20);

    c.print("HTTP/1.1 101 Switching Protocols\r\n");
    c.print("Upgrade: websocket\r\n");
    c.print("Connection: Upgrade\r\n");
    c.print("Sec-WebSocket-Accept: ");
    c.print(accept);
    c.print("\r\n\r\n");
}

/* ── page sender: 12 KB through a ~6 KB socket buffer, without waiting ──
 * "print the page, then close" is only safe if something waits for the tail to
 * be acknowledged. When the FIN overtakes it the browser receives a truncated
 * <script> - and every button on the page is bound at the END of that script, so
 * the page looks perfect while nothing on it works at all. So each connection
 * owns a small pump: every poll pushes one segment at a time (sockWrite), and the
 * socket is closed once the last byte has been accepted, plus a short grace for
 * the FIN, or when the deadline passes. Nothing blocks. */
static const char HTTP_HDR[] =
    "HTTP/1.1 200 OK\r\n"
    "Content-Type: text/html; charset=utf-8\r\n"
    "Connection: close\r\n"
    "Cache-Control: no-store\r\n\r\n";

#define PAGE_OUT_MAX  2
#define PAGE_OUT_MS   8000   /* a page that cannot be delivered is dropped */
#define PAGE_CLOSE_MS 25     /* let the last bytes leave the send buffer before FIN */
#define WEB_CHUNK     1460   /* one MSS per syscall: never queue more than a segment */
#define WS_FRAME_MAX  256    /* telemetry/log frame buffer (truncates long log lines) */

struct PageOut {
    WiFiClient c;
    uint32_t   off;      /* bytes the socket has accepted so far */
    uint32_t   t0;
    uint32_t   tFin;     /* when the last byte was accepted, 0 = not yet */
};

static PageOut  s_out[PAGE_OUT_MAX];
static uint32_t s_pageLen = 0;    /* header + page, set in begin() */

static void pageClose(PageOut &p) { p.c.stop(); p.c = WiFiClient(); p.off = 0; p.tFin = 0; }

/* Non-blocking write - the only one this socket actually has.
 *
 * WiFiClient has NO working availableForWrite(): the one that answers is
 * Print::availableForWrite(), and it returns 0 ("a write may block") because
 * NetworkClient does not override it. So a loop written as
 *      room = availableForWrite(); if (room <= 0) break;
 * writes ZERO bytes forever, and the page reaches the browser as an empty
 * response (Chrome: ERR_EMPTY_RESPONSE).
 *
 * WiFiClient::write() is not the answer either: with a full send buffer it loops
 * up to 10 x 1 s inside select() and stalls the whole control loop.
 *
 * send(fd, ..., MSG_DONTWAIT) is what the core itself does internally. It returns
 *   > 0  bytes accepted (may be fewer than asked: the buffer is partly full)
 *     0  buffer full, come back on a later poll
 *    -1  socket is gone
 */
static int sockWrite(WiFiClient &c, const uint8_t *b, size_t n) {
    const int fd = c.fd();
    if (fd < 0) return -1;
    const ssize_t r = send(fd, b, n, MSG_DONTWAIT);
    if (r >= 0) return (int)r;
    if (errno == EAGAIN || errno == EWOULDBLOCK) return 0;
    return -1;
}

static void pageOpen(WiFiClient &c) {
    int slot = -1;
    for (int i = 0; i < PAGE_OUT_MAX; i++) if (!s_out[i].c) { slot = i; break; }
    if (slot < 0) {                        /* table full: drop the oldest, keep the news */
        slot = 0;
        for (int i = 1; i < PAGE_OUT_MAX; i++)
            if ((int32_t)(s_out[i].t0 - s_out[slot].t0) < 0) slot = i;
        pageClose(s_out[slot]);
    }
    PageOut &p = s_out[slot];
    p.c    = c;
    p.off  = 0;
    p.t0   = millis();
    p.tFin = 0;
}

static void pagePump(void) {
    const uint32_t hdrLen = (uint32_t)(sizeof(HTTP_HDR) - 1);
    for (int i = 0; i < PAGE_OUT_MAX; i++) {
        PageOut &p = s_out[i];
        if (!p.c) continue;
        if (!s_pageLen || !p.c.connected()) { pageClose(p); continue; }

        bool dead = false;
        while (p.off < s_pageLen) {
            const uint8_t *src;
            uint32_t room;
            if (p.off < hdrLen) {
                src  = (const uint8_t *)HTTP_HDR + p.off;
                room = hdrLen - p.off;
            } else {
                src  = (const uint8_t *)PAGE_HTML + (p.off - hdrLen);
                room = s_pageLen - p.off;
            }
            if (room > WEB_CHUNK) room = WEB_CHUNK;
            const int w = sockWrite(p.c, src, room);
            if (w <= 0) { dead = (w < 0); break; }
            p.off += (uint32_t)w;
            if (w < (int)room) break;            /* buffer is full for now */
        }

        if (p.off >= s_pageLen) {                /* every byte is in the socket */
            if (!p.tFin) p.tFin = millis() | 1;  /* 0 means "not finished yet" */
            if ((uint32_t)(millis() - p.tFin) >= PAGE_CLOSE_MS) {
                s_pages++;
                pageClose(p);
            }
            continue;
        }
        if (dead || (uint32_t)(millis() - p.t0) > PAGE_OUT_MS) {
            /* The one line that names this class of bug: a page that never left the
             * board must not look like a working connection. */
            s_pageFail++;
            if (Serial) {
                Serial.print("[web] page FAILED after ");
                Serial.print((unsigned long)p.off);
                Serial.print(" of ");
                Serial.print((unsigned long)s_pageLen);
                Serial.println(" bytes - the browser gets an empty response");
            }
            pageClose(p);
        }
    }
}

/* ── WebSocket frames ── */
static void wsPong(WiFiClient &c) {
    uint8_t h[2] = {0x8A, 0x00};
    c.write(h, 2);
}

static void wsFill(WiFiClient &c) {
    while (c.available() > 0 && s_have < sizeof(s_buf)) s_buf[s_have++] = (uint8_t)c.read();
}

/* 1 = text (msg ready), 2 = ping, 0 = close, -1 = need more data */
static int wsParse(char *msg, size_t max) {
    size_t n = s_have, pos = 2;
    if (n < 2) return -1;

    uint8_t opcode = s_buf[0] & 0x0F;
    bool    masked = s_buf[1] & 0x80;
    uint64_t plen  = s_buf[1] & 0x7F;

    if (plen == 126) {
        if (n < 4) return -1;
        plen = ((uint64_t)s_buf[2] << 8) | s_buf[3];
        pos = 4;
    } else if (plen == 127) {
        if (n < 10) return -1;
        plen = 0;
        for (int i = 2; i < 10; i++) plen = (plen << 8) | s_buf[i];
        pos = 10;
    }

    uint8_t mask[4];
    if (masked) {
        if (n < pos + 4) return -1;
        for (int i = 0; i < 4; i++) mask[i] = s_buf[pos + i];
        pos += 4;
    }

    if (plen > (uint64_t)(max - 1)) plen = max - 1;
    if (n < pos + plen) return -1;

    size_t i;
    for (i = 0; i < plen; i++) {
        uint8_t b = s_buf[pos + i];
        if (masked) b ^= mask[i & 3];
        msg[i] = (char)b;
    }
    msg[i] = '\0';

    size_t consumed = pos + plen;
    memmove(s_buf, s_buf + consumed, s_have - consumed);
    s_have -= consumed;

    if (opcode == 0x8) return 0;
    if (opcode == 0x9) return 2;
    if (opcode == 0x1) return 1;
    return -1;
}

/* ── CarWeb ── */
bool CarWeb::begin(const char *ssid, const char *pass, const char *hostname, uint16_t port) {
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);                 /* keep the control loop responsive */
    /* the re-connect logic below owns the retries; the core doing it too makes
     * the IDF print "sta is connecting, return error" on every clash */
    WiFi.setAutoReconnect(false);
    WiFi.begin(ssid, pass);               /* returns at once - nothing waits here */

    s_pageLen = (uint32_t)(sizeof(HTTP_HDR) - 1) + (uint32_t)strlen(PAGE_HTML);

    s_host[0] = '\0';
    if (hostname) { strncpy(s_host, hostname, sizeof(s_host) - 1); s_host[sizeof(s_host) - 1] = '\0'; }
    s_port = port;
    s_server = WiFiServer(port);
    s_server.begin(port);
    s_server.setNoDelay(true);
    return true;      /* the request was made; the LINK is reported by wifiUp() */
}

bool CarWeb::wifiUp() { return WiFi.status() == WL_CONNECTED; }

bool CarWeb::connected() { return s_ws && s_ws.connected(); }


uint32_t CarWeb::requests() { return s_reqs; }
uint32_t CarWeb::pagesServed() { return s_pages; }
uint32_t CarWeb::pagesFailed() { return s_pageFail; }
int      CarWeb::parked() { int n = 0; for (int i = 0; i < WEB_PEND_MAX; i++) if (s_pend[i].c) n++; return n; }
int      CarWeb::pageOuts() { int n = 0; for (int i = 0; i < PAGE_OUT_MAX; i++) if (s_out[i].c) n++; return n; }
int      CarWeb::lastCmdAgeMs() {
    return s_lastCmd ? (int)(millis() - s_lastCmd) : -1;
}

/* Sending is the one place where a phone can freeze the car: WiFiClient::write()
 * blocks while TCP accepts the bytes, so a page that went to sleep (or a link that
 * died without FIN) would stall the whole loop with the motors still driven. That
 * is why the frame goes out through sockWrite() and a frame that does not fit is
 * DROPPED - the next telemetry tick is 200 ms away - while a peer that keeps
 * refusing is disconnected so the page can reconnect, which also lets the command
 * watchdog stop the car.
 *
 * Drop the controller: clear every trace of the conversation so the page can
 * reconnect into a clean state. */
static void wsDrop(void) {
    if (s_ws) s_ws.stop();
    s_ws = WiFiClient();
    s_have = 0;
    s_sendDrop = 0;
}

void CarWeb::send(const char *msg) {
    if (!s_ws || !s_ws.connected()) return;

    /* One frame, built in one buffer, handed to the socket in one non-blocking
     * syscall. A frame is atomic on the wire: a partial frame corrupts the stream
     * for good, so a partial write ends the session instead of being retried. */
    uint8_t f[WS_FRAME_MAX];
    size_t  n = strlen(msg);
    if (n > WS_FRAME_MAX - 4) n = WS_FRAME_MAX - 4;    /* a log line, not a record */
    size_t total;
    if (n < 126) {
        f[0] = 0x81; f[1] = (uint8_t)n;
        memcpy(f + 2, msg, n);
        total = n + 2;
    } else {
        f[0] = 0x81; f[1] = 126; f[2] = (uint8_t)(n >> 8); f[3] = (uint8_t)n;
        memcpy(f + 4, msg, n);
        total = n + 4;
    }

    const int w = sockWrite(s_ws, f, total);
    if (w == (int)total) { s_sendDrop = 0; return; }

    if (w != 0) {                                  /* dead, or a torn frame */
        if (Serial) Serial.println(w < 0 ? "[web] peer gone - dropping the session"
                                         : "[web] torn frame - dropping the session");
        wsDrop();
        return;
    }
    if (s_sendDrop++ == 0 && Serial)
        Serial.println("[web] peer not draining - frame dropped, loop not blocked");
    if (s_sendDrop >= 40) {                        /* ~8 s of a wedged peer */
        wsDrop();
        if (Serial) Serial.println("[web] peer dropped, waiting for a new page");
    }
}

void CarWeb::poll() {
    pagePump();                        /* push whatever the page sockets take */

    /* 1. new clients: either the page or a WebSocket upgrade.
     *
     * A browser also opens speculative sockets that never send a request, and a
     * phone that is slow to answer makes the header arrive late. Waiting for it
     * here used to block this function - and therefore the whole control loop -
     * for up to 2 s, which the user sees as "press the button, the car reacts a
     * second later". So a new socket is parked and drained across polls, and
     * nothing in this function ever waits. */
    for (int i = 0; i < WEB_PEND_MAX; i++) {
        WebPending &p = s_pend[i];
        if (!p.c) continue;
        if (!p.c.connected() || (uint32_t)(millis() - p.t0) > WEB_HDR_TIMEOUT_MS) {
            p.c.stop();
            p.c = WiFiClient();
            p.n = 0;
        }
    }

    WiFiClient nc = s_server.accept();
    while (nc) {
        int slot = -1;
        for (int i = 0; i < WEB_PEND_MAX; i++)
            if (!s_pend[i].c) { slot = i; break; }
        if (slot < 0) {
            /* A full table must not cost us the ONE socket that matters: browsers
             * open speculative sockets first, so refuse the OLDEST parked request
             * (it has had its whole timeout to send a header) and take the new
             * one. Refusing the new one made the page retry forever while the car
             * sat there getting no commands at all. */
            slot = 0;
            for (int i = 1; i < WEB_PEND_MAX; i++)
                if ((int32_t)(s_pend[i].t0 - s_pend[slot].t0) < 0) slot = i;
            s_pend[slot].c.stop();
        }
        s_pend[slot].c = nc;
        s_pend[slot].n = 0;
        s_pend[slot].t0 = millis();
        nc = s_server.accept();
    }

    for (int i = 0; i < WEB_PEND_MAX; i++) {
        WebPending &p = s_pend[i];
        if (!p.c) continue;
        while (p.n < sizeof(p.buf) - 1 && p.c.available() > 0) {
            p.buf[p.n++] = (char)p.c.read();
            if (headerDone(p.buf, p.n)) break;
        }
        p.buf[p.n] = 0;
        /* a buffer that filled up without a CRLFCRLF will not get one: answer with
         * what arrived instead of sitting on it until the timeout */
        if (!headerDone(p.buf, p.n) && p.n < sizeof(p.buf) - 1) continue;

        WiFiClient c = p.c;
        p.c = WiFiClient();                 /* slot is free before any I/O */
        p.n = 0;
        s_reqs++;
        /* Echo the request line. This is the single most useful line when the
         * page "cannot be reached": it separates "the board never saw you"
         * (Wi-Fi/IP/firewall/DHCP) from "the board saw you and answered"
         * (then the problem is the response, not the network). */
        if (Serial) {
            char line[64];
            size_t k = 0;
            while (k < sizeof(line) - 1 && p.buf[k] && p.buf[k] != '\r' && p.buf[k] != '\n')
                k++;
            memcpy(line, p.buf, k);
            line[k] = 0;
            Serial.print("[web] req #");
            Serial.print((unsigned long)s_reqs);
            Serial.print(" \"");
            Serial.print(line);
            Serial.println("\"");
        }
        char key[96];
        if (headerVal(p.buf, "Sec-WebSocket-Key:", key, sizeof(key))) {
            wsHandshake(c, key);
            if (s_ws) s_ws.stop();          /* one controller at a time */
            s_ws = c;
            s_have = 0;
            s_sendDrop = 0;
            /* the one line that separates "the page is not connected" from "the
             * page is connected but sends nothing" - without it both look the
             * same from the serial side: the car simply never moves. */
            if (Serial) Serial.println("[web] controller connected (WebSocket up)");
        } else {
            pageOpen(c);               /* served across polls, never in one go */
        }
    }

    /* 2. frames from the active controller */
    char msg[64];
    int r;
    if (s_ws && s_ws.connected()) {
        wsFill(s_ws);
        while ((r = wsParse(msg, sizeof(msg))) >= 1) {
            if (r == 2) {
                wsPong(s_ws);
            } else {
                s_lastCmd = millis();
                if (_fn) _fn(msg);
            }
        }
        if (r == 0) {
            s_ws.stop();
            if (Serial) Serial.println("[web] controller closed the page");
        }
    } else if (s_ws) {
        s_ws = WiFiClient();               /* dropped: clear the slot */
        s_have = 0;
        if (Serial) Serial.println("[web] controller lost - motors stop in 1.5 s");
    }

    /* 3. keep Wi-Fi alive.
     * Only ask for a reconnect when the station really is down: calling
     * reconnect() while the stack is still connecting is refused and the IDF
     * logs "sta is connecting, return error" every time - pure noise. */
    static uint32_t lastTry = 0;
    wl_status_t st = WiFi.status();
    if (st != WL_CONNECTED) {
        if (s_linkUp) {
            s_linkUp = false;
            lastTry = millis() - 5000;      /* retry once immediately */
            if (Serial) Serial.println("[wifi] link lost - retrying every 5 s");
        }
        const bool reallyDown = (st == WL_DISCONNECTED || st == WL_CONNECTION_LOST ||
                                 st == WL_CONNECT_FAILED || st == WL_NO_SSID_AVAIL);
        if (reallyDown && (uint32_t)(millis() - lastTry) >= 5000) {
            lastTry = millis();
            WiFi.reconnect();
        }
    } else if (!s_linkUp) {
        s_linkUp = true;
        if (Serial) {
            Serial.print("[wifi] connected, IP ");
            Serial.println(WiFi.localIP().toString());
            /* the "open http://..." hint lives in appLink(), which also mirrors it
             * to the page - one line per fact, not two */
        }
        if (s_host[0] && MDNS.begin(s_host)) MDNS.addService("http", "tcp", s_port);
    }
}
