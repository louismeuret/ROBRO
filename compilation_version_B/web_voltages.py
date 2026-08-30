#!/usr/bin/env python3
"""Live web dashboard for the ROBRO STM32 telemetry (voltages + currents).

Reads "VDC,VU,VV,VW,IU,IV,IW" raw-value lines from the STM32 serial link and
serves a browser UI with live values and a small history chart.

Requirements: pyserial (stdlib http.server / SSE for the web side)

Usage:
    sudo python3 web_voltages.py                 # /dev/ttyUSB0, port 8080
    sudo python3 web_voltages.py /dev/ttyUSB1 8000

Then open http://localhost:8080/ in a browser.
"""
import json
import re
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import serial

SERIAL_PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"   # ESP32-C3 native USB
BAUD = 115200   # ESP32 bridge (was 57600 for the old Arduino SoftwareSerial bridge)
HTTP_PORT = int(sys.argv[2]) if len(sys.argv) > 2 else 8080

# Values are shown as raw 12-bit ADC counts (0-4095).
pat = re.compile(r"VDC,(\d+),VU,(\d+),VV,(\d+),VW,(\d+),IU,(\d+),IV,(\d+),IW,(\d+)")

state = {"vdc": 0, "vu": 0, "vv": 0, "vw": 0, "iu": 0, "iv": 0, "iw": 0, "count": 0}
lock = threading.Lock()
HISTORY_LEN = 300
history = []  # list of dicts


def serial_reader():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD, timeout=0.2)
    except Exception as e:
        print("serial error:", e)
        return
    ser.reset_input_buffer()
    print("reading", SERIAL_PORT, "at", BAUD, "-> http://localhost:%d/" % HTTP_PORT)
    while True:
        try:
            line = ser.readline().decode(errors="ignore")
        except serial.SerialException:
            break
        m = pat.search(line)
        if m:
            with lock:
                state["vdc"] = int(m.group(1))
                state["vu"] = int(m.group(2))
                state["vv"] = int(m.group(3))
                state["vw"] = int(m.group(4))
                state["iu"] = int(m.group(5))
                state["iv"] = int(m.group(6))
                state["iw"] = int(m.group(7))
                state["count"] += 1
                history.append(dict(state))
                if len(history) > HISTORY_LEN:
                    del history[0]


VCH = ["vdc", "vu", "vv", "vw"]
ACH = ["iu", "iv", "iw"]


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def do_GET(self):
        if self.path == "/":
            body = HTML.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif self.path == "/events":
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            try:
                while True:
                    with lock:
                        payload = json.dumps({
                            "vals": {k: state[k] for k in VCH + ACH},
                            "count": state["count"],
                            "hist": [[state[k] for k in VCH + ACH]
                                     for state in history[-HISTORY_LEN:]],
                        })
                    self.wfile.write(("data: %s\n\n" % payload).encode())
                    self.wfile.flush()
                    time.sleep(0.05)
            except (BrokenPipeError, ConnectionResetError):
                pass
        else:
            self.send_response(404)
            self.end_headers()


HTML = """<!DOCTYPE html>
<html><head><meta charset="utf-8"><title>ROBRO voltages & currents</title>
<style>
body { font-family: sans-serif; background:#111; color:#eee; margin:0; padding:20px; }
h1 { font-size:20px; margin:0 0 14px 4px; }
h1 span { font-weight:normal; color:#888; font-size:13px; }
.sec { font-size:12px; color:#aaa; margin:14px 4px 6px; text-transform:uppercase; letter-spacing:1px; }
.cards { display:flex; gap:12px; flex-wrap:wrap; }
.card { background:#1b1b1b; border:1px solid #333; border-radius:10px;
        padding:12px 18px; min-width:130px; }
.card .name { font-size:12px; color:#aaa; text-transform:uppercase; letter-spacing:1px; }
.card .raw { font-size:26px; font-weight:bold; margin-top:4px; }
#vdc .raw { color:#f55; } #vu .raw { color:#5f5; } #vv .raw { color:#5bf; } #vw .raw { color:#fc5; }
#iu .raw { color:#f8f; } #iv .raw { color:#5ff; } #iw .raw { color:#cf5; }
canvas { margin-top:14px; background:#131313; border:1px solid #333; border-radius:10px; }
.legend { margin-top:4px; font-size:12px; color:#bbb; }
 .legend b { margin-right:12px; }
.yctrl { margin:6px 4px; font-size:12px; color:#bbb; display:flex; align-items:center; gap:8px; }
.yctrl input[type=range] { width:220px; background:#222; }
.yctrl .val { color:#eee; min-width:40px; }
.meta { color:#777; font-size:12px; margin-top:10px; }
</style></head><body>
<h1>ROBRO telemetry <span>raw ADC counts (0-4095)</span></h1>
<div class="sec">Voltages</div>
<div class="cards">
  <div class="card" id="vdc"><div class="name">VDC</div><div class="raw">-</div></div>
  <div class="card" id="vu"><div class="name">VU</div><div class="raw">-</div></div>
  <div class="card" id="vv"><div class="name">VV</div><div class="raw">-</div></div>
  <div class="card" id="vw"><div class="name">VW</div><div class="raw">-</div></div>
</div>
<div class="yctrl">Y max:
  <input type="range" id="maxv" min="1" max="4095" step="10" value="4095">
  <span class="val" id="maxvv">4095</span></div>
<canvas id="chartv" width="940" height="180"></canvas>
<div class="legend">
  <b style="color:#f55">VDC</b><b style="color:#5f5">VU</b>
  <b style="color:#5bf">VV</b><b style="color:#fc5">VW</b>
</div>
<div class="sec">Currents</div>
<div class="cards">
  <div class="card" id="iu"><div class="name">IU</div><div class="raw">-</div></div>
  <div class="card" id="iv"><div class="name">IV</div><div class="raw">-</div></div>
  <div class="card" id="iw"><div class="name">IW</div><div class="raw">-</div></div>
</div>
<div class="yctrl">Y max:
  <input type="range" id="maxa" min="1" max="4095" step="10" value="4095">
  <span class="val" id="maxaa">4095</span></div>
<canvas id="charta" width="940" height="180"></canvas>
<div class="legend">
  <b style="color:#f8f">IU</b><b style="color:#5ff">IV</b><b style="color:#cf5">IW</b>
</div>
<div class="meta" id="meta">waiting for data...</div>
<script>
var vkeys = ["vdc","vu","vv","vw"];
var akeys = ["iu","iv","iw"];
var vcols = {vdc:"#f55", vu:"#5f5", vv:"#5bf", vw:"#fc5"};
var acols = {iu:"#f8f", iv:"#5ff", iw:"#cf5"};
var es = new EventSource("/events");
var maxv = document.getElementById("maxv");
var maxa = document.getElementById("maxa");
maxv.oninput = function(){ document.getElementById("maxvv").textContent = maxv.value; };
maxa.oninput = function(){ document.getElementById("maxaa").textContent = maxa.value; };
es.onmessage = function(e) {
  var d = JSON.parse(e.data);
  var all = vkeys.concat(akeys);
  all.forEach(function(k){
    document.getElementById(k).querySelector(".raw").textContent = d.vals[k];
  });
  document.getElementById("meta").textContent = "samples: " + d.count +
    " &middot; history: " + d.hist.length + " points";
  draw("chartv", "maxv", d.hist, vkeys, vcols);
  draw("charta", "maxa", d.hist, akeys, acols);
};
function draw(cid, maxid, hist, keys, cols) {
  var cv = document.getElementById(cid), ctx = cv.getContext("2d");
  ctx.clearRect(0,0,cv.width,cv.height);
  var n = hist.length; if (!n) return;
  var max = parseInt(document.getElementById(maxid).value, 10);
  if (!max || max < 1) max = 4095;
  var w = cv.width, h = cv.height, pad = 20;
  ctx.strokeStyle = "#2a2a2a"; ctx.beginPath();
  for (var g=0; g<=4; g++){ var y = pad + (h-2*pad)*g/4; ctx.moveTo(0,y); ctx.lineTo(w,y); }
  ctx.stroke();
  var idx = 0;
  for (var i=0;i<keys.length;i++){
    idx = vkeys.indexOf(keys[i]) >= 0 ? vkeys.indexOf(keys[i]) : 4 + akeys.indexOf(keys[i]);
    ctx.strokeStyle = cols[keys[i]]; ctx.lineWidth = 1.5; ctx.beginPath();
    for (var x=0;x<n;x++){
      var yy = pad + (h-2*pad) * (1 - Math.min(hist[x][idx], max)/max);
      var xx = (n===1?0:w*(x/(n-1)));
      if (x===0) ctx.moveTo(xx,yy); else ctx.lineTo(xx,yy);
    }
    ctx.stroke();
  }
  ctx.fillStyle = "#888"; ctx.font = "11px sans-serif"; ctx.fillText(max, 4, pad+4);
  ctx.fillText("0", 4, h-4);
}
</script></body></html>"""


def main():
    threading.Thread(target=serial_reader, daemon=True).start()
    httpd = ThreadingHTTPServer(("", HTTP_PORT), Handler)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
