/*
 * ROBRO — ESP32-C3 SPI bridge for the STM32G474 motor controller.
 *
 * The STM32 is the SPI master; this ESP32 is the slave. Every 50 ms the STM32
 * clocks one fixed 32-byte transfer: a telemetry frame goes out on MOSI while
 * the command frame this sketch staged comes back on MISO. One transfer,
 * both directions.
 *
 * Wiring — all four STM32 signals are on the existing header, so nothing has
 * to be soldered to the LED pads:
 *
 *     STM32 PB3 (GPIO4, SPI_SCK)   ->  C3 GPIO4  (SCLK)
 *     STM32 PA7 (GPIO6, SPI_MOSI)  ->  C3 GPIO5  (MOSI)
 *     STM32 PB4 (GPIO5, SPI_MISO)  <-  C3 GPIO6  (MISO)
 *     STM32 PB9 (free header pin)  ->  C3 GPIO7  (CS, active low)
 *     STM32 GND                    ——  C3 GND
 *
 * Board: ESP32-C3 SuperMini. In the Arduino IDE set "USB CDC On Boot: Enabled"
 * so Serial goes to the native USB port — the board then enumerates as
 * /dev/ttyACM0, NOT /dev/ttyUSB0. Pass that path to web_voltages.py.
 *
 * NOTE: the two RJ45 jacks on the STM32 board are NOT Ethernet. They carry the
 * RS485 daisy-chain bus. Nothing here plugs into them.
 *
 * Leave WIFI_SSID empty to skip WiFi and run as a plain USB bridge.
 */

#include <WiFi.h>
#include <WebServer.h>
#include "driver/spi_slave.h"
#include "driver/gpio.h"

// ---------------------------------------------------------------- config ---
static const char *WIFI_SSID = "Kobox";        // "" -> no WiFi, USB output only
static const char *WIFI_PASS = "Vivelakolok";

// ESP32-C3 SuperMini. GPIO4/5/6/7 is the only clean contiguous block on this
// board: GPIO2/8/9 are strapping pins (driving them at boot can stop the board
// starting or force download mode), GPIO18/19 are the native USB D-/D+ that
// the SuperMini's USB port uses, and GPIO20/21 are UART0.
#define PIN_SCLK    4
#define PIN_MOSI    5
#define PIN_MISO    6
#define PIN_CS      7

// The C3 has a single user-available SPI controller. SPI2_HOST is also valid
// on the classic ESP32 (where it is HSPI), so this builds on both.
#define SPI_HOST_ID SPI2_HOST

static const long USB_BAUD = 115200;

// Raw 12-bit ADC counts -> engineering units: the value at 4095 counts.
// Set these to your divider / shunt ratios.
static const float VSCALE_FS = 400.0f;    // volts at 4095 counts
static const float ISCALE_FS = 10.0f;     // amps  at 4095 counts

// ------------------------------------------------------------- protocol ----
static const size_t FRAME_LEN = 72;
static const size_t MAX_PEERS  = 4;

static const uint8_t TX_SYNC0 = 0xA5, TX_SYNC1 = 0x5A;   // STM32 -> ESP32
static const uint8_t RX_SYNC0 = 0xC3, RX_SYNC1 = 0x3C;   // ESP32 -> STM32

enum : uint8_t {
  CMD_NOP = 0, CMD_SET_FREQ = 1, CMD_SET_MOD = 2,
  CMD_START = 3, CMD_STOP = 4, CMD_PING = 5,
  CMD_SET_MOD_START = 6, CMD_SET_RAMP_MS = 7,
  CMD_SET_DIR = 8, CMD_SET_LED = 9
};

// LED override modes, matching ESP_LED_* in main.c.
enum : uint8_t { LED_AUTO = 0, LED_ON = 1, LED_OFF = 2 };

// DMA-capable buffers must be word aligned.
WORD_ALIGNED_ATTR static uint8_t spiRx[FRAME_LEN];   // telemetry from STM32
WORD_ALIGNED_ATTR static uint8_t spiTx[FRAME_LEN];   // command to STM32

// ----------------------------------------------------------------- state ---
WebServer server(80);

struct Peer { uint32_t id = 0; uint16_t age10ms = 0; };

struct Telemetry {
  uint16_t vdc = 0, vu = 0, vv = 0, vw = 0, iu = 0, iv = 0, iw = 0;
  bool fault15v = false;
  bool outputsEnabled = false;
  bool reverse = false;
  bool pb10 = false;
  uint8_t peerCount = 0;
  uint32_t peerFrames = 0;
  uint16_t frqCentiHz = 0;
  uint16_t modMilli = 0;
  uint32_t boardId = 0;
  uint16_t modStartMilli = 0;
  uint16_t rampMs = 0;
  uint16_t rampFracMilli = 0;
  uint16_t pwmArr = 0;
  uint8_t ledMode[2] = { LED_AUTO, LED_AUTO };
  Peer peers[MAX_PEERS];
  uint32_t frames = 0;      // good frames received
  uint32_t badFrames = 0;   // sync/checksum rejects
  unsigned long lastMs = 0;
};

static Telemetry tel;

static uint8_t pendingCmd = CMD_NOP;
static uint16_t pendingArg = 0;
static uint8_t cmdSeq = 0;
static bool queued = false;

// ------------------------------------------------------------- helpers -----
static uint16_t rd16(const uint8_t *p) { return (uint16_t)p[0] | ((uint16_t)p[1] << 8); }
static uint32_t rd32(const uint8_t *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static uint8_t xorsum(const uint8_t *p, size_t n) {
  uint8_t x = 0;
  for (size_t i = 0; i < n; i++) x ^= p[i];
  return x;
}

// Stage the command frame the STM32 will collect on the NEXT transfer.
static void buildCommandFrame() {
  memset(spiTx, 0, FRAME_LEN);
  spiTx[0] = RX_SYNC0;
  spiTx[1] = RX_SYNC1;
  spiTx[2] = pendingCmd;
  spiTx[3] = cmdSeq;
  spiTx[4] = (uint8_t)pendingArg;
  spiTx[5] = (uint8_t)(pendingArg >> 8);
  spiTx[FRAME_LEN - 1] = xorsum(spiTx, FRAME_LEN - 1);
}

static void postCommand(uint8_t cmd, uint16_t arg) {
  pendingCmd = cmd;
  pendingArg = arg;
  cmdSeq++;                 // the STM32 acts only when the sequence changes
}

static bool parseTelemetry(const uint8_t *f) {
  if (f[0] != TX_SYNC0 || f[1] != TX_SYNC1) return false;
  if (xorsum(f, 70) != f[70]) return false;

  tel.vdc = rd16(&f[2]);
  tel.vu  = rd16(&f[4]);
  tel.vv  = rd16(&f[6]);
  tel.vw  = rd16(&f[8]);
  tel.iu  = rd16(&f[10]);
  tel.iv  = rd16(&f[12]);
  tel.iw  = rd16(&f[14]);
  tel.fault15v       = (f[16] & 0x01) != 0;
  tel.outputsEnabled = (f[16] & 0x02) != 0;
  tel.reverse        = (f[16] & 0x04) != 0;
  tel.pb10           = (f[16] & 0x08) != 0;
  tel.peerCount     = f[17];
  tel.peerFrames    = rd32(&f[18]);
  tel.frqCentiHz    = rd16(&f[22]);
  tel.modMilli      = rd16(&f[24]);
  tel.boardId       = rd32(&f[26]);
  tel.modStartMilli = rd16(&f[30]);
  tel.rampMs        = rd16(&f[32]);
  tel.rampFracMilli = rd16(&f[34]);
  tel.pwmArr        = rd16(&f[36]);
  tel.ledMode[0]    = f[38];
  tel.ledMode[1]    = f[39];
  for (size_t i = 0; i < MAX_PEERS; i++) {
    tel.peers[i].id      = rd32(&f[40 + i * 6]);
    tel.peers[i].age10ms = rd16(&f[44 + i * 6]);
  }
  tel.frames++;
  tel.lastMs = millis();
  return true;
}

// Mirror to USB in the original ASCII format, so web_voltages.py and
// plot_currents.py keep working against the ESP32's USB port unchanged.
static void mirrorToUsb() {
  Serial.printf("VDC,%u,VU,%u,VV,%u,VW,%u,IU,%u,IV,%u,IW,%u,FLT,%u,PEER,%u,FRQ,%u,MOD,%u,EN,%u\r\n",
                tel.vdc, tel.vu, tel.vv, tel.vw, tel.iu, tel.iv, tel.iw,
                tel.fault15v ? 1u : 0u, tel.peerFrames,
                tel.frqCentiHz, tel.modMilli, tel.outputsEnabled ? 1u : 0u);
}

// ---------------------------------------------------------------- web -------
static String jsonState() {
  String s = "{";
  s += "\"vdc\":"   + String(tel.vdc);
  s += ",\"vu\":"   + String(tel.vu);
  s += ",\"vv\":"   + String(tel.vv);
  s += ",\"vw\":"   + String(tel.vw);
  s += ",\"iu\":"   + String(tel.iu);
  s += ",\"iv\":"   + String(tel.iv);
  s += ",\"iw\":"   + String(tel.iw);
  s += ",\"flt\":"  + String(tel.fault15v ? 1 : 0);
  s += ",\"en\":"   + String(tel.outputsEnabled ? 1 : 0);
  s += ",\"rev\":"  + String(tel.reverse ? 1 : 0);
  s += ",\"pb10\":" + String(tel.pb10 ? 1 : 0);
  s += ",\"freq\":" + String(tel.frqCentiHz / 100.0f, 2);
  s += ",\"mod\":"  + String(tel.modMilli / 1000.0f, 3);
  s += ",\"mod0\":" + String(tel.modStartMilli / 1000.0f, 3);
  s += ",\"ramp\":" + String(tel.rampMs);
  s += ",\"prog\":" + String(tel.rampFracMilli / 10.0f, 1);
  s += ",\"arr\":"  + String(tel.pwmArr);
  s += ",\"pwmhz\":" + String(tel.pwmArr ? (16000000UL / (2UL * tel.pwmArr)) : 0);
  s += ",\"led0\":" + String(tel.ledMode[0]);
  s += ",\"led1\":" + String(tel.ledMode[1]);
  s += ",\"id\":"   + String(tel.boardId);
  s += ",\"pframes\":" + String(tel.peerFrames);
  s += ",\"frames\":"  + String(tel.frames);
  s += ",\"bad\":"     + String(tel.badFrames);
  s += ",\"age\":"     + String(tel.lastMs ? (millis() - tel.lastMs) : 999999);
  s += ",\"vfs\":"     + String(VSCALE_FS, 1);
  s += ",\"ifs\":"     + String(ISCALE_FS, 1);
  s += ",\"peers\":[";
  for (uint8_t i = 0; i < tel.peerCount && i < MAX_PEERS; i++) {
    if (i) s += ",";
    s += "{\"id\":" + String(tel.peers[i].id) +
         ",\"age\":" + String(tel.peers[i].age10ms * 10) + "}";
  }
  s += "]}";
  return s;
}

static const char PAGE_HTML[] PROGMEM = R"HTML(<!doctype html>
<html><head><meta charset="utf-8"><title>ROBRO control</title></head><body>
<h2>ROBRO - STM32G474 motor controller</h2>
<p id="stat">connecting...</p>

<h3>Sensors</h3>
<table border="1" cellpadding="3" id="sens"></table>

<h3>Digital inputs</h3>
<table border="1" cellpadding="3" id="dig"></table>

<h3>Waveform / motor settings</h3>
<table border="1" cellpadding="3">
<tr><th>Setting</th><th>Now</th><th>Set</th></tr>
<tr><td>Electrical frequency (Hz)</td><td id="v_freq">-</td>
    <td><input id="i_freq" type="number" step="0.5" min="0" max="100" value="20" size="6">
        <button onclick="cmd('freq',f('i_freq'))">Apply</button></td></tr>
<tr><td>Modulation index target (0-0.95)</td><td id="v_mod">-</td>
    <td><input id="i_mod" type="number" step="0.05" min="0" max="0.95" value="0.8" size="6">
        <button onclick="cmd('mod',f('i_mod'))">Apply</button></td></tr>
<tr><td>Modulation index at ramp start (boost)</td><td id="v_mod0">-</td>
    <td><input id="i_mod0" type="number" step="0.01" min="0" max="0.95" value="0.10" size="6">
        <button onclick="cmd('modstart',f('i_mod0'))">Apply</button></td></tr>
<tr><td>Ramp time (ms, min 100)</td><td id="v_ramp">-</td>
    <td><input id="i_ramp" type="number" step="100" min="100" max="65535" value="5000" size="8">
        <button onclick="cmd('ramp',f('i_ramp'))">Apply</button></td></tr>
<tr><td>Direction</td><td id="v_rev">-</td>
    <td><button onclick="cmd('dir',0)">Forward</button>
        <button onclick="cmd('dir',1)">Reverse</button></td></tr>
<tr><td>Ramp progress (%)</td><td id="v_prog">-</td><td>read-only</td></tr>
<tr><td>PWM carrier</td><td id="v_pwm">-</td><td>compile-time (PWM_ARR)</td></tr>
<tr><td>Outputs (BDTR.MOE)</td><td id="v_en">-</td>
    <td><button onclick="cmd('start',0)">START</button>
        <button onclick="cmd('stop',0)">STOP</button></td></tr>
</table>

<h3>LEDs</h3>
<table border="1" cellpadding="3">
<tr><th>LED</th><th>Mode</th><th>Toggle</th></tr>
<tr><td>LED1 (PB6) - normally "15V present"</td><td id="v_led0">-</td>
    <td><button id="b_led0" onclick="ledNext(0)">toggle</button></td></tr>
<tr><td>LED2 (PB7) - normally RS485 activity</td><td id="v_led1">-</td>
    <td><button id="b_led1" onclick="ledNext(1)">toggle</button></td></tr>
</table>
<p>Each button cycles AUTO -&gt; ON -&gt; OFF -&gt; AUTO. In AUTO the firmware owns the LED.</p>

<h3>Daisy chain (RS485)</h3>
<p>This board id: <span id="v_id">-</span> &nbsp;
   Frames from other boards: <span id="v_pframes">-</span> &nbsp;
   <button onclick="cmd('ping',0)">Send PING</button></p>
<table border="1" cellpadding="3" id="peers"></table>

<script>
function f(id){return document.getElementById(id).value;}
function cmd(c,v){fetch('/cmd?c='+c+'&v='+encodeURIComponent(v));}
function ledCmd(i,m){fetch('/cmd?c=led&i='+i+'&v='+m);}
var ledNow=[0,0];
function ledNext(i){ledCmd(i,(ledNow[i]+1)%3);}
function modeName(m){return m==0?'AUTO':(m==1?'ON':'OFF');}
function txt(id,v){document.getElementById(id).textContent=v;}

function row(n,raw,scaled){return '<tr><td>'+n+'</td><td align="right">'+raw+
  '</td><td align="right">'+scaled+'</td></tr>';}

function poll(){
 fetch('/api').then(function(r){return r.json()}).then(function(d){
  var live=d.age<2000;
  txt('stat',(live?'LIVE':'NO DATA')+' - frames '+d.frames+', bad '+d.bad+
      ', last '+d.age+' ms ago');

  var v=d.vfs/4095, i=d.ifs/4095;
  document.getElementById('sens').innerHTML=
    '<tr><th>Signal</th><th>Raw (0-4095)</th><th>Scaled</th></tr>'+
    row('VDC (PB12)',d.vdc,(d.vdc*v).toFixed(1)+' V')+
    row('VU (PB1)',d.vu,(d.vu*v).toFixed(1)+' V')+
    row('VV','n/a','PB2 is a GPIO output, no ADC')+
    row('VW (PA6)',d.vw,(d.vw*v).toFixed(1)+' V')+
    row('IU (PA0)',d.iu,(d.iu*i).toFixed(2)+' A')+
    row('IV (PA4)',d.iv,(d.iv*i).toFixed(2)+' A')+
    row('IW (PB0)',d.iw,(d.iw*i).toFixed(2)+' A');

  document.getElementById('dig').innerHTML=
    '<tr><th>Input</th><th>State</th></tr>'+
    '<tr><td>15V rail (PA15)</td><td>'+(d.flt?'FAULT - no 15V':'OK')+'</td></tr>'+
    '<tr><td>PB10 (EXTI input)</td><td>'+(d.pb10?'HIGH':'LOW')+'</td></tr>';

  txt('v_freq',d.freq.toFixed(2)+' Hz');
  txt('v_mod',d.mod.toFixed(3));
  txt('v_mod0',d.mod0.toFixed(3));
  txt('v_ramp',d.ramp+' ms');
  txt('v_rev',d.rev?'REVERSE':'FORWARD');
  txt('v_prog',d.prog.toFixed(1)+' %');
  txt('v_pwm',d.pwmhz+' Hz (ARR '+d.arr+')');
  txt('v_en',d.en?'ENABLED':'DISABLED');

  ledNow=[d.led0,d.led1];
  txt('v_led0',modeName(d.led0));
  txt('v_led1',modeName(d.led1));
  document.getElementById('b_led0').textContent='-> '+modeName((d.led0+1)%3);
  document.getElementById('b_led1').textContent='-> '+modeName((d.led1+1)%3);

  txt('v_id',d.id);
  txt('v_pframes',d.pframes);
  var h='<tr><th>#</th><th>Board id</th><th>Last seen</th></tr>';
  if(d.peers.length==0){h+='<tr><td colspan="3">no other boards seen</td></tr>';}
  for(var k=0;k<d.peers.length;k++){
    h+='<tr><td>'+(k+1)+'</td><td>'+d.peers[k].id+'</td><td>'+
       d.peers[k].age+' ms ago</td></tr>';
  }
  document.getElementById('peers').innerHTML=h;
 }).catch(function(){txt('stat','ESP32 unreachable');});
}
setInterval(poll,500);poll();
</script>
</body></html>)HTML";

static void handleRoot() { server.send_P(200, "text/html", PAGE_HTML); }
static void handleApi()  { server.send(200, "application/json", jsonState()); }

static void handleCmd() {
  String c = server.arg("c");
  float v = server.arg("v").toFloat();

  if      (c == "freq")      postCommand(CMD_SET_FREQ,      (uint16_t)(constrain(v, 0.0f, 100.0f) * 100.0f));
  else if (c == "mod")       postCommand(CMD_SET_MOD,       (uint16_t)(constrain(v, 0.0f, 0.95f) * 1000.0f));
  else if (c == "modstart")  postCommand(CMD_SET_MOD_START, (uint16_t)(constrain(v, 0.0f, 0.95f) * 1000.0f));
  else if (c == "ramp")      postCommand(CMD_SET_RAMP_MS,   (uint16_t)constrain(v, 100.0f, 65535.0f));
  else if (c == "dir")       postCommand(CMD_SET_DIR,       (uint16_t)(v != 0.0f ? 1 : 0));
  else if (c == "start")     postCommand(CMD_START, 0);
  else if (c == "stop")      postCommand(CMD_STOP, 0);
  else if (c == "ping")      postCommand(CMD_PING, 0);
  else if (c == "led") {
    uint8_t idx  = (uint8_t)server.arg("i").toInt();
    uint8_t mode = (uint8_t)constrain(v, 0.0f, 2.0f);
    if (idx > 1) { server.send(400, "text/plain", "bad led index"); return; }
    postCommand(CMD_SET_LED, (uint16_t)(idx | (mode << 8)));
  }
  else { server.send(400, "text/plain", "unknown command"); return; }

  server.send(200, "text/plain", "queued");
}

// ----------------------------------------------------------------- setup ---
void setup() {
  Serial.begin(USB_BAUD);
  delay(200);
  Serial.println();
  Serial.println("ROBRO ESP32-C3 SPI bridge — slave on SPI2 (SCLK4 MOSI5 MISO6 CS7)");

  spi_bus_config_t buscfg = {};
  buscfg.mosi_io_num = PIN_MOSI;
  buscfg.miso_io_num = PIN_MISO;
  buscfg.sclk_io_num = PIN_SCLK;
  buscfg.quadwp_io_num = -1;
  buscfg.quadhd_io_num = -1;

  spi_slave_interface_config_t slvcfg = {};
  slvcfg.mode = 0;                 // CPOL=0 CPHA=0, matches the STM32 config
  slvcfg.spics_io_num = PIN_CS;
  slvcfg.queue_size = 3;
  slvcfg.flags = 0;

  // Idle-high pulls so the bus is defined while the STM32 is in reset.
  gpio_set_pull_mode((gpio_num_t)PIN_MOSI, GPIO_PULLUP_ONLY);
  gpio_set_pull_mode((gpio_num_t)PIN_SCLK, GPIO_PULLUP_ONLY);
  gpio_set_pull_mode((gpio_num_t)PIN_CS,   GPIO_PULLUP_ONLY);

  esp_err_t err = spi_slave_initialize(SPI_HOST_ID, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);
  if (err != ESP_OK) {
    Serial.printf("spi_slave_initialize failed: %d\n", (int)err);
    while (true) delay(1000);
  }

  if (WIFI_SSID[0] != '\0') {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.print("WiFi: connecting");
    unsigned long t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
      delay(300);
      Serial.print('.');
    }
    Serial.println();
    if (WiFi.status() == WL_CONNECTED) {
      Serial.print("WiFi: dashboard at http://");
      Serial.println(WiFi.localIP());
    } else {
      Serial.println("WiFi: failed — USB output only");
    }
    server.on("/", handleRoot);
    server.on("/api", handleApi);
    server.on("/cmd", handleCmd);
    server.begin();
  } else {
    Serial.println("WiFi: disabled (WIFI_SSID empty) — USB output only");
  }
}

// ------------------------------------------------------------------ loop ---
void loop() {
  static spi_slave_transaction_t trans;

  // Stage the next transfer. Whatever command is pending now is what the
  // STM32 will collect on its next exchange.
  if (!queued) {
    buildCommandFrame();
    memset(spiRx, 0, FRAME_LEN);
    memset(&trans, 0, sizeof(trans));
    trans.length    = FRAME_LEN * 8;
    trans.tx_buffer = spiTx;
    trans.rx_buffer = spiRx;
    if (spi_slave_queue_trans(SPI_HOST_ID, &trans, 0) == ESP_OK) queued = true;
  }

  // Non-blocking collect, so the web server stays responsive between transfers.
  if (queued) {
    spi_slave_transaction_t *done = nullptr;
    if (spi_slave_get_trans_result(SPI_HOST_ID, &done, 0) == ESP_OK) {
      queued = false;
      if (parseTelemetry(spiRx)) mirrorToUsb();
      else                       tel.badFrames++;
    }
  }

  if (WIFI_SSID[0] != '\0') server.handleClient();
}
