/*
 * ============================================================
 *  IR Backup System v3 — Seeed XIAO ESP32-C6
 * ============================================================
 *  Wiring:
 *    D0 (GPIO0) → IR LED Transmitter (+ resistor 100Ω ke GND)
 *    D2 (GPIO4) → TSOP1838 OUT (Receiver)  ← PENTING: D1/GPIO3 adalah RF enable!
 *    3V3        → TSOP1838 Vcc
 *    GND        → TSOP1838 GND & IR LED Kathode
 *
 *  Cara pakai:
 *    1. Upload ke XIAO ESP32-C6
 *    2. Connect ke WiFi: "IR-Backup" password: "12345678"
 *    3. Buka browser: http://192.168.4.1
 *       (Captive Portal otomatis muncul di HP)
 *    4. Ketik nama sinyal → klik Capture → tekan remote
 *    5. Klik ▶ Kirim untuk replay sinyal
 *    6. Install sebagai PWA: klik "Add to Home Screen" di browser
 *
 *  Storage:
 *    - 50 slot sinyal (cukup untuk 1 remote 40 tombol + cadangan)
 *    - Disimpan permanen di Flash (LittleFS) — tidak hilang saat reset
 *
 *  Library: hanya built-in Arduino ESP32 3.x, tidak perlu install tambahan
 *  Board  : Seeed XIAO ESP32C6 (esp32 board package >= 3.0.0)
 * ============================================================
 */

#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <LittleFS.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/rmt_rx.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"
#include "esp_wifi.h"   // force 802.11bgn on ESP32-C6

// ── Pin Config ───────────────────────────────────────────────
// XIAO ESP32-C6 pinout: D0=GPIO0, D1=GPIO1, D2=GPIO2, D3=GPIO21
// Referensi: https://wiki.seeedstudio.com/xiao_esp32c6_getting_started/
#define PIN_IR_TX   GPIO_NUM_0    // D0 header → IR LED Transmitter (D0=GPIO0)
#define PIN_IR_RX   GPIO_NUM_2    // D2 header → TSOP1838 OUT (D2=GPIO2)
#define PIN_LED     15            // GPIO15 = USER LED onboard (LOW=ON, HIGH=OFF)
#define REPLAY_DURATION_MS  2000  // kirim sinyal berulang selama 2 detik

// ── WiFi AP ──────────────────────────────────────────────────
const char* WIFI_SSID = "IR-Backup";
const char* WIFI_PASS = "12345678";   // min 8 karakter, kosong = open (mudah disconnect)

// ── Captive Portal DNS ───────────────────────────────────────
DNSServer dnsServer;
#define DNS_PORT 53

// ── RMT ──────────────────────────────────────────────────────
#define RMT_RESOLUTION_HZ    1000000UL
#define IR_CARRIER_HZ        38000
#define MAX_RMT_SYMBOLS      64          // ESP32-C6 hardware max 48/channel; 64 pakai 2 blok
#define CAPTURE_TIMEOUT_MS   5000

// ── Storage ──────────────────────────────────────────────────
#define MAX_SIGNALS    50             // ← 50 slot, cukup 40 tombol + cadangan
#define MAX_TIMINGS    300            // naik dari 256 → 300 untuk remote AC kompleks
#define FS_DIR         "/ir"          // folder di LittleFS

struct IRSignal {
  String   name;
  uint16_t timings[MAX_TIMINGS];     // mark/space alternating (µs)
  int      count;
  bool     valid;
};

IRSignal  g_signals[MAX_SIGNALS];
int       g_signal_count = 0;

// ── RMT Handles ──────────────────────────────────────────────
static rmt_channel_handle_t  rx_chan  = NULL;
static rmt_channel_handle_t  tx_chan  = NULL;
static rmt_encoder_handle_t  copy_enc = NULL;
static QueueHandle_t          rx_queue = NULL;
static rmt_symbol_word_t      rx_buf[MAX_RMT_SYMBOLS];

// ── Capture / Replay State ────────────────────────────────────
volatile bool  g_capturing     = false;
volatile bool  g_replaying     = false;
unsigned long  g_capture_start = 0;
String         g_pending_name  = "";
bool           g_last_ok       = false;
String         g_last_msg      = "";

// ── LED State Machine ─────────────────────────────────────────
unsigned long  g_led_timer     = 0;
int            g_led_phase     = 0;

WebServer server(80);

// Frekuensi carrier aktif (bisa diubah via web)
uint32_t g_carrier_hz = IR_CARRIER_HZ;  // default 38000

// ─────────────────────────────────────────────────────────────
//  LittleFS — Simpan/Load sinyal ke Flash
// ─────────────────────────────────────────────────────────────
//  Format binary per file /ir/N.bin:
//    [1 byte]  panjang nama
//    [N bytes] nama string
//    [2 bytes] count (jumlah timings)
//    [count*2] timings uint16_t

bool fs_save(int idx) {
  char path[24];
  snprintf(path, sizeof(path), "%s/%d.bin", FS_DIR, idx);
  File f = LittleFS.open(path, "w", true);
  if (!f) { Serial.println("[FS] Gagal buka " + String(path)); return false; }

  IRSignal& s  = g_signals[idx];
  uint8_t nlen = (uint8_t)min((int)s.name.length(), 63);
  f.write(nlen);
  f.write((const uint8_t*)s.name.c_str(), nlen);
  uint16_t cnt = (uint16_t)s.count;
  f.write((uint8_t*)&cnt, 2);
  f.write((uint8_t*)s.timings, cnt * 2);
  f.close();
  Serial.println("[FS] Saved idx=" + String(idx) + " '" + s.name + "'");
  return true;
}

bool fs_load(int idx) {
  char path[24];
  snprintf(path, sizeof(path), "%s/%d.bin", FS_DIR, idx);
  if (!LittleFS.exists(path)) return false;

  File f = LittleFS.open(path, "r");
  if (!f) return false;

  uint8_t nlen = f.read();
  char    nbuf[64] = {};
  f.read((uint8_t*)nbuf, nlen);

  uint16_t cnt = 0;
  f.read((uint8_t*)&cnt, 2);
  if (cnt > MAX_TIMINGS) cnt = MAX_TIMINGS;

  g_signals[idx].name  = String(nbuf);
  g_signals[idx].count = (int)cnt;
  f.read((uint8_t*)g_signals[idx].timings, cnt * 2);
  g_signals[idx].valid = (cnt >= 6);
  f.close();
  return g_signals[idx].valid;
}

void fs_load_all() {
  g_signal_count = 0;
  if (!LittleFS.exists(FS_DIR)) {
    LittleFS.mkdir(FS_DIR);
    return;
  }
  for (int i = 0; i < MAX_SIGNALS; i++) {
    if (fs_load(i)) g_signal_count++;
    else break;
  }
  Serial.println("[FS] Loaded " + String(g_signal_count) + " sinyal dari flash");
}

void fs_delete(int idx) {
  // Hapus file yang dituju
  char path[24];
  snprintf(path, sizeof(path), "%s/%d.bin", FS_DIR, idx);
  LittleFS.remove(path);

  // Geser file berikutnya supaya nomor urut tetap rapi
  for (int i = idx; i < g_signal_count - 1; i++) {
    char from[24], to[24];
    snprintf(from, sizeof(from), "%s/%d.bin", FS_DIR, i + 1);
    snprintf(to,   sizeof(to),   "%s/%d.bin", FS_DIR, i);
    LittleFS.rename(from, to);
  }
}

void fs_clear_all() {
  for (int i = 0; i < g_signal_count; i++) {
    char path[24];
    snprintf(path, sizeof(path), "%s/%d.bin", FS_DIR, i);
    LittleFS.remove(path);
  }
  g_signal_count = 0;
  Serial.println("[FS] Semua sinyal dihapus");
}

// ─────────────────────────────────────────────────────────────
//  RMT — ISR Callback
// ─────────────────────────────────────────────────────────────
static bool IRAM_ATTR rmt_rx_done_cb(rmt_channel_handle_t chan,
                                      const rmt_rx_done_event_data_t* edata,
                                      void* ctx) {
  BaseType_t woken = pdFALSE;
  xQueueSendFromISR((QueueHandle_t)ctx, edata, &woken);
  return woken == pdTRUE;
}

// ─────────────────────────────────────────────────────────────
//  RMT Setup
// ─────────────────────────────────────────────────────────────
void rmt_setup() {
  rx_queue = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));

  // RX (TSOP1838)
  // ESP32-C6: SOC_RMT_MEM_WORDS_PER_CHANNEL = 48; gunakan tepat 48 agar tidak overflow
  rmt_rx_channel_config_t rx_cfg = {};
  rx_cfg.gpio_num          = PIN_IR_RX;
  rx_cfg.clk_src           = RMT_CLK_SRC_DEFAULT;
  rx_cfg.resolution_hz     = RMT_RESOLUTION_HZ;
  rx_cfg.mem_block_symbols = 48;          // ← tetap dalam 1 hardware block
  ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_cfg, &rx_chan));

  rmt_rx_event_callbacks_t cbs = {};
  cbs.on_recv_done = rmt_rx_done_cb;
  ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rx_chan, &cbs, rx_queue));
  ESP_ERROR_CHECK(rmt_enable(rx_chan));

  // TX (IR LED)
  rmt_tx_channel_config_t tx_cfg = {};
  tx_cfg.gpio_num          = PIN_IR_TX;
  tx_cfg.clk_src           = RMT_CLK_SRC_DEFAULT;
  tx_cfg.resolution_hz     = RMT_RESOLUTION_HZ;
  tx_cfg.mem_block_symbols = 48;          // ← 1 hardware block, TX chunking via queue
  tx_cfg.trans_queue_depth = 4;
  ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_cfg, &tx_chan));

  rmt_carrier_config_t carrier = {};
  carrier.frequency_hz = IR_CARRIER_HZ;
  carrier.duty_cycle   = 0.33f;
  // polarity & always_on: default (zero-init) sudah benar untuk ESP-IDF v5.1+
  // polarity_active_low=0 → carrier aktif saat level HIGH (benar untuk IR LED)
  // always_on=0 → carrier hanya aktif saat transmit (bukan idle)
  ESP_ERROR_CHECK(rmt_apply_carrier(tx_chan, &carrier));
  ESP_ERROR_CHECK(rmt_enable(tx_chan));

  rmt_copy_encoder_config_t enc_cfg = {};
  ESP_ERROR_CHECK(rmt_new_copy_encoder(&enc_cfg, &copy_enc));

  Serial.println("[RMT] OK — TX=GPIO0(D0) RX=GPIO2(D2) carrier=38kHz");
}

// ─────────────────────────────────────────────────────────────
//  Capture & Replay
// ─────────────────────────────────────────────────────────────
void capture_start(const String& name) {
  xQueueReset(rx_queue);
  rmt_receive_config_t rcfg = {};
  rcfg.signal_range_min_ns = 1250;
  rcfg.signal_range_max_ns = 12000000;
  ESP_ERROR_CHECK(rmt_receive(rx_chan, rx_buf, sizeof(rx_buf), &rcfg));
  g_capturing     = true;
  g_pending_name  = name;
  g_capture_start = millis();
  Serial.println("[CAP] Waiting: " + name);
}

void capture_process(const rmt_rx_done_event_data_t& ev, IRSignal& sig) {
  sig.count = 0;
  sig.valid = false;

  // TSOP active-LOW: level=0 → burst (carrier ON), level=1 → space (carrier OFF)
  // Simpan level di bit-15 tiap timing: 0=burst, 1=space
  // Durasi di bit[14:0] — max IR signal = ~12000µs << 32767, jadi aman
  for (size_t i = 0; i < ev.num_symbols && sig.count < MAX_TIMINGS - 2; i++) {
    const rmt_symbol_word_t& s = ev.received_symbols[i];
    if (s.duration0 > 0) {
      uint16_t t = (uint16_t)(s.duration0 & 0x7FFF);
      if (s.level0 == 1) t |= 0x8000;  // HIGH = space → set bit-15
      sig.timings[sig.count++] = t;
    }
    if (s.duration1 > 0) {
      uint16_t t = (uint16_t)(s.duration1 & 0x7FFF);
      if (s.level1 == 1) t |= 0x8000;  // HIGH = space → set bit-15
      sig.timings[sig.count++] = t;
    }
  }

  // Buang leading spaces (TSOP idle HIGH sebelum burst pertama)
  int skip = 0;
  while (skip < sig.count && (sig.timings[skip] & 0x8000)) skip++;
  if (skip > 0 && skip < sig.count) {
    sig.count -= skip;
    memmove(sig.timings, sig.timings + skip, sig.count * sizeof(uint16_t));
  }

  sig.valid = (sig.count >= 6);
  Serial.printf("[CAP] %d symbols → %d timings (skip=%d), valid=%s\n",
    (int)ev.num_symbols, sig.count, skip, sig.valid?"YES":"NO");
  if (sig.valid) {
    Serial.print("[CAP] First 8: ");
    for (int j = 0; j < 8 && j < sig.count; j++) {
      bool isBurst = !(sig.timings[j] & 0x8000);
      Serial.printf("%d%s ", sig.timings[j] & 0x7FFF, isBurst?"B":"S");
    }
    Serial.println();
  }
}

void signal_replay(int idx) {
  IRSignal& sig = g_signals[idx];
  if (!sig.valid || sig.count < 2) return;

  // Rekonstruksi simbol RMT dari timings tersimpan.
  // Bit-15 tiap timing = level TSOP saat capture:
  //   0 = burst (TSOP LOW) → TX: level=1 (carrier ON)
  //   1 = space  (TSOP HIGH) → TX: level=0 (carrier OFF)
  // Pasangkan dua timing berturut → 1 rmt_symbol_word_t
  static rmt_symbol_word_t syms[MAX_TIMINGS / 2 + 2];
  int n = 0;

  for (int i = 0; i < sig.count - 1 && n < (int)(MAX_TIMINGS / 2); i += 2) {
    uint16_t t0 = sig.timings[i];
    uint16_t t1 = sig.timings[i + 1];
    uint16_t dur0 = t0 & 0x7FFF;
    uint16_t dur1 = t1 & 0x7FFF;
    if (dur0 == 0) continue;
    syms[n].level0    = (t0 & 0x8000) ? 0 : 1;  // burst→1, space→0
    syms[n].duration0 = dur0 > 32767 ? 32767 : dur0;
    syms[n].level1    = (t1 & 0x8000) ? 0 : 1;
    syms[n].duration1 = dur1 > 32767 ? 32767 : dur1;
    n++;
  }
  if (n == 0) { Serial.println("[TX] ERROR: 0 symbols"); return; }
  syms[n].val = 0; // terminator

  Serial.printf("[TX] %s: %d timings → %d symbols\n", sig.name.c_str(), sig.count, n);
  for (int j = 0; j < 4 && j < n; j++) {
    Serial.printf("[TX]   [%d] %s=%dµs %s=%dµs\n", j,
      syms[j].level0?"ON ":"OFF", syms[j].duration0,
      syms[j].level1?"ON ":"OFF", syms[j].duration1);
  }

  rmt_transmit_config_t tcfg = {};
  tcfg.loop_count = 0;

  // Kirim berulang selama REPLAY_DURATION_MS (seperti tahan tombol remote)
  g_replaying = true;
  digitalWrite(PIN_LED, LOW);  // LED ON solid selama replay

  unsigned long replayEnd = millis() + REPLAY_DURATION_MS;
  int sent = 0;
  do {
    esp_err_t err = rmt_transmit(tx_chan, copy_enc,
                                 syms, n * sizeof(rmt_symbol_word_t), &tcfg);
    if (err != ESP_OK) {
      Serial.printf("[TX] ERROR: 0x%x\n", (unsigned)err);
      break;
    }
    rmt_tx_wait_all_done(tx_chan, 500);
    sent++;
    delay(40);  // gap antar frame ~40ms
  } while (millis() < replayEnd);

  g_replaying = false;
  g_led_phase = 0;
  g_led_timer = millis();
  digitalWrite(PIN_LED, HIGH);  // LED OFF, kembali ke heartbeat
  Serial.printf("[TX] Done. %s sent x%d\n", sig.name.c_str(), sent);
}

// ─────────────────────────────────────────────────────────────
//  PWA Manifest
// ─────────────────────────────────────────────────────────────
const char MANIFEST_JSON[] PROGMEM = R"=====(
{
  "name": "IR Backup System",
  "short_name": "IR Backup",
  "description": "Capture & replay IR remote signals via XIAO ESP32-C6",
  "start_url": "/",
  "display": "standalone",
  "orientation": "portrait",
  "background_color": "#faf8f4",
  "theme_color": "#cc7a4a",
  "icons": [
    {
      "src": "/icon.svg",
      "sizes": "any",
      "type": "image/svg+xml",
      "purpose": "any maskable"
    }
  ]
}
)=====";

// ─────────────────────────────────────────────────────────────
//  PWA Icon (SVG inline)
// ─────────────────────────────────────────────────────────────
const char ICON_SVG[] PROGMEM = R"=====(
<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 192 192">
  <rect width="192" height="192" rx="40" fill="#cc7a4a"/>
  <text x="96" y="130" font-size="110" text-anchor="middle" fill="white" font-family="Georgia,serif" font-style="italic">ir</text>
</svg>
)=====";

// ─────────────────────────────────────────────────────────────
//  Service Worker (PWA offline cache)
// ─────────────────────────────────────────────────────────────
const char SW_JS[] PROGMEM = R"=====(
const CACHE = 'ir-backup-v3';
const ASSETS = ['/', '/manifest.json', '/icon.svg'];

self.addEventListener('install', e => {
  e.waitUntil(
    caches.open(CACHE).then(c => c.addAll(ASSETS))
  );
  self.skipWaiting();
});

self.addEventListener('activate', e => {
  e.waitUntil(
    caches.keys().then(keys =>
      Promise.all(keys.filter(k => k !== CACHE).map(k => caches.delete(k)))
    )
  );
  self.clients.claim();
});

self.addEventListener('fetch', e => {
  const url = new URL(e.request.url);
  // API calls: network first (jangan cache)
  if (url.pathname.startsWith('/api/')) {
    e.respondWith(fetch(e.request).catch(() =>
      new Response('{"ok":false,"msg":"offline"}', {headers:{'Content-Type':'application/json'}})
    ));
    return;
  }
  // Static assets: cache first
  e.respondWith(
    caches.match(e.request).then(r => r || fetch(e.request).then(res => {
      const clone = res.clone();
      caches.open(CACHE).then(c => c.put(e.request, clone));
      return res;
    }))
  );
});
)=====";

// ─────────────────────────────────────────────────────────────
//  HTML — Single Page Web App (PWA)
// ─────────────────────────────────────────────────────────────
const char INDEX_HTML[] PROGMEM = R"=====(
<!DOCTYPE html>
<html lang="id">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<meta name="mobile-web-app-capable" content="yes">
<meta name="apple-mobile-web-app-capable" content="yes">
<meta name="apple-mobile-web-app-status-bar-style" content="default">
<meta name="apple-mobile-web-app-title" content="IR Backup">
<meta name="theme-color" content="#cc7a4a">
<link rel="manifest" href="/manifest.json">
<link rel="apple-touch-icon" href="/icon.svg">
<title>IR Backup</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{
  font-family:Georgia,serif;
  background-color:#faf8f4;
  background-image:linear-gradient(#e0dbd2 1px,transparent 1px),
                   linear-gradient(90deg,#e0dbd2 1px,transparent 1px);
  background-size:28px 28px;
  color:#2c2520;
  min-height:100vh;
}
/* ── Header ── */
.hdr{
  background:rgba(250,248,244,0.97);
  border-bottom:1.5px solid #c8c0b4;
  padding:13px 18px;
  display:flex;align-items:center;gap:10px;
  position:sticky;top:0;z-index:10;
}
.hdr-logo{
  width:34px;height:34px;border-radius:9px;
  background:#cc7a4a;
  display:flex;align-items:center;justify-content:center;
  font-size:16px;color:#fff;font-style:italic;font-weight:bold;
}
.hdr-title{font-size:15px;color:#2c2520;font-weight:bold;flex:1}
.hdr-sub{font-size:10px;color:#9c8f82;font-family:monospace;margin-top:2px}
.ap-badge{
  font-size:9px;color:#fff;background:#4a7a3a;
  padding:4px 10px;border-radius:10px;font-family:monospace;font-weight:600;
}
/* ── Layout ── */
.wrap{max-width:640px;margin:0 auto;padding:14px}
/* ── Card ── */
.card{
  background:rgba(255,253,250,0.92);
  border:1.5px solid #c8c0b4;
  border-radius:11px;
  padding:14px;
  margin-bottom:13px;
}
.ctitle{
  font-size:9px;color:#9c8f82;text-transform:uppercase;
  letter-spacing:1.3px;margin-bottom:11px;
  font-family:monospace;font-weight:600;
  display:flex;align-items:center;gap:8px;
}
/* ── Inputs ── */
input[type=text],input[type=search]{
  width:100%;
  background:#fff;
  border:1.5px solid #c8c0b4;
  color:#2c2520;
  padding:9px 11px;
  border-radius:7px;
  font:13px Georgia,serif;
  margin-bottom:9px;
  outline:none;
  transition:border-color .2s;
}
input:focus{border-color:#cc7a4a}
input::placeholder{color:#c0b4a8;font-size:12px}
/* ── Buttons — semua pakai #cc7a4a + teks putih ── */
.btn{
  display:inline-flex;align-items:center;gap:5px;
  padding:9px 16px;border-radius:7px;border:none;cursor:pointer;
  font:700 12px monospace;transition:filter .15s,background .1s;white-space:nowrap;
  -webkit-user-select:none;user-select:none;
  -webkit-tap-highlight-color:transparent;
  touch-action:manipulation;
}
.btn:hover{filter:brightness(1.1)}
.btn:active{transform:scale(.96);filter:brightness(0.8)!important}
.btn:disabled{opacity:.4;cursor:not-allowed;filter:none}
/* Utama (Capture, Play, Download, dll) */
.btn-clay {background:#cc7a4a;color:#fff}
.btn-clay:active{background:#a05830!important}
/* Bahaya (Hapus) */
.btn-danger{background:#c0503a;color:#fff}
.btn-danger:active{background:#8c3020!important}
/* Outline ringan */
.btn-outline{background:#fff;border:1.5px solid #cc7a4a;color:#cc7a4a}
.btn-outline:active{background:#f5ddd0!important}
/* Latch — tombol yang sedang dipilih/aktif */
.btn-selected{background:#7a3010!important;color:#fff!important;
  box-shadow:inset 0 2px 5px rgba(0,0,0,.35);outline:none!important}
.btn-clay.btn-selected:active{background:#5a2008!important}
/* ── Status ── */
.st{padding:9px 12px;border-radius:7px;font-size:12px;margin-top:9px;line-height:1.5;font-family:monospace;border:1.5px solid}
.idle {background:#f5f2ed;color:#8c7f72;border-color:#ddd7ce}
.blink{background:#fef4ec;color:#8b4a20;border-color:#e8b880;animation:blk 1s infinite}
.ok   {background:#eaf3e4;color:#2d6020;border-color:#a8d090}
.er   {background:#fdf0ee;color:#8b3020;border-color:#e0b0a8}
@keyframes blk{0%,100%{opacity:1}50%{opacity:.45}}
/* ── Signal list row ── */
.sig{
  display:flex;align-items:center;gap:7px;
  padding:9px 11px;
  background:#fff;border:1.5px solid #ddd7ce;
  border-radius:8px;margin-bottom:6px;
  transition:border-color .15s;
}
.sig:hover{border-color:#cc7a4a}
.sn{flex:1;font-size:13px;color:#2c2520;font-family:monospace;font-weight:600;overflow:hidden;text-overflow:ellipsis;white-space:nowrap}
.si{font-size:10px;color:#9c8f82;background:#f0ece6;padding:2px 7px;border-radius:5px;font-family:monospace;border:1px solid #ddd7ce;white-space:nowrap}
/* ── Misc ── */
.sbar{display:flex;gap:7px;align-items:center;flex-wrap:wrap;margin-bottom:10px}
hr{border:none;border-top:1.5px solid #ddd7ce;margin:11px 0}
.badge{background:#cc7a4a;color:#fff;padding:2px 9px;border-radius:10px;font-size:10px;font-family:monospace;font-weight:600}
.prog{height:4px;background:#ddd7ce;border-radius:2px;margin-bottom:10px}
.prog-fill{height:100%;background:#cc7a4a;border-radius:2px;transition:width .3s}
.info-row{display:flex;gap:10px;font-size:11px;color:#9c8f82;margin-bottom:10px;font-family:monospace}
.info-row span{color:#5c5048;font-weight:600}
.empty{color:#c0b4a8;font-size:12px;text-align:center;padding:22px 0;font-family:monospace}
</style>
</head>
<body>

<!-- PWA Install Banner -->
<div id="installBanner" style="display:none;background:#cc7a4a;color:#fff;padding:10px 16px;align-items:center;gap:10px;font-family:monospace;font-size:12px">
  <span style="flex:1">&#128241; Install IR Backup ke Home Screen untuk akses offline!</span>
  <button onclick="installPWA()" style="background:#fff;color:#cc7a4a;border:none;padding:6px 12px;border-radius:6px;font:700 11px monospace;cursor:pointer">Install</button>
  <button onclick="this.parentElement.style.display='none'" style="background:transparent;border:none;color:#fff;font-size:16px;cursor:pointer;padding:0 4px">&times;</button>
</div>

<div class="hdr">
  <div class="hdr-logo">ir</div>
  <div style="flex:1">
    <div class="hdr-title">IR Backup System</div>
    <div class="hdr-sub">XIAO ESP32-C6 &nbsp;·&nbsp; 192.168.4.1</div>
  </div>
  <div class="ap-badge">&#9679; WiFi AP</div>
</div>

<!-- Logic Analyzer Modal -->
<div id="waveModal" style="display:none;position:fixed;inset:0;background:rgba(0,0,0,.7);z-index:999;align-items:center;justify-content:center;padding:16px">
  <div style="background:#faf8f4;border-radius:14px;padding:18px;max-width:760px;width:100%;box-shadow:0 8px 32px rgba(0,0,0,.4)">
    <div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:10px">
      <div>
        <div style="font-weight:700;font-size:15px;font-family:monospace;color:#5a3820" id="waveName"></div>
        <div style="font-size:11px;color:#9c8f82;font-family:monospace;margin-top:2px" id="waveMeta"></div>
      </div>
      <button class="btn btn-clay" style="padding:6px 12px" onclick="document.getElementById('waveModal').style.display='none'">&#10005; Tutup</button>
    </div>
    <div id="waveSvg" style="overflow-x:auto;overflow-y:hidden"></div>
    <div style="margin-top:10px;font-size:10px;color:#9c8f82;font-family:monospace">
      &#9632; <span style="color:#cc7a4a">━━</span> Burst (carrier ON) &nbsp;|&nbsp; &#9135;&#9135; Space (silence) &nbsp;|&nbsp; 1 tick = 1 µs
    </div>
  </div>
</div>

<div class="wrap">

  <!-- Capture Card -->
  <div class="card">
    <div class="ctitle">&#11044; Capture Sinyal Baru</div>
    <input type="text" id="sname" placeholder="Nama tombol: AC_power, TV_vol+, fan_speed_3 ...">
    <div style="margin:8px 0 4px 0">
      <div style="font-size:11px;color:#9c8f82;font-family:monospace;margin-bottom:6px">&#9889; Frekuensi RX sensor:</div>
      <div style="display:flex;flex-wrap:wrap;gap:6px" id="capFreqBtns">
        <button class="btn btn-clay" id="cf33"  style="padding:4px 12px;font-size:12px" onclick="setFreq(33000)">33 kHz</button>
        <button class="btn btn-clay" id="cf36"  style="padding:4px 12px;font-size:12px" onclick="setFreq(36000)">36 kHz</button>
        <button class="btn btn-clay" id="cf37"  style="padding:4px 12px;font-size:12px" onclick="setFreq(37000)">37 kHz</button>
        <button class="btn btn-clay" id="cf38"  style="padding:4px 12px;font-size:12px;outline:2px solid #fff" onclick="setFreq(38000)">38 kHz &#9733;</button>
        <button class="btn btn-clay" id="cf40"  style="padding:4px 12px;font-size:12px" onclick="setFreq(40000)">40 kHz</button>
        <button class="btn btn-clay" id="cf56"  style="padding:4px 12px;font-size:12px" onclick="setFreq(56000)">56 kHz</button>
      </div>
    </div>
    <div class="sbar">
      <button class="btn btn-clay" id="btnCap" onclick="startCap()">&#9210; Mulai Capture</button>
      <small style="color:#9c8f82;font-size:11px;font-family:monospace">arahkan remote ke sensor, max 5 detik</small>
    </div>
    <div class="st idle" id="stCap">Siap &#8212; ketik nama tombol lalu klik Capture</div>
  </div>

  <!-- Signal List Card -->
  <div class="card">
    <div class="ctitle">
      &#9670; Sinyal Tersimpan
      <span class="badge" id="cnt">0 / 50</span>
    </div>

    <div class="prog"><div class="prog-fill" id="progFill" style="width:0%"></div></div>

    <div class="info-row">
      <div>Tersimpan: <span id="infoCnt">0</span></div>
      <div>Sisa slot: <span id="infoFree">50</span></div>
      <div style="flex:1"></div>
      <div id="infoFs"></div>
    </div>

    <input type="search" id="srch" placeholder="Cari nama sinyal..." oninput="filterList()">

    <div id="list"><div class="empty">Belum ada sinyal. Capture dulu!</div></div>

    <hr>
    <div class="sbar">
      <button class="btn btn-danger" onclick="clearAll()">&#128465; Hapus Semua</button>
    </div>
  </div>


  <!-- Frequency Test Card -->
  <div class="card">
    <div class="ctitle">&#9889; Pilih Frekuensi Carrier IR</div>
    <div style="font-size:11px;color:#9c8f82;font-family:monospace;margin-bottom:12px">
      Pilih frekuensi sesuai sensor kamu. Kebanyakan remote = 38 kHz.<br>
      Setelah pilih, klik Test IR LED untuk loopback, atau coba Capture.
    </div>
    <div style="display:flex;flex-wrap:wrap;gap:8px;margin-bottom:12px" id="freqBtns">
      <button class="btn btn-clay" id="f33"   onclick="setFreq(33000)">33 kHz</button>
      <button class="btn btn-clay" id="f36"   onclick="setFreq(36000)">36 kHz</button>
      <button class="btn btn-clay" id="f37"   onclick="setFreq(37000)">37 kHz</button>
      <button class="btn btn-clay" style="outline:2px solid #fff" id="f38" onclick="setFreq(38000)">38 kHz ★</button>
      <button class="btn btn-clay" id="f40"   onclick="setFreq(40000)">40 kHz</button>
      <button class="btn btn-clay" id="f56"   onclick="setFreq(56000)">56 kHz</button>
    </div>
    <div class="st idle" id="stFreq">Aktif: 38 kHz (default)</div>
    <hr>
    <div class="sbar">
      <button class="btn btn-outline" onclick="rawTest()">&#128104; Test Sensor (pakai remote)</button>
      <button class="btn btn-outline" onclick="ledBlink()">&#128294; Nyalakan LED 1 detik (kamera)</button>
      <button class="btn btn-outline" onclick="txTest()">&#128308; Test Loopback TX→RX</button>
    </div>
  </div>

  <!-- Backup & Restore Card -->
  <div class="card">
    <div class="ctitle">&#128190; Backup &amp; Restore ke HP</div>
    <div style="display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-bottom:10px">
      <div style="background:#fff;border:1.5px solid #ddd7ce;border-radius:9px;padding:12px;text-align:center">
        <div style="font-size:22px;margin-bottom:5px">&#11015;</div>
        <div style="font-size:11px;color:#5c5048;font-family:monospace;font-weight:600;margin-bottom:8px">Simpan ke HP</div>
        <div style="font-size:10px;color:#9c8f82;font-family:monospace;margin-bottom:10px">Download semua sinyal<br>ke folder Downloads HP</div>
        <button class="btn btn-clay" style="width:100%;justify-content:center" onclick="dlJSON()">&#11015; Download</button>
      </div>
      <div style="background:#fff;border:1.5px solid #ddd7ce;border-radius:9px;padding:12px;text-align:center">
        <div style="font-size:22px;margin-bottom:5px">&#11014;</div>
        <div style="font-size:11px;color:#5c5048;font-family:monospace;font-weight:600;margin-bottom:8px">Restore dari HP</div>
        <div style="font-size:10px;color:#9c8f82;font-family:monospace;margin-bottom:10px">Upload file backup<br>JSON ke ESP32</div>
        <button class="btn btn-clay" style="width:100%;justify-content:center" onclick="document.getElementById('fileImport').click()">&#11014; Upload</button>
        <input type="file" id="fileImport" accept=".json" style="display:none" onchange="importJSON(this)">
      </div>
    </div>
    <div class="st idle" id="stBackup" style="font-size:11px">Backup rutin agar data tidak hilang jika ESP32 di-flash ulang</div>
  </div>

</div>

<script>
let pollTimer = null;
let allSignals = [];

function startCap() {
  const n = document.getElementById('sname').value.trim();
  if (!n) { alert('Isi nama sinyal dulu!'); return; }
  if (allSignals.length >= 50) { alert('Storage penuh! Hapus beberapa sinyal dulu.'); return; }
  document.getElementById('btnCap').disabled = true;
  setSt('blink','⏺ Menunggu sinyal IR... tekan tombol remote sekarang!');
  fetch('/api/capture?name=' + encodeURIComponent(n))
    .then(r=>r.json())
    .then(d=>{
      if(!d.ok){setSt('er','❌ '+d.msg);document.getElementById('btnCap').disabled=false;return;}
      pollTimer = setInterval(checkDone, 600);
    })
    .catch(()=>{setSt('er','❌ Koneksi gagal');document.getElementById('btnCap').disabled=false;});
}

function checkDone(){
  fetch('/api/status').then(r=>r.json()).then(d=>{
    if(!d.capturing){
      clearInterval(pollTimer);
      document.getElementById('btnCap').disabled=false;
      setSt(d.ok?'ok':'er',(d.ok?'✅ ':'❌ ')+d.msg);
      if(d.ok){document.getElementById('sname').value='';loadList();


function setFreq(hz){
  const labels = {33000:'33 kHz',36000:'36 kHz',37000:'37 kHz',
                  38000:'38 kHz ★',40000:'40 kHz',56000:'56 kHz'};
  document.getElementById('stFreq').className='st blink';
  document.getElementById('stFreq').textContent='Mengubah ke '+labels[hz]+'...';
  // highlight tombol aktif
  [33000,36000,37000,38000,40000,56000].forEach(f=>{
    const b=document.getElementById('f'+f);
    if(b){ b.style.outline=''; b.classList.toggle('btn-selected', f===hz); }
    const c=document.getElementById('cf'+f);
    if(c){ c.style.outline=''; c.classList.toggle('btn-selected', f===hz); }
  });
  fetch('/api/set_freq?hz='+hz).then(r=>r.json()).then(d=>{
    const el=document.getElementById('stFreq');
    if(d.ok){
      el.className='st ok';
      el.textContent='✅ Aktif: '+labels[hz]+' — sekarang coba Test IR LED atau Capture';
    } else {
      el.className='st er';
      el.textContent='❌ Gagal: '+d.msg;
    }
  }).catch(()=>{
    document.getElementById('stFreq').className='st er';
    document.getElementById('stFreq').textContent='❌ Koneksi gagal';
  });
}
function ledBlink(){
  showTestResult('Menyalakan IR LED selama 1 detik... arahkan kamera HP ke LED sekarang!');
  fetch('/api/led_blink').then(r=>r.text()).then(t=>{showTestResult(t);});
}

function txTest(){
  showTestResult('Mengirim burst IR... harap tunggu...');
  fetch('/api/test_tx').then(r=>r.text()).then(t=>{showTestResult(t);});
}

function showTestResult(txt){
  const old=document.getElementById('rawResult');
  if(old) old.remove();
  const pre=document.createElement('pre');
  pre.id='rawResult';
  pre.style='font-size:11px;padding:10px;background:#1a1a1a;color:#0f0;border-radius:8px;max-height:250px;overflow:auto;margin-top:10px;white-space:pre-wrap';
  pre.textContent=txt;
  // Sisipkan di card terakhir (Backup card)
  document.querySelectorAll('.card')[2].appendChild(pre);
}

function rawTest(){
  showTestResult('Menunggu sinyal remote... tekan tombol remote ke arah sensor sekarang!');
  fetch('/api/raw_test').then(r=>r.text()).then(t=>{showTestResult(t);});
}}
    }
  }).catch(()=>clearInterval(pollTimer));
}

function setSt(cls,msg){
  const el=document.getElementById('stCap');
  el.className='st '+cls; el.textContent=msg;
}

function loadList(){
  fetch('/api/signals').then(r=>r.json()).then(data=>{
    allSignals=data;
    updateStats(data);
    renderList(data);
  });
}

function updateStats(data){
  const n=data.length, max=50, pct=Math.round(n/max*100);
  document.getElementById('cnt').textContent=n+' / 50';
  document.getElementById('infoCnt').textContent=n;
  document.getElementById('infoFree').textContent=max-n;
  const fill=document.getElementById('progFill');
  fill.style.width=pct+'%';
  fill.style.background=pct>90?'#c0503a':pct>70?'#c08030':'#cc7a4a';
}

function filterList(){
  const q=document.getElementById('srch').value.toLowerCase();
  renderList(q?allSignals.filter(s=>s.name.toLowerCase().includes(q)):allSignals);
}

function renderList(data){
  const el=document.getElementById('list');
  if(!data.length){
    el.innerHTML='<div class="empty">'+(allSignals.length?'Tidak ada hasil pencarian.':'Belum ada sinyal. Capture dulu!')+'</div>';
    return;
  }
  el.innerHTML=data.map((s)=>{
    const ri=allSignals.indexOf(s);
    return `<div class="sig">
      <span class="sn" title="${s.name}">${s.name}</span>
      <span class="si">${s.count}p</span>
      <button class="btn btn-outline" style="padding:7px 10px;font-size:11px" onclick="showWave(${ri})" title="Logic Analyzer">&#128202;</button>
      <button class="btn btn-clay"    style="padding:7px 13px;font-size:11px" onclick="replay(${ri},this)">&#9658; Kirim</button>
      <button class="btn btn-danger"  style="padding:7px 10px;font-size:11px" onclick="del(${ri})">&#10005;</button>
    </div>`;
  }).join('');
}

// ─── Logic Analyzer Waveform ───────────────────────────────────────
function showWave(idx){
  fetch('/api/signal?id='+idx).then(r=>r.json()).then(d=>{
    if(!d.timings||!d.timings.length){alert('Data kosong');return;}
    document.getElementById('waveName').textContent=d.name;
    const totalUs=d.timings.reduce((a,t)=>a+t.d,0);
    const bursts=d.timings.filter(t=>t.b).length;
    document.getElementById('waveMeta').textContent=
      d.count+' timings · '+bursts+' burst · '+(totalUs/1000).toFixed(1)+' ms total';
    document.getElementById('waveSvg').innerHTML=buildWaveSvg(d.timings);
    document.getElementById('waveModal').style.display='flex';
  }).catch(()=>alert('Gagal ambil data'));
}

function buildWaveSvg(timings){
  const W=Math.min(window.innerWidth-48, 700);
  const totalUs=timings.reduce((a,t)=>a+t.d,0);
  const scale=totalUs>0?(W-4)/totalUs:1;
  const H=60, HIGH=8, LOW=40, PAD=2;
  // Mulai dari LOW (idle / space)
  let path='M'+PAD+','+(LOW);
  let x=PAD;
  for(const t of timings){
    const w=Math.max(2, t.d*scale);
    if(t.b){
      // burst: naik ke HIGH, lebar w, turun ke LOW
      path+=` V${HIGH} H${(x+w).toFixed(1)} V${LOW}`;
    } else {
      // space: tetap di LOW
      path+=` H${(x+w).toFixed(1)}`;
    }
    x+=w;
  }
  // Time axis labels — tiap 5ms atau sesuai total
  let labels='';
  const stepMs=totalUs<=10000?1:totalUs<=50000?5:10;
  const stepUs=stepMs*1000;
  let acc=0, lx=PAD;
  for(const t of timings){
    const w=Math.max(2,t.d*scale);
    const prev=acc;
    acc+=t.d;
    // cetak label jika melewati step boundary
    const crossed=Math.floor(acc/stepUs)-Math.floor(prev/stepUs);
    if(crossed>0){
      const lpos=lx+(acc%stepUs==0?w:w*(acc%stepUs)/t.d);
      const lms=Math.round(acc/stepUs)*stepMs;
      labels+=`<text x="${lpos.toFixed(0)}" y="${H-2}" fill="#9c8f82" font-size="9" text-anchor="middle">${lms}ms</text>`;
    }
    lx+=w;
  }
  return `<svg width="${W}" height="${H}" style="display:block;border-radius:6px;background:#1a1614">
    <line x1="${PAD}" y1="${LOW}" x2="${x.toFixed(1)}" y2="${LOW}" stroke="#333" stroke-width="0.5"/>
    <line x1="${PAD}" y1="${HIGH}" x2="${x.toFixed(1)}" y2="${HIGH}" stroke="#333" stroke-width="0.5" stroke-dasharray="3,4"/>
    <path d="${path}" fill="none" stroke="#cc7a4a" stroke-width="2" stroke-linejoin="miter"/>
    <text x="${PAD}" y="${H-2}" fill="#9c8f82" font-size="9">0</text>
    ${labels}
    <text x="${(x-2).toFixed(0)}" y="${H-2}" fill="#9c8f82" font-size="9" text-anchor="end">${(totalUs/1000).toFixed(1)}ms</text>
  </svg>`;
}

function replay(i,btn){
  if(i<0)return;
  const orig=btn.innerHTML;
  btn.disabled=true; btn.textContent='...';
  fetch('/api/replay?id='+i).then(r=>r.json()).then(d=>{
    if(d.ok){
      btn.textContent='✓ Terkirim';btn.style.background='#4a7a3a';
      setSt('ok','✅ Sinyal "'+allSignals[i].name+'" berhasil dikirim!');
      setTimeout(()=>{btn.innerHTML=orig;btn.style.background='';btn.disabled=false;},1400);
    } else {
      setSt('er','❌ Gagal replay');btn.innerHTML=orig;btn.disabled=false;
    }
  }).catch(()=>{btn.innerHTML=orig;btn.disabled=false;});
}

function del(i){
  if(!confirm('Hapus "'+allSignals[i].name+'"?'))return;
  fetch('/api/delete?id='+i).then(()=>loadList());
}

function clearAll(){
  if(!allSignals.length){alert('Tidak ada sinyal.');return;}
  if(!confirm('Hapus SEMUA '+allSignals.length+' sinyal? Tidak bisa di-undo!'))return;
  fetch('/api/clear').then(()=>{setSt('ok','✅ Semua sinyal dihapus');loadList();});
}

function dlJSON(){
  if(!allSignals.length){alert('Tidak ada sinyal untuk didownload.');return;}
  // Buat link download langsung agar works di mobile
  const a=document.createElement('a');
  a.href='/api/download';
  a.download='ir_backup.json';
  document.body.appendChild(a);
  a.click();
  document.body.removeChild(a);
  setBk('ok','✅ File ir_backup.json disimpan ke Downloads HP');
}

function setBk(cls,msg){
  const el=document.getElementById('stBackup');
  el.className='st '+cls; el.textContent=msg;
}

function importJSON(input){
  const file=input.files[0];
  if(!file){return;}
  setBk('blink','⏫ Mengupload '+file.name+'...');
  const reader=new FileReader();
  reader.onload=function(e){
    let data;
    try{data=JSON.parse(e.target.result);}
    catch(err){setBk('er','❌ File JSON tidak valid');input.value='';return;}
    if(!Array.isArray(data)){setBk('er','❌ Format salah — harus array []');input.value='';return;}
    fetch('/api/import',{
      method:'POST',
      headers:{'Content-Type':'application/json'},
      body:JSON.stringify(data)
    }).then(r=>r.json()).then(d=>{
      setBk(d.ok?'ok':'er',(d.ok?'✅ ':'❌ ')+d.msg);
      if(d.ok) loadList();
    }).catch(()=>setBk('er','❌ Gagal upload'));
    input.value='';
  };
  reader.readAsText(file);
}

loadList();

// ── PWA Service Worker Registration ──
if ('serviceWorker' in navigator) {
  window.addEventListener('load', () => {
    navigator.serviceWorker.register('/sw.js')
      .then(r => console.log('[SW] Registered:', r.scope))
      .catch(e => console.log('[SW] Error:', e));
  });
}

// ── PWA Install Prompt ──
let deferredPrompt = null;
const installBanner = document.getElementById('installBanner');
window.addEventListener('beforeinstallprompt', e => {
  e.preventDefault();
  deferredPrompt = e;
  if (installBanner) installBanner.style.display = 'flex';
});
window.addEventListener('appinstalled', () => {
  if (installBanner) installBanner.style.display = 'none';
});
function installPWA() {
  if (!deferredPrompt) return;
  deferredPrompt.prompt();
  deferredPrompt.userChoice.then(() => { deferredPrompt = null; });
}
</script>
</body>
</html>
)=====";

// ─────────────────────────────────────────────────────────────
//  Web Handlers

// ─────────────────────────────────────────────────────────────
//  LED Blink Test — raw GPIO bit-bang, bypass RMT sepenuhnya
// ─────────────────────────────────────────────────────────────
void rmt_tx_reinit() {
  // Re-inisialisasi RMT TX setelah raw GPIO test
  rmt_tx_channel_config_t tx_cfg = {};
  tx_cfg.gpio_num          = PIN_IR_TX;
  tx_cfg.clk_src           = RMT_CLK_SRC_DEFAULT;
  tx_cfg.resolution_hz     = RMT_RESOLUTION_HZ;
  tx_cfg.mem_block_symbols = 48;
  tx_cfg.trans_queue_depth = 4;
  if (rmt_new_tx_channel(&tx_cfg, &tx_chan) == ESP_OK) {
    rmt_carrier_config_t carrier = {};
    carrier.frequency_hz = g_carrier_hz;
    carrier.duty_cycle   = 0.33f;
    rmt_apply_carrier(tx_chan, &carrier);
    rmt_enable(tx_chan);
    Serial.println("[TX-REINIT] RMT TX OK");
  } else {
    Serial.println("[TX-REINIT] RMT TX gagal re-init!");
  }
}

void web_test_led_blink() {
  Serial.println("\n[LED-TEST] RMT carrier burst test pada PIN_IR_TX...");

  Serial.printf("[LED-TEST] tx_chan=%p  copy_enc=%p  carrier=%uHz\n",
               (void*)tx_chan, (void*)copy_enc, (unsigned)g_carrier_hz);
  if (!tx_chan || !copy_enc) {
    server.send(500, "text/plain",
      "RMT TX belum siap!\ntx_chan=" + String((uint32_t)tx_chan, HEX) +
      " copy_enc=" + String((uint32_t)copy_enc, HEX) +
      "\nCoba reset ESP32.");
    return;
  }

  // Kirim burst carrier ~3 detik langsung via RMT (TIDAK perlu hapus channel)
  // 47 symbol × (32767µs + 32767µs) ≈ 3.08 detik IR aktif
  // Dengan carrier 38kHz → LED berkedip 38000x/detik → terlihat jelas di kamera
  rmt_symbol_word_t burst[48];
  for (int i = 0; i < 47; i++) {
    burst[i].level0    = 1;      // carrier ON
    burst[i].duration0 = 32767;  // 32.7ms
    burst[i].level1    = 1;      // carrier ON
    burst[i].duration1 = 32767;  // 32.7ms
  }
  burst[47].val = 0;  // terminator

  rmt_transmit_config_t tcfg = {};
  tcfg.loop_count = 0;
  esp_err_t err = rmt_transmit(tx_chan, copy_enc,
                               burst, 47 * sizeof(rmt_symbol_word_t), &tcfg);

  Serial.printf("[LED-TEST] rmt_transmit → %s (0x%x)\n",
                err == ESP_OK ? "OK" : "GAGAL", (unsigned)err);

  if (err == ESP_OK) {
    server.send(200, "text/plain",
      "IR LED burst dikirim ~3 detik (GPIO" + String((int)PIN_IR_TX) + ", " +
      String(g_carrier_hz/1000) + " kHz)!\n\n"
      "Selama 3 detik arahkan kamera HP ke LED:\n"
      "  TERLIHAT cahaya ungu/putih → LED & RMT OK\n"
      "  TIDAK terlihat → cek wiring:\n"
      "    Anoda (+/kaki panjang) → resistor 100Ω → D0\n"
      "    Katoda (-/kaki pendek) → GND\n"
      "    Coba swap polaritas"
    );
  } else {
    server.send(500, "text/plain",
      "rmt_transmit gagal: 0x" + String((unsigned)err, HEX) + "\n"
      "Coba reset ESP32 dan ulangi."
    );
  }
}


// ─────────────────────────────────────────────────────────────
//  Set Carrier Frequency (36/37/38/40/56 kHz)
// ─────────────────────────────────────────────────────────────
void web_set_freq() {
  if (!server.hasArg("hz")) {
    server.send(400, "application/json", "{\"ok\":false,\"msg\":\"parameter hz tidak ada\"}");
    return;
  }
  uint32_t hz = (uint32_t)server.arg("hz").toInt();
  // Validasi — hanya frekuensi yang dikenal
  if (hz != 33000 && hz != 36000 && hz != 37000 &&
      hz != 38000 && hz != 40000 && hz != 56000) {
    server.send(400, "application/json", "{\"ok\":false,\"msg\":\"frekuensi tidak valid\"}");
    return;
  }
  g_carrier_hz = hz;

  // Terapkan ke TX channel: disable → ubah carrier → enable
  rmt_disable(tx_chan);
  rmt_carrier_config_t carrier = {};
  carrier.frequency_hz = g_carrier_hz;
  carrier.duty_cycle   = 0.33f;
  esp_err_t err = rmt_apply_carrier(tx_chan, &carrier);
  rmt_enable(tx_chan);

  if (err == ESP_OK) {
    Serial.printf("[FREQ] Carrier diubah ke %u Hz\n", (unsigned)g_carrier_hz);
    String msg = "{\"ok\":true,\"hz\":" + String(g_carrier_hz) + "}";
    server.send(200, "application/json", msg);
  } else {
    server.send(500, "application/json", "{\"ok\":false,\"msg\":\"gagal apply carrier\"}");
  }
}


// ─────────────────────────────────────────────────────────────
//  TX Loopback Test — kirim burst IR, cek apakah RX menerimanya
// ─────────────────────────────────────────────────────────────
void web_test_tx() {
  Serial.println("\n[TX-TEST] ===== TX LOOPBACK TEST =====");
  Serial.printf("[TX-TEST] TX=GPIO%d  RX=GPIO%d\n", (int)PIN_IR_TX, (int)PIN_IR_RX);

  String result = "=== TX LOOPBACK TEST ===\n";
  result += "TX GPIO" + String((int)PIN_IR_TX) + " → RX GPIO" + String((int)PIN_IR_RX) + "\n\n";

  // Kirim 5 burst: 600µs ON + 400µs OFF via RMT
  static rmt_symbol_word_t test_syms[6];
  for (int i = 0; i < 5; i++) {
    test_syms[i].level0    = 1;
    test_syms[i].duration0 = 600;   // 600µs mark (dengan carrier 38kHz)
    test_syms[i].level1    = 0;
    test_syms[i].duration1 = 400;   // 400µs space
  }
  test_syms[5].val = 0;   // terminator

  rmt_transmit_config_t tcfg = {};
  tcfg.loop_count = 0;

  // Mulai transmit, langsung cek RX dengan pulseIn
  rmt_transmit(tx_chan, copy_enc, test_syms, 6 * sizeof(rmt_symbol_word_t), &tcfg);

  // TSOP butuh beberapa ms untuk respond — tunggu sedikit lalu cek
  delay(2);
  unsigned long lo1 = pulseIn(PIN_IR_RX, LOW, 15000UL);
  unsigned long lo2 = pulseIn(PIN_IR_RX, LOW, 10000UL);
  rmt_tx_wait_all_done(tx_chan, 300);

  bool detected = (lo1 > 0 || lo2 > 0);

  Serial.printf("[TX-TEST] pulseIn: %luus / %luus — %s\n",
                lo1, lo2, detected ? "DETECTED" : "NOT DETECTED");

  if (detected) {
    result += "[OK] IR LED berfungsi!\n";
    result += "[OK] Sensor berfungsi!\n";
    result += "Loopback sukses: " + String(lo1) + "µs + " + String(lo2) + "µs terdeteksi\n";
    result += "\nKedua komponen siap dipakai.";
    Serial.println("[TX-TEST] PASS");
  } else {
    result += "[GAGAL] Tidak ada sinyal loopback.\n\n";
    result += "Kemungkinan penyebab:\n";
    result += "1. IR LED tidak terhubung ke D0 (GPIO2)\n";
    result += "2. IR LED rusak atau resistor salah\n";
    result += "3. Sensor terlalu jauh dari LED\n";
    result += "4. Sensor tidak terhubung ke D2 (GPIO4)\n";
    result += "5. Sensor belum mendapat 3V3\n";
    Serial.println("[TX-TEST] FAIL — no loopback detected");
  }
  Serial.println("[TX-TEST] ============================\n");
  server.send(200, "text/plain", result);
}


// ─────────────────────────────────────────────────────────────
//  Raw GPIO Test — bypass RMT, pakai pulseIn() untuk debug
// ─────────────────────────────────────────────────────────────
void web_raw_test() {
  Serial.println("\n[RAW] ===== RAW GPIO TEST =====");
  Serial.printf("[RAW] Monitoring GPIO%d selama 5 detik...\n", (int)PIN_IR_RX);
  Serial.println("[RAW] Tekan tombol remote sekarang!");

  pinMode(PIN_IR_RX, INPUT);
  int current = digitalRead(PIN_IR_RX);
  Serial.printf("[RAW] GPIO level sekarang: %s (idle harusnya HIGH)\n",
                current ? "HIGH" : "LOW");

  String result = "=== RAW IR TEST GPIO" + String((int)PIN_IR_RX) + " ===\n";
  result += "Level idle: " + String(current ? "HIGH (OK)" : "LOW (cek wiring!)") + "\n\n";

  unsigned long t0 = millis();
  int count = 0;

  while (count < 30 && (millis() - t0) < 5000) {
    // Tunggu sinyal aktif (LOW) max 200ms
    unsigned long lo = pulseIn(PIN_IR_RX, LOW, 200000UL);
    if (lo == 0) continue;
    unsigned long hi = pulseIn(PIN_IR_RX, HIGH, 200000UL);
    Serial.printf("[RAW] #%02d MARK=%5luus SPACE=%5luus\n", count, lo, hi);
    result += "#" + String(count) + " MARK=" + String(lo) + "us SPACE=" + String(hi) + "us\n";
    count++;
  }

  if (count == 0) {
    Serial.println("[RAW] TIDAK ADA SINYAL TERDETEKSI!");
    Serial.println("[RAW] Kemungkinan: GPIO salah / sensor rusak / remote terlalu jauh");
    result += "\n[GAGAL] Tidak ada sinyal!\n";
    result += "Cek:\n";
    result += "1. TSOP OUT -> D1 (GPIO3)\n";
    result += "2. TSOP VCC -> 3V3\n";
    result += "3. TSOP GND -> GND\n";
    result += "4. Remote diarahkan langsung ke sensor\n";
  } else {
    Serial.printf("[RAW] OK! %d pulse diterima\n", count);
    result += "\n[OK] " + String(count) + " pulse diterima!\n";
    result += "Sensor berfungsi. Jika Capture masih gagal, cek frekuensi sensor.\n";
  }
  Serial.println("[RAW] ===========================\n");
  server.send(200, "text/plain", result);
}

// ─────────────────────────────────────────────────────────────
void web_root() {
  server.send_P(200, "text/html", INDEX_HTML);
}

void web_manifest() {
  server.send_P(200, "application/manifest+json", MANIFEST_JSON);
}

void web_icon() {
  server.send_P(200, "image/svg+xml", ICON_SVG);
}

void web_sw() {
  server.sendHeader("Cache-Control", "no-cache");
  server.send_P(200, "application/javascript", SW_JS);
}

// ── Captive Portal Handlers ──────────────────────────────────
// Android checks: /generate_204 → harus PERSIS 204 No Content (no body!)
void web_204() {
  server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  server.sendHeader("Content-Length", "0");
  server.send(204);
}

// iOS/macOS checks: /hotspot-detect.html → butuh "<Success>" di body
void web_ios_hotspot() {
  server.sendHeader("Cache-Control", "no-cache");
  server.send(200, "text/html",
    "<HTML><HEAD><TITLE>Success</TITLE></HEAD><BODY>Success</BODY></HTML>");
}

// Windows checks: /connecttest.txt & /ncsi.txt
void web_msft() {
  server.send(200, "text/plain", "Microsoft Connect Test");
}

// Semua lain → redirect ke home page
void web_captive() {
  server.sendHeader("Location", String("http://") + WiFi.softAPIP().toString() + "/");
  server.send(302, "text/plain", "");
}

void web_import() {
  if (!server.hasArg("plain")) {
    server.send(400, "application/json", "{\"ok\":false,\"msg\":\"body kosong\"}");
    return;
  }
  String body = server.arg("plain");

  // Parse JSON array sederhana: cari "name" dan "timings"
  int imported = 0;
  int pos = 0;
  while (g_signal_count < MAX_SIGNALS) {
    // Cari "name":"..."
    int ni = body.indexOf("\"name\":", pos);
    if (ni < 0) break;
    int ns = body.indexOf('"', ni + 7) + 1;
    int ne = body.indexOf('"', ns);
    if (ns <= 0 || ne <= 0) break;
    String name = body.substring(ns, ne);

    // Cari "timings":[...]
    int ti = body.indexOf("\"timings\":", ne);
    if (ti < 0) break;
    int ts = body.indexOf('[', ti) + 1;
    int te = body.indexOf(']', ts);
    if (ts <= 0 || te <= 0) break;
    String timStr = body.substring(ts, te);

    // Parse timings
    IRSignal sig;
    sig.name  = name;
    sig.count = 0;
    sig.valid = false;
    int tp = 0;
    while (tp < (int)timStr.length() && sig.count < MAX_TIMINGS) {
      while (tp < (int)timStr.length() && (timStr[tp] == ' ' || timStr[tp] == ',')) tp++;
      if (tp >= (int)timStr.length()) break;
      int tend = tp;
      while (tend < (int)timStr.length() && timStr[tend] != ',') tend++;
      uint16_t val = (uint16_t)timStr.substring(tp, tend).toInt();
      if (val > 0) sig.timings[sig.count++] = val;
      tp = tend + 1;
    }
    sig.valid = (sig.count >= 6);

    if (sig.valid) {
      g_signals[g_signal_count] = sig;
      fs_save(g_signal_count);
      g_signal_count++;
      imported++;
    }
    pos = te + 1;
  }

  if (imported > 0) {
    String msg = "{\"ok\":true,\"msg\":\"" + String(imported) + " sinyal berhasil di-restore\"}";
    server.send(200, "application/json", msg);
  } else {
    server.send(200, "application/json", "{\"ok\":false,\"msg\":\"Tidak ada sinyal valid di file\"}");
  }
}

void web_capture() {
  if (!server.hasArg("name") || server.arg("name").isEmpty()) {
    server.send(400, "application/json", "{\"ok\":false,\"msg\":\"nama kosong\"}");
    return;
  }
  if (g_capturing) {
    server.send(400, "application/json", "{\"ok\":false,\"msg\":\"sedang capture\"}");
    return;
  }
  if (g_signal_count >= MAX_SIGNALS) {
    server.send(400, "application/json", "{\"ok\":false,\"msg\":\"storage penuh (50/50)\"}");
    return;
  }
  capture_start(server.arg("name"));
  server.send(200, "application/json", "{\"ok\":true}");
}

void web_status() {
  String msg = g_last_msg;
  msg.replace("\"", "'");
  String j = "{\"capturing\":" + String(g_capturing ? "true" : "false")
           + ",\"ok\":"        + String(g_last_ok   ? "true" : "false")
           + ",\"msg\":\""     + msg + "\"}";
  server.send(200, "application/json", j);
}

void web_signals() {
  String j = "[";
  for (int i = 0; i < g_signal_count; i++) {
    if (i) j += ",";
    j += "{\"name\":\"" + g_signals[i].name + "\","
       + "\"count\":"   + g_signals[i].count + "}";
  }
  j += "]";
  server.send(200, "application/json", j);
}

// Endpoint: GET /api/signal?id=N — kirim timings lengkap untuk logic analyzer
void web_signal_data() {
  if (!server.hasArg("id")) { server.send(400); return; }
  int id = server.arg("id").toInt();
  if (id < 0 || id >= g_signal_count || !g_signals[id].valid) {
    server.send(404, "application/json", "{\"ok\":false}");
    return;
  }
  IRSignal& s = g_signals[id];
  // Retun {d:duration, b:1=burst 0=space} per timing, level di bit-15
  String j = "{\"name\":\"" + s.name + "\",\"count\":" + s.count + ",\"timings\":[";
  for (int i = 0; i < s.count; i++) {
    if (i) j += ",";
    uint16_t raw  = s.timings[i];
    int      isBurst = (raw & 0x8000) ? 0 : 1;
    j += "{\"d\":" + String(raw & 0x7FFF) + ",\"b\":" + isBurst + "}";
  }
  j += "]}";
  server.send(200, "application/json", j);
}

void web_replay() {
  if (!server.hasArg("id")) { server.send(400); return; }
  int id = server.arg("id").toInt();
  if (id < 0 || id >= g_signal_count || !g_signals[id].valid) {
    server.send(404, "application/json", "{\"ok\":false,\"msg\":\"tidak ditemukan\"}");
    return;
  }
  server.send(200, "application/json", "{\"ok\":true}");
  delay(10);  // beri waktu response terkirim sebelum block 3 detik
  signal_replay(id);
}


void web_delete() {
  if (!server.hasArg("id")) { server.send(400); return; }
  int id = server.arg("id").toInt();
  if (id < 0 || id >= g_signal_count) { server.send(404); return; }
  for (int i = id; i < g_signal_count - 1; i++) g_signals[i] = g_signals[i + 1];
  g_signal_count--;
  fs_delete(id);
  server.send(200, "application/json", "{\"ok\":true}");
}

void web_clear() {
  fs_clear_all();
  for (int i = 0; i < MAX_SIGNALS; i++) g_signals[i] = IRSignal();
  g_signal_count = 0;
  server.send(200, "application/json", "{\"ok\":true}");
}

void web_download() {
  String j = "[\n";
  for (int i = 0; i < g_signal_count; i++) {
    if (i) j += ",\n";
    j += "  {\"name\":\"" + g_signals[i].name + "\",\"timings\":[";
    for (int k = 0; k < g_signals[i].count; k++) {
      if (k) j += ",";
      j += String(g_signals[i].timings[k]);
    }
    j += "]}";
  }
  j += "\n]";
  server.sendHeader("Content-Disposition", "attachment; filename=\"ir_backup.json\"");
  server.send(200, "application/json", j);
}

// ─────────────────────────────────────────────────────────────
//  Setup
// ─────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=================================");
  Serial.println(" IR Backup v3 -- XIAO ESP32-C6  ");
  Serial.println("=================================");

  // ── USER LED (GPIO15) ────────────────────────────────────────
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);   // LED ON saat boot
  g_led_timer = millis();

  // ── TES LED AWAL (sebelum RMT & WiFi) ──────────────────────
  // Jika LED menyala di sini → GPIO0(D0) & wiring OK
  // Jika tidak → masalah hardware/wiring, bukan firmware
  Serial.println("[BOOT-TEST] Tes GPIO0(D0) → HIGH 2 detik...");
  pinMode(0, OUTPUT);
  digitalWrite(0, HIGH);
  delay(2000);
  digitalWrite(0, LOW);
  Serial.println("[BOOT-TEST] Selesai. Jika LED menyala 2 detik → GPIO OK");
  // ────────────────────────────────────────────────────────────

  // LittleFS
  if (!LittleFS.begin(true)) {
    LittleFS.format();
    LittleFS.begin(true);
  }
  Serial.println("[FS] OK");
  fs_load_all();

  // XIAO ESP32-C6 antenna control — WAJIB sebelum WiFi!
  // GPIO3  : LOW = enable RF amplifier (jangan dipakai GPIO lain!)
  // GPIO14 : LOW = antena internal, HIGH = antena eksternal
  pinMode(3, OUTPUT);
  digitalWrite(3, LOW);   // aktifkan RF amplifier
  pinMode(14, OUTPUT);
  digitalWrite(14, LOW);  // pilih antena internal
  delay(50);
  Serial.println("[ANT] GPIO3=LOW (RF on), GPIO14=LOW (internal antenna)");

  // WiFi AP — ESP32-C6 fix: force 802.11 b/g/n (bukan WiFi 6)
  // Urutan WAJIB: mode() → set_protocol() → softAPConfig() → softAP()
  WiFi.mode(WIFI_AP);
  delay(100);

  // Paksa 802.11 b/g/n — harus SEBELUM softAP() dipanggil
  esp_wifi_set_protocol(WIFI_IF_AP,
    WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);

  WiFi.softAPConfig(
    IPAddress(192,168,4,1),
    IPAddress(192,168,4,1),
    IPAddress(255,255,255,0)
  );
  WiFi.softAP(WIFI_SSID, WIFI_PASS, 1, 0, 4);
  delay(500);

  Serial.println("[WiFi] SSID : " + String(WIFI_SSID));
  Serial.println("[WiFi] Pass : " + String(WIFI_PASS));
  Serial.println("[WiFi] IP   : " + WiFi.softAPIP().toString());
  Serial.println("[WiFi] Proto: 802.11 b/g/n (forced)");

  // DNS captive portal
  dnsServer.start(DNS_PORT, "*", IPAddress(192,168,4,1));
  Serial.println("[DNS] OK");

  // RMT
  rmt_setup();

  // Web routes
  server.on("/",              HTTP_GET,  web_root);
  server.on("/index.html",    HTTP_GET,  web_root);
  server.on("/manifest.json", HTTP_GET,  web_manifest);
  server.on("/icon.svg",      HTTP_GET,  web_icon);
  server.on("/sw.js",         HTTP_GET,  web_sw);
  server.on("/api/capture",   HTTP_GET,  web_capture);
  server.on("/api/status",    HTTP_GET,  web_status);
  server.on("/api/signals",   HTTP_GET,  web_signals);
  server.on("/api/signal",     HTTP_GET,  web_signal_data);
  server.on("/api/replay",    HTTP_GET,  web_replay);
  server.on("/api/delete",    HTTP_GET,  web_delete);
  server.on("/api/clear",     HTTP_GET,  web_clear);
  server.on("/api/download",  HTTP_GET,  web_download);
  server.on("/api/import",    HTTP_POST, web_import);
  server.on("/api/raw_test",   HTTP_GET,  web_raw_test);
  server.on("/api/test_tx",     HTTP_GET,  web_test_tx);
  server.on("/api/set_freq",    HTTP_GET,  web_set_freq);
  server.on("/api/led_blink",   HTTP_GET,  web_test_led_blink);

  // Connectivity checks — supaya HP tidak anggap "no internet"
  server.on("/generate_204",              HTTP_GET, web_204);
  server.on("/gen_204",                   HTTP_GET, web_204);
  server.on("/hotspot-detect.html",       HTTP_GET, web_ios_hotspot);
  server.on("/library/test/success.html", HTTP_GET, web_ios_hotspot);
  server.on("/success.html",              HTTP_GET, web_ios_hotspot);
  server.on("/connecttest.txt",           HTTP_GET, web_msft);
  server.on("/ncsi.txt",                  HTTP_GET, web_msft);
  server.on("/redirect",                  HTTP_GET, web_204);
  server.onNotFound(web_captive);
  server.begin();

  Serial.println("[Web] http://" + WiFi.softAPIP().toString());
  Serial.println("=================================\n");
}

// ─────────────────────────────────────────────────────────────
//  LED State Machine (non-blocking, dipanggil tiap loop)
//  GPIO15 active-LOW: LOW=ON, HIGH=OFF
//  IDLE      : slow heartbeat — 50ms ON per 2 detik
//  CAPTURING : fast blink — 150ms ON / 150ms OFF
//  REPLAYING : solid ON (dikontrol signal_replay)
// ─────────────────────────────────────────────────────────────
void led_update() {
  if (g_replaying) return;  // signal_replay mengontrol LED

  unsigned long now = millis();
  if (g_capturing) {
    if (now - g_led_timer >= 150) {
      g_led_timer = now;
      g_led_phase ^= 1;
      digitalWrite(PIN_LED, g_led_phase ? LOW : HIGH);
    }
  } else {
    // Heartbeat: 50ms ON, 1950ms OFF
    unsigned long period = now - g_led_timer;
    if (g_led_phase == 0 && period >= 950) {
      g_led_phase = 1; g_led_timer = now;
      digitalWrite(PIN_LED, LOW);   // ON
    } else if (g_led_phase == 1 && period >= 50) {
      g_led_phase = 0; g_led_timer = now;
      digitalWrite(PIN_LED, HIGH);  // OFF
    }
  }
}

// ─────────────────────────────────────────────────────────────
//  Loop
// ─────────────────────────────────────────────────────────────
void loop() {
  dnsServer.processNextRequest();
  server.handleClient();
  led_update();

  if (!g_capturing) return;

  if (millis() - g_capture_start > CAPTURE_TIMEOUT_MS) {
    g_capturing = false;
    g_last_ok   = false;
    g_last_msg  = "Timeout 5 detik -- tidak ada sinyal IR";
    Serial.println("[CAP] Timeout");
    return;
  }

  rmt_rx_done_event_data_t ev;
  if (xQueueReceive(rx_queue, &ev, 0) == pdTRUE) {
    g_capturing = false;
    IRSignal sig;
    sig.valid = false;
    capture_process(ev, sig);
    if (sig.valid) {
      if (g_signal_count < MAX_SIGNALS) {
        sig.name = g_pending_name;
        g_signals[g_signal_count] = sig;
        fs_save(g_signal_count);
        g_signal_count++;
        g_last_ok  = true;
        g_last_msg = "'" + g_pending_name + "' tersimpan (" + String(sig.count) + " pulses)";
        Serial.println("[CAP] OK " + String(g_signal_count) + "/50: " + g_pending_name);
      } else {
        g_last_ok  = false;
        g_last_msg = "Storage penuh! 50/50 slot terpakai";
      }
    } else {
      g_last_ok  = false;
      g_last_msg = "Sinyal tidak valid. Coba lebih dekat ke sensor.";
    }
  }
}
