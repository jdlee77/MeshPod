/*
 * TX-only LoRa beacon for Heltec (unofficial lib, SSD1306Wire OLED)
 *
 * Features
 * - Random delay between bursts: [TX_MIN_INTERVAL_S .. TX_MAX_INTERVAL_S]
 * - Burst length: TX_ON_TIME_S seconds, packets spaced by TX_IFG_MS
 * - Frequency list with optional hopping:
 *      • FREQ_HOP_ENABLED on/off
 *      • FREQ_SEQ or FREQ_RANDOM
 *      • Hop per burst (default) or per packet
 * - OLED draws static UI once; thereafter only tiny regions update:
 *      • Seconds countdown (always)
 *      • Frequency line (only when hopping)
 *      • Sent counter (optional)
 *      • Battery % (optional timed refresh)
 * - LED blink on TX with user functions: setBlinkEnabled(bool), toggleBlink()
 *
 * NOTE: You are responsible for regulatory compliance in your region.
 */

#define HELTEC_POWER_BUTTON
#include <heltec_unofficial.h>
#include <esp_system.h>  // esp_random()

// ---------------- User Settings ----------------
const char*  DEVICE_NAME            = "Fox Node-01"; // Change the device name between the "Quotes"

// Randomized wait between bursts (inclusive)
const uint32_t TX_MIN_INTERVAL_S    = 5;     // min seconds between bursts
const uint32_t TX_MAX_INTERVAL_S    = 45;    // max seconds between bursts

// TX burst behavior
const uint32_t TX_ON_TIME_S         = 3;     // seconds to transmit per burst
const uint16_t TX_IFG_MS            = 50;    // inter-frame gap (ms) between packets in a burst
const bool     TX_IMMEDIATE_ON_BOOT = true;  // start with a burst immediately?

// -----------------Frequency list & hopping----------------
static const float FREQ_LIST[] = { 905.2, 906.2, 907.8, 908.6 }; // Add or remove frequencies from this list. ISM Band US 902.5 - 927.5 (Do a sweep to see whats in use first)
static const size_t FREQ_COUNT = sizeof(FREQ_LIST) / sizeof(FREQ_LIST[0]);

const bool     FREQ_HOP_ENABLED = true;     // set false to lock to first frequency in list
enum FreqMode { FREQ_SEQ, FREQ_RANDOM };
const FreqMode FREQ_SELECT_MODE = FREQ_RANDOM; // sequential or random
const bool     HOP_EACH_PACKET  = false;    // false = hop per burst, true = hop per packet

// ---------------- LED Blink Control ----------------
static bool BLINK_ENABLED = true;                 // default ON. Change to false if you DON'T want the node to blink during transmission
void setBlinkEnabled(bool on) { BLINK_ENABLED = on; }
void toggleBlink()           { BLINK_ENABLED = !BLINK_ENABLED; }

// -----------------Transmit Settings------------------
#define BANDWIDTH_KHZ        500.0 // LoRa bandwidth (kHz), as a float: 7.8, 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125.0, 250.0, 500.0
#define SPREADING_FACTOR     9 // Set a value from 5–12. Higher values reduce speed but increase processor gain, yielding longer range and better interference resistance.
#define TRANSMIT_POWER_DBM   0 // TX power (dBm). 0 dBm = 1 mW (good for bench tests). Range: -9 to 22 dBm (~0.125–158 mW). Use a proper antenna—don’t let the magic smoke out.

// Throttle frequency UI updates when hopping per packet (avoid flicker)
const uint32_t FREQ_UI_MIN_INTERVAL_MS = 500;  // 0 = update every change

// Transmission Payload
static const char LORIS_MSG[] =
  "Loris ipsum dolor sit amet, nocte velox et lente; umbrosae arborum secreta servant, amice.";

//----------------------------CAUTION CHANGING SETTINGS BELOW-----------------------------------------

// ---------------- OLED Layout (128x64 typical) ----------------
const int ROW1_Y = 0;     // Device
const int ROW2_Y = 12;    // Frequency (label + value; value part updates on hop)
const int ROW3_Y = 24;    // Sent (label + value; value part updates on TX if enabled)
const int ROW4_Y = 36;    // Countdown line (label + seconds) — seconds region updates
const int ROW5_Y = 48;    // Battery line (label + percent) — optional periodic update

// Countdown label & value regions
const int COUNT_LABEL_X = 0;
const int COUNT_LABEL_W = 40;   // px to clear when switching "Next:" <-> "TX:"
const int COUNT_LABEL_H = 12;

const int COUNT_VALUE_X = 64;   // where seconds text starts
const int COUNT_VALUE_W = 60;   // width of area to wipe for seconds
const int COUNT_VALUE_H = 12;

// "Sent" repaint (visible by default; set false if you truly want ONLY seconds to move)
bool UPDATE_SENT_ON_TX = true;

// Optional battery refresh interval (ms). 0 = draw once only (keeps “only seconds” policy).
const uint32_t BATTERY_REFRESH_MS = 0;

// ---- Geometry computed at runtime (so we only wipe numeric/value regions) ----
int SENT_VALUE_X = 0, SENT_VALUE_W = 0;
int FREQ_VALUE_X = 0, FREQ_VALUE_W = 0;
int BATT_VALUE_X = 0, BATT_VALUE_W = 0;

// ---------------- State ----------------
static uint32_t sentCount      = 0;

static uint64_t nextTxDueAtMs  = 0;
static bool     inTxWindow     = false;
static uint64_t txWindowEndMs  = 0;
static uint64_t lastPacketMs   = 0;

static uint32_t lastShownSecs  = 0xFFFFFFFF; // force initial draw
static bool     lastPhaseWasTx = false;

static size_t   freqIndex      = 0;
static float    currentFreqMHz = 0.0f;

static uint64_t lastFreqUiUpdateMs = 0;

static int       lastBatteryPct     = -1;
static uint64_t  lastBatteryReadMs  = 0;

// ---------------- Timing Helpers ----------------
static uint32_t secsUntil(uint64_t whenMs) {
  uint64_t now = millis();
  if (whenMs <= now) return 0;
  return (uint32_t)((whenMs - now + 999ULL) / 1000ULL); // ceil to whole seconds
}

static void scheduleNextBurst() {
  uint32_t min_ms = TX_MIN_INTERVAL_S * 1000UL;
  uint32_t max_ms = TX_MAX_INTERVAL_S * 1000UL;
  if (max_ms < min_ms) { uint32_t t = min_ms; min_ms = max_ms; max_ms = t; }
  uint32_t interval_ms = (min_ms == max_ms) ? min_ms
                         : (uint32_t)random((long)min_ms, (long)max_ms + 1L);
  nextTxDueAtMs = millis() + interval_ms;
}

// ---------------- Frequency Helpers ----------------
static void applyFrequency(size_t idx) {
  if (FREQ_COUNT == 0) {
    while (true) { both.println("ERROR: FREQ_LIST is empty."); delay(2000); }
  }
  if (idx >= FREQ_COUNT) idx = 0;
  freqIndex = idx;
  currentFreqMHz = FREQ_LIST[freqIndex];
  RADIOLIB_OR_HALT(radio.setFrequency(currentFreqMHz));
}

static void updateFreqUI(bool force = false) {
  uint64_t now = millis();
  if (!force && FREQ_UI_MIN_INTERVAL_MS > 0 &&
      (now - lastFreqUiUpdateMs) < FREQ_UI_MIN_INTERVAL_MS) {
    return;
  }
  lastFreqUiUpdateMs = now;

  display.setColor(BLACK);
  display.fillRect(FREQ_VALUE_X, ROW2_Y, FREQ_VALUE_W, 12);
  display.setColor(WHITE);

  char fbuf[24];
  snprintf(fbuf, sizeof(fbuf), "%.2f MHz", (double)currentFreqMHz);
  display.drawString(FREQ_VALUE_X, ROW2_Y, String(fbuf));
  display.display();
}

static void hopNextFrequency(bool forceUi = false) {
  if (!FREQ_HOP_ENABLED || FREQ_COUNT == 0) return;
  size_t nextIdx = (FREQ_SELECT_MODE == FREQ_RANDOM)
                   ? (size_t)random((long)FREQ_COUNT)
                   : (freqIndex + 1) % FREQ_COUNT;
  applyFrequency(nextIdx);
  updateFreqUI(forceUi); // show the new frequency line (tiny region only)
}

static void startTxWindow() {
  inTxWindow    = true;
  txWindowEndMs = millis() + (uint64_t)TX_ON_TIME_S * 1000ULL;
  lastPacketMs  = 0;

  if (!HOP_EACH_PACKET) {
    hopNextFrequency(true); // hop at the start of each burst; force UI update
  }
}

// ---------------- Battery Helpers (from heltec_unofficial) ----------------
static int readBatteryPercent() {
  // returns 0..100 (estimated), or negative/NaN if not available
  float pct = heltec_battery_percent();
  if (!(pct >= 0.0f && pct <= 100.0f)) return -1;
  return (int)(pct + 0.5f);
}

static void maybeUpdateBatteryUI() {
  if (BATTERY_REFRESH_MS == 0) return;                // disabled by default
  uint64_t now = millis();
  if (now - lastBatteryReadMs < BATTERY_REFRESH_MS) return;
  lastBatteryReadMs = now;

  int pct = readBatteryPercent();
  if (pct == lastBatteryPct) return;                  // no change => skip redraw
  lastBatteryPct = pct;

  display.setColor(BLACK);
  display.fillRect(BATT_VALUE_X, ROW5_Y, BATT_VALUE_W, 12);
  display.setColor(WHITE);

  char bbuf[16];
  if (pct < 0) snprintf(bbuf, sizeof(bbuf), "--%%");
  else         snprintf(bbuf, sizeof(bbuf), "%d%%", pct);
  display.drawString(BATT_VALUE_X, ROW5_Y, String(bbuf));
  display.display();
}

// ---------------- OLED (SSD1306Wire) ----------------
static void drawStaticUI() {
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);

  // Device
  display.drawString(0, ROW1_Y, String("Device:  ") + DEVICE_NAME);

  // Frequency label + value (value part will be updated on hop)
  String freqLabel = "Frequency: ";
  display.drawString(0, ROW2_Y, freqLabel);
  FREQ_VALUE_X = display.getStringWidth(freqLabel);
  FREQ_VALUE_W = display.getWidth() - FREQ_VALUE_X - 2;

  char fbuf[24];
  snprintf(fbuf, sizeof(fbuf), "%.2f MHz", (double)currentFreqMHz);
  display.drawString(FREQ_VALUE_X, ROW2_Y, String(fbuf));

  // Sent label + initial value (value part may be updated on TX)
  String sentLabel = "Total Packets Sent: ";
  display.drawString(0, ROW3_Y, sentLabel);
  SENT_VALUE_X = display.getStringWidth(sentLabel);
  SENT_VALUE_W = display.getWidth() - SENT_VALUE_X - 2;

  char nbuf[16];
  snprintf(nbuf, sizeof(nbuf), "%lu", (unsigned long)sentCount);
  display.drawString(SENT_VALUE_X, ROW3_Y, String(nbuf));

  // Countdown label (seconds value will be updated elsewhere)
  display.drawString(COUNT_LABEL_X, ROW4_Y, inTxWindow ? "TX:" : "Next TX:");

  // Battery line (label + initial percent; value may update on timer)
  String battLabel = "Battery: ";
  display.drawString(0, ROW5_Y, battLabel);
  BATT_VALUE_X = display.getStringWidth(battLabel);
  BATT_VALUE_W = display.getWidth() - BATT_VALUE_X - 2;

  int pct = readBatteryPercent();
  lastBatteryPct = pct;
  char bbuf[16];
  if (pct < 0) snprintf(bbuf, sizeof(bbuf), "--%%");
  else         snprintf(bbuf, sizeof(bbuf), "%d%%", pct);
  display.drawString(BATT_VALUE_X, ROW5_Y, String(bbuf));

  display.display(); // push initial buffer
}

static void updateCountdownUI(bool txPhase, uint32_t secs) {
  // If phase changed (Next<->TX), clear the label region and redraw it
  if (txPhase != lastPhaseWasTx) {
    lastPhaseWasTx = txPhase;

    display.setColor(BLACK);
    display.fillRect(COUNT_LABEL_X, ROW4_Y, COUNT_LABEL_W, COUNT_LABEL_H);
    display.setColor(WHITE);

    display.drawString(COUNT_LABEL_X, ROW4_Y, txPhase ? "TX:" : "Next:");
    // (seconds update below will push the buffer)
  }

  // Only update seconds if changed
  if (secs != lastShownSecs) {
    lastShownSecs = secs;

    // Wipe small seconds area then draw new value
    display.setColor(BLACK);
    display.fillRect(COUNT_VALUE_X, ROW4_Y, COUNT_VALUE_W, COUNT_VALUE_H);
    display.setColor(WHITE);

    char sbuf[16];
    snprintf(sbuf, sizeof(sbuf), "%lus", (unsigned long)secs);
    display.drawString(COUNT_VALUE_X, ROW4_Y, String(sbuf));

    display.display(); // push the small update
  }
}

static void maybeUpdateSentUI() {
  if (!UPDATE_SENT_ON_TX) return;

  // Wipe just the Sent numeric field, then redraw current count
  display.setColor(BLACK);
  display.fillRect(SENT_VALUE_X, ROW3_Y, SENT_VALUE_W, 12);
  display.setColor(WHITE);

  char nbuf[16];
  snprintf(nbuf, sizeof(nbuf), "%lu", (unsigned long)sentCount);
  display.drawString(SENT_VALUE_X, ROW3_Y, String(nbuf));

  display.display();
}

// ---------------- Radio TX ----------------
static void sendPacket() {
  if (BLINK_ENABLED) heltec_led(50);
  RADIOLIB(radio.transmit(LORIS_MSG));
  if (BLINK_ENABLED) heltec_led(0);

  if (_radiolib_status == RADIOLIB_ERR_NONE) {
    sentCount++;
    maybeUpdateSentUI(); // optional, controlled by UPDATE_SENT_ON_TX
  }
  lastPacketMs = millis();
}

// ---------------- Arduino Hooks ----------------
void setup() {
  heltec_setup();
  

  // Seed PRNG from hardware RNG
  randomSeed((uint32_t)esp_random());

  // Radio base config
  RADIOLIB_OR_HALT(radio.begin());
  RADIOLIB_OR_HALT(radio.setBandwidth(BANDWIDTH_KHZ));
  RADIOLIB_OR_HALT(radio.setSpreadingFactor(SPREADING_FACTOR));
  RADIOLIB_OR_HALT(radio.setOutputPower(TRANSMIT_POWER_DBM));

  // Frequency
  applyFrequency(0); // initial frequency (index 0)

  // Initial scheduling
  inTxWindow     = false;
  lastPhaseWasTx = false;
  if (TX_IMMEDIATE_ON_BOOT) startTxWindow();
  else                      scheduleNextBurst();

  // Draw static UI once
  drawStaticUI();

  // Seed countdown
  updateCountdownUI(inTxWindow,
                    inTxWindow ? secsUntil(txWindowEndMs) : secsUntil(nextTxDueAtMs));
}

void loop() {
  heltec_loop();
  uint64_t now = millis();

  if (inTxWindow) {
    // End of TX window
    if (now >= txWindowEndMs) {
      inTxWindow = false;
      scheduleNextBurst();
      updateCountdownUI(false, secsUntil(nextTxDueAtMs));
      return;
    }

    // Hop-per-packet (frequency change) if enabled
    if (HOP_EACH_PACKET && FREQ_HOP_ENABLED && (now - lastPacketMs >= TX_IFG_MS)) {
      hopNextFrequency(false); // UI throttled by FREQ_UI_MIN_INTERVAL_MS
    }

    // Send next packet when IFG elapsed
    if (now - lastPacketMs >= TX_IFG_MS) {
      sendPacket();
    }

    // Update TX countdown (seconds only)
    updateCountdownUI(true, secsUntil(txWindowEndMs));

  } else {
    // Time to start a new burst?
    if (now >= nextTxDueAtMs) {
      startTxWindow();          // will hop (and force freq UI) if enabled
      sendPacket();             // kick off immediately
      updateCountdownUI(true, secsUntil(txWindowEndMs));
      return;
    }

    // Update NEXT countdown (seconds only)
    updateCountdownUI(false, secsUntil(nextTxDueAtMs));
  }

  // Optional timed battery update (off by default)
  maybeUpdateBatteryUI();
}
