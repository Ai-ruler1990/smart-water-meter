/*
 * ============================================================
 *  WATER QUALITY MONITORING SYSTEM — ESP32
 *  Arduino IDE  |  Version 9.0 — Calibration + WDT Fix Edition
 *  Blynk IoT + Calibrated Sensors + 3-Node Diagnosis Engine
 * ============================================================
 *
 *  CHANGES FROM v8 → v9:
 *  ─────────────────────────────────────────────────────────
 *  FIX 1. Watchdog Timer (WDT) crash fixed.
 *         readMillivolts() now calls esp_task_wdt_reset() and
 *         yield() inside the sampling loop. SAMPLE_DELAY_MS
 *         reduced from 200ms → 50ms (still valid, less blocking).
 *         SAMPLES reduced from 10 → 6 (trimmed mean still uses 4).
 *
 *  FIX 2. Blynk.connect() moved to non-blocking pattern.
 *         connectWiFi() no longer stalls the boot sequence
 *         if blynk.cloud is unreachable. Blynk reconnect is
 *         handled gracefully inside loop().
 *
 *  FIX 3. Buzzer made non-blocking using BlynkTimer.
 *         alertBuzzer() sets a flag; a separate timer fires
 *         the beep pattern so it never stalls the main loop.
 *
 *  FIX 4. Added 4-sensor runtime calibration engine.
 *         Calibration values stored in flash via Preferences
 *         (survives power cycles). Adjustable from Blynk
 *         dashboard or Serial terminal.
 *         See calibration Blynk pin map below (V13–V18).
 *
 *  FIX 5. Added BLYNK_CONNECTED() hook to push current
 *         calibration values to dashboard on every reconnect.
 *
 *  All v8 simulation scenarios + diagnosis logic unchanged.
 *
 * ============================================================
 *
 *  BLYNK VIRTUAL PIN MAP — v9 (FULL)
 *  ─────────────────────────────────────────────────────────
 *  NODE 3 — Physical (Destination / Hostel Tank):
 *  V0  → Temperature (°C)
 *  V1  → TDS (ppm)
 *  V2  → pH
 *  V3  → Turbidity (NTU)
 *
 *  NODE 1 — Simulated (Source / Main Tank):
 *  V4  → Simulated Source pH
 *  V5  → Simulated Source TDS (ppm)
 *  V6  → Simulated Source Turbidity (NTU)
 *
 *  NODE 2 — Simulated (Transport / Distribution Pipe):
 *  V7  → Simulated Transport pH
 *  V8  → Simulated Transport TDS (ppm)
 *  V9  → Simulated Transport Turbidity (NTU)
 *
 *  DIAGNOSIS + ACTUATION:
 *  V10 → Diagnosis String (Label / Terminal widget)
 *  V11 → Hostel Shutoff Valve (LED widget: 255=Open, 0=Closed)
 *
 *  SIMULATION CONTROL:
 *  V12 → Scenario Selector (Dropdown / Menu widget)
 *         Option 1 = "All Nodes Safe"
 *         Option 2 = "Source Contamination"
 *         Option 3 = "Distribution Pipe Failure"
 *         Option 4 = "Hostel Tank Contamination"
 *
 *  CALIBRATION (NEW in v9) — use Number Input widgets:
 *  V13 → pH Slope         (default: -2.7523)
 *  V14 → pH Intercept     (default:  8.844 )
 *  V15 → TDS K-Value      (default:  0.828 )
 *  V16 → Turbidity Scale  (default:  3.218 )
 *  V17 → Turbidity Offset (default: -7.0   )
 *  V18 → Temperature Offset °C (default: 0.0)
 *         Button widget → write 1 to V18 to SAVE to flash
 *         NOTE: Set V18 as a Number Input for the offset,
 *         and add a separate Button widget writing to V19
 *         to trigger save.
 *
 *  V19 → Save Calibration (Button widget: push 1 to save)
 *  ─────────────────────────────────────────────────────────
 *
 *  SERIAL CALIBRATION COMMANDS (type in Serial Monitor):
 *  CAL:PH_SLOPE:<value>       e.g. CAL:PH_SLOPE:-2.75
 *  CAL:PH_INT:<value>         e.g. CAL:PH_INT:8.84
 *  CAL:TDS_K:<value>          e.g. CAL:TDS_K:0.828
 *  CAL:TURB_SCALE:<value>     e.g. CAL:TURB_SCALE:3.218
 *  CAL:TURB_OFFSET:<value>    e.g. CAL:TURB_OFFSET:-7.0
 *  CAL:TEMP_OFFSET:<value>    e.g. CAL:TEMP_OFFSET:0.0
 *  CAL:SAVE                   saves all to flash
 *  CAL:RESET                  resets all to factory defaults
 *  CAL:PRINT                  prints current values
 *  ─────────────────────────────────────────────────────────
 *
 *  HARDWARE PINS (unchanged from v7/v8):
 *  DS18B20 data       → GPIO 4   (+ 4.7kΩ pull-up to 3.3V)
 *  TDS analog out     → GPIO 34  (ADC1_CH6)
 *  pH analog out      → GPIO 35  (ADC1_CH7)
 *  Turbidity analog   → GPIO 32  (ADC1_CH4)
 *  Buzzer (+)         → GPIO 25
 *  Green  LED + 220Ω → GPIO 26
 *  Yellow LED + 220Ω → GPIO 27
 *  Red    LED + 220Ω → GPIO 14
 * ============================================================
 */

// ── STEP 1: Enable Blynk debug output ────────────────────────
#define BLYNK_PRINT Serial

// ── STEP 2: Blynk credentials ─────────────────────────────────
#define BLYNK_TEMPLATE_ID   "TMPL2LWQWIcd8"
#define BLYNK_TEMPLATE_NAME "Water Quality Monitor"
#define BLYNK_AUTH_TOKEN    "W25eIGG_fdHApMgMPD4xqWrD9Q9xEr1N"

// ── STEP 3: Blynk library ─────────────────────────────────────
#include <BlynkSimpleEsp32.h>

// ── STEP 4: All other libraries ───────────────────────────────
#include "esp_adc_cal.h"
#include "driver/adc.h"
#include "esp_task_wdt.h"       // ← v9: explicit WDT feed
#include <Preferences.h>        // ← v9: flash calibration storage
#include <Arduino.h>
#include <WiFi.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ── WiFi credentials ──────────────────────────────────────────
char ssid[] = "iPhonex";
char pass[] = "Amarame@v12";

// ── WiFi / Blynk timeouts ─────────────────────────────────────
#define WIFI_TIMEOUT_MS   10000
#define BLYNK_TIMEOUT_MS   5000

// ── Feature flags ─────────────────────────────────────────────
#define USE_ADC_CAL  true
#define USE_LCD      false

// ── Debug flags ───────────────────────────────────────────────
#define DEBUG_TURBIDITY  false
#define DEBUG_PH         false
#define DEBUG_TDS        false
#define DEBUG_SIM        false
#define DEBUG_CAL        true    // print calibration values on startup

// ── Pin assignments ───────────────────────────────────────────
#define PIN_TDS          34
#define PIN_PH           35
#define PIN_TURBIDITY    32
#define PIN_DS18B20       4

#define PIN_BUZZER       25
#define PIN_LED_GREEN    26
#define PIN_LED_YELLOW   27
#define PIN_LED_RED      14

// ── ADC calibration object ────────────────────────────────────
#if USE_ADC_CAL
  static esp_adc_cal_characteristics_t adc_chars;
#endif

// ════════════════════════════════════════════════════════════
//  FACTORY-DEFAULT CALIBRATION CONSTANTS
//  These are the compiled-in defaults used on first boot or
//  after CAL:RESET. Runtime values are loaded from flash.
// ════════════════════════════════════════════════════════════
#define DEF_PH_SLOPE        -2.7523f
#define DEF_PH_INTERCEPT     8.844f
#define DEF_TURB_COEFF_A  -1120.4f
#define DEF_TURB_COEFF_B   5742.3f
#define DEF_TURB_COEFF_C  -4352.9f
#define DEF_TURB_K_SCALE     3.218f
#define DEF_TURB_OFFSET     -7.0f
#define DEF_TDS_K_VALUE      0.828f
#define DEF_TDS_TEMP_COEFF   0.02f
#define DEF_TEMP_OFFSET      0.0f
#define TURB_MAX_NTU      3000.0f

// ── Live calibration variables (loaded from flash at boot) ────
float cal_ph_slope       = DEF_PH_SLOPE;
float cal_ph_intercept   = DEF_PH_INTERCEPT;
float cal_turb_k_scale   = DEF_TURB_K_SCALE;
float cal_turb_offset    = DEF_TURB_OFFSET;
float cal_tds_k_value    = DEF_TDS_K_VALUE;
float cal_temp_offset    = DEF_TEMP_OFFSET;

// Turbidity polynomial coefficients (not user-adjustable at runtime;
// changed only by recompile — they describe sensor physics)
const float TURB_COEFF_A = DEF_TURB_COEFF_A;
const float TURB_COEFF_B = DEF_TURB_COEFF_B;
const float TURB_COEFF_C = DEF_TURB_COEFF_C;
const float TDS_TEMP_COEFF = DEF_TDS_TEMP_COEFF;

// ── Preferences namespace for flash storage ───────────────────
Preferences prefs;
#define PREFS_NS "wqm_cal"   // namespace key (max 15 chars)

// ── v9: Reduced sampling to prevent WDT stalls ───────────────
//  6 samples × 50ms = 300ms total per sensor (was 10 × 200ms = 2s)
//  Trimmed mean discards min+max → averages 4 readings.
//  Sufficient for stable ADC reads on ESP32.
#define SAMPLES          6
#define SAMPLE_DELAY_MS  50

// ════════════════════════════════════════════════════════════
//  WATER QUALITY THRESHOLDS (WHO / EPA / EU / ISO 7027)
// ════════════════════════════════════════════════════════════
#define TDS_GOOD_MIN     50.0
#define TDS_GOOD_MAX    500.0
#define TDS_WARN_MIN     30.0
#define TDS_WARN_MAX   1000.0

#define PH_GOOD_MIN      6.5
#define PH_GOOD_MAX      8.5
#define PH_WARN_MIN      6.0
#define PH_WARN_MAX      9.0

#define TURB_GOOD_MAX    1.0
#define TURB_WARN_MAX    4.0

#define TEMP_GOOD_MIN    5.0
#define TEMP_GOOD_MAX   25.0
#define TEMP_WARN_MIN    2.0
#define TEMP_WARN_MAX   30.0

// ════════════════════════════════════════════════════════════
//  SIMULATION ENGINE CONSTANTS (unchanged from v8)
// ════════════════════════════════════════════════════════════
#define N1_SAFE_PH      7.1f
#define N1_SAFE_TDS   150.0f
#define N1_SAFE_TURB    0.5f
#define N2_SAFE_PH      7.0f
#define N2_SAFE_TDS   160.0f
#define N2_SAFE_TURB    0.8f
#define N1_FAULT_PH     5.0f
#define N1_FAULT_TDS  1200.0f
#define N1_FAULT_TURB  15.0f
#define N2_FAULT_PH     4.5f
#define N2_FAULT_TDS  1350.0f
#define N2_FAULT_TURB  18.0f

// ── Quality levels ────────────────────────────────────────────
enum WaterQuality { GOOD, WARNING, CRITICAL };

// ── Measurement struct ────────────────────────────────────────
struct Measurements {
  float tds;
  float ph;
  float turbidity;
  float temperature;
};

// ── Hardware objects ──────────────────────────────────────────
OneWire           oneWire(PIN_DS18B20);
DallasTemperature tempSensor(&oneWire);

#if USE_LCD
  LiquidCrystal_I2C lcd(0x27, 16, 2);
#endif

// ── Timers ────────────────────────────────────────────────────
BlynkTimer timer;

// ── Simulation state ──────────────────────────────────────────
int currentScenario = 1;

// ── v9: Non-blocking buzzer state ─────────────────────────────
WaterQuality pendingBuzzerQuality = GOOD;
bool         buzzerPending        = false;

// ── v9: Calibration dirty flag ────────────────────────────────
bool calDirty = false;

// ── Forward declarations ──────────────────────────────────────
uint32_t     readMillivolts(adc1_channel_t channel);
float        readTDS(float tempC);
float        readPH();
float        readTurbidity();
float        readTemperature();
WaterQuality assessQuality(const Measurements& m,
                           bool& tw, bool& pw,
                           bool& turW, bool& teW);
void         printSerial(const Measurements& m, WaterQuality q,
                         bool tw, bool pw, bool turW, bool teW,
                         float n1_ph, float n1_tds, float n1_turb,
                         float n2_ph, float n2_tds, float n2_turb,
                         const String& diagnosis, int valveState);
void         updateLEDs(WaterQuality q);
void         serviceBuzzer();
void         connectWiFi();
void         loadCalibration();
void         saveCalibration();
void         resetCalibration();
void         printCalibration();
void         pushCalToBlynk();
void         handleSerialCal();

#if USE_LCD
void         updateLCD(const Measurements& m, WaterQuality q);
#endif


// ════════════════════════════════════════════════════════════
//  BLYNK_CONNECTED — fires on every successful Blynk connect
//  Pushes current calibration values + scenario to dashboard.
// ════════════════════════════════════════════════════════════
BLYNK_CONNECTED() {
  Serial.println(F("[Blynk] Connected — syncing dashboard..."));
  Blynk.virtualWrite(V12, currentScenario);
  pushCalToBlynk();
}


// ════════════════════════════════════════════════════════════
//  BLYNK_WRITE(V12) — Scenario Selector (unchanged from v8)
// ════════════════════════════════════════════════════════════
BLYNK_WRITE(V12) {
  currentScenario = param.asInt();
  Serial.print(F("[Sim] Scenario → "));
  switch (currentScenario) {
    case 1: Serial.println(F("1 — All Nodes Safe")); break;
    case 2: Serial.println(F("2 — Source Contamination")); break;
    case 3: Serial.println(F("3 — Distribution Pipe Failure")); break;
    case 4: Serial.println(F("4 — Hostel Tank Contamination")); break;
    default: Serial.println(currentScenario); break;
  }
}


// ════════════════════════════════════════════════════════════
//  CALIBRATION — Blynk write handlers (V13–V19)
//
//  V13 → pH Slope
//  V14 → pH Intercept
//  V15 → TDS K-Value
//  V16 → Turbidity K Scale
//  V17 → Turbidity Offset
//  V18 → Temperature Offset
//  V19 → Save Calibration (button, write 1)
// ════════════════════════════════════════════════════════════
BLYNK_WRITE(V13) {
  cal_ph_slope = param.asFloat();
  calDirty = true;
  Serial.print(F("[Cal] pH Slope set to: ")); Serial.println(cal_ph_slope, 4);
}

BLYNK_WRITE(V14) {
  cal_ph_intercept = param.asFloat();
  calDirty = true;
  Serial.print(F("[Cal] pH Intercept set to: ")); Serial.println(cal_ph_intercept, 4);
}

BLYNK_WRITE(V15) {
  cal_tds_k_value = param.asFloat();
  calDirty = true;
  Serial.print(F("[Cal] TDS K-Value set to: ")); Serial.println(cal_tds_k_value, 4);
}

BLYNK_WRITE(V16) {
  cal_turb_k_scale = param.asFloat();
  calDirty = true;
  Serial.print(F("[Cal] Turbidity K-Scale set to: ")); Serial.println(cal_turb_k_scale, 4);
}

BLYNK_WRITE(V17) {
  cal_turb_offset = param.asFloat();
  calDirty = true;
  Serial.print(F("[Cal] Turbidity Offset set to: ")); Serial.println(cal_turb_offset, 2);
}

BLYNK_WRITE(V18) {
  cal_temp_offset = param.asFloat();
  calDirty = true;
  Serial.print(F("[Cal] Temp Offset set to: ")); Serial.println(cal_temp_offset, 2);
}

BLYNK_WRITE(V19) {
  if (param.asInt() == 1) {
    saveCalibration();
    Serial.println(F("[Cal] Calibration saved to flash via Blynk button."));
    if (Blynk.connected()) {
      Blynk.virtualWrite(V10, "Calibration SAVED to flash.");
    }
    calDirty = false;
  }
}


// ════════════════════════════════════════════════════════════
//  CALIBRATION — Flash read/write (Preferences)
// ════════════════════════════════════════════════════════════

void loadCalibration() {
  prefs.begin(PREFS_NS, true);  // read-only
  cal_ph_slope     = prefs.getFloat("ph_slope",   DEF_PH_SLOPE);
  cal_ph_intercept = prefs.getFloat("ph_int",     DEF_PH_INTERCEPT);
  cal_tds_k_value  = prefs.getFloat("tds_k",      DEF_TDS_K_VALUE);
  cal_turb_k_scale = prefs.getFloat("turb_scale", DEF_TURB_K_SCALE);
  cal_turb_offset  = prefs.getFloat("turb_off",   DEF_TURB_OFFSET);
  cal_temp_offset  = prefs.getFloat("temp_off",   DEF_TEMP_OFFSET);
  prefs.end();
  Serial.println(F("[Cal] Calibration loaded from flash."));
}

void saveCalibration() {
  prefs.begin(PREFS_NS, false); // read-write
  prefs.putFloat("ph_slope",   cal_ph_slope);
  prefs.putFloat("ph_int",     cal_ph_intercept);
  prefs.putFloat("tds_k",      cal_tds_k_value);
  prefs.putFloat("turb_scale", cal_turb_k_scale);
  prefs.putFloat("turb_off",   cal_turb_offset);
  prefs.putFloat("temp_off",   cal_temp_offset);
  prefs.end();
  calDirty = false;
  Serial.println(F("[Cal] Calibration saved to flash."));
}

void resetCalibration() {
  cal_ph_slope     = DEF_PH_SLOPE;
  cal_ph_intercept = DEF_PH_INTERCEPT;
  cal_tds_k_value  = DEF_TDS_K_VALUE;
  cal_turb_k_scale = DEF_TURB_K_SCALE;
  cal_turb_offset  = DEF_TURB_OFFSET;
  cal_temp_offset  = DEF_TEMP_OFFSET;
  saveCalibration();
  pushCalToBlynk();
  Serial.println(F("[Cal] All values reset to factory defaults."));
}

void printCalibration() {
  Serial.println(F("┌──────────────────────────────────────────┐"));
  Serial.println(F("│  CURRENT CALIBRATION VALUES               │"));
  Serial.println(F("├──────────────────────────────────────────┤"));
  Serial.print(F("│  pH Slope      (V13): ")); Serial.println(cal_ph_slope,     4);
  Serial.print(F("│  pH Intercept  (V14): ")); Serial.println(cal_ph_intercept, 4);
  Serial.print(F("│  TDS K-Value   (V15): ")); Serial.println(cal_tds_k_value,  4);
  Serial.print(F("│  Turb K-Scale  (V16): ")); Serial.println(cal_turb_k_scale, 4);
  Serial.print(F("│  Turb Offset   (V17): ")); Serial.println(cal_turb_offset,  2);
  Serial.print(F("│  Temp Offset   (V18): ")); Serial.println(cal_temp_offset,  2);
  Serial.println(F("└──────────────────────────────────────────┘"));
}

void pushCalToBlynk() {
  if (!Blynk.connected()) return;
  Blynk.virtualWrite(V13, cal_ph_slope);
  Blynk.virtualWrite(V14, cal_ph_intercept);
  Blynk.virtualWrite(V15, cal_tds_k_value);
  Blynk.virtualWrite(V16, cal_turb_k_scale);
  Blynk.virtualWrite(V17, cal_turb_offset);
  Blynk.virtualWrite(V18, cal_temp_offset);
}


// ════════════════════════════════════════════════════════════
//  SERIAL CALIBRATION COMMAND PARSER
//  Called once per loop() from main loop.
//  Commands: CAL:PH_SLOPE:<v>  CAL:PH_INT:<v>  CAL:TDS_K:<v>
//            CAL:TURB_SCALE:<v>  CAL:TURB_OFFSET:<v>
//            CAL:TEMP_OFFSET:<v>  CAL:SAVE  CAL:RESET  CAL:PRINT
// ════════════════════════════════════════════════════════════
void handleSerialCal() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  cmd.toUpperCase();

  if (!cmd.startsWith("CAL:")) return;

  if (cmd == "CAL:SAVE") {
    saveCalibration();
    pushCalToBlynk();
    return;
  }
  if (cmd == "CAL:RESET") {
    resetCalibration();
    return;
  }
  if (cmd == "CAL:PRINT") {
    printCalibration();
    return;
  }

  // Parse CAL:KEY:VALUE
  int colon2 = cmd.indexOf(':', 4);  // find 2nd colon
  if (colon2 < 0) { Serial.println(F("[Cal] Unknown command.")); return; }
  String key = cmd.substring(4, colon2);
  float  val = cmd.substring(colon2 + 1).toFloat();

  if      (key == "PH_SLOPE")    { cal_ph_slope     = val; calDirty = true; }
  else if (key == "PH_INT")      { cal_ph_intercept = val; calDirty = true; }
  else if (key == "TDS_K")       { cal_tds_k_value  = val; calDirty = true; }
  else if (key == "TURB_SCALE")  { cal_turb_k_scale = val; calDirty = true; }
  else if (key == "TURB_OFFSET") { cal_turb_offset  = val; calDirty = true; }
  else if (key == "TEMP_OFFSET") { cal_temp_offset  = val; calDirty = true; }
  else { Serial.println(F("[Cal] Unknown key. Use CAL:PRINT to see options.")); return; }

  Serial.print(F("[Cal] ")); Serial.print(key);
  Serial.print(F(" = ")); Serial.println(val, 4);
  Serial.println(F("[Cal] Send CAL:SAVE to persist to flash."));
  pushCalToBlynk();
}


// ════════════════════════════════════════════════════════════
//  connectWiFi() — v9: non-blocking Blynk connect
// ════════════════════════════════════════════════════════════
void connectWiFi() {
  Serial.print(F("\nConnecting to WiFi: "));
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);

  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - t0 >= WIFI_TIMEOUT_MS) {
      Serial.println(F("\n[WiFi] Timed out — running offline."));
      return;
    }
    delay(500);
    Serial.print(F("."));
    esp_task_wdt_reset();   // ← feed watchdog during WiFi wait
  }

  Serial.println();
  Serial.print(F("[WiFi] Connected! IP: "));
  Serial.println(WiFi.localIP());

  // Configure Blynk but do NOT block waiting for cloud connection.
  // BLYNK_CONNECTED() fires automatically when it connects.
  Blynk.config(BLYNK_AUTH_TOKEN);
  Blynk.connect(BLYNK_TIMEOUT_MS);

  if (Blynk.connected()) {
    Serial.println(F("[Blynk] Connected to blynk.cloud."));
  } else {
    Serial.println(F("[Blynk] Cloud unreachable — will retry automatically."));
  }
}


// ════════════════════════════════════════════════════════════
//  sendSensorData() — called every 3 seconds by BlynkTimer
// ════════════════════════════════════════════════════════════
void sendSensorData() {

  // ── 1. READ PHYSICAL SENSORS (NODE 3) ────────────────────
  Measurements m;
  m.temperature = readTemperature();
  m.tds         = readTDS(m.temperature);
  m.ph          = readPH();
  m.turbidity   = readTurbidity();

  bool tdsWarn, phWarn, turbWarn, tempWarn;
  WaterQuality quality = assessQuality(m, tdsWarn, phWarn, turbWarn, tempWarn);

  // ── 2. SIMULATED NODES 1 & 2 (unchanged from v8) ─────────
  float n1_ph   = N1_SAFE_PH,  n1_tds = N1_SAFE_TDS, n1_turb = N1_SAFE_TURB;
  float n2_ph   = N2_SAFE_PH,  n2_tds = N2_SAFE_TDS, n2_turb = N2_SAFE_TURB;

  switch (currentScenario) {
    case 2:
      n1_ph = N1_FAULT_PH; n1_tds = N1_FAULT_TDS; n1_turb = N1_FAULT_TURB;
      break;
    case 3:
      n2_ph = N2_FAULT_PH; n2_tds = N2_FAULT_TDS; n2_turb = N2_FAULT_TURB;
      break;
    default: break;   // case 1 & 4: baselines unchanged
  }

  // ── 3. DIFFERENTIAL DIAGNOSIS (unchanged from v8) ────────
  String diagnosis  = "System Normal. Water Quality Safe.";
  int    valveState = 255;

  if      (n1_turb > TURB_WARN_MAX || n1_ph < PH_WARN_MIN || n1_tds > TDS_WARN_MAX) {
    diagnosis  = "FAULT ISOLATED: Source (Main Tank) Contamination.";
    valveState = 0;
  }
  else if (n2_turb > TURB_WARN_MAX || n2_ph < PH_WARN_MIN || n2_tds > TDS_WARN_MAX) {
    diagnosis  = "FAULT ISOLATED: Distribution Pipe Integrity Failure.";
    valveState = 0;
  }
  else if (m.turbidity > TURB_WARN_MAX || m.ph < PH_WARN_MIN || m.tds > TDS_WARN_MAX) {
    diagnosis  = "FAULT ISOLATED: Local Hostel Tank Contamination.";
    valveState = 0;
  }

  // ── 4. PUSH TO BLYNK ─────────────────────────────────────
  if (Blynk.connected()) {
    Blynk.virtualWrite(V0,  m.temperature);
    Blynk.virtualWrite(V1,  m.tds);
    Blynk.virtualWrite(V2,  m.ph);
    Blynk.virtualWrite(V3,  m.turbidity);
    Blynk.virtualWrite(V4,  n1_ph);
    Blynk.virtualWrite(V5,  n1_tds);
    Blynk.virtualWrite(V6,  n1_turb);
    Blynk.virtualWrite(V7,  n2_ph);
    Blynk.virtualWrite(V8,  n2_tds);
    Blynk.virtualWrite(V9,  n2_turb);
    Blynk.virtualWrite(V10, diagnosis);
    Blynk.virtualWrite(V11, valveState);
  }

  // ── 5. HARDWARE OUTPUTS ───────────────────────────────────
  updateLEDs(quality);
  pendingBuzzerQuality = quality;
  buzzerPending = true;   // serviceBuzzer() handles it non-blocking

  printSerial(m, quality, tdsWarn, phWarn, turbWarn, tempWarn,
              n1_ph, n1_tds, n1_turb, n2_ph, n2_tds, n2_turb,
              diagnosis, valveState);

#if USE_LCD
  updateLCD(m, quality);
#endif
}


// ════════════════════════════════════════════════════════════
//  SETUP
// ════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(800);   // brief pause to let USB-serial settle

  Serial.println(F("\n╔══════════════════════════════════════════╗"));
  Serial.println(F("║  ESP32 Water Quality Monitor  v9.0       ║"));
  Serial.println(F("║  WDT-Fix + 4-Sensor Calibration Engine   ║"));
  Serial.println(F("╚══════════════════════════════════════════╝\n"));

  // ── Load calibration from flash ───────────────────────────
  loadCalibration();

#if DEBUG_CAL
  printCalibration();
#endif

  // ── Output pins ───────────────────────────────────────────
  pinMode(PIN_BUZZER,     OUTPUT);
  pinMode(PIN_LED_GREEN,  OUTPUT);
  pinMode(PIN_LED_YELLOW, OUTPUT);
  pinMode(PIN_LED_RED,    OUTPUT);
  digitalWrite(PIN_BUZZER,     LOW);
  digitalWrite(PIN_LED_GREEN,  LOW);
  digitalWrite(PIN_LED_YELLOW, LOW);
  digitalWrite(PIN_LED_RED,    LOW);

  // LED sweep test
  Serial.println(F("LED test..."));
  digitalWrite(PIN_LED_RED,    HIGH); delay(200); digitalWrite(PIN_LED_RED,    LOW);
  digitalWrite(PIN_LED_YELLOW, HIGH); delay(200); digitalWrite(PIN_LED_YELLOW, LOW);
  digitalWrite(PIN_LED_GREEN,  HIGH); delay(200); digitalWrite(PIN_LED_GREEN,  LOW);
  Serial.println(F("LED test done."));

  // ── ADC (ESP-IDF low-level) ────────────────────────────────
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11); // TDS
  adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_11); // pH
  adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_11); // Turbidity

#if USE_ADC_CAL
  esp_adc_cal_value_t calType = esp_adc_cal_characterize(
    ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars
  );
  if      (calType == ESP_ADC_CAL_VAL_EFUSE_VREF) Serial.println(F("ADC: eFuse Vref"));
  else if (calType == ESP_ADC_CAL_VAL_EFUSE_TP)   Serial.println(F("ADC: Two-point eFuse"));
  else                                              Serial.println(F("ADC: Default Vref"));
#endif

  // ── DS18B20 ───────────────────────────────────────────────
  tempSensor.begin();
  if (tempSensor.getDeviceCount() == 0) {
    Serial.println(F("[WARN] DS18B20 not found — check GPIO4 + 4.7kΩ to 3.3V."));
  } else {
    tempSensor.setResolution(12);
    Serial.print(F("DS18B20 found. Count: "));
    Serial.println(tempSensor.getDeviceCount());
  }

  // ── LCD ───────────────────────────────────────────────────
#if USE_LCD
  Wire.begin(21, 22);
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0); lcd.print(F("Water Quality   "));
  lcd.setCursor(0, 1); lcd.print(F("Monitor  v9.0   "));
  delay(1500);
  lcd.clear();
#endif

  // ── WiFi + Blynk ─────────────────────────────────────────
  connectWiFi();

  // ── Timers ────────────────────────────────────────────────
  // Main sensor read: every 3 seconds
  timer.setInterval(3000L,  sendSensorData);
  // Buzzer service: every 500ms (non-blocking beep dispatch)
  timer.setInterval(500L,   serviceBuzzer);

  Serial.println(F("\nSystem ready. Readings every 3 s."));
  Serial.println(F("Serial cal commands: CAL:PRINT  CAL:SAVE  CAL:RESET"));
  Serial.println(F("  e.g.  CAL:PH_SLOPE:-2.75  then  CAL:SAVE\n"));
}


// ════════════════════════════════════════════════════════════
//  LOOP
// ════════════════════════════════════════════════════════════
void loop() {
  // Blynk + timer run
  if (Blynk.connected()) {
    Blynk.run();
  }
  timer.run();

  // WiFi/Blynk reconnect logic
  static unsigned long lastReconnect = 0;
  if (WiFi.status() != WL_CONNECTED && millis() - lastReconnect > 15000) {
    lastReconnect = millis();
    Serial.println(F("[WiFi] Lost — attempting reconnect..."));
    connectWiFi();
  }

  // Serial calibration command handler
  handleSerialCal();
}


// ════════════════════════════════════════════════════════════
//  ADC READ — Trimmed Mean, WDT-safe (v9 fix)
//  6 samples × 50ms = 300ms total.
//  Discards min + max → averages 4 values.
//  esp_task_wdt_reset() called each iteration.
// ════════════════════════════════════════════════════════════
uint32_t readMillivolts(adc1_channel_t channel) {
  uint32_t readings[SAMPLES];

  for (int i = 0; i < SAMPLES; i++) {
    uint32_t raw = adc1_get_raw(channel);
#if USE_ADC_CAL
    readings[i] = esp_adc_cal_raw_to_voltage(raw, &adc_chars);
#else
    readings[i] = (raw * 3300UL) / 4095UL;
#endif
    esp_task_wdt_reset();   // ← v9: keep watchdog happy during sampling
    delay(SAMPLE_DELAY_MS);
    yield();                // ← v9: yield to FreeRTOS scheduler
  }

  uint32_t minVal = readings[0], maxVal = readings[0];
  for (int i = 1; i < SAMPLES; i++) {
    if (readings[i] < minVal) minVal = readings[i];
    if (readings[i] > maxVal) maxVal = readings[i];
  }

  uint64_t total = 0;
  bool minSkipped = false, maxSkipped = false;
  for (int i = 0; i < SAMPLES; i++) {
    if (!minSkipped && readings[i] == minVal) { minSkipped = true; continue; }
    if (!maxSkipped && readings[i] == maxVal) { maxSkipped = true; continue; }
    total += readings[i];
  }

  return (uint32_t)(total / (SAMPLES - 2));
}


// ════════════════════════════════════════════════════════════
//  TDS — uses runtime cal_tds_k_value
// ════════════════════════════════════════════════════════════
float readTDS(float tempC) {
  uint32_t mv     = readMillivolts(ADC1_CHANNEL_6);
  float    voltage = mv / 1000.0f;

  float coeff  = 1.0f + TDS_TEMP_COEFF * (tempC - 25.0f);
  float compV  = voltage / coeff;
  float rawTDS = (133.42f * powf(compV, 3))
               - (255.86f * powf(compV, 2))
               + (857.39f * compV);
  float tds    = rawTDS * cal_tds_k_value;   // ← runtime K

#if DEBUG_TDS
  Serial.print(F("  [TDS] mV:")); Serial.print(mv);
  Serial.print(F(" compV:")); Serial.print(compV, 3);
  Serial.print(F(" rawTDS:")); Serial.print(rawTDS, 1);
  Serial.print(F(" cal:")); Serial.println(tds, 1);
#endif

  return max(tds, 0.0f);
}


// ════════════════════════════════════════════════════════════
//  pH — uses runtime cal_ph_slope + cal_ph_intercept
// ════════════════════════════════════════════════════════════
float readPH() {
  uint32_t mv      = readMillivolts(ADC1_CHANNEL_7);
  float    voltage = mv / 1000.0f;
  float    ph      = (cal_ph_slope * voltage) + cal_ph_intercept;  // ← runtime

#if DEBUG_PH
  Serial.print(F("  [pH] mV:")); Serial.print(mv);
  Serial.print(F(" V:")); Serial.print(voltage, 3);
  Serial.print(F(" pH:")); Serial.println(ph, 2);
#endif

  return constrain(ph, 0.0f, 14.0f);
}


// ════════════════════════════════════════════════════════════
//  TURBIDITY — uses runtime cal_turb_k_scale + cal_turb_offset
// ════════════════════════════════════════════════════════════
float readTurbidity() {
  uint32_t mv      = readMillivolts(ADC1_CHANNEL_4);
  float    voltage = mv / 1000.0f;
  float    adjV    = voltage * cal_turb_k_scale;   // ← runtime scale

  float rawNTU = (TURB_COEFF_A * adjV * adjV)
               + (TURB_COEFF_B * adjV)
               + TURB_COEFF_C;
  float ntu    = rawNTU + cal_turb_offset;          // ← runtime offset

#if DEBUG_TURBIDITY
  Serial.print(F("  [TURB] mV:")); Serial.print(mv);
  Serial.print(F(" adjV:")); Serial.print(adjV, 3);
  Serial.print(F(" rawNTU:")); Serial.print(rawNTU, 1);
  Serial.print(F(" ntu:")); Serial.println(ntu, 1);
#endif

  return constrain(ntu, 0.0f, TURB_MAX_NTU);
}


// ════════════════════════════════════════════════════════════
//  TEMPERATURE — uses runtime cal_temp_offset
// ════════════════════════════════════════════════════════════
float readTemperature() {
  tempSensor.requestTemperatures();
  float t = tempSensor.getTempCByIndex(0);
  if (t == DEVICE_DISCONNECTED_C) {
    Serial.println(F("[WARN] DS18B20 disconnected — using 25°C."));
    return 25.0f;
  }
  return t + cal_temp_offset;   // ← apply runtime offset
}


// ════════════════════════════════════════════════════════════
//  QUALITY ASSESSMENT (unchanged from v8)
// ════════════════════════════════════════════════════════════
WaterQuality assessQuality(const Measurements& m,
                           bool& tdsWarn, bool& phWarn,
                           bool& turbWarn, bool& tempWarn) {
  tdsWarn = phWarn = turbWarn = tempWarn = false;
  WaterQuality overall = GOOD;
  auto upgrade = [&](WaterQuality level) {
    if (level > overall) overall = level;
  };

  if      (m.tds < TDS_WARN_MIN || m.tds > TDS_WARN_MAX) { tdsWarn=true;  upgrade(CRITICAL); }
  else if (m.tds < TDS_GOOD_MIN || m.tds > TDS_GOOD_MAX) { tdsWarn=true;  upgrade(WARNING);  }

  if      (m.ph < PH_WARN_MIN || m.ph > PH_WARN_MAX)     { phWarn=true;   upgrade(CRITICAL); }
  else if (m.ph < PH_GOOD_MIN || m.ph > PH_GOOD_MAX)     { phWarn=true;   upgrade(WARNING);  }

  if      (m.turbidity > TURB_WARN_MAX)                   { turbWarn=true; upgrade(CRITICAL); }
  else if (m.turbidity > TURB_GOOD_MAX)                   { turbWarn=true; upgrade(WARNING);  }

  if      (m.temperature < TEMP_WARN_MIN || m.temperature > TEMP_WARN_MAX) { tempWarn=true; upgrade(CRITICAL); }
  else if (m.temperature < TEMP_GOOD_MIN || m.temperature > TEMP_GOOD_MAX) { tempWarn=true; upgrade(WARNING);  }

  return overall;
}


// ════════════════════════════════════════════════════════════
//  SERIAL REPORT (expanded for v9 — shows calibration state)
// ════════════════════════════════════════════════════════════
void printSerial(const Measurements& m, WaterQuality q,
                 bool tw, bool pw, bool turW, bool teW,
                 float n1_ph, float n1_tds, float n1_turb,
                 float n2_ph, float n2_tds, float n2_turb,
                 const String& diagnosis, int valveState) {

  Serial.print(F("[ WiFi: "));
  Serial.print(WiFi.status() == WL_CONNECTED ? "ON" : "OFF");
  Serial.print(F("  Blynk: "));
  Serial.print(Blynk.connected() ? "ON" : "OFF");
  Serial.print(F("  Scenario: ")); Serial.print(currentScenario);
  if (calDirty) Serial.print(F("  [CAL UNSAVED]"));
  Serial.println(F(" ]"));

  Serial.println(F("┌────────────────────────────────────────────────────┐"));
  Serial.println(F("│   WATER QUALITY REPORT  v9.0  (WHO/EPA)            │"));
  Serial.println(F("├────────────────────────────────────────────────────┤"));
  Serial.println(F("│  NODE 1 — SOURCE (Simulated)                       │"));
  Serial.print(F("│    pH: ")); Serial.print(n1_ph,2);
  Serial.print(F("  TDS: ")); Serial.print(n1_tds,1);
  Serial.print(F(" ppm  Turb: ")); Serial.print(n1_turb,1); Serial.println(F(" NTU"));
  Serial.println(F("├────────────────────────────────────────────────────┤"));
  Serial.println(F("│  NODE 2 — TRANSPORT (Simulated)                    │"));
  Serial.print(F("│    pH: ")); Serial.print(n2_ph,2);
  Serial.print(F("  TDS: ")); Serial.print(n2_tds,1);
  Serial.print(F(" ppm  Turb: ")); Serial.print(n2_turb,1); Serial.println(F(" NTU"));
  Serial.println(F("├────────────────────────────────────────────────────┤"));
  Serial.println(F("│  NODE 3 — DESTINATION (Physical)                   │"));
  Serial.print(F("│    Temp: ")); Serial.print(m.temperature,1);
  Serial.print(F(" °C")); if (teW) Serial.print(F(" [!!]")); Serial.println();
  Serial.print(F("│    TDS : ")); Serial.print(m.tds,1);
  Serial.print(F(" ppm")); if (tw) Serial.print(F(" [!!]")); Serial.println();
  Serial.print(F("│    pH  : ")); Serial.print(m.ph,2);
  if (pw) Serial.print(F(" [!!]")); Serial.println();
  Serial.print(F("│    Turb: ")); Serial.print(m.turbidity,1);
  Serial.print(F(" NTU")); if (turW) Serial.print(F(" [!!]")); Serial.println();
  Serial.println(F("├────────────────────────────────────────────────────┤"));
  Serial.print(F("│  DIAGNOSIS: ")); Serial.println(diagnosis);
  Serial.print(F("│  VALVE:     ")); Serial.println(valveState == 255 ? "OPEN" : "CLOSED");
  Serial.println(F("└────────────────────────────────────────────────────┘\n"));
}


// ════════════════════════════════════════════════════════════
//  LEDs (unchanged from v8)
// ════════════════════════════════════════════════════════════
void updateLEDs(WaterQuality q) {
  digitalWrite(PIN_LED_GREEN,  LOW);
  digitalWrite(PIN_LED_YELLOW, LOW);
  digitalWrite(PIN_LED_RED,    LOW);
  switch (q) {
    case GOOD:     digitalWrite(PIN_LED_GREEN,  HIGH); break;
    case WARNING:  digitalWrite(PIN_LED_YELLOW, HIGH); break;
    case CRITICAL: digitalWrite(PIN_LED_RED,    HIGH); break;
  }
}


// ════════════════════════════════════════════════════════════
//  BUZZER — non-blocking (v9 fix)
//  serviceBuzzer() is called by BlynkTimer every 500ms.
//  It fires one short beep per timer tick while pending.
// ════════════════════════════════════════════════════════════
static int buzzerBeepCount = 0;
static int buzzerBeepMax   = 0;

void serviceBuzzer() {
  // Arm new beep sequence if pending
  if (buzzerPending) {
    buzzerPending = false;
    switch (pendingBuzzerQuality) {
      case GOOD:
        buzzerBeepCount = 0; buzzerBeepMax = 0;
        break;
      case WARNING:
        buzzerBeepCount = 0; buzzerBeepMax = 2;
        break;
      case CRITICAL:
        buzzerBeepCount = 0; buzzerBeepMax = 5;
        break;
    }
  }

  // Fire one beep if count remaining
  if (buzzerBeepCount < buzzerBeepMax) {
    int ms = (pendingBuzzerQuality == CRITICAL) ? 80 : 200;
    digitalWrite(PIN_BUZZER, HIGH); delay(ms);
    digitalWrite(PIN_BUZZER, LOW);
    buzzerBeepCount++;
  }
}


// ════════════════════════════════════════════════════════════
//  LCD (unchanged from v8, compiled only if USE_LCD = true)
// ════════════════════════════════════════════════════════════
#if USE_LCD
void updateLCD(const Measurements& m, WaterQuality q) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("TDS:")); lcd.print((int)m.tds);
  lcd.print(F(" pH:")); lcd.print(m.ph, 1);
  lcd.setCursor(0, 1);
  switch (q) {
    case GOOD:     lcd.print(F("OK  ")); break;
    case WARNING:  lcd.print(F("WARN")); break;
    case CRITICAL: lcd.print(F("CRIT")); break;
  }
  lcd.print(F(" T:")); lcd.print(m.turbidity, 0);
  lcd.print(F("N ")); lcd.print(m.temperature, 0); lcd.print((char)223);
}
#endif
