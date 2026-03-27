#include <Arduino.h>
#include "BluetoothSerial.h"
#include "SPIFFS.h"
#include <ESP32Servo.h>
#include "HX711.h"

BluetoothSerial SerialBT;
Servo esc;
HX711 scale;

// ===================== ESC SETTINGS =====================
static const int ESC_PIN = 27;

static const int MIN_US = 1000;
static const int MAX_US = 2000;

static const float MAX_LIMIT = 0.70f;
static const unsigned long FAILSAFE_MS = 30 * 1000;
static const unsigned long ARM_MIN_HOLD_MS = 2000;

static bool motorEnabled = false;
static int requestedPercent = 0;
static unsigned long lastMotorCmdMs = 0;
static bool failsafeActive = false;

// ===================== CURRENT SENSOR SETTINGS =====================
static const int PIN_ADC = 34;
static const float VREF = 3.3f;
static const int ADC_MAX = 4095;

static float SENS_V_PER_A = 0.040f;
static const float ALPHA = 0.10f;
static const uint32_t ZERO_CAL_MS = 3000;

static const char* LOG_PATH = "/log.csv";
static bool logEnabled = true;
static uint32_t logIntervalMs = 100;
static uint32_t lastLogMs = 0;

static float filteredRaw = 0;
static float zeroRaw = 0;
static bool zeroDone = false;
static uint32_t t0 = 0;

// ===================== HX711 SETTINGS =====================
const byte HX_DOUT_PIN = 35;
const byte HX_SCK_PIN  = 32;

float calibrationFactor = 1.0f;   // counts per gram
bool isTared = false;
bool isCalibrated = false;
bool continuousRead = true;

// latest load cell values for CSV
long latestLcRaw = 0;
float latestLcGrams = 0.0f;
bool lcValid = false;

// ===================== COMMAND BUFFERS =====================
String serialBuffer = "";
String btBuffer = "";

// ===================== HELPERS =====================
bool btConnected() {
  return SerialBT.hasClient();
}

void dualPrint(const String& s) {
  Serial.print(s);
  if (btConnected()) SerialBT.print(s);
}

void dualPrintln(const String& s) {
  Serial.println(s);
  if (btConnected()) SerialBT.println(s);
}

int readAdcAveraged(int samples = 64) {
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(PIN_ADC);
    delayMicroseconds(200);
  }
  return (int)(sum / samples);
}

float rawToVolt(float raw) {
  return (raw / (float)ADC_MAX) * VREF;
}

// ===================== SPIFFS LOG =====================
void appendLogLine(const String& line) {
  File f = SPIFFS.open(LOG_PATH, FILE_APPEND);
  if (!f) return;
  f.println(line);
  f.close();
}

void clearLog() {
  if (SPIFFS.exists(LOG_PATH)) SPIFFS.remove(LOG_PATH);
  File f = SPIFFS.open(LOG_PATH, FILE_WRITE);
  if (f) {
    f.println("ms,raw,v,currentA,lc_raw,lc_grams");
    f.close();
    dualPrintln("OK: log cleared");
  } else {
    dualPrintln("ERR: Failed to create log");
  }
}

void dumpLogToBT() {
  motorEnabled = false;
  requestedPercent = 0;
  esc.writeMicroseconds(MIN_US);

  File f = SPIFFS.open(LOG_PATH, FILE_READ);
  if (!f) {
    dualPrintln("ERR: no log file");
    return;
  }

  dualPrintln("BEGIN_DUMP");
  while (f.available()) {
    String line = f.readStringUntil('\n');
    Serial.println(line);
    if (btConnected()) SerialBT.println(line);
    delay(5);
  }
  f.close();
  dualPrintln("END_DUMP");
}

// ===================== ESC CONTROL =====================
int percentToPulseUs(int percent0to100) {
  if (percent0to100 < 0) percent0to100 = 0;
  if (percent0to100 > 100) percent0to100 = 100;

  float limited = percent0to100 / 100.0f;
  if (limited > MAX_LIMIT) limited = MAX_LIMIT;

  int span = MAX_US - MIN_US;
  int us = MIN_US + (int)(limited * span + 0.5f);
  return us;
}

void setThrottlePercent(int percent0to100) {
  requestedPercent = constrain(percent0to100, 0, 100);
  int us = percentToPulseUs(requestedPercent);
  esc.writeMicroseconds(us);

  dualPrint("OK Throttle=");
  dualPrint(String(requestedPercent));
  dualPrint("% (capped 70%) pulse=");
  dualPrint(String(us));
  dualPrintln("us");
}

void armSequence() {
  esc.writeMicroseconds(MIN_US);
  dualPrintln("Arming: holding MIN throttle...");
  delay(ARM_MIN_HOLD_MS);
  dualPrintln("Armed (motor enabled).");
}

void printMotorStatus() {
  int us = percentToPulseUs(requestedPercent);
  dualPrint("ENABLED=");
  dualPrint(motorEnabled ? "YES" : "NO");
  dualPrint(" | REQ=");
  dualPrint(String(requestedPercent));
  dualPrint("% | PULSE=");
  dualPrint(String(us));
  dualPrintln("us (capped 70%)");
}

// ===================== HX711 HELPERS =====================
long getStableRaw(int samples = 30, int delayMs = 5) {
  long sum = 0;
  int count = 0;

  for (int i = 0; i < samples; i++) {
    if (scale.is_ready()) {
      sum += scale.read();
      count++;
    }
    delay(delayMs);
  }

  if (count == 0) return 0;
  return sum / count;
}

void updateLoadcellValues() {
  if (!continuousRead) return;

  if (!scale.is_ready()) {
    lcValid = false;
    return;
  }

  long raw = getStableRaw(10, 2);
  long offset = scale.get_offset();

  latestLcRaw = raw;

  if (isCalibrated && calibrationFactor != 0.0f) {
    latestLcGrams = (raw - offset) / calibrationFactor;
    lcValid = true;
  } else {
    latestLcGrams = 0.0f;
    lcValid = false;
  }
}

void printLoadcellStatus() {
  dualPrintln("");
  dualPrintln("=== LOAD CELL STATUS ===");
  dualPrint("HX711 ready: ");
  dualPrintln(scale.is_ready() ? "YES" : "NO");

  dualPrint("Tared: ");
  dualPrintln(isTared ? "YES" : "NO");

  dualPrint("Calibrated: ");
  dualPrintln(isCalibrated ? "YES" : "NO");

  dualPrint("Offset: ");
  dualPrintln(String(scale.get_offset()));

  dualPrint("Calibration factor (counts/gram): ");
  dualPrintln(String(calibrationFactor, 6));

  dualPrint("Continuous read: ");
  dualPrintln(continuousRead ? "ON" : "OFF");

  dualPrint("Latest LC raw: ");
  dualPrintln(String(latestLcRaw));

  dualPrint("Latest LC grams: ");
  if (lcValid) dualPrintln(String(latestLcGrams, 2));
  else dualPrintln("not calibrated");

  dualPrintln("");
}

void doTare() {
  if (!scale.is_ready()) {
    dualPrintln("ERROR: HX711 not ready");
    return;
  }

  dualPrintln("Taring... remove all weight.");
  delay(1000);

  long offset = getStableRaw(80, 5);
  scale.set_offset(offset);
  isTared = true;

  latestLcRaw = 0;
  latestLcGrams = 0;
  lcValid = false;

  dualPrint("Tare complete. Offset = ");
  dualPrintln(String(offset));
}

void doCalibrate(float knownWeight) {
  if (!scale.is_ready()) {
    dualPrintln("ERROR: HX711 not ready");
    return;
  }

  if (!isTared) {
    dualPrintln("ERROR: Please tare first using TARE");
    return;
  }

  if (knownWeight <= 0.0f) {
    dualPrintln("ERROR: Weight must be > 0");
    return;
  }

  dualPrint("Calibrating with known weight = ");
  dualPrint(String(knownWeight, 3));
  dualPrintln(" g");
  dualPrintln("Make sure the weight is already on the load cell.");
  delay(1500);

  long rawWithWeight = getStableRaw(100, 5);
  long offset = scale.get_offset();
  long netRaw = rawWithWeight - offset;

  if (netRaw == 0) {
    dualPrintln("ERROR: Net raw is zero. Calibration failed.");
    return;
  }

  calibrationFactor = (float)netRaw / knownWeight;
  isCalibrated = true;

  latestLcRaw = rawWithWeight;
  latestLcGrams = knownWeight;
  lcValid = true;

  dualPrint("Raw with weight: ");
  dualPrintln(String(rawWithWeight));
  dualPrint("Offset: ");
  dualPrintln(String(offset));
  dualPrint("Net raw: ");
  dualPrintln(String(netRaw));
  dualPrint("Calibration factor: ");
  dualPrintln(String(calibrationFactor, 6));
  dualPrintln("Calibration complete.");
}

// ===================== COMMAND HANDLER =====================
void printHelp() {
  dualPrintln("");
  dualPrintln("=== COMMANDS ===");
  dualPrintln("Motor:");
  dualPrintln("  ON");
  dualPrintln("  OFF");
  dualPrintln("  SPEED <0-100>");
  dualPrintln("  STOP");
  dualPrintln("  STATUS");
  dualPrintln("");
  dualPrintln("Current logger:");
  dualPrintln("  START or LOGSTART");
  dualPrintln("  LOGSTOP");
  dualPrintln("  CLEAR");
  dualPrintln("  DUMP");
  dualPrintln("  SENS=0.040");
  dualPrintln("  RATE=100");
  dualPrintln("");
  dualPrintln("Load cell:");
  dualPrintln("  TARE");
  dualPrintln("  CAL=100");
  dualPrintln("  LCSTATUS");
  dualPrintln("  READON");
  dualPrintln("  READOFF");
  dualPrintln("  HELP");
  dualPrintln("");
}

void handleCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  String up = cmd;
  up.toUpperCase();

  dualPrint("CMD: ");
  dualPrintln(cmd);

  // -------- Motor --------
  if (up == "ON") {
    motorEnabled = true;
    failsafeActive = false;
    lastMotorCmdMs = millis();
    setThrottlePercent(0);
    armSequence();
    setThrottlePercent(0);
    return;
  }

  if (up == "OFF") {
    motorEnabled = false;
    failsafeActive = false;
    setThrottlePercent(0);
    dualPrintln("OK OFF (motor disabled, throttle MIN)");
    return;
  }

  if (up == "STATUS") {
    printMotorStatus();
    dualPrintln(String("LOGGING=") + (logEnabled ? "ON" : "OFF") +
                " | RATE=" + String(logIntervalMs) + "ms | SENS=" + String(SENS_V_PER_A, 5));
    dualPrintln(String("ZERO=") + (zeroDone ? "DONE" : "CAL") + " | zeroRaw=" + String(zeroRaw, 1));
    dualPrintln(String("FAILSAFE=") + (failsafeActive ? "ACTIVE" : "NORMAL"));
    return;
  }

  if (up == "STOP") {
    if (motorEnabled) {
      lastMotorCmdMs = millis();
      failsafeActive = false;
      setThrottlePercent(0);
    } else {
      logEnabled = false;
      dualPrintln("OK: logging OFF");
    }
    return;
  }

  if (up.startsWith("SPEED")) {
    if (!motorEnabled) {
      dualPrintln("ERR: Send ON first");
      return;
    }

    int spaceIdx = cmd.indexOf(' ');
    if (spaceIdx < 0) {
      dualPrintln("ERR: use SPEED <0-100>");
      return;
    }

    int val = cmd.substring(spaceIdx + 1).toInt();
    val = constrain(val, 0, 100);
    lastMotorCmdMs = millis();
    failsafeActive = false;
    setThrottlePercent(val);
    return;
  }

  // -------- Current logger --------
  if (up == "START" || up == "LOGSTART") {
    logEnabled = true;
    dualPrintln("OK: logging ON");
    return;
  }

  if (up == "LOGSTOP") {
    logEnabled = false;
    dualPrintln("OK: logging OFF");
    return;
  }

  if (up == "CLEAR") {
    clearLog();
    return;
  }

  if (up == "DUMP") {
    dumpLogToBT();
    return;
  }

  if (up.startsWith("SENS=")) {
    float v = cmd.substring(5).toFloat();
    if (v > 0.0001f) {
      SENS_V_PER_A = v;
      dualPrintln("OK: SENS=" + String(SENS_V_PER_A, 5));
    } else {
      dualPrintln("ERR: SENS must be > 0");
    }
    return;
  }

  if (up.startsWith("RATE=")) {
    int r = cmd.substring(5).toInt();
    if (r >= 20) {
      logIntervalMs = (uint32_t)r;
      dualPrintln("OK: RATE=" + String(logIntervalMs));
    } else {
      dualPrintln("ERR: RATE must be >= 20ms");
    }
    return;
  }

  // -------- Load cell --------
  if (up == "TARE") {
    doTare();
    return;
  }

  if (up.startsWith("CAL=")) {
    float knownWeight = cmd.substring(4).toFloat();
    doCalibrate(knownWeight);
    return;
  }

  if (up == "LCSTATUS") {
    printLoadcellStatus();
    return;
  }

  if (up == "READON") {
    continuousRead = true;
    dualPrintln("Load cell reading in CSV: ON");
    return;
  }

  if (up == "READOFF") {
    continuousRead = false;
    dualPrintln("Load cell reading in CSV: OFF");
    return;
  }

  if (up == "HELP") {
    printHelp();
    return;
  }

  dualPrintln("Unknown command. Type HELP");
}

// ===================== INPUT READERS =====================
void readSerialCommands() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (serialBuffer.length() > 0) {
        handleCommand(serialBuffer);
        serialBuffer = "";
      }
    } else {
      serialBuffer += c;
    }
  }
}

void readBluetoothCommands() {
  while (SerialBT.available()) {
    char c = SerialBT.read();

    if (c == '\n' || c == '\r') {
      if (btBuffer.length() > 0) {
        handleCommand(btBuffer);
        btBuffer = "";
      }
    } else {
      btBuffer += c;
    }
  }
}

// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);

  // ESC init
  esc.setPeriodHertz(50);
  esc.attach(ESC_PIN, MIN_US, MAX_US);
  esc.writeMicroseconds(MIN_US);
  delay(ARM_MIN_HOLD_MS);

  // ADC init
  pinMode(PIN_ADC, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_ADC, ADC_11db);

  // SPIFFS init
  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS mount failed");
  } else if (!SPIFFS.exists(LOG_PATH)) {
    clearLog();
  }

  // Bluetooth init
  SerialBT.begin("ESP32_Motor_Current_LoadCell");
  Serial.println("Bluetooth ready: ESP32_Motor_Current_LoadCell");

  // Current init
  filteredRaw = readAdcAveraged(64);
  t0 = millis();

  // HX711 init
  scale.begin(HX_DOUT_PIN, HX_SCK_PIN);

  lastMotorCmdMs = millis();
  lastLogMs = millis();

  dualPrintln("Ready.");
  dualPrintln("Bluetooth device: ESP32_Motor_Current_LoadCell");
  dualPrintln("CSV header:");
  dualPrintln("ms,raw,v,currentA,lc_raw,lc_grams");
  printHelp();

  if (scale.is_ready()) {
    dualPrintln("HX711 is ready.");
  } else {
    dualPrintln("WARNING: HX711 not ready.");
  }
}

// ===================== LOOP =====================
void loop() {
  // ESC failsafe
  if (motorEnabled) {
    if (millis() - lastMotorCmdMs > FAILSAFE_MS) {
      esc.writeMicroseconds(MIN_US);
      motorEnabled = false;
      requestedPercent = 0;

      if (!failsafeActive) {
        dualPrintln("⚠️ FAILSAFE at ms=" + String(millis()));
        dualPrintln("Motor disabled due to command timeout.");
        failsafeActive = true;
      }
    } else {
      failsafeActive = false;
    }
  } else {
    esc.writeMicroseconds(MIN_US);
  }

  // command input
  readSerialCommands();
  readBluetoothCommands();

  // current sensor
  int raw = readAdcAveraged(64);
  filteredRaw = (1.0f - ALPHA) * filteredRaw + ALPHA * raw;

  // current zero calibration
  if (!zeroDone) {
    zeroRaw = (zeroRaw * 0.95f) + (filteredRaw * 0.05f);
    if (millis() - t0 > ZERO_CAL_MS) {
      zeroDone = true;
      dualPrintln("Zero DONE. Baseline raw: " + String(zeroRaw, 1));
    }
  }

  // update load cell before CSV line is generated
  updateLoadcellValues();

  // combined CSV stream + log
  uint32_t now = millis();
  if (zeroDone && (now - lastLogMs >= logIntervalMs)) {
    lastLogMs = now;

    float v = rawToVolt(filteredRaw);
    float v0 = rawToVolt(zeroRaw);
    float currentA = (v - v0) / SENS_V_PER_A;

    String lcRawStr = continuousRead ? String(latestLcRaw) : "NA";
    String lcGramStr = (continuousRead && lcValid) ? String(latestLcGrams, 2) : "NA";

    String line = String(now) + "," +
                  String((int)filteredRaw) + "," +
                  String(v, 3) + "," +
                  String(currentA, 3) + "," +
                  lcRawStr + "," +
                  lcGramStr;

    Serial.println(line);
    if (btConnected()) SerialBT.println(line);
    if (logEnabled) appendLogLine(line);
  }
}