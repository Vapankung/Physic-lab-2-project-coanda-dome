/*
  ESP32 Motor (ESC) + Current Sensor Logger over Bluetooth
  --------------------------------------------------------
  - Controls ESC via Bluetooth commands (ON/OFF/SPEED/STOP/STATUS)
  - Measures current from analog sensor (e.g., ACS758 analog output) on ADC pin
  - Streams CSV lines: ms,raw,v,currentA
  - Logs to SPIFFS /log.csv and supports CLEAR / DUMP
  - Supports calibration commands: SENS=0.040  (V per A), RATE=100 (ms)

  Notes:
  - Command "STOP" is ambiguous (motor stop vs log stop).
    Rule used here:
      * If motor is enabled -> STOP means motor throttle to 0
      * If motor is NOT enabled -> STOP means logging OFF
    Also added aliases: LOGSTART / LOGSTOP to avoid confusion.

  Wiring reminders (common):
  - Current sensor analog OUT -> GPIO34 (ADC1)
  - Sensor GND -> ESP32 GND
  - Sensor VCC -> 3.3V (ONLY if your sensor supports 3.3V). Many ACS sensors need 5V.
    If your sensor output can exceed 3.3V, you MUST scale it down (divider/op-amp) to protect ESP32 ADC.
*/

#include <Arduino.h>
#include "BluetoothSerial.h"
#include "SPIFFS.h"
#include <ESP32Servo.h>

BluetoothSerial SerialBT;
Servo esc;

// ===================== ESC SETTINGS =====================
static const int ESC_PIN = 27;

static const int MIN_US = 1000;
static const int MAX_US = 2000;

static const float MAX_LIMIT = 0.30f;              // cap at 30%
static const unsigned long FAILSAFE_MS = 2000;     // no command -> MIN
static const unsigned long ARM_MIN_HOLD_MS = 2000; // arming hold MIN

// ESC state
static bool motorEnabled = false;
static int requestedPercent = 0;     // 0..100
static unsigned long lastMotorCmdMs = 0;

// ===================== CURRENT SENSOR SETTINGS =====================
static const int PIN_ADC = 34;        // ADC1 recommended
static const float VREF = 3.3f;
static const int ADC_MAX = 4095;

// Adjust after comparing with known current
static float SENS_V_PER_A = 0.040f;   // 40 mV/A example (depends on your sensor + conditioning)

// Filter
static const float ALPHA = 0.10f;

// Zero calibration
static const uint32_t ZERO_CAL_MS = 3000;

// Logging
static const char* LOG_PATH = "/log.csv";
static bool logEnabled = true;        // logging ON by default
static uint32_t logIntervalMs = 100;  // 10 Hz
static uint32_t lastLogMs = 0;

// Current state
static float filteredRaw = 0;
static float zeroRaw = 0;
static bool zeroDone = false;
static uint32_t t0 = 0;

// ===================== HELPERS =====================
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

bool btConnected() {
  // SerialBT.connected() exists in some cores; hasClient() in others.
  // Using hasClient() (works well for SPP).
  return SerialBT.hasClient();
}

void btPrintln(const String& s) {
  Serial.println(s);
  if (btConnected()) SerialBT.println(s);
}

void btPrint(const String& s) {
  Serial.print(s);
  if (btConnected()) SerialBT.print(s);
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
    f.println("ms,raw,v,currentA");
    f.close();
    btPrintln("OK: log cleared");
  } else {
    btPrintln("ERR: Failed to create log");
  }
}

void dumpLogToBT() {
  // SAFETY: ensure motor is at MIN while we block sending the dump
  motorEnabled = false;
  esc.writeMicroseconds(MIN_US);

  File f = SPIFFS.open(LOG_PATH, FILE_READ);
  if (!f) {
    btPrintln("ERR: no log file");
    return;
  }

  btPrintln("BEGIN_DUMP");
  while (f.available()) {
    String line = f.readStringUntil('\n');
    if (btConnected()) SerialBT.println(line);
    delay(5); // helps stability on some PC BT serial buffers
  }
  f.close();
  btPrintln("END_DUMP");
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

  // echo
  btPrint("OK Throttle=");
  btPrint(String(requestedPercent));
  btPrint("% (capped 30%) pulse=");
  btPrint(String(us));
  btPrintln("us");
}

void armSequence() {
  esc.writeMicroseconds(MIN_US);
  btPrintln("Arming: holding MIN throttle...");
  delay(ARM_MIN_HOLD_MS);
  btPrintln("Armed (motor enabled).");
}

void printMotorStatus() {
  int us = percentToPulseUs(requestedPercent);
  btPrint("ENABLED=");
  btPrint(motorEnabled ? "YES" : "NO");
  btPrint(" | REQ=");
  btPrint(String(requestedPercent));
  btPrint("% | PULSE=");
  btPrint(String(us));
  btPrintln("us (capped 30%)");
}

// ===================== COMMAND HANDLER =====================
void handleCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  // Keep a copy for numeric parsing, but compare uppercase for keywords
  String up = cmd;
  up.toUpperCase();

  // -------- ESC commands --------
  if (up == "ON") {
    motorEnabled = true;
    lastMotorCmdMs = millis();
    setThrottlePercent(0);
    armSequence();
    setThrottlePercent(0);
    return;
  }

  if (up == "OFF") {
    motorEnabled = false;
    setThrottlePercent(0);
    btPrintln("OK OFF (motor disabled, throttle MIN)");
    return;
  }

  if (up == "STATUS") {
    printMotorStatus();
    btPrintln(String("LOGGING=") + (logEnabled ? "ON" : "OFF") +
              " | RATE=" + String(logIntervalMs) + "ms | SENS=" + String(SENS_V_PER_A, 5));
    btPrintln(String("ZERO=") + (zeroDone ? "DONE" : "CAL") + " | zeroRaw=" + String(zeroRaw, 1));
    return;
  }

  // STOP ambiguity rule:
  // - if motor enabled => STOP motor
  // - else => STOP logging
  if (up == "STOP") {
    if (motorEnabled) {
      lastMotorCmdMs = millis();
      setThrottlePercent(0);
      return;
    } else {
      logEnabled = false;
      btPrintln("OK: logging OFF");
      return;
    }
  }

  if (up.startsWith("SPEED")) {
    if (!motorEnabled) {
      btPrintln("ERR: Send ON first");
      return;
    }
    int spaceIdx = cmd.indexOf(' ');
    if (spaceIdx < 0) {
      btPrintln("ERR: use SPEED <0-100>");
      return;
    }
    int val = cmd.substring(spaceIdx + 1).toInt();
    val = constrain(val, 0, 100);
    lastMotorCmdMs = millis();
    setThrottlePercent(val);
    return;
  }

  // -------- Logger commands --------
  if (up == "START" || up == "LOGSTART") {
    logEnabled = true;
    btPrintln("OK: logging ON");
    return;
  }

  if (up == "LOGSTOP") {
    logEnabled = false;
    btPrintln("OK: logging OFF");
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
      btPrintln("OK: SENS=" + String(SENS_V_PER_A, 5));
    } else {
      btPrintln("ERR: SENS must be > 0");
    }
    return;
  }

  if (up.startsWith("RATE=")) {
    int r = cmd.substring(5).toInt();
    if (r >= 20) {
      logIntervalMs = (uint32_t)r;
      btPrintln("OK: RATE=" + String(logIntervalMs));
    } else {
      btPrintln("ERR: RATE must be >= 20ms");
    }
    return;
  }

  // Help
  btPrintln("CMD? Motor: ON | OFF | SPEED <0-100> | STOP | STATUS");
  btPrintln("CMD? Log: START(or LOGSTART) | LOGSTOP | CLEAR | DUMP | SENS=0.040 | RATE=100");
}

// ===================== SETUP / LOOP =====================
void setup() {
  Serial.begin(115200);

  // --- ESC init ---
  esc.setPeriodHertz(50);
  esc.attach(ESC_PIN, MIN_US, MAX_US);
  esc.writeMicroseconds(MIN_US);
  delay(ARM_MIN_HOLD_MS);

  // --- ADC init ---
  pinMode(PIN_ADC, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_ADC, ADC_11db);

  // --- SPIFFS init ---
  if (!SPIFFS.begin(true)) {
    btPrintln("SPIFFS mount failed");
  } else if (!SPIFFS.exists(LOG_PATH)) {
    clearLog();
  }

  // --- Bluetooth init ---
  SerialBT.begin("ESP32_Motor_Current");
  Serial.println("Bluetooth ready: ESP32_Motor_Current");

  // --- Current filter init ---
  filteredRaw = readAdcAveraged(64);
  t0 = millis();

  lastMotorCmdMs = millis();
  lastLogMs = millis();

  btPrintln("Ready.");
  btPrintln("Motor: ON/OFF/SPEED n/STOP/STATUS");
  btPrintln("Log: START/LOGSTOP/CLEAR/DUMP/SENS=..../RATE=....");
}

void loop() {
  // ----- ESC failsafe -----
  if (motorEnabled) {
    if (millis() - lastMotorCmdMs > FAILSAFE_MS) {
      esc.writeMicroseconds(MIN_US);
    }
  } else {
    esc.writeMicroseconds(MIN_US);
  }

  // ----- Read Bluetooth commands -----
  if (SerialBT.available()) {
    String cmd = SerialBT.readStringUntil('\n');
    Serial.print("BT: ");
    Serial.println(cmd);
    handleCommand(cmd);
  }

  // ----- Current sampling & filtering -----
  int raw = readAdcAveraged(64);
  filteredRaw = (1.0f - ALPHA) * filteredRaw + ALPHA * raw;

  // ----- Zero calibration (first few seconds) -----
  if (!zeroDone) {
    zeroRaw = (zeroRaw * 0.95f) + (filteredRaw * 0.05f);
    if (millis() - t0 > ZERO_CAL_MS) {
      zeroDone = true;
      btPrintln("Zero DONE. Baseline raw: " + String(zeroRaw, 1));
    }
  }

  // ----- Stream + log -----
  uint32_t now = millis();
  if (zeroDone && (now - lastLogMs >= logIntervalMs)) {
    lastLogMs = now;

    float v = rawToVolt(filteredRaw);
    float v0 = rawToVolt(zeroRaw);
    float currentA = (v - v0) / SENS_V_PER_A;

    // CSV line (same format as your logger)
    String line = String(now) + "," +
                  String((int)filteredRaw) + "," +
                  String(v, 3) + "," +
                  String(currentA, 3);

    Serial.println(line);
    if (btConnected()) SerialBT.println(line);
    if (logEnabled) appendLogLine(line);
  }
}