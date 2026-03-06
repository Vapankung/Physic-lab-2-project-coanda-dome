#include <Arduino.h>
#include "BluetoothSerial.h"
#include "SPIFFS.h"

BluetoothSerial SerialBT;

// ===== Hardware setup =====
static const int PIN_ADC = 34;        
static const float VREF = 3.3f;
static const int ADC_MAX = 4095;

// ===== Sensor sensitivity =====
static float SENS_V_PER_A = 0.040f;   // 40 mV/A example
static const float ALPHA = 0.10f;     // Filter strength
static const uint32_t ZERO_CAL_MS = 3000;

// Logging
static const char* LOG_PATH = "/log.csv";
static bool logEnabled = true;        
static uint32_t logIntervalMs = 100;  
static uint32_t lastLogMs = 0;

float filteredRaw = 0;
float zeroRaw = 0;
bool zeroDone = false;
uint32_t t0 = 0;

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

void btPrintln(const String& s) {
  Serial.println(s);
  if (SerialBT.connected()) { // Only send if a device is connected
    SerialBT.println(s);      
  }
}

void appendLogLine(const String& line) {
  File f = SPIFFS.open(LOG_PATH, FILE_APPEND);
  if (!f) return;
  f.println(line);
  f.close();
}

void dumpLogToBT() {
  File f = SPIFFS.open(LOG_PATH, FILE_READ);
  if (!f) {
    btPrintln("ERR: no log file");
    return;
  }

  btPrintln("BEGIN_DUMP");
  while (f.available()) {
    String line = f.readStringUntil('\n');
    SerialBT.println(line);  
    delay(5); // Increased delay for Windows buffer stability
  }
  f.close();
  btPrintln("END_DUMP");
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

void handleCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();

  if (cmd == "START") { logEnabled = true; btPrintln("OK: logging ON"); }
  else if (cmd == "STOP") { logEnabled = false; btPrintln("OK: logging OFF"); }
  else if (cmd == "CLEAR") { clearLog(); }
  else if (cmd == "DUMP") { dumpLogToBT(); }
  else if (cmd.startsWith("SENS=")) {
    float v = cmd.substring(5).toFloat();
    if (v > 0.0001f) { SENS_V_PER_A = v; btPrintln("OK: SENS=" + String(SENS_V_PER_A, 4)); }
  } else if (cmd.startsWith("RATE=")) {
    int r = cmd.substring(5).toInt();
    if (r >= 20) { logIntervalMs = (uint32_t)r; btPrintln("OK: RATE=" + String(logIntervalMs)); }
  } else {
    btPrintln("CMD? Use: START|STOP|CLEAR|DUMP|SENS=0.040|RATE=100");
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_ADC, INPUT); // Explicitly set pin mode

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_ADC, ADC_11db);

  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS mount failed");
  } else if (!SPIFFS.exists(LOG_PATH)) {
    clearLog();
  }

  // Bluetooth Device Name
  SerialBT.begin("ESP32_CurrentLogger"); 
  Serial.println("Bluetooth Started. Connect via COM port on Windows.");

  filteredRaw = readAdcAveraged(64);
  t0 = millis();
}

void loop() {
  // 1. Check for Bluetooth Commands
  if (SerialBT.available()) {
    String cmd = SerialBT.readStringUntil('\n');
    handleCommand(cmd);
  }

  // 2. Sampling and Filtering
  int raw = readAdcAveraged(64);
  filteredRaw = (1.0f - ALPHA) * filteredRaw + ALPHA * raw;

  // 3. Zero Calibration Logic
  if (!zeroDone) {
    zeroRaw = (zeroRaw * 0.95f) + (filteredRaw * 0.05f);
    if (millis() - t0 > ZERO_CAL_MS) {
      zeroDone = true;
      btPrintln("Zero DONE. Baseline: " + String(zeroRaw));
    }
  }

  // 4. Data Streaming & Logging
  uint32_t now = millis();
  if (zeroDone && (now - lastLogMs >= logIntervalMs)) {
    lastLogMs = now;
    float v = rawToVolt(filteredRaw);
    float v0 = rawToVolt(zeroRaw);
    float currentA = (v - v0) / SENS_V_PER_A;

    String line = String(now) + "," + String((int)filteredRaw) + "," + String(v, 3) + "," + String(currentA, 3);
    
    Serial.println(line);
    if (SerialBT.connected()) SerialBT.println(line);
    if (logEnabled) appendLogLine(line);
  }
}