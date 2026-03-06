#include "HX711.h"
#include <EEPROM.h>

const int LOADCELL_DOUT_PIN = 2;
const int LOADCELL_SCK_PIN  = 3;

HX711 scale;

// EEPROM address to store float calibration factor
const int EEPROM_ADDR = 0;

float calibFactor = 1.0f;          // grams per raw-unit (depends on library convention)
bool streaming = false;
const unsigned long STREAM_MS = 50; // 20 Hz

unsigned long lastStream = 0;

float readEEPROMFloat(int addr) {
  float val;
  EEPROM.get(addr, val);
  // Basic sanity check
  if (!isfinite(val) || val == 0.0f) return 1.0f;
  return val;
}

void writeEEPROMFloat(int addr, float val) {
  EEPROM.put(addr, val);
}

long readRawAverage(int n) {
  // HX711 library: scale.read_average(n) gives raw average
  return scale.read_average(n);
}

void setup() {
  Serial.begin(115200);
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

  // Load saved calibration factor
  calibFactor = readEEPROMFloat(EEPROM_ADDR);

  // IMPORTANT:
  // In HX711 library, set_scale(factor) means: get_units() = (raw - offset) / factor
  // So "factor" is typically counts-per-unit.
  // We'll store "counts-per-gram" as calibFactor.
  scale.set_scale(calibFactor);

  // Tare at startup
  scale.tare();

  Serial.println("READY");
  Serial.print("CAL=");
  Serial.println(calibFactor, 6);
}

String readLine() {
  static String s = "";
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      String out = s;
      s = "";
      out.trim();
      return out;
    } else if (c != '\r') {
      s += c;
    }
  }
  return "";
}

void processCommand(const String &cmd) {
  if (cmd.length() == 0) return;

  if (cmd == "TARE") {
    scale.tare();
    Serial.println("OK:TARE");
    return;
  }

  if (cmd == "RAW") {
    long raw = readRawAverage(1000);
    Serial.print("RAW:");
    Serial.println(raw);
    return;
  }

  if (cmd.startsWith("CAL:")) {
    String valStr = cmd.substring(4);
    float f = valStr.toFloat();
    if (isfinite(f) && f != 0.0f) {
      calibFactor = f;
      scale.set_scale(calibFactor);
      writeEEPROMFloat(EEPROM_ADDR, calibFactor);
      Serial.print("OK:CAL:");
      Serial.println(calibFactor, 6);
    } else {
      Serial.println("ERR:CAL");
    }
    return;
  }

  if (cmd.startsWith("STREAM:")) {
    int on = cmd.substring(7).toInt();
    streaming = (on != 0);
    Serial.print("OK:STREAM:");
    Serial.println(streaming ? 1 : 0);
    return;
  }

  Serial.println("ERR:UNKNOWN");
}

void loop() {
  // Handle incoming serial commands
  String cmd = readLine();
  if (cmd.length() > 0) processCommand(cmd);

  // Stream grams
  if (streaming) {
    unsigned long now = millis();
    if (now - lastStream >= STREAM_MS) {
      lastStream = now;
      // get_units() returns (raw - offset) / scale_factor
      // If scale_factor = counts-per-gram, then result is grams.
      float grams = scale.get_units(5);
      Serial.println(grams, 2);
    }
  }
}