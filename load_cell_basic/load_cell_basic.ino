#include "HX711.h"
#include <BluetoothSerial.h>

BluetoothSerial SerialBT;
HX711 scale;

// =========================
// Pin setup
// =========================
const byte DOUT_PIN = 25;
const byte SCK_PIN  = 26;

// =========================
// Globals
// =========================
float calibrationFactor = 1.0;   // counts per gram
bool isTared = false;
bool isCalibrated = false;
bool continuousRead = true;

String serialBuffer = "";
String btBuffer = "";

// =========================
// Helper print functions
// =========================
void dualPrint(const String &msg) {
  Serial.print(msg);
  SerialBT.print(msg);
}

void dualPrintln(const String &msg) {
  Serial.println(msg);
  SerialBT.println(msg);
}

// =========================
// Read stable average raw
// =========================
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

// =========================
// Command handlers
// =========================
void printHelp() {
  dualPrintln("");
  dualPrintln("=== COMMANDS ===");
  dualPrintln("HELP       -> show commands");
  dualPrintln("TARE       -> tare with no load");
  dualPrintln("CAL=100    -> calibrate using 100 g known weight");
  dualPrintln("STATUS     -> show current status");
  dualPrintln("READON     -> start continuous reading");
  dualPrintln("READOFF    -> stop continuous reading");
  dualPrintln("");
  dualPrintln("How to calibrate:");
  dualPrintln("1) Remove all weight");
  dualPrintln("2) Send: TARE");
  dualPrintln("3) Put known weight");
  dualPrintln("4) Send: CAL=your_weight_in_grams");
  dualPrintln("");
}

void printStatus() {
  dualPrintln("");
  dualPrintln("=== STATUS ===");
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
  dualPrintln("");
}

void doTare() {
  if (!scale.is_ready()) {
    dualPrintln("ERROR: HX711 not ready");
    return;
  }

  dualPrintln("");
  dualPrintln("Taring... remove all weight now.");
  delay(1000);

  long offset = getStableRaw(80, 5);
  scale.set_offset(offset);

  isTared = true;

  dualPrint("Tare complete. Offset = ");
  dualPrintln(String(offset));
  dualPrintln("");
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
    dualPrintln("ERROR: Weight must be greater than 0");
    return;
  }

  dualPrintln("");
  dualPrint("Calibrating with known weight = ");
  dualPrint( String(knownWeight, 3) );
  dualPrintln(" g");
  dualPrintln("Make sure the weight is already placed on the load cell.");
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

  dualPrint("Raw with weight: ");
  dualPrintln(String(rawWithWeight));

  dualPrint("Offset: ");
  dualPrintln(String(offset));

  dualPrint("Net raw: ");
  dualPrintln(String(netRaw));

  dualPrint("New calibration factor (counts/gram): ");
  dualPrintln(String(calibrationFactor, 6));
  dualPrintln("Calibration complete.");
  dualPrintln("");
}

void processCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();

  if (cmd.length() == 0) return;

  dualPrint("Received command: ");
  dualPrintln(cmd);

  if (cmd == "HELP") {
    printHelp();
  }
  else if (cmd == "TARE") {
    doTare();
  }
  else if (cmd.startsWith("CAL=")) {
    String val = cmd.substring(4);
    float knownWeight = val.toFloat();
    doCalibrate(knownWeight);
  }
  else if (cmd == "STATUS") {
    printStatus();
  }
  else if (cmd == "READON") {
    continuousRead = true;
    dualPrintln("Continuous reading: ON");
  }
  else if (cmd == "READOFF") {
    continuousRead = false;
    dualPrintln("Continuous reading: OFF");
  }
  else {
    dualPrintln("Unknown command. Type HELP");
  }
}

// =========================
// Read commands from Serial
// =========================
void readSerialCommands() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (serialBuffer.length() > 0) {
        processCommand(serialBuffer);
        serialBuffer = "";
      }
    } else {
      serialBuffer += c;
    }
  }
}

// =========================
// Read commands from Bluetooth
// =========================
void readBluetoothCommands() {
  while (SerialBT.available()) {
    char c = SerialBT.read();

    if (c == '\n' || c == '\r') {
      if (btBuffer.length() > 0) {
        processCommand(btBuffer);
        btBuffer = "";
      }
    } else {
      btBuffer += c;
    }
  }
}

// =========================
// Setup
// =========================
void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_LoadCell");

  scale.begin(DOUT_PIN, SCK_PIN);

  dualPrintln("");
  dualPrintln("=== ESP32 HX711 Bluetooth Load Cell ===");
  dualPrintln("Bluetooth device name: ESP32_LoadCell");

  if (scale.is_ready()) {
    dualPrintln("HX711 is ready.");
  } else {
    dualPrintln("WARNING: HX711 not ready.");
  }

  printHelp();
}

// =========================
// Main loop
// =========================
void loop() {
  readSerialCommands();
  readBluetoothCommands();

  static unsigned long lastRead = 0;

  if (continuousRead && millis() - lastRead >= 500) {
    lastRead = millis();

    if (!scale.is_ready()) {
      dualPrintln("HX711 not ready");
      return;
    }

    long raw = getStableRaw(10, 2);
    long offset = scale.get_offset();

    dualPrint("RAW: ");
    dualPrint(raw >= 0 ? String(raw) : String(raw));
    
    if (isCalibrated && calibrationFactor != 0.0f) {
      float grams = (raw - offset) / calibrationFactor;
      dualPrint(" | grams: ");
      dualPrintln(String(grams, 2));
    } else {
      dualPrintln(" | grams: not calibrated");
    }
  }
}