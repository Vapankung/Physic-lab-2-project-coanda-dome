#include "HX711.h"

HX711 scale;

// ===== Pins for Arduino Uno =====
const byte DOUT_PIN = 25;
const byte SCK_PIN  = 26;

// ===== Calibration =====
float knownWeightGrams = 100.0;     // change when needed
float calibrationFactor = 1.0;      // counts per gram

// ===== State =====
bool tareDone = false;
bool calibrated = false;
bool continuousRead = false;

String inputBuffer = "";

// ===== Accuracy settings =====
const int FAST_SAMPLES = 100;        // normal reading
const int ACCURATE_SAMPLES = 200;    // tare / calibration
const float SMOOTH_ALPHA = 0.15;    // lower = smoother, slower
bool smoothedValid = false;
float smoothedGrams = 0.0;

// --------------------------------------------------
// Utility: sort array
// --------------------------------------------------
void sortArray(long arr[], int n) {
  for (int i = 0; i < n - 1; i++) {
    for (int j = i + 1; j < n; j++) {
      if (arr[j] < arr[i]) {
        long temp = arr[i];
        arr[i] = arr[j];
        arr[j] = temp;
      }
    }
  }
}

// --------------------------------------------------
// Read multiple HX711 samples and return trimmed mean
// Removes lowest and highest values to reduce noise
// trim = number removed from each side
// --------------------------------------------------
long readTrimmedMean(int samples, int trim) {
  if (samples <= 0) return 0;
  if (2 * trim >= samples) trim = 0;

  long values[40];  // enough for our chosen sample sizes
  if (samples > 40) samples = 40;

  for (int i = 0; i < samples; i++) {
    values[i] = scale.read();
  }

  sortArray(values, samples);

  long sum = 0;
  int count = 0;

  for (int i = trim; i < samples - trim; i++) {
    sum += values[i];
    count++;
  }

  if (count == 0) return 0;
  return sum / count;
}

// --------------------------------------------------
// Check whether scale readings are stable
// spread = max - min among sampled values
// smaller spread = more stable
// --------------------------------------------------
bool isScaleStable(int samples = 15, long allowedSpread = 1000) {
  if (!scale.is_ready()) return false;

  long minVal = scale.read();
  long maxVal = minVal;

  for (int i = 1; i < samples; i++) {
    long v = scale.read();
    if (v < minVal) minVal = v;
    if (v > maxVal) maxVal = v;
  }

  long spread = maxVal - minVal;

  Serial.print(F("Stability spread: "));
  Serial.println(spread);

  return (spread <= allowedSpread);
}

// --------------------------------------------------
// Convert raw to grams
// --------------------------------------------------
float rawToGrams(long raw) {
  return (raw - scale.get_offset()) / calibrationFactor;
}

// --------------------------------------------------
// Read weight with filtering
// --------------------------------------------------
float getFilteredWeightGrams() {
  long raw = readTrimmedMean(FAST_SAMPLES, 3);
  float grams = rawToGrams(raw);

  if (!smoothedValid) {
    smoothedGrams = grams;
    smoothedValid = true;
  } else {
    smoothedGrams = SMOOTH_ALPHA * grams + (1.0 - SMOOTH_ALPHA) * smoothedGrams;
  }

  return smoothedGrams;
}

// --------------------------------------------------
// Print commands
// --------------------------------------------------
void printHelp() {
  Serial.println(F("\n=== COMMANDS ==="));
  Serial.println(F("help         -> show commands"));
  Serial.println(F("tare         -> tare with no load"));
  Serial.println(F("cal          -> calibrate using known weight"));
  Serial.println(F("weight=XXX   -> set known weight in grams"));
  Serial.println(F("read         -> one filtered reading"));
  Serial.println(F("raw          -> one raw trimmed reading"));
  Serial.println(F("stable       -> check if scale is stable"));
  Serial.println(F("start        -> continuous reading"));
  Serial.println(F("stop         -> stop continuous reading"));
  Serial.println(F("status       -> show status"));
  Serial.println(F("================\n"));
}

void printStatus() {
  Serial.println(F("\n=== STATUS ==="));
  Serial.print(F("HX711 ready: "));
  Serial.println(scale.is_ready() ? F("YES") : F("NO"));

  Serial.print(F("Tare done: "));
  Serial.println(tareDone ? F("YES") : F("NO"));

  Serial.print(F("Calibrated: "));
  Serial.println(calibrated ? F("YES") : F("NO"));

  Serial.print(F("Known weight (g): "));
  Serial.println(knownWeightGrams, 3);

  Serial.print(F("Offset: "));
  Serial.println(scale.get_offset());

  Serial.print(F("Calibration factor (counts/g): "));
  Serial.println(calibrationFactor, 6);

  if (smoothedValid) {
    Serial.print(F("Smoothed grams: "));
    Serial.println(smoothedGrams, 3);
  }

  Serial.println(F("================\n"));
}

// --------------------------------------------------
// Tare
// --------------------------------------------------
void tareScaleAccurate() {
  if (!scale.is_ready()) {
    Serial.println(F("ERROR: HX711 not ready. Check wiring."));
    return;
  }

  Serial.println(F("Remove all weight from the load cell."));
  Serial.println(F("Checking stability before tare..."));

  if (!isScaleStable(15, 1500)) {
    Serial.println(F("Scale not stable enough. Wait and try again."));
    return;
  }

  Serial.println(F("Taring... please wait"));
  scale.tare(30);

  // refine tare by using trimmed mean as offset
  long refinedOffset = readTrimmedMean(ACCURATE_SAMPLES, 5);
  scale.set_offset(refinedOffset);

  tareDone = true;
  smoothedValid = false;

  Serial.print(F("Tare complete. Refined offset = "));
  Serial.println(scale.get_offset());
}

// --------------------------------------------------
// Calibration
// --------------------------------------------------
void calibrateScaleAccurate() {
  if (!scale.is_ready()) {
    Serial.println(F("ERROR: HX711 not ready. Check wiring."));
    return;
  }

  if (!tareDone) {
    Serial.println(F("ERROR: Please run 'tare' first."));
    return;
  }

  if (knownWeightGrams <= 0.0) {
    Serial.println(F("ERROR: known weight must be > 0."));
    return;
  }

  Serial.print(F("Place known weight: "));
  Serial.print(knownWeightGrams, 3);
  Serial.println(F(" g"));
  Serial.println(F("Checking stability before calibration..."));

  if (!isScaleStable(15, 1500)) {
    Serial.println(F("Scale not stable enough. Wait and try again."));
    return;
  }

  Serial.println(F("Calibrating... please wait"));

  long rawWithWeight = readTrimmedMean(ACCURATE_SAMPLES, 5);
  long offset = scale.get_offset();
  long netRaw = rawWithWeight - offset;

  calibrationFactor = (float)netRaw / knownWeightGrams;

  if (calibrationFactor == 0.0) {
    Serial.println(F("ERROR: Calibration failed. Factor = 0."));
    calibrated = false;
    return;
  }

  calibrated = true;
  smoothedValid = false;

  Serial.print(F("Raw with weight: "));
  Serial.println(rawWithWeight);

  Serial.print(F("Offset: "));
  Serial.println(offset);

  Serial.print(F("Net raw: "));
  Serial.println(netRaw);

  Serial.print(F("Calibration factor (counts/g): "));
  Serial.println(calibrationFactor, 6);

  Serial.println(F("Calibration complete."));
}

// --------------------------------------------------
// Single read
// --------------------------------------------------
void readWeightOnce() {
  if (!scale.is_ready()) {
    Serial.println(F("HX711 not ready"));
    return;
  }

  if (!tareDone) {
    Serial.println(F("Please run 'tare' first."));
    return;
  }

  long raw = readTrimmedMean(FAST_SAMPLES, 3);

  Serial.print(F("RAW: "));
  Serial.print(raw);

  if (calibrated && calibrationFactor != 0.0) {
    float grams = getFilteredWeightGrams();
    Serial.print(F(" | grams: "));
    Serial.println(grams, 3);
  } else {
    Serial.println(F(" | grams: not calibrated"));
  }
}

void readRawOnce() {
  if (!scale.is_ready()) {
    Serial.println(F("HX711 not ready"));
    return;
  }

  long raw = readTrimmedMean(FAST_SAMPLES, 3);
  Serial.print(F("Trimmed RAW: "));
  Serial.println(raw);
}

// --------------------------------------------------
// Commands
// --------------------------------------------------
void processCommand(String cmd) {
  cmd.trim();
  cmd.toLowerCase();

  if (cmd.length() == 0) return;

  Serial.print(F("Received command: "));
  Serial.println(cmd);

  if (cmd == "help") {
    printHelp();
  }
  else if (cmd == "tare") {
    tareScaleAccurate();
  }
  else if (cmd == "cal") {
    calibrateScaleAccurate();
  }
  else if (cmd.startsWith("weight=")) {
    String valueStr = cmd.substring(7);
    float value = valueStr.toFloat();

    if (value <= 0) {
      Serial.println(F("ERROR: weight must be > 0"));
    } else {
      knownWeightGrams = value;
      Serial.print(F("Known weight set to "));
      Serial.print(knownWeightGrams, 3);
      Serial.println(F(" g"));
    }
  }
  else if (cmd == "read") {
    readWeightOnce();
  }
  else if (cmd == "raw") {
    readRawOnce();
  }
  else if (cmd == "stable") {
    if (isScaleStable()) {
      Serial.println(F("Scale is stable."));
    } else {
      Serial.println(F("Scale is not stable."));
    }
  }
  else if (cmd == "start") {
    if (!calibrated) {
      Serial.println(F("Please calibrate first."));
      return;
    }
    continuousRead = true;
    Serial.println(F("Continuous reading started."));
  }
  else if (cmd == "stop") {
    continuousRead = false;
    Serial.println(F("Continuous reading stopped."));
  }
  else if (cmd == "status") {
    printStatus();
  }
  else {
    Serial.println(F("Unknown command."));
    printHelp();
  }
}

// --------------------------------------------------
// Serial input
// --------------------------------------------------
void readSerialCommands() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (inputBuffer.length() > 0) {
        processCommand(inputBuffer);
        inputBuffer = "";
      }
    } else {
      inputBuffer += c;
    }
  }
}

// --------------------------------------------------
// Setup / Loop
// --------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println(F("\n=== HX711 High Accuracy Serial Control ==="));

  scale.begin(DOUT_PIN, SCK_PIN);

  if (scale.is_ready()) {
    Serial.println(F("HX711 is ready."));
  } else {
    Serial.println(F("WARNING: HX711 not ready. Check wiring."));
  }

  Serial.println(F("\nRecommended process:"));
  Serial.println(F("1) Remove all weight"));
  Serial.println(F("2) Type: tare"));
  Serial.println(F("3) Put known weight on load cell"));
  Serial.println(F("4) Type: weight=100   (or your real weight)"));
  Serial.println(F("5) Type: cal"));
  Serial.println(F("6) Type: read or start"));

  printHelp();
}

void loop() {
  readSerialCommands();

  if (continuousRead) {
    readWeightOnce();
    delay(250);
  }
}