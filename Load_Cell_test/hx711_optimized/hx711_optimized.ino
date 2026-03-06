#include "HX711.h"
#include <EEPROM.h>

// =====================================================================
// HX711 Optimized Scale with Live Teach / Adaptive Calibration
// Arduino Uno
// =====================================================================

// =====================================================================
// Pin definitions
// =====================================================================
const byte DOUT_PIN = 26;
const byte SCK_PIN  = 25;

// =====================================================================
// Sample sizes
// =====================================================================
const int FAST_SAMPLES     = 32;   // normal reads
const int ACCURATE_SAMPLES = 64;   // tare / calibration / teach
const int MAX_SAMPLES      = 64;   // must be >= both above

// =====================================================================
// Stability thresholds
// =====================================================================
const int   STABILITY_SAMPLES    = 20;
const float STABILITY_MAX_STDDEV = 80.0f;   // tighten if needed

// =====================================================================
// Kalman filter parameters
// Tune these for your setup
// =====================================================================
const float KALMAN_Q = 0.5f;     // higher = faster response
const float KALMAN_R = 150.0f;   // higher = smoother, less trust in sensor

// =====================================================================
// IQR outlier rejection
// =====================================================================
const float IQR_FACTOR = 1.5f;

// =====================================================================
// Live teach strength
// 0.0 -> ignore new teach
// 1.0 -> fully replace that calibration point
// =====================================================================
const float TEACH_ALPHA = 0.35f;

// =====================================================================
// 2-point calibration storage
// rawToGrams uses a linear model between two points
// =====================================================================
struct CalPoint {
  long  raw;
  float grams;
};

// =====================================================================
// EEPROM layout
// =====================================================================
const int EEPROM_MAGIC_ADDR  = 0;
const byte EEPROM_MAGIC      = 0xA7;
const int EEPROM_OFFSET_ADDR = 1;   // long
const int EEPROM_CAL_ADDR    = 5;   // calLow + calHigh

// =====================================================================
// State
// =====================================================================
HX711 scale;

bool tareDone       = false;
bool calibrated     = false;
bool continuousRead = false;

CalPoint calLow  = {0, 0.0f};
CalPoint calHigh = {0, 0.0f};

// Linear model:
// grams = calSlope * raw + calIntercept
float calSlope     = 1.0f;   // grams per count
float calIntercept = 0.0f;   // grams

// Kalman state
float kEstimate = 0.0f;
float kError    = 1.0f;
bool  kValid    = false;

// Serial input
char    inputBuffer[64];
uint8_t inputLen = 0;

// Teach queue
bool  teachPending     = false;
bool  resumeAfterTeach = false;
float teachTargetGrams = 0.0f;

// =====================================================================
// Sorting — insertion sort
// =====================================================================
void sortArray(long arr[], int n) {
  for (int i = 1; i < n; i++) {
    long key = arr[i];
    int j = i - 1;
    while (j >= 0 && arr[j] > key) {
      arr[j + 1] = arr[j];
      j--;
    }
    arr[j + 1] = key;
  }
}

// =====================================================================
// Collect raw HX711 samples
// =====================================================================
int collectSamples(long buf[], int n) {
  if (n > MAX_SAMPLES) n = MAX_SAMPLES;

  for (int i = 0; i < n; i++) {
    while (!scale.is_ready()) {}
    buf[i] = scale.read();
  }
  return n;
}

// =====================================================================
// IQR-based trimmed mean
// Sorts buf in place
// =====================================================================
long iqrTrimmedMean(long buf[], int n) {
  if (n <= 0) return 0;

  sortArray(buf, n);

  float q1  = buf[n / 4];
  float q3  = buf[(3 * n) / 4];
  float iqr = q3 - q1;

  float lo = q1 - IQR_FACTOR * iqr;
  float hi = q3 + IQR_FACTOR * iqr;

  long sum = 0;
  int count = 0;

  for (int i = 0; i < n; i++) {
    if (buf[i] >= lo && buf[i] <= hi) {
      sum += buf[i];
      count++;
    }
  }

  return (count > 0) ? (sum / count) : buf[n / 2];
}

// =====================================================================
// Stability check using standard deviation
// =====================================================================
bool isScaleStable() {
  if (!scale.is_ready()) return false;

  long buf[STABILITY_SAMPLES];
  int n = collectSamples(buf, STABILITY_SAMPLES);

  float mean = 0.0f;
  for (int i = 0; i < n; i++) mean += buf[i];
  mean /= n;

  float variance = 0.0f;
  for (int i = 0; i < n; i++) {
    float d = buf[i] - mean;
    variance += d * d;
  }

  float stddev = sqrt(variance / n);

  Serial.print(F("Stability std-dev: "));
  Serial.println(stddev, 2);

  return (stddev <= STABILITY_MAX_STDDEV);
}

// =====================================================================
// Recompute linear calibration model from calLow and calHigh
// grams = m*raw + b
// =====================================================================
bool recomputeCalibration() {
  long dRaw = calHigh.raw - calLow.raw;
  float dGrams = calHigh.grams - calLow.grams;

  if (dRaw == 0) return false;
  if (dGrams <= 0.0f) return false;

  calSlope     = dGrams / (float)dRaw;               // grams per count
  calIntercept = calLow.grams - calSlope * calLow.raw;

  calibrated = true;
  kValid = false;
  return true;
}

// =====================================================================
// Convert raw to grams
// =====================================================================
float rawToGrams(long raw) {
  if (!calibrated) return (float)raw;
  return calSlope * raw + calIntercept;
}

// =====================================================================
// Kalman filter
// =====================================================================
float kalmanUpdate(float measurement) {
  if (!kValid) {
    kEstimate = measurement;
    kError    = KALMAN_R;
    kValid    = true;
    return measurement;
  }

  float predictedError = kError + KALMAN_Q;
  float gain = predictedError / (predictedError + KALMAN_R);

  kEstimate = kEstimate + gain * (measurement - kEstimate);
  kError    = (1.0f - gain) * predictedError;

  return kEstimate;
}

// =====================================================================
// Read filtered value
// =====================================================================
float readFiltered(int samples) {
  long buf[MAX_SAMPLES];
  int n = collectSamples(buf, samples);
  long raw = iqrTrimmedMean(buf, n);
  return kalmanUpdate(rawToGrams(raw));
}

// =====================================================================
// EEPROM
// =====================================================================
void saveToEEPROM() {
  EEPROM.write(EEPROM_MAGIC_ADDR, EEPROM_MAGIC);
  EEPROM.put(EEPROM_OFFSET_ADDR, scale.get_offset());
  EEPROM.put(EEPROM_CAL_ADDR, calLow);
  EEPROM.put(EEPROM_CAL_ADDR + sizeof(CalPoint), calHigh);

  Serial.println(F("Calibration saved to EEPROM."));
}

bool loadFromEEPROM() {
  if (EEPROM.read(EEPROM_MAGIC_ADDR) != EEPROM_MAGIC) return false;

  long offset;
  EEPROM.get(EEPROM_OFFSET_ADDR, offset);
  scale.set_offset(offset);

  EEPROM.get(EEPROM_CAL_ADDR, calLow);
  EEPROM.get(EEPROM_CAL_ADDR + sizeof(CalPoint), calHigh);

  if (!recomputeCalibration()) return false;

  tareDone = true;
  return true;
}

// =====================================================================
// Print helpers
// =====================================================================
void printHelp() {
  Serial.println(F("\n=== COMMANDS ==="));
  Serial.println(F("help          -> show this list"));
  Serial.println(F("tare          -> zero with no load"));
  Serial.println(F("cal1=XXX      -> place low known weight, e.g. cal1=50"));
  Serial.println(F("cal2=XXX      -> place high known weight, e.g. cal2=200"));
  Serial.println(F("teach=XXX     -> send true weight to optimize calibration"));
  Serial.println(F("read          -> single filtered reading"));
  Serial.println(F("raw           -> single raw trimmed reading"));
  Serial.println(F("stable        -> check scale stability"));
  Serial.println(F("start         -> continuous reading"));
  Serial.println(F("stop          -> stop continuous reading"));
  Serial.println(F("save          -> save calibration to EEPROM"));
  Serial.println(F("load          -> load calibration from EEPROM"));
  Serial.println(F("status        -> show current status"));
  Serial.println(F("================\n"));
}

void printStatus() {
  Serial.println(F("\n=== STATUS ==="));
  Serial.print(F("HX711 ready:        "));
  Serial.println(scale.is_ready() ? F("YES") : F("NO"));

  Serial.print(F("Tare done:          "));
  Serial.println(tareDone ? F("YES") : F("NO"));

  Serial.print(F("Calibrated:         "));
  Serial.println(calibrated ? F("YES") : F("NO"));

  Serial.print(F("Offset:             "));
  Serial.println(scale.get_offset());

  if (calibrated) {
    Serial.print(F("Cal low raw/g:      "));
    Serial.print(calLow.raw);
    Serial.print(F(" / "));
    Serial.println(calLow.grams, 3);

    Serial.print(F("Cal high raw/g:     "));
    Serial.print(calHigh.raw);
    Serial.print(F(" / "));
    Serial.println(calHigh.grams, 3);

    Serial.print(F("Slope (g/count):    "));
    Serial.println(calSlope, 8);

    Serial.print(F("Intercept (g):      "));
    Serial.println(calIntercept, 6);

    Serial.print(F("Kalman estimate:    "));
    Serial.println(kEstimate, 3);
  }

  Serial.println(F("================\n"));
}

// =====================================================================
// Tare
// =====================================================================
void doTare() {
  if (!scale.is_ready()) {
    Serial.println(F("ERROR: HX711 not ready."));
    return;
  }

  Serial.println(F("Remove all weight. Checking stability..."));
  if (!isScaleStable()) {
    Serial.println(F("Not stable. Wait and retry."));
    return;
  }

  Serial.println(F("Taring..."));

  long buf[MAX_SAMPLES];
  int n = collectSamples(buf, ACCURATE_SAMPLES);
  long refined = iqrTrimmedMean(buf, n);
  scale.set_offset(refined);

  tareDone = true;
  calibrated = false;
  kValid = false;

  calLow.raw = 0;
  calLow.grams = 0.0f;
  calHigh.raw = 0;
  calHigh.grams = 0.0f;

  Serial.print(F("Tare done. Offset = "));
  Serial.println(scale.get_offset());
}

// =====================================================================
// 2-point calibration low
// =====================================================================
void doCalLow(float knownGrams) {
  if (!tareDone) {
    Serial.println(F("Run 'tare' first."));
    return;
  }

  if (knownGrams <= 0) {
    Serial.println(F("Weight must be > 0."));
    return;
  }

  Serial.print(F("Place "));
  Serial.print(knownGrams, 2);
  Serial.println(F(" g. Checking stability..."));

  if (!isScaleStable()) {
    Serial.println(F("Not stable. Wait and retry."));
    return;
  }

  long buf[MAX_SAMPLES];
  int n = collectSamples(buf, ACCURATE_SAMPLES);

  calLow.raw   = iqrTrimmedMean(buf, n);
  calLow.grams = knownGrams;

  Serial.print(F("Cal point 1 set: raw="));
  Serial.print(calLow.raw);
  Serial.print(F(", grams="));
  Serial.println(calLow.grams, 3);

  Serial.println(F("Now run: cal2=XXX with your heavier weight."));
}

// =====================================================================
// 2-point calibration high
// =====================================================================
void doCalHigh(float knownGrams) {
  if (!tareDone) {
    Serial.println(F("Run 'tare' first."));
    return;
  }

  if (calLow.grams <= 0) {
    Serial.println(F("Run 'cal1=XXX' first."));
    return;
  }

  if (knownGrams <= calLow.grams) {
    Serial.println(F("cal2 weight must be greater than cal1 weight."));
    return;
  }

  Serial.print(F("Place "));
  Serial.print(knownGrams, 2);
  Serial.println(F(" g. Checking stability..."));

  if (!isScaleStable()) {
    Serial.println(F("Not stable. Wait and retry."));
    return;
  }

  long buf[MAX_SAMPLES];
  int n = collectSamples(buf, ACCURATE_SAMPLES);

  calHigh.raw   = iqrTrimmedMean(buf, n);
  calHigh.grams = knownGrams;

  if (!recomputeCalibration()) {
    Serial.println(F("ERROR: calibration recompute failed."));
    return;
  }

  Serial.print(F("Cal point 2 set: raw="));
  Serial.print(calHigh.raw);
  Serial.print(F(", grams="));
  Serial.println(calHigh.grams, 3);

  Serial.print(F("Slope (g/count): "));
  Serial.println(calSlope, 8);

  Serial.println(F("Calibration complete. Type 'save' to persist."));
}

// =====================================================================
// Read once
// =====================================================================
void doRead() {
  if (!scale.is_ready()) {
    Serial.println(F("HX711 not ready."));
    return;
  }

  if (!tareDone) {
    Serial.println(F("Run 'tare' first."));
    return;
  }

  float grams = readFiltered(FAST_SAMPLES);

  if (calibrated) {
    Serial.print(F("Weight: "));
    Serial.print(grams, 3);
    Serial.println(F(" g"));
  } else {
    Serial.print(F("Raw (no cal): "));
    Serial.println((long)grams);
  }
}

// =====================================================================
// Raw read once
// =====================================================================
void doRaw() {
  if (!scale.is_ready()) {
    Serial.println(F("HX711 not ready."));
    return;
  }

  long buf[MAX_SAMPLES];
  int n = collectSamples(buf, FAST_SAMPLES);
  long raw = iqrTrimmedMean(buf, n);

  Serial.print(F("Trimmed RAW: "));
  Serial.println(raw);
}

// =====================================================================
// Live teach
// While reading, user can send teach=XXX
// It stops stream, captures stable accurate raw value,
// updates the nearest calibration side, then recomputes calibration
// =====================================================================
void queueTeach(float correctGrams) {
  if (correctGrams <= 0.0f) {
    Serial.println(F("ERROR: teach value must be > 0."));
    return;
  }

  teachTargetGrams = correctGrams;
  teachPending = true;
  resumeAfterTeach = continuousRead;
  continuousRead = false;

  Serial.print(F("Teach queued with true weight = "));
  Serial.print(correctGrams, 3);
  Serial.println(F(" g"));
  Serial.println(F("Hold that weight steady..."));
}

void doTeach(float correctGrams) {
  if (!scale.is_ready()) {
    Serial.println(F("ERROR: HX711 not ready."));
    return;
  }

  if (!tareDone) {
    Serial.println(F("ERROR: Run 'tare' first."));
    return;
  }

  if (!calibrated) {
    Serial.println(F("ERROR: Do normal calibration first (cal1 / cal2), then use teach."));
    return;
  }

  Serial.print(F("Teaching with known weight: "));
  Serial.print(correctGrams, 3);
  Serial.println(F(" g"));

  Serial.println(F("Checking stability before teach..."));
  if (!isScaleStable()) {
    Serial.println(F("Scale not stable enough. Keep the weight still and try again."));
    return;
  }

  long buf[MAX_SAMPLES];
  int n = collectSamples(buf, ACCURATE_SAMPLES);
  long newRaw = iqrTrimmedMean(buf, n);

  float oldEstimate = rawToGrams(newRaw);
  float errorBefore = correctGrams - oldEstimate;

  Serial.print(F("Measured before correction: "));
  Serial.print(oldEstimate, 3);
  Serial.println(F(" g"));

  Serial.print(F("Target true weight:        "));
  Serial.print(correctGrams, 3);
  Serial.println(F(" g"));

  Serial.print(F("Error before correction:   "));
  Serial.print(errorBefore, 3);
  Serial.println(F(" g"));

  float midpoint = 0.5f * (calLow.grams + calHigh.grams);

  if (correctGrams <= midpoint) {
    long blendedRaw = (long)((1.0f - TEACH_ALPHA) * calLow.raw + TEACH_ALPHA * newRaw);
    calLow.raw = blendedRaw;
    calLow.grams = correctGrams;
    Serial.println(F("Updated LOW calibration point."));
  } else {
    long blendedRaw = (long)((1.0f - TEACH_ALPHA) * calHigh.raw + TEACH_ALPHA * newRaw);
    calHigh.raw = blendedRaw;
    calHigh.grams = correctGrams;
    Serial.println(F("Updated HIGH calibration point."));
  }

  if (!recomputeCalibration()) {
    Serial.println(F("ERROR: calibration recompute failed."));
    return;
  }

  float newEstimate = rawToGrams(newRaw);
  float errorAfter = correctGrams - newEstimate;

  // Reset filter close to corrected value
  kEstimate = correctGrams;
  kError = KALMAN_R;
  kValid = true;

  Serial.print(F("Measured after correction: "));
  Serial.print(newEstimate, 3);
  Serial.println(F(" g"));

  Serial.print(F("Error after correction:    "));
  Serial.print(errorAfter, 3);
  Serial.println(F(" g"));

  Serial.println(F("Teach complete. Type 'save' if you want to store this improvement."));
}

// =====================================================================
// Command dispatcher
// =====================================================================
void processCommand(const char* rawCmd) {
  char cmd[64];
  int len = 0;

  while (rawCmd[len] && len < 63) {
    cmd[len] = tolower((unsigned char)rawCmd[len]);
    len++;
  }
  cmd[len] = '\0';

  while (len > 0 && (cmd[len - 1] == ' ' || cmd[len - 1] == '\r' || cmd[len - 1] == '\n' || cmd[len - 1] == '\t')) {
    cmd[--len] = '\0';
  }

  if (len == 0) return;

  Serial.print(F("CMD: "));
  Serial.println(cmd);

  if (strcmp(cmd, "help") == 0) {
    printHelp();
  }
  else if (strcmp(cmd, "tare") == 0) {
    doTare();
  }
  else if (strncmp(cmd, "cal1=", 5) == 0) {
    float g = atof(cmd + 5);
    if (g <= 0.0f) Serial.println(F("Value must be > 0."));
    else doCalLow(g);
  }
  else if (strncmp(cmd, "cal2=", 5) == 0) {
    float g = atof(cmd + 5);
    if (g <= 0.0f) Serial.println(F("Value must be > 0."));
    else doCalHigh(g);
  }
  else if (strncmp(cmd, "teach=", 6) == 0) {
    float g = atof(cmd + 6);
    if (g <= 0.0f) Serial.println(F("Value must be > 0."));
    else queueTeach(g);
  }
  else if (strcmp(cmd, "read") == 0) {
    doRead();
  }
  else if (strcmp(cmd, "raw") == 0) {
    doRaw();
  }
  else if (strcmp(cmd, "stable") == 0) {
    Serial.println(isScaleStable() ? F("Scale is stable.") : F("Not stable."));
  }
  else if (strcmp(cmd, "start") == 0) {
    if (!calibrated) {
      Serial.println(F("Calibrate first."));
      return;
    }
    continuousRead = true;
    Serial.println(F("Grams"));
  }
  else if (strcmp(cmd, "stop") == 0) {
    continuousRead = false;
    resumeAfterTeach = false;
    Serial.println(F("Stopped."));
  }
  else if (strcmp(cmd, "save") == 0) {
    if (!calibrated) {
      Serial.println(F("Nothing to save."));
      return;
    }
    saveToEEPROM();
  }
  else if (strcmp(cmd, "load") == 0) {
    if (loadFromEEPROM()) {
      Serial.println(F("Calibration loaded from EEPROM."));
      printStatus();
    } else {
      Serial.println(F("No valid calibration found in EEPROM."));
    }
  }
  else if (strcmp(cmd, "status") == 0) {
    printStatus();
  }
  else {
    Serial.print(F("Unknown command: "));
    Serial.println(cmd);
    printHelp();
  }
}

// =====================================================================
// Serial input
// =====================================================================
void readSerialCommands() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\n' || c == '\r') {
      if (inputLen > 0) {
        inputBuffer[inputLen] = '\0';
        processCommand(inputBuffer);
        inputLen = 0;
      }
    } else if (inputLen < (sizeof(inputBuffer) - 1)) {
      inputBuffer[inputLen++] = c;
    }
  }
}

// =====================================================================
// Setup
// =====================================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println(F("\n=== HX711 Optimized Scale with Teach ==="));
  scale.begin(DOUT_PIN, SCK_PIN);

  if (scale.is_ready()) {
    Serial.println(F("HX711 ready."));
  } else {
    Serial.println(F("WARNING: HX711 not ready — check wiring."));
  }

  if (loadFromEEPROM()) {
    Serial.println(F("Previous calibration restored from EEPROM."));
    printStatus();
  } else {
    Serial.println(F("\nRecommended setup:"));
    Serial.println(F("  1) Remove all weight  -> tare"));
    Serial.println(F("  2) Place light weight -> cal1=50"));
    Serial.println(F("  3) Place heavy weight -> cal2=200"));
    Serial.println(F("  4) Save               -> save"));
    Serial.println(F("  5) Weigh              -> read or start"));
    Serial.println(F("  6) Fine tune          -> teach=XXX"));
  }

  printHelp();
}

// =====================================================================
// Loop
// =====================================================================
void loop() {
  readSerialCommands();

  if (teachPending) {
    float g = teachTargetGrams;
    teachPending = false;
    doTeach(g);

    if (resumeAfterTeach) {
      resumeAfterTeach = false;
      continuousRead = true;
      Serial.println(F("Grams"));
    }
  }

  if (continuousRead && scale.is_ready()) {
    long buf[FAST_SAMPLES];
    int n = collectSamples(buf, FAST_SAMPLES);
    long raw = iqrTrimmedMean(buf, n);
    float grams = kalmanUpdate(rawToGrams(raw));

    Serial.println(grams, 3);
    delay(100);
  }
}