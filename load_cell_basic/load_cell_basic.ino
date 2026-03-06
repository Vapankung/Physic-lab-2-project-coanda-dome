#include "HX711.h"

HX711 scale;

const byte DOUT_PIN = 25;
const byte SCK_PIN  = 26;

const float KNOWN_WEIGHT_GRAMS = 244;  // <-- change this

float calibrationFactor = 1.0;

void setup() {
  Serial.begin(9600);
  scale.begin(DOUT_PIN, SCK_PIN);

  Serial.println("\n=== HX711 Calibration ===");
  Serial.println("1) Remove all weight.");
  delay(3000);

  // Tare
  scale.tare(500);  // average 15 readings
  long offset = scale.get_offset();
  Serial.print("Offset (tare): ");
  Serial.println(offset);

  Serial.println("\n2) Put the KNOWN weight on the load cell now.");
  Serial.print("Known weight (g): ");
  Serial.println(KNOWN_WEIGHT_GRAMS);
  delay(5000);

  // Read average raw value with known weight
  long rawWithWeight = scale.read_average(500); // average 20 readings
  Serial.print("Raw with known weight: ");
  Serial.println(rawWithWeight);

  // After tare, the “net raw” is basically (rawWithWeight - offset)
  long netRaw = rawWithWeight - offset;
  Serial.print("Net raw (raw - offset): ");
  Serial.println(netRaw);

  // Compute calibration factor (raw counts per gram)
  calibrationFactor = (float)netRaw / KNOWN_WEIGHT_GRAMS;
  Serial.print("\n✅ Calibration factor (counts per gram): ");
  Serial.println(calibrationFactor, 6);

  Serial.println("\nNow reading weight...");
}

void loop() {
  if (!scale.is_ready()) {
    Serial.println("HX711 not ready");
    delay(500);
    return;
  }

  long raw = scale.read_average(10);
  long offset = scale.get_offset();

  float grams = (raw - offset) / calibrationFactor;

  Serial.print("RAW: ");
  Serial.print(raw);
  Serial.print(" | grams: ");
  Serial.println(grams, 2);

  delay(300);
}
