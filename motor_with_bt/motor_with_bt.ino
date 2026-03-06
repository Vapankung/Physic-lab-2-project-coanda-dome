#include <BluetoothSerial.h>
#include <ESP32Servo.h>

BluetoothSerial SerialBT;
Servo esc;

// ====== SETTINGS ======
const int ESC_PIN = 27;          // PWM signal pin to ESC (change if needed)

const int MIN_US = 1000;         // typical ESC min throttle pulse
const int MAX_US = 2000;         // typical ESC max throttle pulse

const float MAX_LIMIT = 0.30f;   // 30% max throttle limit

// Safety timeout: if no valid speed command for a while -> drop to MIN
const unsigned long FAILSAFE_MS = 2000;

// Arming behavior
const unsigned long ARM_MIN_HOLD_MS = 2000; // hold MIN for 2s on ON/boot

// ====== STATE ======
bool enabled = false;
int requestedPercent = 0;        // 0..100 from user
unsigned long lastCmdMs = 0;

int percentToPulseUs(int percent0to100) {
  if (percent0to100 < 0) percent0to100 = 0;
  if (percent0to100 > 100) percent0to100 = 100;

  // Apply 30% limit
  float limited = percent0to100 / 100.0f;
  if (limited > MAX_LIMIT) limited = MAX_LIMIT;

  // Map 0..MAX_LIMIT to MIN_US..(MIN_US + MAX_LIMIT*(MAX_US-MIN_US))
  int span = MAX_US - MIN_US;
  int us = MIN_US + (int)(limited * span + 0.5f);
  return us;
}

void setThrottlePercent(int percent0to100) {
  requestedPercent = percent0to100;
  int us = percentToPulseUs(requestedPercent);
  esc.writeMicroseconds(us);

  Serial.print("Throttle req = ");
  Serial.print(requestedPercent);
  Serial.print("%  | limited pulse = ");
  Serial.print(us);
  Serial.println(" us");

  if (SerialBT.hasClient()) {
    SerialBT.print("OK Throttle=");
    SerialBT.print(requestedPercent);
    SerialBT.print("% (capped 30%) pulse=");
    SerialBT.print(us);
    SerialBT.println("us");
  }
}

void armSequence() {
  // Send MIN for a while to arm safely
  esc.writeMicroseconds(MIN_US);
  Serial.println("Arming: holding MIN throttle...");
  if (SerialBT.hasClient()) SerialBT.println("Arming: holding MIN throttle...");

  delay(ARM_MIN_HOLD_MS);

  Serial.println("Armed (enabled).");
  if (SerialBT.hasClient()) SerialBT.println("Armed (enabled).");
}

void printStatus() {
  int us = percentToPulseUs(requestedPercent);
  SerialBT.print("ENABLED=");
  SerialBT.print(enabled ? "YES" : "NO");
  SerialBT.print(" | REQ=");
  SerialBT.print(requestedPercent);
  SerialBT.print("% | PULSE=");
  SerialBT.print(us);
  SerialBT.println("us (capped 30%)");
}

void setup() {
  Serial.begin(115200);

  // Setup servo signal at 50Hz, 1000-2000us
  esc.setPeriodHertz(50);
  esc.attach(ESC_PIN, MIN_US, MAX_US);

  // Boot safe: MIN throttle
  esc.writeMicroseconds(MIN_US);
  delay(ARM_MIN_HOLD_MS);

  SerialBT.begin("ESP32-ESC-CTRL");
  Serial.println("Bluetooth ready: ESP32-ESC-CTRL");
  Serial.println("Commands: ON, OFF, SPEED n(0-100), STOP, STATUS");

  lastCmdMs = millis();
}

void loop() {
  // ----- Failsafe -----
  if (enabled) {
    if (millis() - lastCmdMs > FAILSAFE_MS) {
      // Drop to MIN throttle if no commands recently
      esc.writeMicroseconds(MIN_US);
    }
  } else {
    // If disabled, always keep MIN
    esc.writeMicroseconds(MIN_US);
  }

  // ----- Read Bluetooth -----
  if (SerialBT.available()) {
    String cmd = SerialBT.readStringUntil('\n');
    cmd.trim();
    lastCmdMs = millis();

    Serial.print("BT: ");
    Serial.println(cmd);

    if (cmd.equalsIgnoreCase("ON")) {
      enabled = true;
      setThrottlePercent(0);
      armSequence();  // safe arming hold at MIN
      setThrottlePercent(0);
    }
    else if (cmd.equalsIgnoreCase("OFF")) {
      enabled = false;
      setThrottlePercent(0);
      SerialBT.println("OK OFF (disabled, throttle MIN)");
    }
    else if (cmd.equalsIgnoreCase("STOP")) {
      if (enabled) setThrottlePercent(0);
      else SerialBT.println("ERR: Send ON first");
    }
    else if (cmd.equalsIgnoreCase("STATUS")) {
      printStatus();
    }
    else if (cmd.startsWith("SPEED") || cmd.startsWith("speed")) {
      if (!enabled) {
        SerialBT.println("ERR: Send ON first");
        return;
      }

      // Parse: "SPEED 25"
      int spaceIdx = cmd.indexOf(' ');
      if (spaceIdx < 0) {
        SerialBT.println("ERR: use SPEED <0-100>");
        return;
      }
      int val = cmd.substring(spaceIdx + 1).toInt();
      if (val < 0) val = 0;
      if (val > 100) val = 100;

      setThrottlePercent(val);
    }
    else {
      SerialBT.println("ERR: ON / OFF / SPEED <0-100> / STOP / STATUS");
    }
  }
}