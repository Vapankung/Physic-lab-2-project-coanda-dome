#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

const int LED_PIN = 2;   // onboard LED (try 4 if 2 doesn't work)

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(115200);

  SerialBT.begin("ESP32-WROOM-CTRL");
  Serial.println("Bluetooth Ready");
  Serial.println("Send: ON or OFF");
}

void loop() {
  if (SerialBT.available()) {

    String command = SerialBT.readStringUntil('\n');
    command.trim();  // remove spaces and newline

    Serial.print("Received: ");
    Serial.println(command);

    if (command.equalsIgnoreCase("ON")) {
      digitalWrite(LED_PIN, HIGH);
      SerialBT.println("LED ON");
      Serial.println("LED ON");
    }
    else if (command.equalsIgnoreCase("OFF")) {
      digitalWrite(LED_PIN, LOW);
      SerialBT.println("LED OFF");
      Serial.println("LED OFF");
    }
    else {
      SerialBT.println("Use ON or OFF");
    }
  }
}