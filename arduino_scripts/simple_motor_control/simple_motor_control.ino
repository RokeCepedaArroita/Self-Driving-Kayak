#include <SoftwareSerial.h>

SoftwareSerial HM10Serial(0, 1);  // RX, TX - Change these pin numbers to match your actual connections.

void setup() {
  Serial.begin(9600);
  HM10Serial.begin(9600);
}

void loop() {
  // Check if there's data available from the HM-10 Bluetooth module
  if (HM10Serial.available()) {
    char data = HM10Serial.read();
    Serial.print(data);
  }
}
