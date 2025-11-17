#include <Wire.h>
#include "BQ40Z80.h"

// Use the default I2C address (0x0B)
BQ40Z80 bms; 

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  Serial.println("BQ40Z80 SimpleRead Example");

  // Wire.begin() is called inside bms.begin()
  if (!bms.begin()) {
    Serial.println("Failed to find BQ40Z80 chip. Check wiring.");
    while (1) { delay(10); }
  }
  
  Serial.println("BQ40Z80 Found!");
  
  // Optional: Enable debug messages
  // bms.setDebug(true);
}

void loop() {
  Serial.print("Voltage: ");
  Serial.print(bms.getVoltage());
  Serial.println(" V");

  Serial.print("Current: ");
  Serial.print(bms.getCurrent());
  Serial.println(" A");

  Serial.print("Temperature: ");
  Serial.print(bms.getTemperature());
  Serial.println(" C");

  Serial.print("SOC: ");
  Serial.print(bms.getSOC());
  Serial.println(" %");

  Serial.print("Cell 1: ");
  Serial.print(bms.getCellVoltage(1));
  Serial.println(" mV");

  Serial.print("Cell 7: ");
  Serial.print(bms.getCellVoltage(7));
  Serial.println(" mV");

  Serial.println("--------------------");
  delay(2000);
}