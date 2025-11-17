# WIP / Personal Project

**This is a personal, work-in-progress library.** It is not feature-complete and likely contains bugs. I'm sharing it for my own use and for anyone who might find it helpful.

**Please use at your own risk!**

---

# My_BQ40Z80_Lib

A personal, work-in-progress library for the TI BQ40Z80. Reads all 7 cell voltages, pack voltage, current, temp, and SOC. Accesses diagnostics like Safety Status and Lifetime data. Includes helpers to unseal, configure protections (CUV/COV), and seal the device.

## Installation

### PlatformIO

This library is not (yet) in the PlatformIO registry. You can add it to your `platformio.ini` by pointing to this GitHub repository:

```ini
lib_deps =
    https://github.com/johanwloubser99-max/My_BQ40Z80_Lib.git
```

## Basic Example

```cpp
#include <Wire.h>
#include "BQ40Z80.h"

// Use the default I2C address (0x0B)
BQ40Z80 bms; 

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  // Wire.begin() is called inside bms.begin()
  if (!bms.begin()) {
    Serial.println("Failed to find BQ40Z80 chip");
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

  Serial.print("SOC: ");
  Serial.print(bms.getSOC());
  Serial.println(" %");

  Serial.print("Cell 1: ");
  Serial.print(bms.getCellVoltage(1));
  Serial.println(" mV");

  Serial.println("--------------------");
  delay(2000);
}
```