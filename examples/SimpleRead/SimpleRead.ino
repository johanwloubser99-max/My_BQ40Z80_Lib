#include <SoftwareSerial.h>

// --- Pin Definitions for SoftwareSerial ---
// Connect this (Relay) Arduino's Pin 10 to the AVR64DD32's TX1 pin
const int BMS_RX_PIN = 10; 
// Connect this (Relay) Arduino's Pin 11 to the AVR64DD32's RX1 pin
const int BMS_TX_PIN = 11; 

// --- Constants ---
const byte SERIAL_ENQUIRY = 0x05;         // ENQ byte to request data
const unsigned long ENQUIRY_INTERVAL = 10000;  // Send enquiry every 5 seconds

// --- Global Variables ---
unsigned long lastEnquiryTime = 0;      // Timestamp for enquiry timer

// Initialize the SoftwareSerial port
SoftwareSerial bmsSerial(BMS_RX_PIN, BMS_TX_PIN);

/**
 * @brief Main setup function.
 */
void setup() {
  // Initialize the Serial Monitor (for you to see the output)
  Serial.begin(57600);
  while (!Serial) delay(10);
  
  // Initialize the SoftwareSerial port to talk to the BMS Monitor
  // This baud rate MUST match the LISTENER_SERIAL rate on your AVR64DD32
  bmsSerial.begin(57600);

  Serial.println("\r\n--- BMS Serial Relay/Tester ---");
  Serial.println("This sketch will send an ENQ (0x05) byte to the BMS monitor");
  Serial.println("every 5 seconds and print any response received.");
  Serial.println("-----------------------------------------------------");
}

/**
 * @brief Main loop: Sends requests and relays responses.
 */
void loop() {
  
  // Task 1: Periodically send the enquiry byte
  if (millis() - lastEnquiryTime >= ENQUIRY_INTERVAL) {
    Serial.println("\r\nSending ENQ (0x05) to BMS Monitor...");
    bmsSerial.write(SERIAL_ENQUIRY);
    lastEnquiryTime = millis();
  }

  // Task 2: Listen for a response from the BMS Monitor and relay it
  if (bmsSerial.available() > 0) {
    // Read all available characters and print them to the Serial Monitor
    while (bmsSerial.available() > 0) {
      Serial.write(bmsSerial.read());
    }
  }

  // (Task 3: Optional pass-through from your monitor TO the BMS)
  // If you type anything in the Serial Monitor, it will be sent to the BMS Monitor.
  // Be careful, as your BMS_Monitor sketch only expects 0x05.
  /*
  if (Serial.available() > 0) {
    bmsSerial.write(Serial.read());
  }
  */
}
