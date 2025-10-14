#include <Arduino.h>
#include <SoftwareSerial.h>
#include <ReefwingMSP.h>

// Pins for FC communication
#define RX_PIN 8
#define TX_PIN 9

SoftwareSerial fcSerial(RX_PIN, TX_PIN);
ReefwingMSP msp;
static unsigned long lastPoll = 0;

// Buffer for received payload
uint8_t payload[32];
uint8_t messageID;
uint8_t recvSize;

void setup() {
  Serial.begin(115200);       // USB Serial to PC
  fcSerial.begin(115200);     // FC UART
  delay(1000);

  Serial.println(F("Reefwing MSP Example - Nano <-> Betaflight FC"));

  msp.begin(fcSerial);        // start MSP
  delay(2000);

  // Request some telemetry
  msp.send(MSP_ATTITUDE, NULL, 0);
  msp.send(MSP_STATUS, NULL, 0);
}

void loop() {
  // Check for any received MSP message
  if (msp.recv(&messageID, payload, sizeof(payload), &recvSize)) {
    switch (messageID) {
      case MSP_ATTITUDE: {
        // Payload: 3 x int16_t (roll, pitch, yaw)
        int16_t roll  = payload[0] | (payload[1] << 8);
        int16_t pitch = payload[2] | (payload[3] << 8);
        int16_t yaw   = payload[4] | (payload[5] << 8);

        Serial.println(F("=== Attitude ==="));
        Serial.print(F("Roll: "));  Serial.println(roll / 10.0);
        Serial.print(F("Pitch: ")); Serial.println(pitch / 10.0);
        Serial.print(F("Yaw: "));   Serial.println(yaw);
        break;
      }

      case MSP_STATUS: {
        // Payload example: cycleTime (uint16_t), i2cErrCount (uint16_t)
        uint16_t cycleTime = payload[0] | (payload[1] << 8);
        uint16_t i2cErrs   = payload[2] | (payload[3] << 8);

        Serial.println(F("=== Status ==="));
        Serial.print(F("Cycle Time: ")); Serial.println(cycleTime);
        Serial.print(F("I2C Errors: ")); Serial.println(i2cErrs);
        break;
      }

      default:
        Serial.print(F("Unknown MSP ID: "));
        Serial.println(messageID);
        break;
    }
    Serial.print(F("Received MSP ID: "));
  }

  // Poll telemetry every 500ms
  if (millis() - lastPoll > 500) {
    lastPoll = millis();
    msp.send(MSP_ATTITUDE, NULL, 0);
  }
}
