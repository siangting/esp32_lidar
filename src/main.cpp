#include <Arduino.h>

#define LIDAR_RX 16
#define LIDAR_TX 17
#define MOTOR_CTRL_PIN 4

HardwareSerial LIDARSerial(1);

void sendResetCommand() {
  LIDARSerial.write(0xA5);
  LIDARSerial.write(0x40); // Reset
  delay(100); // Let it settle
}

void sendScanCommand() {
  LIDARSerial.write(0xA5);
  LIDARSerial.write(0x21); // Normal Scan
}


void sendHealthCheck() {
  LIDARSerial.write(0xA5);
  LIDARSerial.write(0x52); // Get health

  delay(100);

  Serial.println("Reading Health Info:");
  while (LIDARSerial.available()) {
    uint8_t b = LIDARSerial.read();
    Serial.printf("%02X ", b);
  }
  Serial.println();
}

void setup() {
  Serial.begin(256000);
  LIDARSerial.begin(256000, SERIAL_8N1, LIDAR_RX, LIDAR_TX);

  pinMode(MOTOR_CTRL_PIN, OUTPUT);
  digitalWrite(MOTOR_CTRL_PIN, HIGH); // Start motor

  delay(1000);  // let motor stabilize

  sendResetCommand();      // Reset lidar
  delay(200);              // wait after reset

  // Flush any leftover ASCII banner
  Serial.println("Flushing banner...");
  unsigned long flushStart = millis();
  while (millis() - flushStart < 500) {
    while (LIDARSerial.available()) {
      LIDARSerial.read();  // discard
    }
  }

  sendScanCommand();       // Now send scan
  Serial.println("Sent scan command to RPLIDAR A2M12");

  // Wait for descriptor
  Serial.println("Waiting for descriptor...");
  int desc_count = 0;
  while (desc_count < 7) {
    if (LIDARSerial.available()) {
      uint8_t d = LIDARSerial.read();
      Serial.printf("%02X ", d);
      desc_count++;
    }
  }

  Serial.println("\nDescriptor received. Now start parsing scan data...");
}




void loop() {
  static uint8_t buffer[5];
  static int index = 0;

  while (LIDARSerial.available()) {
    uint8_t byteRead = LIDARSerial.read();

    // Shift bytes until we find a valid start flag
    if (index == 0) {
      // Wait for start flags: bit 0 == 1 AND bit 1 == 0
      if ((byteRead & 0x01) && !(byteRead & 0x02)) {
        buffer[index++] = byteRead;
      }
    } else {
      buffer[index++] = byteRead;
    }

    if (index == 5) {
      index = 0;

      // Parse 5-byte scan point
      uint16_t quality = (buffer[0] >> 2);
      uint16_t angle_raw = buffer[1] | (buffer[2] << 8);
      uint16_t distance_raw = buffer[3] | (buffer[4] << 8);

      float angle = (angle_raw >> 1) / 64.0;
      float distance = distance_raw / 4.0;

      Serial.print("Angle: ");
      Serial.print(angle, 2);
      Serial.print(" deg, Distance: ");
      Serial.print(distance, 2);
      Serial.print(" mm, Quality: ");
      Serial.println(quality);
      delay(100);
    }
  }

}
