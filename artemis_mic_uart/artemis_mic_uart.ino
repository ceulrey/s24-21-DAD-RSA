/**
  ******************************************************************************
  * @file           : custom_uart.ino
  * @brief          : Packet transmission for the sensor nodes
  * @date           : 2/20/24
  * @author         : S24-21
  ******************************************************************************
*/

#define BAUD 921600       // using 115200 baud rate
#define CONFIG SERIAL_8N1 // a config value from HardwareSerial.h (defaults to SERIAL_8N1)
#include "Arduino.h"

const pin_size_t TX_PIN = 9;  // Example TX Pin
const pin_size_t RX_PIN = 10; // Example RX Pin

UART NanoSerial(A16, A0);  // Custom UART setup (using pins A16 for TX, A0 for RX)

// Custom UART Packet Design
struct SensorDataPacket {
  uint8_t sop;         // Start of packet
  uint8_t datatype;    // Data type (1 byte)
  uint8_t sensorId;    // Sensor ID (1 byte)
  uint32_t timestamp;  // Timestamp in milliseconds (4 bytes)
  int64_t data;        // Sensor data (8 bytes, fixed-point format)
  uint8_t crc;         // CRC for error checking (1 byte)
  uint8_t eop;         // End of packet (1 byte)
};

void setup() {
  Serial.begin(BAUD, CONFIG);
  NanoSerial.begin(BAUD, CONFIG);
}

void loop() {
  static unsigned long lastSampleTime = 0;  // Stores the last sample time in milliseconds
  unsigned long currentMillis = millis();   // Current time in milliseconds

  // Sampling period in milliseconds for 44.1 kHz sampling rate (T = 1/54211 = 0.0000184 s per sample) 
  const unsigned long samplingPeriod = 18;  // Approximately equals to (1 / 54211) * 1000 = 18.4 ms

  if (currentMillis - lastSampleTime >= samplingPeriod) {
    lastSampleTime += samplingPeriod;  // Update the last sample time to maintain consistent sampling intervals

    int S = analogRead(A15);  // Read the sound sample
    // int64_t fixedPointData = static_cast<int64_t>(S * 100);  // Convert to fixed-point format

    // Construct the packet
    SensorDataPacket packet;
    packet.sop = 0x53;  // 'S' Start of packet
    packet.datatype = 0b10;  // Data Type: Temp = 00, Humidity = 01, Sound = 10, Vibration = 11
    packet.sensorId = 0b100;  // USART Port Connected To: 000, 001, 010, 011, 100, 101, 110, 111 (i.e. Sensor 1-8)
    packet.timestamp = currentMillis;  // Timestamp in milliseconds
    packet.data = S;  // Sound data in fixed-point
    packet.crc = calculateCRC((uint8_t*)&packet, sizeof(packet) - sizeof(packet.crc));  // CRC
    packet.eop = 0x45;  // 'E' End of packet

    sendSensorDataPacket(packet);  // Send the packet
    printSensorDataPacket(packet);  // Print packet details for debugging
  }
}

uint8_t calculateCRC(uint8_t *data, size_t len) {
  uint8_t crc = 0;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
  }
  return crc;
}

void sendSensorDataPacket(SensorDataPacket& packet) {
  NanoSerial.write((uint8_t*)&packet, sizeof(packet));
}

void printSensorDataPacket(const SensorDataPacket& packet) {
  Serial.print("\nSOP: 0x"); Serial.println(packet.sop, HEX);
  Serial.print("Data Type: "); Serial.println(packet.datatype);
  Serial.print("Sensor ID: "); Serial.println(packet.sensorId);
  Serial.print("Timestamp (ms): "); Serial.println(packet.timestamp);
  Serial.print("Data (fixed-point): "); Serial.println(packet.data);
  Serial.print("CRC: 0x"); Serial.println(packet.crc, HEX);
  Serial.print("EOP: 0x"); Serial.println(packet.eop, HEX);
}


