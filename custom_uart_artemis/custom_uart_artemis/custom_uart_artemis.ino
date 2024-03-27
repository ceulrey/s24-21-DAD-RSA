/**
  ******************************************************************************
  * @file           : custom_uart.ino
  * @brief          : Packet transmission for the sensor nodes
  * @date			      : 2/20/24
  * @author			    : S24-21
  *******************************************************************************/

#define BAUD 115200       // using 115200 baud rate
#define CONFIG SERIAL_8N1 // a config value from HardwareSerial.h (defaults to SERIAL_8N1)
#include "Arduino.h"
// A16 is the TX Pin, A0 is the RX Pin
UART NanoSerial(A16, A0);

// Custom UART Packet Design
struct SensorDataPacket {
  uint8_t sop;        // 1 byte
  uint8_t datatype;   // 1 byte
  uint8_t sensorId;   // 1 byte
  uint32_t timestamp; // 4 bytes
  double data;        // 8 bytes
  uint8_t crc;        // 1 byte
  uint8_t eop;        // 1 byte
                      // Total Size: 17 bytes
};

void setup() {
  // NanoSerial.begin(BAUD, CONFIG);
  Serial.begin(BAUD, CONFIG);
  NanoSerial.begin(BAUD, CONFIG);

  // Initialize random number generator with a unique seed
  randomSeed(analogRead(0));
}

void loop() {

  // Construct packet 
  SensorDataPacket packet;
  packet.sop = 0x53;                                                                 // Unique Start Byte ('S' in ASCII)
  packet.datatype = 0b11;                                                            // Data Type: Temp = 00, Humidity = 01, Sound = 10, Vibration = 11
  packet.sensorId = 0b111;                                                           // USART Port Connected To: 000, 001, 010, 011, 100, 101, 110, 111 (i.e. Sensor 1-8)
  packet.timestamp = now();                                                          // Time when Data Captured
  packet.data = random(10, 5000) / 100.0;                                              // Data Field
  packet.crc = calculateCRC((uint8_t*)&packet, sizeof(packet) - sizeof(packet.crc)); // CRC for Error Checking
  packet.eop = 0x45;                                                                 // Stop Byte ('E' in ASCII)

  // Print packet before sending
  printSensorDataPacket(packet);

  // Send packet over UART (serial)
  sendSensorDataPacket(packet);
  delay(3000); // delay 1 second
}

// Placeholder function to return a timestamp (number of seconds since the Arduino started)
uint32_t now() {
  return (uint32_t) millis() / 1000; // in secs
}

// Placeholder function for CRC calculation, need a CRC algorithm here eventually
uint8_t calculateCRC(uint8_t *data, size_t len) {
  uint8_t crc = 0;
  for (size_t i = 0; i < len; ++i) { // simple XOR checksum, not a real CRC
    crc ^= data[i];
  }
  return crc; 
}

// Function to send a packet
void sendSensorDataPacket(SensorDataPacket& packet) {
  Serial.write((uint8_t*)&packet, sizeof(packet));
  // NanoSerial.write((uint8_t*)&packet, sizeof(packet));
  NanoSerial.write((uint8_t*)&packet, sizeof(packet));
}

// void sendSensorDataPacket(SensorDataPacket& packet) {
//   uint8_t *packetPointer = (uint8_t*)&packet;
//   for (size_t i = 0; i < sizeof(packet); i++) {
//     Serial.write(packetPointer[i]);
//     Serial1.write(packetPointer[i]);
//   }
// }

// Print out the UART packet over serial for debugging purposes
void printSensorDataPacket(const SensorDataPacket& packet) {
  Serial.print("\nSOP: 0x"); Serial.println(packet.sop, HEX);
  Serial.print("Data Type: "); Serial.println(packet.datatype);
  Serial.print("Sensor ID: "); Serial.println(packet.sensorId);
  Serial.print("Timestamp: "); Serial.println(packet.timestamp);
  Serial.print("Data: "); Serial.println(packet.data);
  Serial.print("CRC: 0x"); Serial.println(packet.crc, HEX);
  Serial.print("EOP: 0x"); Serial.println(packet.eop, HEX);
}  
