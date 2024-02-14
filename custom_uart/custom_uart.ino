/* 
// This file is subject to the terms and conditions defined in
// file 'LICENSE.md', which is part of this source code package.
*/

/*

  The Apollo3 contains two UART peripherals that can be used with
  the Arduino Serial api.
  https://www.arduino.cc/reference/en/language/functions/communication/serial/
  
  It is possible to route each UART instance (0 or 1) to a number of 
  different pads of the Apollo3. Check the datasheet for all possible 
  mappings.
  https://cdn.sparkfun.com/assets/learn_tutorials/9/0/9/Apollo3_Blue_MCU_Data_Sheet_v0_9_1.pdf
  Table 559
  
  You can create new UART objects (which inherit from the HardwareSerial
  class in Arduino) by specifying the TX and RX pins, and optionally the 
  RTS and CTS pins to enable flow control. Here are the constructor protorypes:
  UART(PinName tx, PinName rx, PinName rts, PinName cts);
  UART(pin_size_t tx, pin_size_t rx, pin_size_t rts, pin_size_t cts);
  
  It is possible to use either the mbed style PinName enumerations or the
  Arduino style pin_size_t pin numbers to specify pins. Either case will
  will determine the actuall Apollo3 pad according to the rules explained
  in the DigitalGPIO example.

*/

/*
          Apollo3 UART pad mapping

       uart0         | |       uart1   
---------------------------------------------
 tx | rx | rts | cts | | tx | rx | rts | cts  
---- ---- ----- ----- - ---- ---- ----- -----
  1    2     3     4 | |  8    2    10    11
  7   11     5     6 | | 10    4    16    17
 16   17    13    12 | | 12    9    20    21
 20   21    18    24 | | 14   13    30    26
 22   23    34    29 | | 18   15    31    29
 26   27    35    33 | | 20   19    34    32
 28   29    37    36 | | 24   21    41    36
 30   31    41    38 | | 35   25    44    45
 39   34             | | 37   36            
 41   40             | | 39   38            
 44   45             | | 42   40            
 48   49             | | 46   43            
                     | |      47            
*/
#define BAUD 115200       // using 115200 baud rate

// A16 is the TX Pin, A0 is the RX Pin
UART NanoSerial(A16, A0);

// Custom UART Packet Design
struct SensorDataPacket {
  uint16_t sop;       // 2 bytes
  uint8_t datatype;   // 1 byte
  uint8_t sensorId;   // 1 byte
  uint8_t length;     // 1 byte
  uint32_t timestamp; // 4 bytes
  uint64_t data;      // 8 bytes
  uint8_t crc;        // 1 byte
  uint16_t eop;       // 2 bytes
                      // Total Size: 20 bytes
};

void setup() {
  NanoSerial.begin(BAUD);
  Serial.begin(BAUD);

  // Initialize random number generator with a unique seed
  randomSeed(analogRead(0));
}

void loop() {

  // Construct packet 
  SensorDataPacket packet;
  packet.sop = 0xABCD;
  packet.datatype = 2;
  packet.sensorId = 1;
  packet.length = sizeof(packet) - sizeof(packet.sop) - sizeof(packet.eop); // Length without SOP and EOP
  packet.timestamp = now();
  packet.data = (uint64_t)random(30, 91);
  packet.crc = calculateCRC((uint8_t*)&packet, sizeof(packet) - sizeof(packet.crc));
  packet.eop = 0xEF;

  // Print packet before sending
  // printSensorDataPacket(packet);

  // Send packet over UART (serial)
  Serial.write((uint8_t*)&packet, sizeof(packet));
  NanoSerial.write((uint8_t*)&packet, sizeof(packet));
  delay(1000);
}

// Placeholder function to return a timestamp (number of seconds since the Arduino started right now)
uint32_t now() {
  return millis() / 1000; // This will just return the number of seconds since the Arduino started
}

// Placeholder function for CRC calculation, need a CRC algorithm here eventually
uint8_t calculateCRC(uint8_t *data, size_t len) {
  uint8_t crc = 0;
  for (size_t i = 0; i < len; ++i) { // simple XOR checksum, not a real CRC
    crc ^= data[i];
  }
  return crc; 
}

// Print out the UART packet over serial for debugging purposes
void printSensorDataPacket(const SensorDataPacket& packet) {
  Serial.print("SOP: 0x"); Serial.println(packet.sop, HEX);
  Serial.print("Data Type: "); Serial.println(packet.datatype);
  Serial.print("Sensor ID: "); Serial.println(packet.sensorId);
  Serial.print("Length: "); Serial.println(packet.length);
  Serial.print("Timestamp: "); Serial.println(packet.timestamp);
  Serial.print("Data: "); Serial.println((long)packet.data);
  Serial.print("CRC: 0x"); Serial.println(packet.crc, HEX);
  Serial.print("EOP: 0x"); Serial.println(packet.eop, HEX);
  Serial.println();
}