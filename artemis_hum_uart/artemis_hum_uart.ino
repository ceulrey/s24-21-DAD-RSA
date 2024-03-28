#define sensor_pin A2
#define RH_constant 2745 // Relative Humidity Constant * 10

#define BAUD 115200       // using 115200 baud rate
#define CONFIG SERIAL_8N1 // a config value from HardwareSerial.h (defaults to SERIAL_8N1)
#include "Arduino.h"

UART NanoSerial(A16, A0);

// Custom UART Packet Design
struct SensorDataPacket {
  uint8_t sop;        // 1 byte
  uint8_t datatype;   // 1 byte
  uint8_t sensorId;   // 1 byte
  uint32_t timestamp; // 4 bytes
  int64_t data;      // 8 bytes
  uint8_t crc;        // 1 byte
  uint8_t eop;        // 1 byte
                      // Total Size: 17 bytes
};

double H;

void setup()
{
  Serial.begin(BAUD, CONFIG);
  NanoSerial.begin(BAUD, CONFIG);
}

void loop()
{
  double T_decay = RCTime();

  double humidity = (T_decay - RH_constant) / 24;

  // Convert humidity to fixed-point representation
  int64_t fixedPointData = static_cast<int64_t>(humidity * 100);  // Assuming T is your temperature in Celsius

  // Construct packet 
  SensorDataPacket packet;
  packet.sop = 0x53;                                                                 // Unique Start Byte ('S' in ASCII)
  packet.datatype = 0b01;                                                            // Data Type: Temp = 00, Humidity = 01, Sound = 10, Vibration = 11
  packet.sensorId = 0b010;                                                           // USART Port Connected To: 000, 001, 010, 011, 100, 101, 110, 111 (i.e. Sensor 1-8)
  packet.timestamp = now();                                                          // Time when Data Captured
  // packet.data = humidity;                                                            // Data Field
  packet.data = fixedPointData;                                                               // Data Field
  packet.crc = calculateCRC((uint8_t*)&packet, sizeof(packet) - sizeof(packet.crc)); // CRC for Error Checking
  packet.eop = 0x45;   
  
  Serial.println(humidity, 10);

  // Print packet before sending
  printSensorDataPacket(packet);

  sendSensorDataPacket(packet);

  delay(500);
}

double RCTime()
{
  // Start and stop time declarations
  unsigned long start;
  unsigned long stop;

  // Set the pin to output high
  pinMode(sensor_pin, OUTPUT);
  digitalWrite(sensor_pin, HIGH);

  // Wait to charge capacitor
  delay(500);

  // Set the pin the input and turn off internal pullup resistor
  pinMode(sensor_pin, INPUT);
  digitalWrite(sensor_pin, LOW);

  // Start time
  start = micros();

  // Loop until the pin goes low
  while (digitalRead(sensor_pin)) {
    // Do nothing
  }

  // Stop time
  stop = micros();

  // Return discharge time in seconds
  return (stop - start) * 2 * 10;
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

// Print out the UART packet over serial for debugging purposes
void printSensorDataPacket(const SensorDataPacket& packet) {
  Serial.print("\nSOP: 0x"); Serial.println(packet.sop, HEX);
  Serial.print("Data Type: "); Serial.println(packet.datatype);
  Serial.print("Sensor ID: "); Serial.println(packet.sensorId);
  Serial.print("Timestamp: "); Serial.println(packet.timestamp);
  Serial.print("Humidity: "); Serial.println(packet.data);
  Serial.print("CRC: 0x"); Serial.println(packet.crc, HEX);
  Serial.print("EOP: 0x"); Serial.println(packet.eop, HEX);
}  