/**
 * Artemis ADC Adjustment
 * ^^^^^^^^^^^^^^^^^^^^^^
 * Outputs from ADXL335 are on a scale from 0 V - 3.3 V
 *    Artemis ADC input only reads from 0 V - 2 V
 * 
 * To fix this, put a 32k Ohm resistor on each output to ground and read
 * data from the output of the ADXL335. The ADXL335 has built in 32k Ohm
 * resistors on each output. So, putting an extra 32k Ohm resistor on the
 * output and measuring input between them effectively creates a voltage
 * divider that cuts the voltage in half to make the ADXL335 output range
 * of 0 V - 1.65 V.
 * 
 * ADXL335
 * ^^^^^^^
 * x:
 *    0.330 V (330 mV) per G Force
 *    0 G is at 1.658 V
 * y:
 *    0.338 V (338 mV) per G Force
 *    0 G is at 1.630 V
 * z:
 *    0.330 V (330 mV) per G Force
 *    0 G is at 1.668 V
 * 
 * Due to Artemis ADC Adjustment, all of these values will be cut in half.
**/

// Pin Definitions
#define x_pin A2
#define y_pin A3
#define z_pin A5

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
  uint64_t data;      // 8 bytes
  uint8_t crc;        // 1 byte
  uint8_t eop;        // 1 byte
                      // Total Size: 17 bytes
};

// Variable Definitions
const double x_VperG = 0.330 / 2.0;
const double x_zeroG = 1.658 / 2.0;
const double y_VperG = 0.338 / 2.0;
const double y_zeroG = 1.630 / 2.0;
const double z_VperG = 0.300 / 2.0;
const double z_zeroG = 1.668 / 2.0;
const double sample_size = 1;

// ADC Definitions
const int adc_resolution = 14;
const double max_input = pow(2, adc_resolution);
const double max_voltage = 2.0;

// Voltage Calculation Variables Declarations
int raw_x;
int raw_y;
int raw_z;
double x_voltage;
double y_voltage;
double z_voltage;

// Calculated Variables Declarations
double x;
double y;
double z;

void setup()
{
  // Set the ADC resolution
  analogReadResolution(adc_resolution);

  // Setup pins
  pinMode(x_pin, INPUT);
  pinMode(y_pin, INPUT);
  pinMode(z_pin, INPUT);

  // Begin transmission
  Serial.begin(BAUD, CONFIG);
  NanoSerial.begin(BAUD, CONFIG);
}

void loop()
{
  // Read raw values
  find_raw_inputs();

  // Convert raw values to voltages
  x_voltage = convert_to_voltage(raw_x);
  y_voltage = convert_to_voltage(raw_y);
  z_voltage = convert_to_voltage(raw_z);

  // Find the G Forces in each direction
  find_G_Forces();

  // Construct packet 
  SensorDataPacket packet;
  packet.sop = 0x53;                                                                 // Unique Start Byte ('S' in ASCII)
  packet.datatype = 0b10;                                                            // Data Type: Temp = 00, Humidity = 01, Sound = 10, Vibration = 11
  packet.sensorId = 0b011;                                                           // USART Port Connected To: 000, 001, 010, 011, 100, 101, 110, 111 (i.e. Sensor 1-8)
  packet.timestamp = now();                                                          // Time when Data Captured
  packet.data = (uint64_t) z;                                                        // Data Field
  packet.crc = calculateCRC((uint8_t*)&packet, sizeof(packet) - sizeof(packet.crc)); // CRC for Error Checking
  packet.eop = 0x45; 

  // Print raw values for debugging
  // print_raw_values();

  // Print G Forces
  // print_G_Forces();

  // Print for Serial Plotter
  print_for_plotter();

  printSensorDataPacket(packet);

  sendSensorDataPacket(packet);

	delay(250);
}

void find_raw_inputs()
{
  // Reset variables
  raw_x = 0;
  raw_y = 0;
  raw_z = 0;

  // Loop to get raw inputs
  for (int i = 0; i < sample_size; i++) {
    raw_x += analogRead(x_pin);
    raw_y += analogRead(y_pin);
    raw_z += analogRead(z_pin);
  }

  // Find average
  raw_x /= sample_size;
  raw_y /= sample_size;
  raw_z /= sample_size;
}

double convert_to_voltage(int raw)
{
  return (raw / max_input) * max_voltage;
}

void find_G_Forces()
{
  x = (x_voltage - x_zeroG) / x_VperG;
  y = (y_voltage - y_zeroG) / y_VperG;
  z = (z_voltage - z_zeroG) / z_VperG;
}

void print_G_Forces()
{
  Serial.print("X: ");
  Serial.print(x);
  Serial.print(" G\t");

  Serial.print("Y: ");
  Serial.print(y);
  Serial.print(" G\t");

  Serial.print("Z: ");
  Serial.print(z);
  Serial.println(" G");

  Serial.println();
}

/* Debugging */
void print_raw_values()
{
  Serial.print("Raw X: ");
  Serial.print(raw_x);
  Serial.print("\tVoltage X: ");
  Serial.print(x_voltage, 10);
  Serial.print("\tG Force X: ");
  Serial.println(x, 10);

  Serial.print("Raw Y: ");
  Serial.print(raw_y);
  Serial.print("\tVoltage Y: ");
  Serial.print(y_voltage, 10);
  Serial.print("\tG Force Y: ");
  Serial.println(y, 10);

  Serial.print("Raw Z: ");
  Serial.print(raw_z);
  Serial.print("\tVoltage Z: ");
  Serial.print(z_voltage, 10);
  Serial.print("\tG Force Z: ");
  Serial.println(z, 10);

  Serial.println();
}

/* Debugging */
void print_for_plotter()
{
  Serial.print(x);
  Serial.print(" ");
  Serial.print(y);
  Serial.print(" ");
  Serial.println(z);
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
  Serial.print("Data: "); Serial.println((long)packet.data);
  Serial.print("CRC: 0x"); Serial.println(packet.crc, HEX);
  Serial.print("EOP: 0x"); Serial.println(packet.eop, HEX);
}  
