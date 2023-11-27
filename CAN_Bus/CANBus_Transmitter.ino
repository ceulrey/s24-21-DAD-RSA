#include <SPI.h>          //Library for using SPI Communication 
#include <mcp2515.h>      //Library for using CAN Communication (https://github.com/autowp/arduino-mcp2515/)
#include <OneWire.h>
#include <DallasTemperature.h>

// Pin used for one wire communication on the ExpressIf ESP32, note that pin 2 is reserved
#define ONE_WIRE_BUS (17)

// Desired time interval between measurements (in milliseconds)
#define DATA_INTERVAL (2000)

// Set to 'true' to print all outputs over USB serial, 'false' to print only the sensor data
#define PRINT_ALL (false)

// The LED is used to indicate an ERROR, note that some ESP32 modules have an on-board LED
#define LED_PIN (19)

// The OneWire instance used for the Dallas sensor
OneWire oneWire(ONE_WIRE_BUS);

// The Sensor instance
DallasTemperature sensor(&oneWire);

// To store the sensor's address per OneWire protocol
DeviceAddress deviceAddress;
 
struct can_frame canMsg;
 
MCP2515 mcp2515(5); 
 
void setup()
{
  while (!Serial);
  Serial.begin(9600);
  SPI.begin();               //Begins SPI communication
  sensor.begin();               //Begins to read temperature & humidity sesnor value
 
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ); //Sets CAN at speed 500KBPS and Clock 8MHz
  mcp2515.setNormalMode();
}
 
 
void loop()
{
  sensor.requestTemperatures();    //Gets Temperature value
  float t = sensor.getTempF(deviceAddress);
  Serial.println(t);
  canMsg.can_id  = 0x036;           //CAN id as 0x036
  canMsg.can_dlc = 8;               //CAN data length as 8
  canMsg.data[0] = t;               //Update humidity value in [0]
  canMsg.data[1] = 0x00;               //Update temperature value in [1]
  canMsg.data[2] = 0x00;            //Rest all with 0
  canMsg.data[3] = 0x00;
  canMsg.data[4] = 0x00;
  canMsg.data[5] = 0x00;
  canMsg.data[6] = 0x00;
  canMsg.data[7] = 0x00;
 
  mcp2515.sendMessage(&canMsg);     //Sends the CAN message
  delay(1000);
}