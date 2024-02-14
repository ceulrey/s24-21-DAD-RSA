/**
 * Wheatstone Bridge Formula
 * ^^^^^^^^^^^^^^^^^^^^^^^^^
 * Vg = Vs [ (Rx / R3+Rx) - (R2 / R1+R2) ]
 * 
 * Vg:  output voltage across bridge
 * Vs:  supply voltage
 * R1:  500ohms (470ohms for testing)
 * R2:  500ohms (470ohms for testing)
 * R3:  500ohms (470ohms for testing)
 * Rx:  RTD Sensor
 * 
 * Rearranged to solve for Rx
 * ^^^^^^^^^^^^^^^^^^^^^^^^^^
 * Rx = -R3 * [ (R1*Vg + R2*Vg + R2*Vs) / (R1*Vg + R2*Vg - R1*Vs) ]
 * 
 * Formula to calculate temperature (in degrees Celsius)
 * ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
 * T >= 0 degrees Celsius:
 *    RT = R0 * [1 + a*T + b*(T^2)]
 * T < 0 degrees Celsius:
 *    RT = R0 * [1 + a*T + b*(T^2) + c*(T-100)*(T^3)]
 * 
 * T:   output temperature
 * R0:  resistance at 0 degrees Celsius (1000 ohms for NB-PTCO-168)
 * RT:  current resistance across the NB-PTCO-168 (RT == Rx)
 * a:   3.9083*(10^-3)
 * b:   -5.775*(10^-7)
 * c:   -4.183*(10^-12)
 * 
 * Rearranged to solve for T
 * ^^^^^^^^^^^^^^^^^^^^^^^^^
 * T >= 0 degrees Celsius:
 *    T = [ -a*R0 + sqrt(R0)*sqrt((a^2)*R0 - 4*b*R0 + 4*b*RT) ] / [ 2*b*R0 ]
 * T < 0 degrees Celsius:
 *    Very long equation
**/

// Pin Definitions
#define rtd_pin A2
#define ref_pin A3

#define BAUD 115200       // any number, common choices: 9600, 115200, 230400, 921600
#define CONFIG SERIAL_8N1 // a config value from HardwareSerial.h (defaults to SERIAL_8N1)
// A16 is the TX Pin, A0 is the RX Pin
UART NanoSerial(A16, A0);

// Variable Definitions
const double a = 3.9083 * pow(10, -3);
const double b = -5.775 * pow(10, -7);
const double c = -4.183 * pow(10, -12);

// Voltage and Resistance Definitions
const double Vs = 3.3;
const double R0 = 1000.0;
const double R1 = 470.0;
const double R2 = 910.0;
const double R3 = 470.0;

// Calculated Variables Declarations
double Vg;
double Rx;
double T;

void setup()
{
  // Setup pins
  pinMode(rtd_pin, INPUT);
  pinMode(ref_pin, INPUT);

  // Begin transmission
  Serial.begin(BAUD);
  NanoSerial.begin(BAUD);
}

void loop()
{
  // Read raw values
  int raw_rtd = analogRead(rtd_pin);
  int raw_ref = analogRead(ref_pin);

  // Print raw values
  print_raw_values(raw_rtd, raw_ref);

  // Convert raw values to voltage
  double rtd_voltage = convert_to_voltage(raw_rtd);
  double ref_voltage = convert_to_voltage(raw_ref);

  // Calculate voltage difference at the Wheatstone Bridge Terminals
  Vg = rtd_voltage - ref_voltage;
  // Vg = convert_to_voltage(raw_rtd - raw_ref);
  // Serial.println(Vg, 10);

  // Find the resistance across the RTD
  Rx = find_resistance(Vg);
  
  // Find the temperature in degrees Celsius
  T = find_temperature(Rx);
  
  // Print the temperature in Celsius and Fahrenheit
  print_temperature(T);

	delay(500);
}

void print_raw_values(int raw_rtd, int raw_ref)
{
  Serial.print("Half Input: \t\t");
  Serial.print(3.3 / 5.0 * 1024.0 / 2.0);
  Serial.print("\tHalf Voltage: \t\t");
  Serial.println(3.3 / 2.0, 10);

  Serial.print("Raw RTD Input: \t\t");
  Serial.print(raw_rtd);
  Serial.print("\tRaw RTD Voltage: \t");
  Serial.println(raw_rtd / 1024.0 * 2.0, 10);

  Serial.print("Raw Ref Input: \t\t");
  Serial.print(raw_ref);
  Serial.print("\tRaw Ref Voltage: \t");
  Serial.println(raw_ref / 1024.0 * 2.0, 10);

  Serial.print("Input Difference: \t");
  Serial.print(raw_rtd - raw_ref);
  Serial.print("\tVoltage Difference: \t");
  Serial.println((raw_rtd - raw_ref) / 1024.0 * 2.0, 10);

  Serial.println();
}

double convert_to_voltage(int raw)
{
  return (raw / 1024.0) * 2.0;
}

double find_resistance(double Vg)
{
  return -R3 * ( (R1*Vg + R2*Vg + R2*Vs) / (R1*Vg + R2*Vg - R1*Vs) );
}

double find_temperature(double RT)
{
  return ( -a*R0 + sqrt(R0)*sqrt(pow(a, 2)*R0 - 4*b*R0 + 4*b*RT) ) / ( 2*b*R0 );
}

void print_temperature(double T)
{
  Serial.print("C: ");
  Serial.println(T, 10);
  Serial.print("F: ");
  Serial.println(T * (9.0/5.0) + 32, 10);
  NanoSerial.print(T, 10);
  NanoSerial.print(" ");

  Serial.println();
}