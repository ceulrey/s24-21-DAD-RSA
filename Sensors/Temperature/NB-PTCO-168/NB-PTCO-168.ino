/**
 * Wheatstone Bridge Formula
 * ^^^^^^^^^^^^^^^^^^^^^^^^^
 * Vg = Vs [ (Rx / R3+Rx) - (R2 / R1+R2) ]
 * 
 * Vg:  output voltage across bridge
 * Vs:  supply voltage
 * R1:  1000 Ohms
 * R2:   500 Ohms (470 Ohms for testing)
 * R3:  1000 Ohms
 * Rx:  RTD Sensor
 * 
 * Resistor Positions
 * ^^^^^^^^^^^^^^^^^^
 *      R1 R3
 *      R2 Rx
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
 * 
 * NB-PTCO-168
 * ^^^^^^^^^^^
 * Range for our application:
 *    Low                   Base                    High
 *    -20 Celsius           0 Celsius               +60 Celsius
 *    -4 Fahrenheit         +32 Fahrenheit          +140 Fahrenheit
 *    922.0609843 Ohms      1000 Ohms               1232.419 Ohms
**/

// Pin Definitions
#define rtd_pin A2
#define ref_pin A3

// Variable Definitions
const double a = 3.9083 * pow(10, -3);
const double b = -5.775 * pow(10, -7);
const double c = -4.183 * pow(10, -12);
const double sample_size = 100;

// ADC Definitions
const int adc_resolution = 14;
const double max_input = pow(2, adc_resolution);
const double max_voltage = 2.0;

// Voltage and Resistance Definitions
const double Vs = 3.3;
const double R0 = 1000.0;
const double R1 = 1000.0;
const double R2 = 470.0;
const double R3 = 1000.0;

// Voltage Calculation Variables Declarations
int raw_rtd;
int raw_ref;
double rtd_voltage;
double ref_voltage;

// Calculated Variables Declarations
double Vg;
double Rx;
double T;

void setup()
{
  // Set the ADC resolution
  analogReadResolution(adc_resolution);

  // Setup pins
  pinMode(rtd_pin, INPUT);
  pinMode(ref_pin, INPUT);

  // Begin transmission
  Serial.begin(9600);
}

void loop()
{
  // Find raw inputs for both Wheatstone Bridge Terminals
  find_raw_inputs();

  // Convert raw values to voltages
  rtd_voltage = convert_to_voltage(raw_rtd);
  ref_voltage = convert_to_voltage(raw_ref);

  // Calculate voltage difference at the Wheatstone Bridge Terminals
  Vg = rtd_voltage - ref_voltage;

  // Print raw values for debugging
  // print_raw_values();

  // Find the resistance across the RTD
  Rx = find_resistance(Vg);
  
  // Find the temperature in degrees Celsius
  T = find_temperature(Rx);
  
  // Print the temperature in Celsius and Fahrenheit
  print_temperature(T);

	delay(200);
}

void find_raw_inputs()
{
  // Reset variables
  raw_rtd = 0;
  raw_ref = 0;

  // Loop to get raw inputs
  for (int i = 0; i < sample_size; i++) {
    raw_rtd += analogRead(rtd_pin);
    raw_ref += analogRead(ref_pin);
    // delay(1);
  }

  // Find average
  raw_rtd /= sample_size;
  raw_ref /= sample_size;
}

double convert_to_voltage(int raw)
{
  return (raw / max_input) * max_voltage;
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
  double C = T;
  double F = C * (9.0/5.0) + 32;

  Serial.print("C: ");
  Serial.println(C, 10);
  Serial.print("F: ");
  Serial.println(F, 10);

  Serial.println();
}

/* Debugging */
void print_raw_values()
{
  Serial.print("Correct Raw Ref: \t\t");
  Serial.print(max_input * ( R2 / (R1+R2) ));
  Serial.print("\tCorrect Raw Ref Voltage: \t\t");
  Serial.println(max_voltage / max_input * max_voltage, 10);

  Serial.print("Raw RTD Input: \t\t");
  Serial.print(raw_rtd);
  Serial.print("\tRaw RTD Voltage: \t");
  Serial.println(rtd_voltage, 10);

  Serial.print("Raw Ref Input: \t\t");
  Serial.print(raw_ref);
  Serial.print("\tRaw Ref Voltage: \t");
  Serial.println(ref_voltage, 10);

  Serial.print("Input Difference: \t");
  Serial.print(raw_rtd - raw_ref);
  Serial.print("\tVoltage Difference: \t");
  Serial.println(Vg, 10);

  Serial.println();
}