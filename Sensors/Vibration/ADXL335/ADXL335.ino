/**
 * Artemis ADC Adjustment
 * ^^^^^^^^^^^^^^^^^^^^^^
 * Outputs from ADXL335 are on a scale from 0 V - 3.3 V
 *    Artemis ADC input only reads from 0 V - 2 V
 * 
 * To fix this, use the outputs in each direction as the voltage source
 * for two series resistors with the same value. Read the voltage between
 * these resistors. This will cut all ADXL335 outputs in half, effectively
 * making the ADXL335 outputs on a scale from 0 V - 1.65 V which falls
 * within the Artemis ADC range.
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

// Variable Definitions
const double x_VperG = 0.330;
const double x_zeroG = 1.658;
const double y_VperG = 0.338;
const double y_zeroG = 1.630;
const double z_VperG = 0.330;
const double z_zeroG = 1.668;
const double sample_size = 100;

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
  Serial.begin(9600);
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

  // Print raw values for debugging
  // print_raw_values();

  // Print G Forces
  print_G_Forces();

	delay(1000);
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