#define sensor_pin A2
#define RH_constant 2000 // Relative Humidity Constant * 10

double H;

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  double T_decay = RCTime();

  double humidity = (T_decay - RH_constant) / 24;

  Serial.println(humidity, 10);

  delay(100);
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