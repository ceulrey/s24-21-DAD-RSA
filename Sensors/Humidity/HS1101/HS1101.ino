#define sensor_pin 4
#define RH_constant 12169 // Relative Humidity Constant * 10

double H;

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  double RC_delay = RCTime();
  // double RC_delay = RC_delay_cycles * (1 / 16000000.0);

  // RC_delay *= 200;

  Serial.println(RC_delay, 20);

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

  // Wait for 500 ms to charge capacitor
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
  return (stop - start) / 1000000.0;
}