#define x_pin A0
#define y_pin A1
#define z_pin A2

#define raw_min 0
#define raw_max 1023

// Multiple samples to reduce noise
int sample_size = 10;

void setup()
{
  // For Arduino debugging
  analogReference(EXTERNAL);
  
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
  int raw_x = read_axis(x_pin);
  int raw_y = read_axis(y_pin);
  int raw_z = read_axis(z_pin);

  // Convert to milli-Gs (-3g to +3gs)
  long x_scaled = map(raw_x, raw_min, raw_max, -3000.0, 3000.0);
  long y_scaled = map(raw_y, raw_min, raw_max, -3000.0, 3000.0);
  long z_scaled = map(raw_z, raw_min, raw_max, -3000.0, 3000.0);

  // Rescale to fractional Gs
  double x = x_scaled / 1000.0;
  double y = y_scaled / 1000.0;
  double z = z_scaled / 1000.0;

  Serial.print("X, Y, Z  :: ");
	Serial.print(raw_x);
	Serial.print(", ");
	Serial.print(raw_y);
	Serial.print(", ");
	Serial.print(raw_z);
	Serial.print(" :: ");
	Serial.print(x);
	Serial.print("G, ");
	Serial.print(y);
	Serial.print("G, ");
	Serial.print(z);
	Serial.println("G");

	delay(100);
}

int read_axis(int pin)
{
  long reading = 0;

  // Delayed initial reading
  analogRead(pin);
  delay(1);

  // // Read samples
  // for (int i = 0; i < sample_size; i++) {
  //   reading += analogRead(pin);
  // }

  // return reading / sample_size;

  return analogRead(pin);
}