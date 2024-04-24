#include <Nextion.h>

// Nextion objects: pageID, componentID, componentName
NexText temperature_C = NexText(0, 6, "t4");
NexText temperature_F = NexText(0, 7, "t5");
NexText humidity_RH = NexText(0, 8, "t6");
NexText vibration_X = NexText(0, 9, "t7");
NexText vibration_Y = NexText(0, 10, "t8");
NexText vibration_Z = NexText(0, 11, "t9");
NexText sound_dB = NexText(0, 12, "t10");

void setup() {
  Serial.begin(9600);
  nexInit();
}

void loop() {
  double temperature = 22.3;
  double humidity = 50.27;
  double x = 0;
  double y = 0;
  double z = 1;
  double dB = 30.15;

  setTemperature(temperature);
  setHumidity(humidity);
  setVibration(x, y, z);
  setSound(dB);

  delay(10);
}

void setTemperature(double t) {
  String C = String(t) + " C";
  String F = String( (t*9/5) + 32 ) + " F";

  temperature_C.setText(C.c_str());
  temperature_F.setText(F.c_str());
}

void setHumidity(double h) {
  String RH = String(h) + "%";

  humidity_RH.setText(RH.c_str());
}

void setVibration(double x, double y, double z) {
  String X = "X: " + String(x) + " g";
  String Y = "Y: " + String(y) + " g";
  String Z = "Z: " + String(z) + " g";

  vibration_X.setText(X.c_str());
  vibration_Y.setText(Y.c_str());
  vibration_Z.setText(Z.c_str());
}

void setSound(double d) {
  String dB = String(d) + " dB";

  sound_dB.setText(dB.c_str());
}
