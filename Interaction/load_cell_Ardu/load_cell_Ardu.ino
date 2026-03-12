#include "HX711.h"

#define DT  3
#define SCK 2

HX711 scale;

float calibration_factor = 317.51;
float filtered_weight = 0.0;
const float alpha = 0.8;

void setup() {
  Serial.begin(9600);
  scale.begin(DT, SCK);
  scale.set_scale(calibration_factor);
  scale.tare();  // start with zero
  Serial.println("Scale ready...");
}

void loop() {
  // --- Weight measurement ---
  float reading = scale.get_units(1); // average of 5 samples
  filtered_weight = alpha * reading + (1 - alpha) * filtered_weight;
  Serial.println(filtered_weight, 2);  // send weight to Python

  // --- Check serial commands ---
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();  // remove newline/whitespace

    if (cmd == "tare") {
      scale.tare();
      filtered_weight = 0;
      // Serial.println("OK: tare done");
    } 
    else if (cmd == "cal") {
      // you could add calibration commands here
      // Serial.println("OK: calibration placeholder");
    }
  }

  // delay(0); // adjust as needed
}
