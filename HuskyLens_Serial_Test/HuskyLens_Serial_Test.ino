// Serial monitor test code
#include "HUSKYLENS.h"
#include <Wire.h>

HUSKYLENS huskylens;

// -------------------- HuskyLens screen center --------------------
const int screenCenterX = 160;

// -------------------- Dead zone (just for reference) --------------------
const int deadZone = 20;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  Serial.println("Initializing HuskyLens...");

  // Try to connect to HuskyLens
  while (!huskylens.begin(Wire)) {
    Serial.println("HuskyLens not connected! Check wiring.");
    delay(1000);
  }

  Serial.println("HuskyLens connected!");

  // Set to Object Tracking mode
  huskylens.writeAlgorithm(ALGORITHM_OBJECT_TRACKING);

  Serial.println("Teach your object now.");
  Serial.println("Place object exactly 30 cm from camera.");
  Serial.println("----------------------------------------");

  delay(2000);
}

void loop() {

  // Request data
  if (!huskylens.request()) {
    Serial.println("❌ Request failed");
    delay(200);
    return;
  }

  // No object detected
  if (!huskylens.available()) {
    Serial.println("👀 No object detected");
    delay(200);
    return;
  }

  // Read detected object
  HUSKYLENSResult result = huskylens.read();

  int x = result.xCenter;
  int width = result.width;
  int height = result.height;

  // -------------------- PRINT VALUES --------------------
  Serial.print("Object X: ");
  Serial.print(x);

  Serial.print(" | Width: ");
  Serial.print(width);

  Serial.print(" | Height: ");
  Serial.println(height);

  // -------------------- OPTIONAL DIRECTION CHECK --------------------
  if (x < screenCenterX - deadZone) {
    Serial.println("→ LEFT side of screen");
  }
  else if (x > screenCenterX + deadZone) {
    Serial.println("→ RIGHT side of screen");
  }
  else {
    Serial.println("→ CENTERED");
  }

  Serial.println("----------------------------------------");

  delay(200);
}
