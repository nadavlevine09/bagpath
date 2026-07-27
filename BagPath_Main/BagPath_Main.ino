#include "HUSKYLENS.h"
#include <Wire.h>

HUSKYLENS huskylens;

// ============================================================
// DEBUG
// ============================================================
const bool DEBUG = true;

// ============================================================
// L298N PINS
// IMPORTANT: with your new wiring:
//   RIGHT motor = OUT1 / OUT2 = ENA, IN1, IN2
//   LEFT  motor = OUT3 / OUT4 = ENB, IN3, IN4
// ============================================================
const int ENA = 5;   // RIGHT motor speed (PWM)
const int IN1 = 6;   // RIGHT motor direction
const int IN2 = 7;   // RIGHT motor direction

const int IN3 = 8;   // LEFT motor direction
const int IN4 = 9;   // LEFT motor direction
const int ENB = 10;  // LEFT motor speed (PWM)

// ============================================================
// OPTIONAL MOTOR REVERSAL
// Change to true if that side spins backward
// ============================================================
const bool REVERSE_LEFT_MOTOR  = false;
const bool REVERSE_RIGHT_MOTOR = false;

// ============================================================
// HUSKYLENS GEOMETRY
// ============================================================
const int screenCenterX = 160;

// ============================================================
// TARGET SETTINGS
// ============================================================
const int TARGET_ID = 1;   // set to -1 for any learned object

// ============================================================
// OBJECT REAL-WORLD SIZE (cm)
// ============================================================
const float REAL_W_CM = 7.0;
const float REAL_H_CM = 13.5;

// ============================================================
// CALIBRATION VALUES
// Measured at exactly 30 cm from HuskyLens lens
// ============================================================
const float CAL_DIST_CM = 30.0;
const float CAL_W_PX    = 173.0;
const float CAL_H_PX    = 173.0;

// Effective focal lengths
const float fX = (CAL_W_PX * CAL_DIST_CM) / REAL_W_CM;
const float fY = (CAL_H_PX * CAL_DIST_CM) / REAL_H_CM;

// ============================================================
// DISTANCE THRESHOLDS
// ============================================================
const float STOP_DIST_CM = 12.0;
const float GO_DIST_CM   = 18.0;

// ============================================================
// DETECTION FILTERS
// ============================================================
const int MIN_BOX_W = 18;
const int MIN_BOX_H = 18;

// ============================================================
// SPEED SETTINGS
// ============================================================
const int MIN_FORWARD_SPEED = 85;
const int MAX_FORWARD_SPEED = 150;

const int MIN_TURN_BIAS = 20;
const int MAX_TURN_BIAS = 95;

const int HARD_PIVOT_SPEED   = 110;
const int GENTLE_PIVOT_SPEED = 70;

// ============================================================
// CONTROL TIMING
// ============================================================
const unsigned long CONTROL_INTERVAL_MS = 50;
unsigned long lastControlTime = 0;

// Lost target behavior
const unsigned long LOST_HOLD_MS    = 350;
const unsigned long SEARCH_AFTER_MS = 900;
unsigned long lastSeenTime = 0;

// ============================================================
// SMOOTHING
// ============================================================
float smoothX = screenCenterX;
float smoothDist = 40.0;
const float alphaX = 0.35;
const float alphaDist = 0.28;

// ============================================================
// RAMPING
// ============================================================
int currentLeftSpeed = 0;
int currentRightSpeed = 0;
int targetLeftSpeed = 0;
int targetRightSpeed = 0;

const int MAX_SPEED_STEP_PER_LOOP = 12;

// -1 = left, +1 = right, 0 = centered
int lastKnownDirection = 0;

// ============================================================
// DEBUG PRINT
// ============================================================
void debugPrint(const String &msg) {
  if (DEBUG) {
    Serial.println(msg);
  }
}

// ============================================================
// APPLY MOTOR OUTPUTS
// Input speed meaning stays:
//   leftSpeed  = LEFT motor command
//   rightSpeed = RIGHT motor command
//
// BUT your wiring is now:
//   RIGHT motor -> ENA/IN1/IN2 -> OUT1/OUT2
//   LEFT motor  -> ENB/IN3/IN4 -> OUT3/OUT4
// ============================================================
void applyMotorRaw(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  if (REVERSE_LEFT_MOTOR)  leftSpeed = -leftSpeed;
  if (REVERSE_RIGHT_MOTOR) rightSpeed = -rightSpeed;

  // ---------------- RIGHT MOTOR (OUT1 / OUT2) ----------------
  if (rightSpeed > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, rightSpeed);
  }
  else if (rightSpeed < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, -rightSpeed);
  }
  else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  }

  // ---------------- LEFT MOTOR (OUT3 / OUT4) ----------------
  if (leftSpeed > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, leftSpeed);
  }
  else if (leftSpeed < 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, -leftSpeed);
  }
  else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 0);
  }
}

// ============================================================
// TARGET SPEEDS
// ============================================================
void setTargetSpeeds(int leftSpeed, int rightSpeed) {
  targetLeftSpeed = constrain(leftSpeed, -255, 255);
  targetRightSpeed = constrain(rightSpeed, -255, 255);
}

// ============================================================
// RAMP CURRENT SPEEDS TOWARD TARGET
// ============================================================
void updateRampedMotors() {
  if (currentLeftSpeed < targetLeftSpeed) {
    currentLeftSpeed += MAX_SPEED_STEP_PER_LOOP;
    if (currentLeftSpeed > targetLeftSpeed) currentLeftSpeed = targetLeftSpeed;
  } else if (currentLeftSpeed > targetLeftSpeed) {
    currentLeftSpeed -= MAX_SPEED_STEP_PER_LOOP;
    if (currentLeftSpeed < targetLeftSpeed) currentLeftSpeed = targetLeftSpeed;
  }

  if (currentRightSpeed < targetRightSpeed) {
    currentRightSpeed += MAX_SPEED_STEP_PER_LOOP;
    if (currentRightSpeed > targetRightSpeed) currentRightSpeed = targetRightSpeed;
  } else if (currentRightSpeed > targetRightSpeed) {
    currentRightSpeed -= MAX_SPEED_STEP_PER_LOOP;
    if (currentRightSpeed < targetRightSpeed) currentRightSpeed = targetRightSpeed;
  }

  applyMotorRaw(currentLeftSpeed, currentRightSpeed);
}

// ============================================================
// STOP
// ============================================================
void stopMotors() {
  setTargetSpeeds(0, 0);
}

// ============================================================
// PICK BEST TARGET
// ============================================================
bool readBestTarget(HUSKYLENSResult &best) {
  if (!huskylens.available()) return false;

  bool found = false;
  int bestArea = -1;

  while (huskylens.available()) {
    HUSKYLENSResult r = huskylens.read();

    if (TARGET_ID != -1 && r.ID != TARGET_ID) continue;
    if (r.width < MIN_BOX_W || r.height < MIN_BOX_H) continue;

    int area = r.width * r.height;
    if (area > bestArea) {
      bestArea = area;
      best = r;
      found = true;
    }
  }

  return found;
}

// ============================================================
// ESTIMATE DISTANCE
// ============================================================
float estimateDistanceCm(int objectW, int objectH) {
  float distFromW = (fX * REAL_W_CM) / (float)max(1, objectW);
  float distFromH = (fY * REAL_H_CM) / (float)max(1, objectH);

  return min(distFromW, distFromH);
}

// ============================================================
// DISTANCE-BASED FORWARD SPEED
// ============================================================
int computeForwardSpeed(float distCm) {
  if (distCm <= GO_DIST_CM) return 0;

  float clamped = distCm;
  if (clamped > 80.0) clamped = 80.0;

  int speed = map((int)(clamped * 10), (int)(GO_DIST_CM * 10), 800,
                  MIN_FORWARD_SPEED, MAX_FORWARD_SPEED);

  return constrain(speed, MIN_FORWARD_SPEED, MAX_FORWARD_SPEED);
}

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  Wire.begin();

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  stopMotors();
  updateRampedMotors();

  while (!huskylens.begin(Wire)) {
    debugPrint("HuskyLens not connected! Check wiring.");
    delay(200);
  }

  huskylens.writeAlgorithm(ALGORITHM_OBJECT_TRACKING);

  debugPrint("HuskyLens connected.");
  debugPrint("Bagpath ready.");
  debugPrint("Right motor = OUT1/OUT2, Left motor = OUT3/OUT4.");
  debugPrint("Make sure object is learned in Object Tracking mode.");
  delay(1200);
}

// ============================================================
// LOOP
// ============================================================
void loop() {
  if (millis() - lastControlTime >= CONTROL_INTERVAL_MS) {
    lastControlTime = millis();

    bool gotFreshData = huskylens.request();

    if (!gotFreshData) {
      debugPrint("Request failed");
      stopMotors();
    }
    else if (!huskylens.isLearned()) {
      debugPrint("No object learned yet");
      stopMotors();
    }
    else {
      HUSKYLENSResult best;
      bool found = readBestTarget(best);

      if (found) {
        lastSeenTime = millis();

        int objectX = best.xCenter;
        int objectW = best.width;
        int objectH = best.height;

        float distanceCm = estimateDistanceCm(objectW, objectH);

        // Smoothing
        smoothX = alphaX * objectX + (1.0 - alphaX) * smoothX;
        smoothDist = alphaDist * distanceCm + (1.0 - alphaDist) * smoothDist;

        // Dynamic deadzone
        int dynamicDeadzone = max(12, objectW / 6);

        float errorX = smoothX - screenCenterX;
        float absErrorX = abs(errorX);

        if (errorX < -dynamicDeadzone) lastKnownDirection = -1;
        else if (errorX > dynamicDeadzone) lastKnownDirection = 1;
        else lastKnownDirection = 0;

        if (DEBUG) {
          Serial.print("ID=");
          Serial.print(best.ID);
          Serial.print(" X=");
          Serial.print(objectX);
          Serial.print(" smoothX=");
          Serial.print(smoothX, 1);
          Serial.print(" W=");
          Serial.print(objectW);
          Serial.print(" H=");
          Serial.print(objectH);
          Serial.print(" dist=");
          Serial.print(distanceCm, 1);
          Serial.print(" smoothDist=");
          Serial.print(smoothDist, 1);
          Serial.print(" deadzone=");
          Serial.print(dynamicDeadzone);
          Serial.print(" errorX=");
          Serial.println(errorX, 1);
        }

        // ---------------- CONTROL DECISION ----------------

        // Too close
        if (smoothDist <= STOP_DIST_CM) {
          debugPrint("ACTION: STOP (too close)");
          stopMotors();
        }

        // Very far off-center -> hard pivot
        else if (absErrorX > 95) {
          if (errorX < 0) {
            debugPrint("ACTION: HARD LEFT PIVOT");
            setTargetSpeeds(-HARD_PIVOT_SPEED, HARD_PIVOT_SPEED);
          } else {
            debugPrint("ACTION: HARD RIGHT PIVOT");
            setTargetSpeeds(HARD_PIVOT_SPEED, -HARD_PIVOT_SPEED);
          }
        }

        // Centered
        else if (absErrorX <= dynamicDeadzone) {
          if (smoothDist > GO_DIST_CM) {
            int fwd = computeForwardSpeed(smoothDist);
            debugPrint("ACTION: STRAIGHT FORWARD");
            setTargetSpeeds(fwd, fwd);
          } else {
            debugPrint("ACTION: STOP (buffer zone)");
            stopMotors();
          }
        }

        // Moderate steering error
        else {
          int steerBias = map((int)absErrorX, dynamicDeadzone, 95,
                              MIN_TURN_BIAS, MAX_TURN_BIAS);
          steerBias = constrain(steerBias, MIN_TURN_BIAS, MAX_TURN_BIAS);

          if (smoothDist > GO_DIST_CM) {
            int baseForward = computeForwardSpeed(smoothDist);

            if (errorX < 0) {
              // target left -> curve left
              int leftCmd  = baseForward - steerBias;
              int rightCmd = baseForward + steerBias;

              leftCmd  = constrain(leftCmd, 0, 255);
              rightCmd = constrain(rightCmd, 0, 255);

              debugPrint("ACTION: CURVE LEFT");
              setTargetSpeeds(leftCmd, rightCmd);
            } else {
              // target right -> curve right
              int leftCmd  = baseForward + steerBias;
              int rightCmd = baseForward - steerBias;

              leftCmd  = constrain(leftCmd, 0, 255);
              rightCmd = constrain(rightCmd, 0, 255);

              debugPrint("ACTION: CURVE RIGHT");
              setTargetSpeeds(leftCmd, rightCmd);
            }
          }
          else {
            // In the buffer zone, recenter gently
            if (errorX < 0) {
              debugPrint("ACTION: GENTLE LEFT PIVOT");
              setTargetSpeeds(-GENTLE_PIVOT_SPEED, GENTLE_PIVOT_SPEED);
            } else {
              debugPrint("ACTION: GENTLE RIGHT PIVOT");
              setTargetSpeeds(GENTLE_PIVOT_SPEED, -GENTLE_PIVOT_SPEED);
            }
          }
        }
      }
      else {
        // ---------------- LOST TARGET HANDLING ----------------
        unsigned long lostFor = millis() - lastSeenTime;

        if (DEBUG) {
          Serial.print("No valid target. Lost for ");
          Serial.print(lostFor);
          Serial.println(" ms");
        }

        if (lostFor < LOST_HOLD_MS) {
          debugPrint("ACTION: HOLD LAST COMMAND");
        }
        else if (lostFor < SEARCH_AFTER_MS) {
          debugPrint("ACTION: STOP (short target loss)");
          stopMotors();
        }
        else {
          if (lastKnownDirection <= 0) {
            debugPrint("ACTION: SEARCH LEFT");
            setTargetSpeeds(-55, 55);
          } else {
            debugPrint("ACTION: SEARCH RIGHT");
            setTargetSpeeds(55, -55);
          }
        }
      }
    }
  }

  // Always ramp smoothly
  updateRampedMotors();
}
