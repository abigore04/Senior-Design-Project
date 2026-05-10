/*
 * metal_detector_fixed.ino
 *
 * Fixes vs original:
 *   1. Baseline is frozen after calibration — no more drift.
 *      The original sketch slowly moved the baseline toward the current
 *      reading, so metal held near the coil for >2 s would be "learned"
 *      as the new normal and stop triggering.
 *   2. Sends "difference,hit" over serial every reading so ROS can see
 *      the raw signal level and tune the threshold without re-flashing.
 *   3. THRESHOLD is a single constant — easy to adjust here.
 *   4. Recalibrate by sending 'r' over serial (no need to power-cycle).
 */

#include "FreqCount.h"

// ── Tune this if detector is too sensitive or not sensitive enough ──
const long THRESHOLD = 3;   // frequency counts difference to trigger HIT
//   Start low (3) and increase if you get false positives.
//   If real metal never triggers, decrease to 2 or 1.

// RGB LED pins
const int redPin   = A5;
const int greenPin = A4;
const int bluePin  = A2;

unsigned long baseLine    = 0;
bool          calibrated  = false;

void silence() {
  digitalWrite(redPin,   LOW);
  digitalWrite(greenPin, LOW);
  digitalWrite(bluePin,  LOW);
}

void calibrate() {
  silence();
  digitalWrite(bluePin, HIGH);   // blue = calibrating

  FreqCount.end();
  FreqCount.begin(100);          // 100 ms gate

  // Wait for first stable reading
  unsigned long sum   = 0;
  int           count = 0;
  unsigned long t0    = millis();

  while (millis() - t0 < 1500) {
    if (FreqCount.available()) {
      sum += FreqCount.read();
      count++;
    }
    delay(10);
  }

  if (count > 0) {
    baseLine   = sum / count;
    calibrated = true;
    silence();
    // Green flash = calibration OK
    digitalWrite(greenPin, HIGH);
    delay(400);
    silence();
    Serial.print("BASELINE=");
    Serial.println(baseLine);
  } else {
    // Red flash = calibration failed (no frequency signal)
    digitalWrite(redPin, HIGH);
    delay(400);
    silence();
    Serial.println("BASELINE_FAIL");
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(redPin,   OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin,  OUTPUT);

  // Startup LED test
  digitalWrite(redPin,   HIGH); delay(150); silence();
  digitalWrite(greenPin, HIGH); delay(150); silence();
  digitalWrite(bluePin,  HIGH); delay(150); silence();

  calibrate();
}

void loop() {
  // Allow recalibration via serial 'r' command
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'r' || c == 'R') {
      Serial.println("RECALIBRATING...");
      calibrate();
      return;
    }
  }

  if (!calibrated)          { delay(50); return; }
  if (!FreqCount.available()) { return; }

  unsigned long count      = FreqCount.read();
  long          difference = (long)baseLine - (long)count;
  difference = abs(difference);

  // Send raw difference AND hit flag: "difference,hit"
  // e.g. "0,0"  "2,0"  "7,1"  "12,1"
  int hit = (difference >= THRESHOLD) ? 1 : 0;

  Serial.print(difference);
  Serial.print(",");
  Serial.println(hit);

  // LED feedback
  if (hit) {
    if (difference > 10) {
      // Strong hit — red
      digitalWrite(redPin,   HIGH);
      digitalWrite(greenPin, LOW);
      digitalWrite(bluePin,  LOW);
    } else {
      // Weak hit — green
      digitalWrite(redPin,   LOW);
      digitalWrite(greenPin, HIGH);
      digitalWrite(bluePin,  LOW);
    }
  } else {
    silence();
  }
}
