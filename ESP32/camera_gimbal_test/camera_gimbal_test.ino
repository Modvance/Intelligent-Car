#include <Arduino.h>
#include <stdio.h>

#include "ESP32_Servo.h"

const uint8_t PAN_PIN = 25;
const uint8_t TILT_PIN = 26;
const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2500;

// Attach both servo horns only after this centered position is commanded.
const int PAN_CENTER = 93;
const int TILT_CENTER = 162;
const int PAN_MIN_ANGLE = 0;
const int PAN_MAX_ANGLE = 180;
const int TILT_MIN_ANGLE = 90;
const int TILT_MAX_ANGLE = 162;

const unsigned long SWEEP_SETTLE_MS = 700;
const bool AUTORUN_SWEEP = false;

Servo panServo;
Servo tiltServo;
bool servosAttached = false;
int panAngle = PAN_CENTER;
int tiltAngle = TILT_CENTER;

enum SweepStep {
  SWEEP_IDLE,
  SWEEP_PAN_MIN,
  SWEEP_PAN_MAX,
  SWEEP_CENTER_AFTER_PAN,
  SWEEP_TILT_MIN,
  SWEEP_TILT_MAX,
  SWEEP_FINAL_CENTER,
  SWEEP_DONE,
};

SweepStep sweepStep = SWEEP_IDLE;
unsigned long nextSweepMoveMs = 0;

int clampAngle(int value, int minimum, int maximum) {
  if (value < minimum) return minimum;
  if (value > maximum) return maximum;
  return value;
}

void ensureServosAttached() {
  if (servosAttached) {
    return;
  }
  panServo.attach(PAN_PIN, SERVO_MIN_US, SERVO_MAX_US);
  tiltServo.attach(TILT_PIN, SERVO_MIN_US, SERVO_MAX_US);
  servosAttached = true;
}

void setGimbalAngles(int pan, int tilt) {
  ensureServosAttached();
  panAngle = clampAngle(pan, PAN_MIN_ANGLE, PAN_MAX_ANGLE);
  tiltAngle = clampAngle(tilt, TILT_MIN_ANGLE, TILT_MAX_ANGLE);
  panServo.write(panAngle);
  tiltServo.write(tiltAngle);
}

void centerGimbal() {
  setGimbalAngles(PAN_CENTER, TILT_CENTER);
}

void cancelSweep() {
  if (sweepStep != SWEEP_IDLE) {
    sweepStep = SWEEP_IDLE;
    Serial.println("OK,sweep cancelled");
  }
}

void startSweep() {
  ensureServosAttached();
  sweepStep = SWEEP_PAN_MIN;
  nextSweepMoveMs = millis();
  Serial.println("OK,sweep started");
}

void updateSweep() {
  if (sweepStep == SWEEP_IDLE || millis() < nextSweepMoveMs) {
    return;
  }

  switch (sweepStep) {
    case SWEEP_PAN_MIN:
      setGimbalAngles(PAN_MIN_ANGLE, TILT_CENTER);
      Serial.println("INFO,sweep pan 0");
      sweepStep = SWEEP_PAN_MAX;
      break;
    case SWEEP_PAN_MAX:
      setGimbalAngles(PAN_MAX_ANGLE, TILT_CENTER);
      Serial.println("INFO,sweep pan 180");
      sweepStep = SWEEP_CENTER_AFTER_PAN;
      break;
    case SWEEP_CENTER_AFTER_PAN:
      centerGimbal();
      Serial.println("INFO,sweep pan center");
      sweepStep = SWEEP_TILT_MIN;
      break;
    case SWEEP_TILT_MIN:
      setGimbalAngles(PAN_CENTER, TILT_MIN_ANGLE);
      Serial.println("INFO,sweep tilt 90");
      sweepStep = SWEEP_TILT_MAX;
      break;
    case SWEEP_TILT_MAX:
      setGimbalAngles(PAN_CENTER, TILT_MAX_ANGLE);
      Serial.println("INFO,sweep tilt 162");
      sweepStep = SWEEP_FINAL_CENTER;
      break;
    case SWEEP_FINAL_CENTER:
      centerGimbal();
      Serial.println("INFO,sweep center");
      sweepStep = SWEEP_DONE;
      break;
    case SWEEP_DONE:
      sweepStep = SWEEP_IDLE;
      Serial.println("OK,sweep complete");
      return;
    case SWEEP_IDLE:
      return;
  }
  nextSweepMoveMs = millis() + SWEEP_SETTLE_MS;
}

void printStatus() {
  Serial.print("INFO,attached,");
  Serial.print(servosAttached ? "true" : "false");
  Serial.print(",pan,");
  Serial.print(panAngle);
  Serial.print(",tilt,");
  Serial.print(tiltAngle);
  Serial.print(",pan_range,0,180,tilt_range,90,162");
  Serial.print(",sweeping,");
  Serial.println(sweepStep == SWEEP_IDLE ? "false" : "true");
}

void printHelp() {
  Serial.println("INFO,camera gimbal commands:");
  Serial.println("INFO,pan ANGLE | tilt ANGLE | set PAN TILT | center");
  Serial.println("INFO,sweep | stop | release | attach | status | help");
  Serial.println("INFO,pan range 0..180; tilt range 90..162");
}

void handleCommand(String line) {
  line.trim();
  line.toLowerCase();
  if (line.length() == 0) {
    return;
  }
  if (line == "help") {
    printHelp();
    return;
  }
  if (line == "status") {
    printStatus();
    return;
  }
  if (line == "sweep") {
    startSweep();
    return;
  }
  if (line == "stop") {
    cancelSweep();
    Serial.println("OK,holding current position");
    return;
  }
  if (line == "center") {
    cancelSweep();
    centerGimbal();
    Serial.println("OK,center");
    return;
  }
  if (line == "release") {
    cancelSweep();
    if (servosAttached) {
      panServo.detach();
      tiltServo.detach();
      servosAttached = false;
    }
    Serial.println("OK,released");
    return;
  }
  if (line == "attach") {
    ensureServosAttached();
    setGimbalAngles(panAngle, tiltAngle);
    Serial.println("OK,attached");
    return;
  }

  int value = 0;
  if (sscanf(line.c_str(), "pan %d", &value) == 1) {
    cancelSweep();
    setGimbalAngles(value, tiltAngle);
    Serial.println("OK,pan");
    return;
  }
  if (sscanf(line.c_str(), "tilt %d", &value) == 1) {
    cancelSweep();
    setGimbalAngles(panAngle, value);
    Serial.println("OK,tilt");
    return;
  }

  int newPan = 0;
  int newTilt = 0;
  if (sscanf(line.c_str(), "set %d %d", &newPan, &newTilt) == 2) {
    cancelSweep();
    setGimbalAngles(newPan, newTilt);
    Serial.println("OK,set");
    return;
  }

  Serial.println("ERR,unknown command; send help");
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(20);
  ensureServosAttached();
  centerGimbal();
  Serial.println("INFO,camera_gimbal_test ready");
  printHelp();
  if (AUTORUN_SWEEP) {
    delay(1000);
    startSweep();
  }
}

void loop() {
  if (Serial.available() > 0) {
    handleCommand(Serial.readStringUntil('\n'));
  }
  updateSweep();
}
