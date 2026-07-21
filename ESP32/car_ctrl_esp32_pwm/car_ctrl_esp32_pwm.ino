#include "RL_ESP32_Motor.h"
#include "ESP32_Servo.h"

enum {
  M0 = 0,
  M1,
  M2,
  M3,
  S1,
  S2,
  CHECK
};

const uint8_t WHEEL_COUNT = 4;
const int16_t CHECK_VAL = -12345;
const uint8_t PACKET_SHORTS = 7;
const uint8_t PACKET_BYTES = PACKET_SHORTS * sizeof(int16_t);

// Atlas sends [M1, M2, M3, M4, pan, tilt, -12345] as a 14-byte little-endian frame.

Motor motors[WHEEL_COUNT] = {
  Motor(1, 12, 13),
  Motor(2, 14, 15),
  Motor(3, 16, 17),
  Motor(4, 18, 19),
};

Servo servo_25;
Servo servo_26;

const int16_t PAN_CENTER = 93;
const int16_t TILT_CENTER = 162;
const int16_t PAN_MIN_ANGLE = 0;
const int16_t PAN_MAX_ANGLE = 180;
const int16_t TILT_MIN_ANGLE = 90;
const int16_t TILT_MAX_ANGLE = 162;

// Order: M1, M2, M3, M4. Change one value to -1 if that wheel is reversed.
const int8_t MOTOR_SIGN[WHEEL_COUNT] = {1, 1, 1, 1};

const int16_t START_KICK_PWM = 50;
const unsigned long START_KICK_MS = 120;

// Apply the kick only when leaving rest or reversing; steady motion stays at target PWM.

int16_t status[6] = {0, 0, 0, 0, -1, -1};
int16_t targetSpeeds[WHEEL_COUNT] = {0, 0, 0, 0};
int16_t appliedSpeeds[WHEEL_COUNT] = {0, 0, 0, 0};
unsigned long kickUntilMs[WHEEL_COUNT] = {0, 0, 0, 0};

int16_t clampMotor(int16_t value) {
  if (value > 100) return 100;
  if (value < -100) return -100;
  return value;
}

int16_t clampServoAngle(int16_t value, int16_t minimum, int16_t maximum) {
  if (value < minimum) return minimum;
  if (value > maximum) return maximum;
  return value;
}

int16_t signedKickSpeed(int16_t target) {
  int16_t magnitude = abs(target);
  if (magnitude < START_KICK_PWM) {
    magnitude = START_KICK_PWM;
  }
  if (magnitude > 100) {
    magnitude = 100;
  }
  return target > 0 ? magnitude : -magnitude;
}

bool isDirectionChange(int16_t oldSpeed, int16_t newSpeed) {
  return oldSpeed != 0 && newSpeed != 0 && ((oldSpeed > 0) != (newSpeed > 0));
}

void applyMotorOutput(uint8_t index, int16_t speed) {
  speed = clampMotor(speed);
  if (speed == appliedSpeeds[index]) {
    return;
  }

  motors[index].Motor_Speed(speed * MOTOR_SIGN[index]);
  appliedSpeeds[index] = speed;
}

void updateMotorOutputs() {
  unsigned long now = millis();
  for (uint8_t i = 0; i < WHEEL_COUNT; i++) {
    int16_t output = targetSpeeds[i];

    if (targetSpeeds[i] == 0) {
      kickUntilMs[i] = 0;
    } else if (kickUntilMs[i] != 0) {
      if ((long)(kickUntilMs[i] - now) > 0) {
        output = signedKickSpeed(targetSpeeds[i]);
      } else {
        kickUntilMs[i] = 0;
      }
    }

    applyMotorOutput(i, output);
  }
}

void setMotorTargets(const int16_t speeds[WHEEL_COUNT]) {
  unsigned long now = millis();
  for (uint8_t i = 0; i < WHEEL_COUNT; i++) {
    int16_t speed = clampMotor(speeds[i]);
    int16_t oldSpeed = targetSpeeds[i];

    if (speed != oldSpeed) {
      targetSpeeds[i] = speed;
      status[i] = speed;

      if (speed == 0) {
        kickUntilMs[i] = 0;
      } else if (oldSpeed == 0 || isDirectionChange(oldSpeed, speed)) {
        kickUntilMs[i] = now + START_KICK_MS;
      }

      updateMotorOutputs();
    }
  }
}

void stopMotors() {
  int16_t speeds[WHEEL_COUNT] = {0, 0, 0, 0};
  setMotorTargets(speeds);
}

void setServos(const int16_t angles[2]) {
  if (angles[0] >= 0) {
    int16_t panAngle = clampServoAngle(angles[0], PAN_MIN_ANGLE, PAN_MAX_ANGLE);
    if (panAngle != status[S1]) {
      servo_25.write(panAngle);
      status[S1] = panAngle;
      delay(20);
    }
  }

  if (angles[1] >= 0) {
    int16_t tiltAngle = clampServoAngle(angles[1], TILT_MIN_ANGLE, TILT_MAX_ANGLE);
    if (tiltAngle != status[S2]) {
      servo_26.write(tiltAngle);
      status[S2] = tiltAngle;
    }
  }
}

void handleBinaryPacket() {
  if (Serial.available() < PACKET_BYTES) {
    return;
  }

  int16_t packet[PACKET_SHORTS] = {0};
  size_t bytesRead = Serial.readBytes((char *)packet, PACKET_BYTES);
  if (bytesRead != PACKET_BYTES || packet[CHECK] != CHECK_VAL) {
    stopMotors();
    Serial.println("FAIL");
    return;
  }

  int16_t speeds[WHEEL_COUNT] = {packet[M0], packet[M1], packet[M2], packet[M3]};
  int16_t angles[2] = {packet[S1], packet[S2]};
  setMotorTargets(speeds);
  setServos(angles);
  Serial.println("SUCC");
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(20);

  for (uint8_t i = 0; i < WHEEL_COUNT; i++) {
    motors[i].mcpwm_begin();
  }

  servo_25.attach(25, 500, 2500);
  servo_26.attach(26, 500, 2500);

  int16_t initAngles[2] = {PAN_CENTER, TILT_CENTER};
  stopMotors();
  setServos(initAngles);
}

void loop() {
  updateMotorOutputs();
  handleBinaryPacket();
}
