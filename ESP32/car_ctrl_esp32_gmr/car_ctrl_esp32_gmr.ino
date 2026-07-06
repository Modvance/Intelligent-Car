#include "RL_ESP32_Motor.h"
#include "ESP32_Servo.h"
#include <math.h>

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

// Huawei ESP32 IOT & Robot board motor driver pin mapping.
Motor motors[WHEEL_COUNT] = {
  Motor(1, 12, 13),
  Motor(2, 14, 15),
  Motor(3, 16, 17),
  Motor(4, 18, 19),
};

Servo servo_25;
Servo servo_26;

const uint8_t TRIG_PIN = 4;
const uint8_t ECHO_PIN = 5;
const float SOUND_SPEED_CM_PER_US = 0.034f;
const bool ENABLE_ULTRASONIC_OBSTACLE_AVOIDANCE = false;

const float ENCODER_PULSES_PER_MOTOR_REV = 500.0f;
const float GEAR_RATIO = 48.0f;
const float ENCODER_PULSES_PER_WHEEL_REV = ENCODER_PULSES_PER_MOTOR_REV * GEAR_RATIO;
const float MAX_WHEEL_RPM = 330.0f;

const unsigned long CONTROL_PERIOD_MS = 50;
const float MIN_START_PWM = 18.0f;
const float PID_KP = 0.22f;
const float PID_KI = 0.55f;
const float INTEGRAL_LIMIT = 90.0f;

const uint8_t ENCODER_PINS[WHEEL_COUNT][2] = {
  {21, 22},
  {23, 27},
  {34, 35},
  {36, 39},
};

// Order: M1, M2, M3, M4. Use -1 for a motor that is physically reversed.
// These values keep the last tested direction compensation from motor_encoder_test.
const int8_t MOTOR_SIGN[WHEEL_COUNT] = {1, 1, 1, 1};

// Use -1 if a wheel's measured rpm sign is opposite to the command sign.
const int8_t ENCODER_SIGN[WHEEL_COUNT] = {1, 1, 1, 1};

struct WheelController {
  Motor *driver;
  uint8_t pinA;
  uint8_t pinB;
  int8_t encoderSign;
  volatile long ticks;
  long lastTicks;
  float targetRpm;
  float measuredRpm;
  float integral;
  int pwm;
};

WheelController wheels[WHEEL_COUNT] = {
  {&motors[0], ENCODER_PINS[0][0], ENCODER_PINS[0][1], ENCODER_SIGN[0], 0, 0, 0, 0, 0, 0},
  {&motors[1], ENCODER_PINS[1][0], ENCODER_PINS[1][1], ENCODER_SIGN[1], 0, 0, 0, 0, 0, 0},
  {&motors[2], ENCODER_PINS[2][0], ENCODER_PINS[2][1], ENCODER_SIGN[2], 0, 0, 0, 0, 0, 0},
  {&motors[3], ENCODER_PINS[3][0], ENCODER_PINS[3][1], ENCODER_SIGN[3], 0, 0, 0, 0, 0, 0},
};

int16_t status[6] = {0, 0, 0, 0, -1, -1};
TaskHandle_t ultrasonicTask;
volatile bool obstacleStop = false;
unsigned long lastControlMs = 0;

float clampFloat(float value, float minValue, float maxValue) {
  if (value < minValue) return minValue;
  if (value > maxValue) return maxValue;
  return value;
}

int roundToInt(float value) {
  return value >= 0.0f ? (int)(value + 0.5f) : (int)(value - 0.5f);
}

void IRAM_ATTR handleEncoder(uint8_t index) {
  bool phaseB = digitalRead(wheels[index].pinB);
  wheels[index].ticks += phaseB ? wheels[index].encoderSign : -wheels[index].encoderSign;
}

void IRAM_ATTR handleEncoder0() { handleEncoder(0); }
void IRAM_ATTR handleEncoder1() { handleEncoder(1); }
void IRAM_ATTR handleEncoder2() { handleEncoder(2); }
void IRAM_ATTR handleEncoder3() { handleEncoder(3); }

void initEncoders() {
  for (uint8_t i = 0; i < WHEEL_COUNT; i++) {
    pinMode(wheels[i].pinA, INPUT);
    pinMode(wheels[i].pinB, INPUT);
  }

  attachInterrupt(digitalPinToInterrupt(wheels[0].pinA), handleEncoder0, RISING);
  attachInterrupt(digitalPinToInterrupt(wheels[1].pinA), handleEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(wheels[2].pinA), handleEncoder2, RISING);
  attachInterrupt(digitalPinToInterrupt(wheels[3].pinA), handleEncoder3, RISING);
}

void resetWheelPid(uint8_t index) {
  wheels[index].integral = 0.0f;
  wheels[index].pwm = 0;
}

void setWheelTarget(uint8_t index, float percent) {
  percent = clampFloat(percent, -100.0f, 100.0f);
  wheels[index].targetRpm = percent * MAX_WHEEL_RPM / 100.0f;

  if (fabsf(percent) < 0.1f) {
    resetWheelPid(index);
    return;
  }
}

void setMotorTargets(const int16_t speeds[WHEEL_COUNT]) {
  for (uint8_t i = 0; i < WHEEL_COUNT; i++) {
    if (speeds[i] != status[i]) {
      setWheelTarget(i, speeds[i]);
      status[i] = speeds[i];
    }
  }
}

void updateMotorControl(bool force = false) {
  unsigned long now = millis();
  unsigned long elapsed = now - lastControlMs;

  if (!force && elapsed < CONTROL_PERIOD_MS) {
    return;
  }

  if (elapsed == 0) {
    elapsed = CONTROL_PERIOD_MS;
  }
  lastControlMs = now;

  long currentTicks[WHEEL_COUNT];
  noInterrupts();
  for (uint8_t i = 0; i < WHEEL_COUNT; i++) {
    currentTicks[i] = wheels[i].ticks;
  }
  interrupts();

  for (uint8_t i = 0; i < WHEEL_COUNT; i++) {
    long deltaTicks = currentTicks[i] - wheels[i].lastTicks;
    wheels[i].lastTicks = currentTicks[i];

    float target = wheels[i].targetRpm;
    if (fabsf(target) < 0.5f) {
      wheels[i].driver->Motor_Speed(0);
      resetWheelPid(i);
      continue;
    }

    float instantRpm = ((float)deltaTicks * 60000.0f) / (ENCODER_PULSES_PER_WHEEL_REV * elapsed);
    wheels[i].measuredRpm = 0.65f * wheels[i].measuredRpm + 0.35f * instantRpm;

    float error = target - wheels[i].measuredRpm;
    wheels[i].integral += error * (elapsed / 1000.0f);
    wheels[i].integral = clampFloat(wheels[i].integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

    float command = target * 100.0f / MAX_WHEEL_RPM + PID_KP * error + PID_KI * wheels[i].integral;
    command = clampFloat(command, -100.0f, 100.0f);

    if (fabsf(command) > 0.0f && fabsf(command) < MIN_START_PWM) {
      command = command > 0.0f ? MIN_START_PWM : -MIN_START_PWM;
    }

    wheels[i].pwm = roundToInt(command);
    wheels[i].driver->Motor_Speed(wheels[i].pwm * MOTOR_SIGN[i]);
  }
}

void stopMotors() {
  int16_t speeds[WHEEL_COUNT] = {0, 0, 0, 0};
  setMotorTargets(speeds);
  updateMotorControl(true);
}

void setServos(const int16_t angles[2]) {
  if (angles[0] >= 0 && angles[0] <= 180 && angles[0] != status[S1]) {
    servo_25.write(angles[0]);
    status[S1] = angles[0];
    delay(20);
  }

  if (angles[1] >= 0 && angles[1] <= 180 && angles[1] != status[S2]) {
    servo_26.write(angles[1]);
    status[S2] = angles[1];
  }
}

void ultrasonicTaskCode(void *pvParameters) {
  while (true) {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    unsigned long duration = pulseIn(ECHO_PIN, HIGH, 30000UL);
    float distance = duration * SOUND_SPEED_CM_PER_US / 2.0f;

    if (distance > 1.0f) {
      if (distance < 10.0f) {
        obstacleStop = true;
        stopMotors();
      } else {
        obstacleStop = false;
      }
    }

    delay(100);
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

  if (ENABLE_ULTRASONIC_OBSTACLE_AVOIDANCE && obstacleStop) {
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
  initEncoders();

  servo_25.attach(25, 500, 2500);
  servo_26.attach(26, 500, 2500);

  if (ENABLE_ULTRASONIC_OBSTACLE_AVOIDANCE) {
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
  }

  int16_t initAngles[2] = {90, 65};
  stopMotors();
  setServos(initAngles);
  lastControlMs = millis();

  if (ENABLE_ULTRASONIC_OBSTACLE_AVOIDANCE) {
    xTaskCreatePinnedToCore(
      ultrasonicTaskCode,
      "Ultrasonic",
      10000,
      NULL,
      tskIDLE_PRIORITY,
      &ultrasonicTask,
      0);
  }
}

void loop() {
  updateMotorControl();
  handleBinaryPacket();
}
