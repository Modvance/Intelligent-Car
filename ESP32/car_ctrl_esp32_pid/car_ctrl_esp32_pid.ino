#include "RL_ESP32_Motor.h"
#include "ESP32_Servo.h"
#include <math.h>

// Closed-loop firmware for the GMR encoder TT motors. The host command packet
// remains Huawei-compatible; ESP32-to-host feedback is sent as binary telemetry.

enum { M0 = 0, M1, M2, M3, S1, S2, CHECK };

const uint8_t WHEEL_COUNT = 4;
const int16_t CHECK_VAL = -12345;
const uint8_t PACKET_SHORTS = 7;
const uint8_t PACKET_BYTES = PACKET_SHORTS * sizeof(int16_t);

Motor motors[WHEEL_COUNT] = {
  Motor(1, 12, 13),
  Motor(2, 14, 15),
  Motor(3, 16, 17),
  Motor(4, 18, 19),
};

Servo servo_25;
Servo servo_26;

// Generic defaults for the supplied 500 PPR, nominal 48:1 GMR motor. The PID
// loop uses real tick deltas; only the displayed/target RPM scale depends on PPR.
const float ENCODER_PULSES_PER_WHEEL_REV = 24000.0f;
const float MAX_WHEEL_RPM = 70.0f;
const unsigned long CONTROL_PERIOD_MS = 50;
const unsigned long TELEMETRY_PERIOD_MS = 100;
const float PID_KP = 0.70f;
const float PID_KI = 0.18f;
const float PID_KD = 0.02f;
const float INTEGRAL_LIMIT = 80.0f;
const float PWM_LIMIT = 100.0f;
const float MIN_RUNNING_PWM = 14.0f;
const float START_PULSE_PWM = 28.0f;
const unsigned long START_PULSE_MS = 140;
const float SPEED_FILTER_ALPHA = 0.45f;

const uint8_t ENCODER_PINS[WHEEL_COUNT][2] = {
  {21, 22},
  {23, 27},
  {34, 35},
  {36, 39},
};

// Keep the final direction convention already used by main.py and ROS2.
const int8_t MOTOR_SIGN[WHEEL_COUNT] = {1, 1, 1, 1};
const int8_t ENCODER_SIGN[WHEEL_COUNT] = {1, 1, 1, 1};

const uint8_t TELEMETRY_MAGIC_0 = 0xA5;
const uint8_t TELEMETRY_MAGIC_1 = 0x5A;
const uint8_t TELEMETRY_VERSION = 1;
const uint8_t TELEMETRY_PAYLOAD_BYTES = 76;
const uint8_t TELEMETRY_FRAME_BYTES = 2 + 1 + 1 + TELEMETRY_PAYLOAD_BYTES + 2;
const uint8_t FLAG_SATURATED = 0x01;
const uint8_t FLAG_START_PULSE = 0x02;

struct WheelController {
  Motor *driver;
  uint8_t pinA;
  uint8_t pinB;
  int8_t encoderSign;
  volatile long ticks;
  long lastTicks;
  int16_t requestedPercent;
  int8_t targetDirection;
  float targetRpm;
  float measuredRpm;
  float integral;
  float previousError;
  float error;
  float pTerm;
  float iTerm;
  float dTerm;
  int16_t tickDelta;
  int pwm;
  bool saturated;
  bool startPulse;
  unsigned long startPulseUntilMs;
};

WheelController wheels[WHEEL_COUNT] = {
  {&motors[0], 21, 22, ENCODER_SIGN[0], 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, false, false, 0},
  {&motors[1], 23, 27, ENCODER_SIGN[1], 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, false, false, 0},
  {&motors[2], 34, 35, ENCODER_SIGN[2], 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, false, false, 0},
  {&motors[3], 36, 39, ENCODER_SIGN[3], 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, false, false, 0},
};

int16_t status[6] = {0, 0, 0, 0, -1, -1};
unsigned long lastControlMs = 0;
unsigned long lastTelemetryMs = 0;
uint32_t telemetrySequence = 0;

float clampFloat(float value, float minValue, float maxValue) {
  if (value < minValue) return minValue;
  if (value > maxValue) return maxValue;
  return value;
}

int signOf(float value) {
  return value > 0.0f ? 1 : (value < 0.0f ? -1 : 0);
}

int16_t roundToInt16(float value) {
  value = clampFloat(value, -32768.0f, 32767.0f);
  return value >= 0.0f ? (int16_t)(value + 0.5f) : (int16_t)(value - 0.5f);
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
  for (uint8_t index = 0; index < WHEEL_COUNT; ++index) {
    pinMode(wheels[index].pinA, INPUT);
    pinMode(wheels[index].pinB, INPUT);
  }
  attachInterrupt(digitalPinToInterrupt(wheels[0].pinA), handleEncoder0, RISING);
  attachInterrupt(digitalPinToInterrupt(wheels[1].pinA), handleEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(wheels[2].pinA), handleEncoder2, RISING);
  attachInterrupt(digitalPinToInterrupt(wheels[3].pinA), handleEncoder3, RISING);
}

void resetWheelController(uint8_t index, bool preserveTicks = true) {
  WheelController &wheel = wheels[index];
  wheel.measuredRpm = 0.0f;
  wheel.integral = 0.0f;
  wheel.previousError = 0.0f;
  wheel.error = 0.0f;
  wheel.pTerm = 0.0f;
  wheel.iTerm = 0.0f;
  wheel.dTerm = 0.0f;
  wheel.tickDelta = 0;
  wheel.pwm = 0;
  wheel.saturated = false;
  wheel.startPulse = false;
  wheel.startPulseUntilMs = 0;
  if (preserveTicks) {
    noInterrupts();
    wheel.lastTicks = wheel.ticks;
    interrupts();
  }
}

void setWheelTarget(uint8_t index, int16_t command) {
  WheelController &wheel = wheels[index];
  command = (int16_t)clampFloat(command, -100.0f, 100.0f);
  int newDirection = signOf(command);
  int oldDirection = wheel.targetDirection;

  if (command == wheel.requestedPercent) return;

  wheel.requestedPercent = command;
  wheel.targetDirection = newDirection;
  wheel.targetRpm = command * MAX_WHEEL_RPM / 100.0f;

  if (newDirection == 0 || (oldDirection != 0 && oldDirection != newDirection)) {
    resetWheelController(index);
  }
  if (newDirection != 0 && oldDirection == 0) {
    resetWheelController(index);
    wheel.startPulseUntilMs = millis() + START_PULSE_MS;
  }
}

void setMotorTargets(const int16_t speeds[WHEEL_COUNT]) {
  for (uint8_t index = 0; index < WHEEL_COUNT; ++index) {
    setWheelTarget(index, speeds[index]);
    status[index] = speeds[index];
  }
}

float conditionalIntegrate(WheelController &wheel, float error, float dtSeconds,
                           float feedForward, float pTerm, float dTerm) {
  float candidateIntegral = clampFloat(
      wheel.integral + error * dtSeconds, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  float candidateITerm = PID_KI * candidateIntegral;
  float candidateOutput = feedForward + pTerm + candidateITerm + dTerm;
  bool pushesHigh = candidateOutput >= PWM_LIMIT && error > 0.0f;
  bool pushesLow = candidateOutput <= -PWM_LIMIT && error < 0.0f;
  if (!pushesHigh && !pushesLow) {
    wheel.integral = candidateIntegral;
  }
  return PID_KI * wheel.integral;
}

void applyWheelControl(uint8_t index, long currentTicks, unsigned long elapsedMs, unsigned long now) {
  WheelController &wheel = wheels[index];
  long deltaTicks = currentTicks - wheel.lastTicks;
  wheel.lastTicks = currentTicks;
  wheel.tickDelta = (int16_t)clampFloat(deltaTicks, -32768.0f, 32767.0f);

  if (wheel.targetDirection == 0) {
    wheel.driver->Motor_Speed(0);
    resetWheelController(index, false);
    return;
  }

  float dtSeconds = elapsedMs / 1000.0f;
  float instantRpm = (deltaTicks * 60000.0f) / (ENCODER_PULSES_PER_WHEEL_REV * elapsedMs);
  wheel.measuredRpm += SPEED_FILTER_ALPHA * (instantRpm - wheel.measuredRpm);
  wheel.error = wheel.targetRpm - wheel.measuredRpm;
  wheel.pTerm = PID_KP * wheel.error;
  wheel.dTerm = PID_KD * (wheel.error - wheel.previousError) / dtSeconds;
  float feedForward = wheel.targetRpm * 100.0f / MAX_WHEEL_RPM;
  wheel.iTerm = conditionalIntegrate(wheel, wheel.error, dtSeconds, feedForward, wheel.pTerm, wheel.dTerm);
  float output = feedForward + wheel.pTerm + wheel.iTerm + wheel.dTerm;

  wheel.saturated = output >= PWM_LIMIT || output <= -PWM_LIMIT;
  output = clampFloat(output, -PWM_LIMIT, PWM_LIMIT);
  wheel.startPulse = (long)(wheel.startPulseUntilMs - now) > 0;
  if (wheel.startPulse) {
    output = wheel.targetDirection * max(fabsf(output), START_PULSE_PWM);
  } else if (fabsf(output) < MIN_RUNNING_PWM) {
    output = wheel.targetDirection * MIN_RUNNING_PWM;
  }

  wheel.pwm = (int)roundToInt16(output);
  wheel.previousError = wheel.error;
  wheel.driver->Motor_Speed(wheel.pwm * MOTOR_SIGN[index]);
}

void updateMotorControl(bool force = false) {
  unsigned long now = millis();
  unsigned long elapsedMs = now - lastControlMs;
  if (!force && elapsedMs < CONTROL_PERIOD_MS) return;
  if (elapsedMs == 0) elapsedMs = CONTROL_PERIOD_MS;
  lastControlMs = now;

  long currentTicks[WHEEL_COUNT];
  noInterrupts();
  for (uint8_t index = 0; index < WHEEL_COUNT; ++index) currentTicks[index] = wheels[index].ticks;
  interrupts();
  for (uint8_t index = 0; index < WHEEL_COUNT; ++index) {
    applyWheelControl(index, currentTicks[index], elapsedMs, now);
  }
}

void stopMotors() {
  int16_t speeds[WHEEL_COUNT] = {0, 0, 0, 0};
  setMotorTargets(speeds);
  updateMotorControl(true);
}

int16_t clampServoAngle(int16_t value, int16_t minValue, int16_t maxValue) {
  return value < minValue ? minValue : (value > maxValue ? maxValue : value);
}

void setServos(const int16_t angles[2]) {
  if (angles[0] >= 0 && angles[0] <= 180 && angles[0] != status[S1]) {
    servo_25.write(clampServoAngle(angles[0], 0, 180));
    status[S1] = angles[0];
  }
  if (angles[1] >= 0 && angles[1] <= 180 && angles[1] != status[S2]) {
    servo_26.write(clampServoAngle(angles[1], 0, 180));
    status[S2] = angles[1];
  }
}

uint16_t crc16Ccitt(const uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t index = 0; index < length; ++index) {
    crc ^= (uint16_t)data[index] << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
    }
  }
  return crc;
}

void putInt16(uint8_t *buffer, uint8_t &offset, int16_t value) {
  buffer[offset++] = value & 0xFF;
  buffer[offset++] = (value >> 8) & 0xFF;
}

void putUInt16(uint8_t *buffer, uint8_t &offset, uint16_t value) {
  putInt16(buffer, offset, (int16_t)value);
}

void putUInt32(uint8_t *buffer, uint8_t &offset, uint32_t value) {
  buffer[offset++] = value & 0xFF;
  buffer[offset++] = (value >> 8) & 0xFF;
  buffer[offset++] = (value >> 16) & 0xFF;
  buffer[offset++] = (value >> 24) & 0xFF;
}

void sendTelemetry() {
  unsigned long now = millis();
  if (now - lastTelemetryMs < TELEMETRY_PERIOD_MS) return;
  lastTelemetryMs = now;

  uint8_t frame[TELEMETRY_FRAME_BYTES] = {0};
  frame[0] = TELEMETRY_MAGIC_0;
  frame[1] = TELEMETRY_MAGIC_1;
  frame[2] = TELEMETRY_VERSION;
  frame[3] = TELEMETRY_PAYLOAD_BYTES;
  uint8_t offset = 4;
  putUInt32(frame, offset, telemetrySequence++);
  putUInt16(frame, offset, CONTROL_PERIOD_MS);
  putUInt16(frame, offset, 0);
  for (uint8_t index = 0; index < WHEEL_COUNT; ++index) {
    WheelController &wheel = wheels[index];
    uint8_t flags = (wheel.saturated ? FLAG_SATURATED : 0) |
                    (wheel.startPulse ? FLAG_START_PULSE : 0);
    putInt16(frame, offset, roundToInt16(wheel.targetRpm * 100.0f));
    putInt16(frame, offset, roundToInt16(wheel.measuredRpm * 100.0f));
    putInt16(frame, offset, wheel.tickDelta);
    putInt16(frame, offset, wheel.pwm);
    putInt16(frame, offset, roundToInt16(wheel.error * 100.0f));
    putInt16(frame, offset, roundToInt16(wheel.pTerm * 100.0f));
    putInt16(frame, offset, roundToInt16(wheel.iTerm * 100.0f));
    putInt16(frame, offset, roundToInt16(wheel.dTerm * 100.0f));
    frame[offset++] = flags;
  }
  uint16_t crc = crc16Ccitt(frame + 2, 2 + TELEMETRY_PAYLOAD_BYTES);
  frame[offset++] = crc & 0xFF;
  frame[offset++] = (crc >> 8) & 0xFF;
  Serial.write(frame, offset);
}

void handleBinaryPacket() {
  if (Serial.available() < PACKET_BYTES) return;
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
  for (uint8_t index = 0; index < WHEEL_COUNT; ++index) motors[index].mcpwm_begin();
  initEncoders();
  servo_25.attach(25, 500, 2500);
  servo_26.attach(26, 500, 2500);
  int16_t initAngles[2] = {93, 162};
  stopMotors();
  setServos(initAngles);
  lastControlMs = millis();
  lastTelemetryMs = lastControlMs;
}

void loop() {
  handleBinaryPacket();
  updateMotorControl();
  sendTelemetry();
}
