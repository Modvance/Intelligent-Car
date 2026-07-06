#include "RL_ESP32_Motor.h"
#include <math.h>

Motor motors[] = {
  Motor(1, 12, 13),
  Motor(2, 14, 15),
  Motor(3, 16, 17),
  Motor(4, 18, 19),
};

const uint8_t WHEEL_COUNT = 4;
// GMR encoder is treated as 500 pulses per wheel revolution for speed display.
const float ENCODER_PULSES_PER_WHEEL_REV = 500.0f;
const float GEAR_RATIO = 48.0f;
const unsigned long SAMPLE_PERIOD_MS = 500;

// Keep this conservative for first power-on tests.
const int MAX_TEST_PWM = 45;
const int AUTO_TEST_PWM = 25;

const uint8_t ENCODER_PINS[WHEEL_COUNT][2] = {
  {21, 22},
  {23, 27},
  {34, 35},
  {36, 39},
};

// If a motor spins the wrong way and wires 1/6 cannot be swapped,
// change that motor's value to -1 and upload again.
const int8_t MOTOR_SIGN[WHEEL_COUNT] = {-1, -1, 1, 1};

// Pulse count sign is not used for closed-loop direction; motor command sign is.
const int8_t ENCODER_SIGN[WHEEL_COUNT] = {1, 1, 1, 1};

volatile long encoderTicks[WHEEL_COUNT] = {0, 0, 0, 0};
long lastTicks[WHEEL_COUNT] = {0, 0, 0, 0};
int currentPwm[WHEEL_COUNT] = {0, 0, 0, 0};
unsigned long lastSampleMs = 0;

void IRAM_ATTR handleEncoder(uint8_t index) {
  encoderTicks[index] += ENCODER_SIGN[index];
}

void IRAM_ATTR handleEncoder0() { handleEncoder(0); }
void IRAM_ATTR handleEncoder1() { handleEncoder(1); }
void IRAM_ATTR handleEncoder2() { handleEncoder(2); }
void IRAM_ATTR handleEncoder3() { handleEncoder(3); }

void initEncoders() {
  for (uint8_t i = 0; i < WHEEL_COUNT; i++) {
    pinMode(ENCODER_PINS[i][0], INPUT);
    pinMode(ENCODER_PINS[i][1], INPUT);
  }

  attachInterrupt(digitalPinToInterrupt(ENCODER_PINS[0][0]), handleEncoder0, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PINS[1][0]), handleEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PINS[2][0]), handleEncoder2, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PINS[3][0]), handleEncoder3, RISING);
}

void setMotor(uint8_t index, int pwm) {
  if (index >= WHEEL_COUNT) {
    return;
  }

  pwm = constrain(pwm, -MAX_TEST_PWM, MAX_TEST_PWM);
  int driverPwm = pwm * MOTOR_SIGN[index];
  currentPwm[index] = pwm;
  motors[index].Motor_Speed(driverPwm);

  Serial.print("M");
  Serial.print(index + 1);
  Serial.print(" pwm=");
  Serial.print(pwm);
  Serial.print(" driver=");
  Serial.println(driverPwm);
}

void stopAll() {
  for (uint8_t i = 0; i < WHEEL_COUNT; i++) {
    setMotor(i, 0);
  }
}

void printHelp() {
  Serial.println();
  Serial.println("ESP32 TT motor + GMR encoder test");
  Serial.println("Serial Monitor: 115200 baud, newline enabled");
  Serial.println("Commands:");
  Serial.println("  help        show this menu");
  Serial.println("  stop        stop all motors");
  Serial.println("  rpm         print tick/rpm now");
  Serial.println("  auto        test M1-M4 forward/backward at low speed");
  Serial.println("  all 25      set all motors to pwm 25");
  Serial.println("  all -25     set all motors to pwm -25");
  Serial.println("  m1 25       set motor 1 pwm 25");
  Serial.println("  m1 -25      set motor 1 pwm -25");
  Serial.println("  m2/m3/m4    same as m1");
  Serial.println();
}

void printStatus(bool force = false) {
  unsigned long now = millis();
  unsigned long elapsed = now - lastSampleMs;

  if (!force && elapsed < SAMPLE_PERIOD_MS) {
    return;
  }
  if (elapsed == 0) {
    elapsed = SAMPLE_PERIOD_MS;
  }

  long ticks[WHEEL_COUNT];
  noInterrupts();
  for (uint8_t i = 0; i < WHEEL_COUNT; i++) {
    ticks[i] = encoderTicks[i];
  }
  interrupts();

  Serial.println("motor,pwm,ticks,delta,motor_rpm,wheel_rpm");
  for (uint8_t i = 0; i < WHEEL_COUNT; i++) {
    long delta = ticks[i] - lastTicks[i];
    float wheelRpmMagnitude = (fabsf((float)delta) * 60000.0f) / (ENCODER_PULSES_PER_WHEEL_REV * elapsed);
    float wheelRpm = currentPwm[i] > 0 ? wheelRpmMagnitude : -wheelRpmMagnitude;
    if (currentPwm[i] == 0) {
      wheelRpm = 0.0f;
    }
    float motorRpm = wheelRpm * GEAR_RATIO;

    Serial.print("M");
    Serial.print(i + 1);
    Serial.print(",");
    Serial.print(currentPwm[i]);
    Serial.print(",");
    Serial.print(ticks[i]);
    Serial.print(",");
    Serial.print(delta);
    Serial.print(",");
    Serial.print(motorRpm, 2);
    Serial.print(",");
    Serial.println(wheelRpm, 2);

    lastTicks[i] = ticks[i];
  }

  lastSampleMs = now;
}

void holdWithStatus(unsigned long durationMs) {
  unsigned long started = millis();
  while (millis() - started < durationMs) {
    printStatus();
    delay(20);
  }
}

void runAutoTest() {
  Serial.println("Auto test starts. Keep wheels lifted from the ground.");
  stopAll();
  holdWithStatus(500);

  for (uint8_t i = 0; i < WHEEL_COUNT; i++) {
    Serial.print("Testing M");
    Serial.print(i + 1);
    Serial.println(" forward");
    setMotor(i, AUTO_TEST_PWM);
    holdWithStatus(1500);

    Serial.print("Testing M");
    Serial.print(i + 1);
    Serial.println(" backward");
    setMotor(i, -AUTO_TEST_PWM);
    holdWithStatus(1500);

    setMotor(i, 0);
    holdWithStatus(700);
  }

  stopAll();
  Serial.println("Auto test finished.");
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

  if (line == "stop") {
    stopAll();
    return;
  }

  if (line == "rpm") {
    printStatus(true);
    return;
  }

  if (line == "auto") {
    runAutoTest();
    return;
  }

  if (line.startsWith("all ")) {
    int pwm = line.substring(4).toInt();
    for (uint8_t i = 0; i < WHEEL_COUNT; i++) {
      setMotor(i, pwm);
    }
    return;
  }

  if (line.length() >= 4 && line.charAt(0) == 'm') {
    int motorNumber = line.charAt(1) - '1';
    if (motorNumber >= 0 && motorNumber < WHEEL_COUNT) {
      String pwmText = line.substring(2);
      pwmText.trim();
      setMotor((uint8_t)motorNumber, pwmText.toInt());
      return;
    }
  }

  Serial.print("Unknown command: ");
  Serial.println(line);
  printHelp();
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  for (uint8_t i = 0; i < WHEEL_COUNT; i++) {
    motors[i].mcpwm_begin();
  }
  initEncoders();
  stopAll();

  lastSampleMs = millis();
  printHelp();
}

void loop() {
  printStatus();

  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    handleCommand(line);
  }
}
