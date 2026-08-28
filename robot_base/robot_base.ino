#include <Servo.h>

#define ENC_L_A 2
#define ENC_L_B 4
#define ENC_R_A 3
#define ENC_R_B 5

#define M_L_PWM 9
#define M_L_IN1 7
#define M_L_IN2 8

#define M_R_PWM 10
#define M_R_IN1 11
#define M_R_IN2 12

#define SERVO_PIN 6 

const float MAX_SPEED = 0.35;
const int   MIN_PWM   = 55;
const int   MAX_PWM   = 255;
const int   DIR_L     = 1;
const int   DIR_R     = 1;

const unsigned long CMD_TIMEOUT_MS = 500;
const unsigned long TX_PERIOD_MS   = 20;
const int   SERVO_CENTER_US = 1500;
const int   SERVO_MIN_US    = 500;
const int   SERVO_MAX_US    = 2500;
const float US_PER_DEG      = 11.1;
const float TILT_MIN_DEG    = -90.0;
const float TILT_MAX_DEG    =  90.0;
const float STEP_DEG        =  0.5;
const unsigned long DWELL_MS = 260;
const float PARK_DEG        =  0.0;
const bool  SWEEP_ON_BOOT   = true;
const bool  BOOT_WIGGLE     = true;

volatile long tickL = 0, tickR = 0;

unsigned long lastCmdMs = 0;
unsigned long lastTxMs  = 0;
unsigned long lastStepMs = 0;

Servo tiltServo;
float tiltDeg = 0.0;
float tiltDir = 1.0;
bool  sweepOn = SWEEP_ON_BOOT;

String rxBuf = "";

void isrLeft() {
  if (digitalRead(ENC_L_B)) tickL++; else tickL--;
}
void isrRight() {
  if (digitalRead(ENC_R_B)) tickR--; else tickR++;
}

void driveSide(int pwmPin, int in1, int in2, int pwmSigned) {
  if (pwmSigned == 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(pwmPin, 0);
    return;
  }
  bool fwd = pwmSigned > 0;
  int mag = constrain(abs(pwmSigned), 0, MAX_PWM);
  digitalWrite(in1, fwd ? HIGH : LOW);
  digitalWrite(in2, fwd ? LOW : HIGH);
  analogWrite(pwmPin, mag);
}

int speedToPwm(float v) {
  if (fabs(v) < 1e-4) return 0;
  int pwm = (int)(fabs(v) / MAX_SPEED * MAX_PWM + 0.5);
  pwm = constrain(pwm, MIN_PWM, MAX_PWM);     // lewati deadzone
  return (v > 0) ? pwm : -pwm;
}

void setWheelSpeeds(float vl, float vr) {
  driveSide(M_L_PWM, M_L_IN1, M_L_IN2, speedToPwm(vl) * DIR_L);
  driveSide(M_R_PWM, M_R_IN1, M_R_IN2, speedToPwm(vr) * DIR_R);
}

void stopMotors() {
  driveSide(M_L_PWM, M_L_IN1, M_L_IN2, 0);
  driveSide(M_R_PWM, M_R_IN1, M_R_IN2, 0);
}

void writeTilt(float deg) {
  deg = constrain(deg, TILT_MIN_DEG, TILT_MAX_DEG);
  tiltDeg = deg;
  int us = SERVO_CENTER_US + (int)(deg * US_PER_DEG);
  tiltServo.writeMicroseconds(constrain(us, SERVO_MIN_US, SERVO_MAX_US));
}

void sweepTick() {
  if (!sweepOn) return;
  unsigned long now = millis();
  if (now - lastStepMs < DWELL_MS) return;
  lastStepMs = now;

  float next = tiltDeg + tiltDir * STEP_DEG;
  if (next > TILT_MAX_DEG) {
    next = TILT_MAX_DEG;
    tiltDir = -1.0;
  } else if (next < TILT_MIN_DEG) {
    next = TILT_MIN_DEG;
    tiltDir = 1.0;
  }
  writeTilt(next);
}

void handleLine(String line) {
  line.trim();
  if (line.length() < 1) return;
  char cmd = line.charAt(0);

  if (cmd == 'M' || cmd == 'm') {
    int sp1 = line.indexOf(' ');
    if (sp1 < 0) return;
    int sp2 = line.indexOf(' ', sp1 + 1);
    if (sp2 < 0) return;
    float vl = line.substring(sp1 + 1, sp2).toFloat();
    float vr = line.substring(sp2 + 1).toFloat();
    setWheelSpeeds(vl, vr);
    lastCmdMs = millis();

  } else if (cmd == 'S' || cmd == 's') {
    int sp1 = line.indexOf(' ');
    int val = (sp1 >= 0) ? line.substring(sp1 + 1).toInt() : 0;
    sweepOn = (val != 0);
    if (!sweepOn) writeTilt(PARK_DEG);
    lastStepMs = millis();

  } else if (cmd == 'P' || cmd == 'p') {
    int sp1 = line.indexOf(' ');
    if (sp1 < 0) return;
    sweepOn = false;
    writeTilt(line.substring(sp1 + 1).toFloat());
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(M_L_PWM, OUTPUT); pinMode(M_L_IN1, OUTPUT); pinMode(M_L_IN2, OUTPUT);
  pinMode(M_R_PWM, OUTPUT); pinMode(M_R_IN1, OUTPUT); pinMode(M_R_IN2, OUTPUT);
  stopMotors();

  pinMode(ENC_L_A, INPUT_PULLUP); pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP); pinMode(ENC_R_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isrLeft,  RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isrRight, RISING);

  tiltServo.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
  writeTilt(PARK_DEG);

  if (BOOT_WIGGLE) {
    delay(400);
    writeTilt(-10.0); delay(400);
    writeTilt( 15.0); delay(400);
    writeTilt(PARK_DEG); delay(400);
  }

  rxBuf.reserve(32);
  lastCmdMs = millis();
  lastStepMs = millis();
}

void loop() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      handleLine(rxBuf);
      rxBuf = "";
    } else if (rxBuf.length() < 28) {
      rxBuf += c;
    }
  }

  if (millis() - lastCmdMs > CMD_TIMEOUT_MS) {
    stopMotors();
  }

  sweepTick();

  unsigned long now = millis();
  if (now - lastTxMs >= TX_PERIOD_MS) {
    lastTxMs = now;

    noInterrupts();
    long l = tickL, r = tickR;
    interrupts();

    long tiltX100 = (long)(tiltDeg * 100.0);
    Serial.print('E');
    Serial.print(' '); Serial.print(l);
    Serial.print(' '); Serial.print(r);
    Serial.print(' '); Serial.println(tiltX100);
  }
}
