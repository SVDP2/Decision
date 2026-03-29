#include <Arduino.h>
#include <math.h>

// ============================================================
// Longitudinal (drive)
// ============================================================
const uint8_t DRIVE_PWM_PIN = 5;   // PWM1
const uint8_t DRIVE_DIR_PIN = 4;   // DIR1

// throttle 양수인데 후진하면 아래 두 값을 서로 바꾸세요.
const uint8_t DRIVE_FORWARD_LEVEL = HIGH;
const uint8_t DRIVE_REVERSE_LEVEL = LOW;
const unsigned long DRIVE_DIR_CHANGE_DEADTIME_MS = 50;

const int MAX_FORWARD_THROTTLE = 200;
const int MAX_REVERSE_THROTTLE = 50;

// ============================================================
// Lateral (steering)
// ============================================================
const uint8_t POT_PIN = A1;

const uint8_t IN1 = 9;
const uint8_t IN2 = 10;
const uint8_t ENA = 2;

const int RIGHT_END = 168;
const int CENTER    = 585;
const int LEFT_END  = 1003;

// 왼쪽으로 갈수록 가변저항 ADC 값이 커지면 true
const bool LEFT_INCREASES_VALUE = true;

// ============================================================
// RC receiver input
// ============================================================
// Mega 2560에서 외부 인터럽트 사용: D19(CH1), D18(CH2)
const uint8_t RC_THROTTLE_PIN = 19;   // CH1 = throttle
const uint8_t RC_STEER_PIN    = 18;   // CH2 = steering

// 마지막 유효 펄스 이후 이 시간(us) 동안 갱신이 없으면 stale로 판단
const unsigned long RC_SIGNAL_STALE_US = 100000;

// 비정상 pulse 보호용 1차 유효 범위
const int RC_RAW_VALID_MIN_US = 900;
const int RC_RAW_VALID_MAX_US = 2100;

// ISR 내부에서 허용할 펄스 범위(약간 여유)
const int RC_ISR_MIN_US = 800;
const int RC_ISR_MAX_US = 2200;

// 실제 제어에 사용할 안전 구간
const int RC_ACTIVE_MIN_US = 1200;
const int RC_ACTIVE_MAX_US = 1800;
const int RC_CENTER_US     = 1500;

// 중립 떨림 방지용 데드밴드
const int RC_THROTTLE_DEADBAND_US = 40;
const int RC_STEER_DEADBAND_US    = 40;

// RC 방향이 반대로 나오면 바꾸세요.
const bool RC_THROTTLE_HIGH_US_IS_FORWARD = true;
const bool RC_STEER_HIGH_US_IS_LEFT       = false;

// 수신기 신호가 잠깐 튀는 경우를 막기 위한 유예 시간
const unsigned long RC_FAILSAFE_DELAY_MS = 150;

// ============================================================
// Control timing / filter
// ============================================================
const unsigned long CONTROL_PERIOD_MS = 20;

// 기존보다 가볍게 조정
const int READ_SAMPLES  = 3;
const int READ_DELAY_MS = 0;
const float POSITION_LPF_ALPHA = 0.8f;

// 목표값 변화율 제한
const float MAX_SETPOINT_RATE = 5000.0f;

// 도착 판정
const float TARGET_TOLERANCE = 8.0f;
const int SETTLE_COUNT_REQUIRED = 4;
const unsigned long STATUS_PERIOD_MS = 200;

// PID
const float POSITION_ERROR_DEADBAND = 4.0f;
const float STEER_KP = 0.65f;
const float STEER_KI = 0.015f;
const float STEER_KD = 0.02f;
const float STEER_INTEGRAL_LIMIT = 300.0f;

// PWM 처리
const float PWM_LPF_ALPHA = 0.8f;
const float PWM_DEADBAND = 6.0f;
const float MIN_DRIVE_PWM = 20.0f;
const float MAX_DRIVE_PWM = 255.0f;

// 모터 방향 반전 시 deadtime
const unsigned long DIR_CHANGE_DEADTIME_MS = 5;

// 미세한 RC steering 변화는 무시
const int STEER_TARGET_UPDATE_EPS = 2;

// ============================================================
// Longitudinal ramp limit
// ============================================================
// PWM 변화 속도 (단위: PWM count / sec)
const float THROTTLE_ACCEL_RATE = 250.0f;
const float THROTTLE_DECEL_RATE = 700.0f;
const float THROTTLE_ZERO_SNAP  = 2.0f;

// ============================================================
// Type / state
// ============================================================
enum Direction { DIR_STOP = 0, DIR_LEFT = 1, DIR_RIGHT = -1 };
Direction lastSteerDir = DIR_STOP;

struct SteeringState {
  bool active;

  int finalTarget;

  float filteredPosition;
  float setpoint;

  float pidIntegral;
  float previousError;
  float pwmFiltered;

  int stableCount;
  Direction lastCommandedDir;

  unsigned long lastControlTime;
  unsigned long lastStatusTime;
};

struct RcState {
  unsigned long throttlePulse;
  unsigned long steerPulse;
  int throttleCmd;
  int steerTarget;
  bool signalValid;
  bool failsafeActive;
  unsigned long lastValidTime;
};

SteeringState steer = {
  false,
  CENTER,
  0.0f, 0.0f,
  0.0f, 0.0f, 0.0f,
  0, DIR_STOP,
  0, 0
};

RcState rc = {
  0, 0,
  0, CENTER,
  false, false,
  0
};

int currentThrottle = 0;
int lastDriveSign = 0;

int driveTargetThrottle = 0;
float driveRampThrottle = 0.0f;
unsigned long lastDriveRampUpdateMs = 0;

// ============================================================
// RC interrupt storage
// ============================================================
volatile uint32_t rcThrottleRiseUs = 0;
volatile uint32_t rcSteerRiseUs    = 0;

volatile uint16_t rcThrottlePulseUs = RC_CENTER_US;
volatile uint16_t rcSteerPulseUs    = RC_CENTER_US;

volatile uint32_t rcThrottleLastUpdateUs = 0;
volatile uint32_t rcSteerLastUpdateUs    = 0;

// ============================================================
// RC ISRs
// ============================================================
void isrRcThrottle() {
  uint32_t now = micros();

  if (digitalRead(RC_THROTTLE_PIN) == HIGH) {
    rcThrottleRiseUs = now;
  } else {
    uint32_t width = now - rcThrottleRiseUs;
    if (width >= RC_ISR_MIN_US && width <= RC_ISR_MAX_US) {
      rcThrottlePulseUs = (uint16_t)width;
      rcThrottleLastUpdateUs = now;
    }
  }
}

void isrRcSteer() {
  uint32_t now = micros();

  if (digitalRead(RC_STEER_PIN) == HIGH) {
    rcSteerRiseUs = now;
  } else {
    uint32_t width = now - rcSteerRiseUs;
    if (width >= RC_ISR_MIN_US && width <= RC_ISR_MAX_US) {
      rcSteerPulseUs = (uint16_t)width;
      rcSteerLastUpdateUs = now;
    }
  }
}

// ============================================================
// Utility
// ============================================================
bool getLatestRcPulses(unsigned long &throttlePulse, unsigned long &steerPulse) {
  uint32_t throttleLast;
  uint32_t steerLast;
  uint32_t nowUs = micros();

  noInterrupts();
  throttlePulse = rcThrottlePulseUs;
  steerPulse    = rcSteerPulseUs;
  throttleLast  = rcThrottleLastUpdateUs;
  steerLast     = rcSteerLastUpdateUs;
  interrupts();

  bool throttleFresh = (nowUs - throttleLast) <= RC_SIGNAL_STALE_US;
  bool steerFresh    = (nowUs - steerLast)    <= RC_SIGNAL_STALE_US;

  return throttleFresh && steerFresh;
}

bool isRcPulseRawValid(unsigned long pulseUs) {
  return (pulseUs >= RC_RAW_VALID_MIN_US && pulseUs <= RC_RAW_VALID_MAX_US);
}

long mapClamped(long x, long inMin, long inMax, long outMin, long outMax) {
  x = constrain(x, inMin, inMax);
  return map(x, inMin, inMax, outMin, outMax);
}

int mapRcPulseToThrottle(unsigned long pulseUs) {
  pulseUs = constrain((int)pulseUs, RC_ACTIVE_MIN_US, RC_ACTIVE_MAX_US);

  const int upperNeutral = RC_CENTER_US + RC_THROTTLE_DEADBAND_US;
  const int lowerNeutral = RC_CENTER_US - RC_THROTTLE_DEADBAND_US;

  int throttleCmd = 0;

  if (RC_THROTTLE_HIGH_US_IS_FORWARD) {
    if ((int)pulseUs > upperNeutral) {
      throttleCmd = (int)mapClamped((long)pulseUs,
                                    upperNeutral,
                                    RC_ACTIVE_MAX_US,
                                    0,
                                    MAX_FORWARD_THROTTLE);
    } else if ((int)pulseUs < lowerNeutral) {
      throttleCmd = (int)mapClamped((long)pulseUs,
                                    RC_ACTIVE_MIN_US,
                                    lowerNeutral,
                                    -MAX_REVERSE_THROTTLE,
                                    0);
    }
  } else {
    if ((int)pulseUs > upperNeutral) {
      throttleCmd = (int)mapClamped((long)pulseUs,
                                    upperNeutral,
                                    RC_ACTIVE_MAX_US,
                                    0,
                                    -MAX_REVERSE_THROTTLE);
    } else if ((int)pulseUs < lowerNeutral) {
      throttleCmd = (int)mapClamped((long)pulseUs,
                                    RC_ACTIVE_MIN_US,
                                    lowerNeutral,
                                    MAX_FORWARD_THROTTLE,
                                    0);
    }
  }

  return constrain(throttleCmd, -MAX_REVERSE_THROTTLE, MAX_FORWARD_THROTTLE);
}

int mapRcPulseToSteerTarget(unsigned long pulseUs) {
  pulseUs = constrain((int)pulseUs, RC_ACTIVE_MIN_US, RC_ACTIVE_MAX_US);

  const int upperNeutral = RC_CENTER_US + RC_STEER_DEADBAND_US;
  const int lowerNeutral = RC_CENTER_US - RC_STEER_DEADBAND_US;

  if ((int)pulseUs > upperNeutral) {
    if (RC_STEER_HIGH_US_IS_LEFT) {
      return (int)mapClamped((long)pulseUs,
                             upperNeutral,
                             RC_ACTIVE_MAX_US,
                             CENTER,
                             LEFT_END);
    } else {
      return (int)mapClamped((long)pulseUs,
                             upperNeutral,
                             RC_ACTIVE_MAX_US,
                             CENTER,
                             RIGHT_END);
    }
  }

  if ((int)pulseUs < lowerNeutral) {
    if (RC_STEER_HIGH_US_IS_LEFT) {
      return (int)mapClamped((long)pulseUs,
                             RC_ACTIVE_MIN_US,
                             lowerNeutral,
                             RIGHT_END,
                             CENTER);
    } else {
      return (int)mapClamped((long)pulseUs,
                             RC_ACTIVE_MIN_US,
                             lowerNeutral,
                             LEFT_END,
                             CENTER);
    }
  }

  return CENTER;
}

// ============================================================
// Longitudinal helpers
// ============================================================
void stopDriveMotor() {
  analogWrite(DRIVE_PWM_PIN, 0);
  currentThrottle = 0;
}

void applyDriveThrottle(int throttle) {
  throttle = constrain(throttle, -MAX_REVERSE_THROTTLE, MAX_FORWARD_THROTTLE);

  if (throttle == 0) {
    analogWrite(DRIVE_PWM_PIN, 0);
    currentThrottle = 0;
    return;
  }

  int newSign = (throttle > 0) ? 1 : -1;
  uint8_t dirLevel = (throttle > 0) ? DRIVE_FORWARD_LEVEL : DRIVE_REVERSE_LEVEL;
  int pwm = abs(throttle);

  if (lastDriveSign != 0 && lastDriveSign != newSign) {
    analogWrite(DRIVE_PWM_PIN, 0);
    delay(DRIVE_DIR_CHANGE_DEADTIME_MS);
  }

  digitalWrite(DRIVE_DIR_PIN, dirLevel);
  analogWrite(DRIVE_PWM_PIN, pwm);

  currentThrottle = throttle;
  lastDriveSign = newSign;
}

void updateDriveThrottleRamp() {
  unsigned long now = millis();

  if (lastDriveRampUpdateMs == 0) {
    lastDriveRampUpdateMs = now;
    return;
  }

  float dt = (now - lastDriveRampUpdateMs) * 0.001f;
  lastDriveRampUpdateMs = now;

  if (dt <= 0.0001f) {
    return;
  }

  float current = driveRampThrottle;
  float target  = constrain((float)driveTargetThrottle,
                            -(float)MAX_REVERSE_THROTTLE,
                            (float)MAX_FORWARD_THROTTLE);
  float diff    = target - current;

  if (fabs(diff) <= 0.5f) {
    driveRampThrottle = target;
    applyDriveThrottle((int)driveRampThrottle);
    return;
  }

  bool signChanged =
      ((current > 0.0f) && (target < 0.0f)) ||
      ((current < 0.0f) && (target > 0.0f));

  bool magnitudeIncreasingSameDirection =
      (!signChanged) && (fabs(target) > fabs(current));

  float rate = magnitudeIncreasingSameDirection ? THROTTLE_ACCEL_RATE
                                                : THROTTLE_DECEL_RATE;

  float maxStep = rate * dt;

  if (diff > maxStep) {
    current += maxStep;
  } else if (diff < -maxStep) {
    current -= maxStep;
  } else {
    current = target;
  }

  // 방향 반전 시 반드시 0을 한 번 거치게 함
  if ((driveRampThrottle > 0.0f && current < 0.0f) ||
      (driveRampThrottle < 0.0f && current > 0.0f)) {
    current = 0.0f;
  }

  // 0 근처 떨림 제거
  if (fabs(current) <= THROTTLE_ZERO_SNAP &&
      fabs(target) <= THROTTLE_ZERO_SNAP) {
    current = 0.0f;
  }

  driveRampThrottle = constrain(current,
                                -(float)MAX_REVERSE_THROTTLE,
                                (float)MAX_FORWARD_THROTTLE);
  applyDriveThrottle((int)driveRampThrottle);
}

// ============================================================
// Steering helpers
// ============================================================
void stopSteerMotor() {
  analogWrite(ENA, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  lastSteerDir = DIR_STOP;
}

void applySteerMotor(Direction dir, int pwm) {
  pwm = constrain(pwm, 0, 255);

  if (dir == DIR_LEFT) {
    if (lastSteerDir == DIR_RIGHT) {
      stopSteerMotor();
      delay(DIR_CHANGE_DEADTIME_MS);
    }
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, pwm);
    lastSteerDir = DIR_LEFT;
  }
  else if (dir == DIR_RIGHT) {
    if (lastSteerDir == DIR_LEFT) {
      stopSteerMotor();
      delay(DIR_CHANGE_DEADTIME_MS);
    }
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, pwm);
    lastSteerDir = DIR_RIGHT;
  }
  else {
    stopSteerMotor();
  }
}

int readSteeringValue() {
  long sum = 0;
  for (int i = 0; i < READ_SAMPLES; i++) {
    sum += analogRead(POT_PIN);
    if (READ_DELAY_MS > 0) {
      delay(READ_DELAY_MS);
    }
  }
  return (int)(sum / READ_SAMPLES);
}

Direction commandToDirection(float signedCommand) {
  if (signedCommand > 0.0f) {
    return LEFT_INCREASES_VALUE ? DIR_LEFT : DIR_RIGHT;
  }
  else if (signedCommand < 0.0f) {
    return LEFT_INCREASES_VALUE ? DIR_RIGHT : DIR_LEFT;
  }
  return DIR_STOP;
}

void printControlStatus(float finalError, float appliedPwm, Direction dir) {
  Serial.print("RC CH1: ");
  Serial.print(rc.throttlePulse);
  Serial.print(" us -> CmdTHR: ");
  Serial.print(rc.throttleCmd);
  Serial.print(" | OutTHR: ");
  Serial.print(currentThrottle);
  Serial.print(" | RC CH2: ");
  Serial.print(rc.steerPulse);
  Serial.print(" us -> Target: ");
  Serial.print(rc.steerTarget);
  Serial.print(" | Cur: ");
  Serial.print(steer.filteredPosition, 1);
  Serial.print(" | Err: ");
  Serial.print(finalError, 1);
  Serial.print(" | PWM: ");
  Serial.print(appliedPwm, 1);
  Serial.print(" | Dir: ");

  if (dir == DIR_LEFT) Serial.println("L");
  else if (dir == DIR_RIGHT) Serial.println("R");
  else Serial.println("STOP");
}

void enterRcFailsafe(const char *reason) {
  driveTargetThrottle = 0;
  driveRampThrottle = 0.0f;
  lastDriveRampUpdateMs = millis();

  stopDriveMotor();
  stopSteerMotor();
  steer.active = false;

  if (!rc.failsafeActive) {
    Serial.print("RC FAILSAFE: ");
    Serial.println(reason);
  }

  rc.failsafeActive = true;
  rc.signalValid = false;
  rc.throttleCmd = 0;
  rc.steerTarget = CENTER;
}

void startSteeringControl(int target) {
  target = constrain(target, RIGHT_END, LEFT_END);

  float currentRaw = (float)readSteeringValue();

  steer.active = true;
  steer.finalTarget = target;
  steer.filteredPosition = currentRaw;
  steer.setpoint = currentRaw;

  steer.pidIntegral = 0.0f;
  steer.previousError = 0.0f;
  steer.pwmFiltered = 0.0f;

  steer.stableCount = 0;
  steer.lastCommandedDir = DIR_STOP;

  steer.lastControlTime = millis();
  steer.lastStatusTime = 0;

  stopSteerMotor();

  Serial.print("조향 제어 시작 / 목표: ");
  Serial.println(steer.finalTarget);
}

void setSteeringTargetContinuous(int target) {
  target = constrain(target, RIGHT_END, LEFT_END);

  if (!steer.active) {
    startSteeringControl(target);
    return;
  }

  if (abs(target - steer.finalTarget) >= STEER_TARGET_UPDATE_EPS) {
    steer.finalTarget = target;
    steer.stableCount = 0;
  }
}

void updateSteeringControl() {
  if (!steer.active) {
    return;
  }

  unsigned long now = millis();
  if (now - steer.lastControlTime < CONTROL_PERIOD_MS) {
    return;
  }

  float dt = (now - steer.lastControlTime) * 0.001f;
  steer.lastControlTime = now;

  if (dt <= 0.0001f) {
    return;
  }

  float currentRaw = (float)readSteeringValue();
  steer.filteredPosition = POSITION_LPF_ALPHA * currentRaw +
                           (1.0f - POSITION_LPF_ALPHA) * steer.filteredPosition;

  float finalError = steer.finalTarget - steer.filteredPosition;

  float setpointDiff = steer.finalTarget - steer.setpoint;
  float maxStep = MAX_SETPOINT_RATE * dt;
  setpointDiff = constrain(setpointDiff, -maxStep, maxStep);
  steer.setpoint += setpointDiff;
  steer.setpoint = constrain(steer.setpoint, (float)RIGHT_END, (float)LEFT_END);

  if (fabs(finalError) <= TARGET_TOLERANCE) {
    stopSteerMotor();
    steer.pwmFiltered = 0.0f;
    steer.pidIntegral = 0.0f;
    steer.previousError = 0.0f;
    steer.lastCommandedDir = DIR_STOP;

    if (steer.stableCount < SETTLE_COUNT_REQUIRED) {
      steer.stableCount++;
    }

    if (now - steer.lastStatusTime >= STATUS_PERIOD_MS) {
      printControlStatus(finalError, 0.0f, DIR_STOP);
      steer.lastStatusTime = now;
    }
    return;
  }

  steer.stableCount = 0;

  float controlError = steer.setpoint - steer.filteredPosition;

  if (fabs(controlError) < POSITION_ERROR_DEADBAND) {
    controlError = 0.0f;
  }

  float derivative = (controlError - steer.previousError) / dt;

  float candidateIntegral = steer.pidIntegral + controlError * dt;
  candidateIntegral = constrain(candidateIntegral,
                                -STEER_INTEGRAL_LIMIT,
                                STEER_INTEGRAL_LIMIT);

  float tentativeOutput = STEER_KP * controlError +
                          STEER_KI * candidateIntegral +
                          STEER_KD * derivative;

  bool saturatingHigh = (tentativeOutput >  MAX_DRIVE_PWM && controlError > 0.0f);
  bool saturatingLow  = (tentativeOutput < -MAX_DRIVE_PWM && controlError < 0.0f);

  if (!(saturatingHigh || saturatingLow)) {
    steer.pidIntegral = candidateIntegral;
  }

  float pidOutput = STEER_KP * controlError +
                    STEER_KI * steer.pidIntegral +
                    STEER_KD * derivative;

  pidOutput = constrain(pidOutput, -MAX_DRIVE_PWM, MAX_DRIVE_PWM);
  steer.previousError = controlError;

  float targetPwm = fabs(pidOutput);
  steer.pwmFiltered = PWM_LPF_ALPHA * targetPwm +
                      (1.0f - PWM_LPF_ALPHA) * steer.pwmFiltered;

  float compensatedPwm = 0.0f;
  Direction cmdDir = DIR_STOP;

  if (steer.pwmFiltered <= PWM_DEADBAND) {
    stopSteerMotor();
    compensatedPwm = 0.0f;
    cmdDir = DIR_STOP;
  } else {
    compensatedPwm = steer.pwmFiltered;

    if (compensatedPwm < MIN_DRIVE_PWM) {
      compensatedPwm = MIN_DRIVE_PWM;
    }

    compensatedPwm = constrain(compensatedPwm, 0.0f, MAX_DRIVE_PWM);
    cmdDir = commandToDirection(pidOutput);
    applySteerMotor(cmdDir, (int)compensatedPwm);
  }

  steer.lastCommandedDir = cmdDir;

  if (now - steer.lastStatusTime >= STATUS_PERIOD_MS) {
    printControlStatus(finalError, compensatedPwm, cmdDir);
    steer.lastStatusTime = now;
  }
}

// ============================================================
// RC update
// ============================================================
void updateRcControl() {
  unsigned long throttlePulse = 0;
  unsigned long steerPulse    = 0;
  bool fresh = getLatestRcPulses(throttlePulse, steerPulse);
  unsigned long now = millis();

  rc.throttlePulse = throttlePulse;
  rc.steerPulse    = steerPulse;

  bool valid = fresh &&
               isRcPulseRawValid(throttlePulse) &&
               isRcPulseRawValid(steerPulse);
  rc.signalValid = valid;

  if (valid) {
    rc.lastValidTime = now;
    rc.throttleCmd = mapRcPulseToThrottle(throttlePulse);
    rc.steerTarget = mapRcPulseToSteerTarget(steerPulse);

    if (rc.failsafeActive) {
      Serial.println("RC signal recovered");
      rc.failsafeActive = false;
    }

    driveTargetThrottle = rc.throttleCmd;
    setSteeringTargetContinuous(rc.steerTarget);
    return;
  }

  if (now - rc.lastValidTime >= RC_FAILSAFE_DELAY_MS) {
    enterRcFailsafe("signal lost or stale pulse");
  }
}

void printCurrentStatus() {
  Serial.println();
  Serial.println("===== 현재 상태 =====");
  Serial.print("RC CH1(us)    : ");
  Serial.println(rc.throttlePulse);
  Serial.print("RC CH2(us)    : ");
  Serial.println(rc.steerPulse);
  Serial.print("Throttle Cmd  : ");
  Serial.println(rc.throttleCmd);
  Serial.print("Throttle Out  : ");
  Serial.println(currentThrottle);
  Serial.print("Steer Target  : ");
  Serial.println(rc.steerTarget);
  Serial.print("Steer ADC     : ");
  Serial.println(readSteeringValue());
  Serial.print("Steer Active  : ");
  Serial.println(steer.active ? "YES" : "NO");
  Serial.print("RC Valid      : ");
  Serial.println(rc.signalValid ? "YES" : "NO");
  Serial.print("RC Failsafe   : ");
  Serial.println(rc.failsafeActive ? "YES" : "NO");
  Serial.println("====================");
  Serial.println();
}

void printHelp() {
  Serial.println();
  Serial.println("===== T870 RC Integrated Controller =====");
  Serial.print("RC throttle pin : ");
  Serial.println(RC_THROTTLE_PIN);
  Serial.print("RC steer pin    : ");
  Serial.println(RC_STEER_PIN);
  Serial.print("RC active range : ");
  Serial.print(RC_ACTIVE_MIN_US);
  Serial.print(" ~ ");
  Serial.println(RC_ACTIVE_MAX_US);
  Serial.print("RC neutral      : ");
  Serial.println(RC_CENTER_US);
  Serial.print("Throttle deadband(us) : ");
  Serial.println(RC_THROTTLE_DEADBAND_US);
  Serial.print("Steer deadband(us)    : ");
  Serial.println(RC_STEER_DEADBAND_US);
  Serial.print("Throttle accel rate   : ");
  Serial.println(THROTTLE_ACCEL_RATE);
  Serial.print("Throttle decel rate   : ");
  Serial.println(THROTTLE_DECEL_RATE);
  Serial.print("Forward max throttle  : ");
  Serial.println(MAX_FORWARD_THROTTLE);
  Serial.print("Reverse max throttle  : ");
  Serial.println(-MAX_REVERSE_THROTTLE);
  Serial.print("Steering ADC range    : ");
  Serial.print(RIGHT_END);
  Serial.print(" ~ ");
  Serial.println(LEFT_END);
  Serial.println("참고:");
  Serial.println("  1) RC 입력선은 Mega 인터럽트 핀으로 옮기세요. CH1 -> D19, CH2 -> D18");
  Serial.println("  2) throttle 방향이 반대면 RC_THROTTLE_HIGH_US_IS_FORWARD 값을 바꾸세요.");
  Serial.println("  3) steering 방향이 반대면 RC_STEER_HIGH_US_IS_LEFT 값을 바꾸세요.");
  Serial.println("  4) neutral에서 떨리면 deadband를 더 키우세요.");
  Serial.println("=========================================");
  Serial.println();
}

// ============================================================
// Arduino entry
// ============================================================
void setup() {
  pinMode(DRIVE_PWM_PIN, OUTPUT);
  pinMode(DRIVE_DIR_PIN, OUTPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  pinMode(RC_THROTTLE_PIN, INPUT);
  pinMode(RC_STEER_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(RC_THROTTLE_PIN), isrRcThrottle, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RC_STEER_PIN), isrRcSteer, CHANGE);

  Serial.begin(115200);

  stopDriveMotor();
  stopSteerMotor();
  digitalWrite(DRIVE_DIR_PIN, DRIVE_FORWARD_LEVEL);

  delay(300);

  // 초기 stale 판단 방지
  noInterrupts();
  uint32_t nowUs = micros();
  rcThrottleLastUpdateUs = nowUs;
  rcSteerLastUpdateUs = nowUs;
  interrupts();

  driveTargetThrottle = 0;
  driveRampThrottle = 0.0f;
  lastDriveRampUpdateMs = millis();

  printHelp();
  printCurrentStatus();
}

void loop() {
  updateRcControl();
  updateDriveThrottleRamp();
  updateSteeringControl();
}