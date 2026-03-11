#include <Arduino.h>
#include "Arduino_LED_Matrix.h"   // LED 매트릭스 라이브러리 포함
#include "frames.h"               // 사용자 정의 아이콘 헤더

//---------------------- 핀 정의 (Arduino UNO R4 wifi 보드) --------------------//
// PWM 가능 여부, 인터럽트 가능 핀 등을 GIGA R1 데이터시트 확인 후 배치
#define motorDirectionPin_Front  11
#define motorDirectionPin_Rear   9
#define motorSpeedPin_Front      10
#define motorSpeedPin_Rear       8

#define STEERING_DIR_PIN 6   // 예: 방향 제어용
#define STEERING_PWM_PIN 7   // 예: PWM 출력용

#define encoderPin_1  2   // 인터럽트 사용 핀
#define encoderPin_2  3   

#define potentiometerPin_ A2

#define remote_GEAR  A3  // ch3, A3: 조종기 키면 약 1000, 끄면 0 
#define remote_AILE  A4  // ch1, A4: 조향 채널 (1000 ~ 2000, 1500 중립)
#define remote_THRO  A5  // ch2, A5: 쓰로틀 채널 (1000 ~ 2000, 1500 중립)

// LED 매트릭스 관련 객체
ArduinoLEDMatrix matrix;

// 시리얼 상태 코드 정의
#define SERIAL_WAITING   0
#define SERIAL_CONNECTED 1
#define SERIAL_ERROR     2

#define MANUAL      0
#define AUTONOMOUS  1

//---------------------- 전역 변수 --------------------//
unsigned long last_time = 0;

int MODE = MANUAL;

int m_aile = 1500;   // ch1, A4: 조향 채널 (1000 ~ 2000, 1500 중립)
int m_thro = 1500;   // ch2, A5: 쓰로틀 채널 (1000 ~ 2000, 1500 중립)

// LPF 필터링된 RC 조종기 속도 값을 저장할 변수
float m_remote_speed_filtered = 0.0; 
// RC 조종기 속도 LPF 계수 (0.0 ~ 1.0, 작을수록 부드러워짐)
const float remote_speed_lpf_alpha = 0.2;

float m_remote_speed = 0.0;
float m_remote_steering_angle = 0.0;

float m_auto_speed = 0.0;
float m_auto_steering_angle = 0.0;

float dt =  0.067; // 제어(루프) 주기 (초 단위, 예: 0.1s)
int serial_state = SERIAL_WAITING;

//----------- speed -----------//
// speed = (Δencoder_count / pulses_per_rev) * wheel_circumference / dt;
const float pulses_per_rev = 15000.0;
const float wheel_circumference = 0.87699;  // meter
volatile int m_encoder_count = 0; // 엔코더 카운트
int speed_encoder_count = 0;

int speed_encoder_count_prev = 0;

float ramped_speed_target = 0.0; // 부드럽게 변화시킬 중간 목표 속도
const float MAX_ACCELERATION = 1.0; // 최대 가속도 (m/s^2)
const float MAX_DECELERATION = 1.0; // 최대 감속도 (m/s^2)
float speed_target = 0.0;

float speed_raw = 0.0; //lpf 전
float speed_filtered = 0.0;   // lpf 후
const float speed_enc_lpf_alpha = 0.143;

// PID 계수
const float speed_Kp = 40.0; //75
const float speed_Ki = 20.0;
const float speed_Kd = 5.0;

const float speed_error_deadband = 0.01; // 2%

// PID 계산 결과 확인용
float speed_error = 0.0;
float speed_derivative = 0.0;
float speed_integral = 0.0;
float speed_pid_output = 0.0;
float previous_speed_error = 0.0;


// 기존 Kff 값은 가속용으로 사용합니다.
const float speed_Kff_accel = 60.0; // 예시: 기존에 사용하던 값
// 감속용 Kff 값은 가속용보다 훨씬 작게 설정합니다.
const float speed_Kff_decel = 10.0; // 예시: 가속 Kff의 1/3 정도에서 시작

float speed_ff_output = 0;
float speed_total_output = 0.0;

int speed_is_break = 0;

float speed_target_PWM = 0.0;
float speed_target_PWM_filtered = 0.0;
const float speed_pwm_lpf_alpha = 1.0;

const float speed_pwm_deadband = 15.0;
const float speed_break_mps = 0.12;

const float speed_pid_min_pwm_threshold =18.0;  // 정지 마찰 극복 기준

float speed_target_PWM_compensated = 0.0;


//----------- steering -----------//
// steering_control.ino 또는 설정 파일 상단

// [추가] 목표 조향각의 최대 변화 속도 (단위: 초당 각도, degrees per second)
// 예: 1초에 최대 180도까지 변할 수 있도록 설정
float max_steer_velocity = 150.0; 
float steer_target_angle = 0.0;

int steer_potentiometer_val = 0;
float steer_raw_angle;
float steer_filtered_angle = 0.0;
const float steer_angle_lpf_alpha = 0.8;

// PID 계수
const float steer_Kp = 13;
const float steer_Ki = 2.5;
const float steer_Kd = 0.2;

const float steer_error_deadband = 0.2;

// PID 계산 결과 확인용
float steer_error = 0.0;
float steer_integral = 0.0;
float steer_derivative = 0.0;
float steer_pid_output = 0.0;
float steer_previous_error = 0.0;

const float steer_pwm_deadband = 10.0;

float steer_target_PWM = 0.0;
float steer_target_PWM_filtered = 0.0;
const float steer_pwm_lpf_alpha = 0.8;

const float steer_pid_min_pwm_threshold = 30.0;

float steer_cur_compensation = 0.0;
float steer_target_PWM_compensated = 0.0;

//------------------- 시리얼 통신 관련 선언 ----------------------// 
uint8_t compute_crc(const uint8_t *data, size_t len) {
  uint8_t crc = 0;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
  }
  return crc;
}
#define RX_BUFFER_SIZE 64
uint8_t rx_buffer[RX_BUFFER_SIZE];  // 수신 버퍼
uint8_t rx_index = 0;

//---------------------- 인터럽트 핸들러 --------------------//
void ISR_EncoderA()
{
  bool a = digitalRead(encoderPin_1);
  bool b = digitalRead(encoderPin_2);

    // 회전 방향 판별
  if (a == b) {
    m_encoder_count++;  // 정방향
  } else {
    m_encoder_count--;  // 역방향
  }
}

//---------------------- 기타 함수 --------------------//
void speed_control(float target_speed_mps)
{
    if (target_speed_mps > ramped_speed_target) {
      ramped_speed_target += MAX_ACCELERATION * dt;
      if (ramped_speed_target > target_speed_mps) {
          ramped_speed_target = target_speed_mps;
      }
  } else if (target_speed_mps < ramped_speed_target) {
      ramped_speed_target -= MAX_DECELERATION * dt;
      if (ramped_speed_target < target_speed_mps) {
          ramped_speed_target = target_speed_mps;
      }
  }
  
  speed_target = constrain(ramped_speed_target, -1.3889, 1.3889);

  // 1. 엔코더 갱신
  noInterrupts();  // m_encoder_count 읽기 전에 인터럽트 방지
  speed_encoder_count = m_encoder_count;
  m_encoder_count = 0;  // 다음 측정 위해 초기화
  interrupts();

  // 1-2. 스파이크 완화 (엔코더 단)
  if (abs(speed_encoder_count - speed_encoder_count_prev) > 1500) {
    speed_encoder_count = 0.8 * speed_encoder_count_prev + 0.2 * speed_encoder_count;
  }
  speed_encoder_count_prev = speed_encoder_count;

  // 2. 현재 속도 계산 (m/s)
  speed_raw = (speed_encoder_count / pulses_per_rev) * wheel_circumference / dt;
  // speed_raw = (speed_encoder_count / pulses_per_rev) * wheel_circumference;

  // 3. speed_raw LPF 적용
  speed_filtered = speed_enc_lpf_alpha * speed_raw + (1.0 - speed_enc_lpf_alpha) * speed_filtered;

  // 4. 현재 속도와 목표 속도 차이
  speed_error = speed_target - speed_filtered;

  // 5. 데드밴드 (속도 오차가 작으면 무시)
  if (abs(speed_error) < speed_target * speed_error_deadband) {
    speed_error = 0;
  }

  // 6. PID 제어
  speed_integral += speed_error * dt;
  speed_integral = constrain(speed_integral, -100.0, 100.0);
  speed_derivative = (speed_error - previous_speed_error) / dt;
  speed_derivative = constrain(speed_derivative, -100.0, 100.0);
  speed_pid_output = speed_Kp * speed_error + speed_Ki * speed_integral + speed_Kd * speed_derivative;
  previous_speed_error = speed_error;

  // 6-2. Feedforward 제어 (속도 비례 항)
  // 목표 속도가 현재 속도보다 클 때 (가속 시)
  if (abs(speed_target) > abs(speed_filtered)) {
    speed_ff_output = speed_target * speed_Kff_accel;
  }
  // 목표 속도가 현재 속도보다 작을 때 (감속 시)
  else {
    speed_ff_output = speed_target * speed_Kff_decel;
  }

  // 6-3. PID + Feedforward 합산 출력
  speed_total_output = speed_pid_output + speed_ff_output;

  // 7. PWM 제한 및 로우패스 필터 적용
  speed_target_PWM = abs(speed_total_output);
  speed_target_PWM_filtered = speed_pwm_lpf_alpha * speed_target_PWM + (1.0 - speed_pwm_lpf_alpha) * speed_target_PWM_filtered;

  // 8. pwm이 0 근처 범위안으로 들어오면 모터드라이브 브레이크
  if ((speed_target_PWM_filtered < speed_pwm_deadband || abs(speed_filtered) < speed_break_mps) & (abs(speed_target) < 0.01)) { 
    digitalWrite(motorDirectionPin_Front, HIGH);
    digitalWrite(motorDirectionPin_Rear, HIGH);
    analogWrite(motorSpeedPin_Front, 0);
    analogWrite(motorSpeedPin_Rear, 0);
    speed_target_PWM_compensated = 0.0;
    speed_is_break = 1;
  } else {
    speed_is_break = 0;
    
    // 9. 최소 출력 보상 (Stiction Compensation)
    float calculated_pwm = speed_target_PWM_filtered;

    // 계산된 PWM이 0보다 크지만, 모터를 움직이기엔 부족한 '최소 구동 기준치'보다 작을 경우
    if (calculated_pwm > 0 && calculated_pwm < speed_pid_min_pwm_threshold) {
      // 출력을 '최소 구동 기준치' 값으로 강제로 설정하여 정지 마찰을 극복합니다.
      speed_target_PWM_compensated = speed_pid_min_pwm_threshold;
    } else {
      // 0이거나 기준치보다 클 경우에는 계산된 값을 그대로 사용합니다.
      speed_target_PWM_compensated = calculated_pwm;
    }
    
    speed_target_PWM_compensated = constrain(speed_target_PWM_compensated, 0, 255);

    // 10. 방향 및 PWM 출력
    if (speed_total_output > 0) { // Forward
      digitalWrite(motorDirectionPin_Front, LOW);
      digitalWrite(motorDirectionPin_Rear, LOW);
    } else {               // Backward
      digitalWrite(motorDirectionPin_Front, HIGH);
      digitalWrite(motorDirectionPin_Rear, HIGH);
    }
    analogWrite(motorSpeedPin_Front, (int)speed_target_PWM_compensated);
    analogWrite(motorSpeedPin_Rear, (int)speed_target_PWM_compensated);
  }
}

void steering_control(float steering_angle)
{
  // 1. 이번 루프에서 허용되는 최대 각도 변화량을 계산합니다.
  float max_angle_change_per_loop = max_steer_velocity * dt;

  // 2. 최종 목표(steering_angle)와 현재 목표(steer_target_angle)의 차이를 계산합니다.
  float angle_diff = steering_angle - steer_target_angle;

  // 3. 차이 값을 최대 변화량 이내로 제한(clamping)합니다.
  float clamped_diff = constrain(angle_diff, -max_angle_change_per_loop, max_angle_change_per_loop);

  // 4. 제한된 변화량을 현재 목표에 더하여, 새로운 목표 각도를 업데이트합니다.
  steer_target_angle += clamped_diff;
  
  // 1-1. 목표 각도의 범위를 -20 ~ +20도로 제한합니다. (이 로직은 그대로 유지)
  steer_target_angle = constrain(steer_target_angle, -20.0, 20.0);

  // 2. 현재 포텐쇼미터 값 읽기 100(우) ~ 1000(좌)값이 나옴..
  steer_potentiometer_val = analogRead(potentiometerPin_);
  if (steer_potentiometer_val >= 512) {
    // 좌회전 (0° ~ +20.96°) // (20.96°-0°)/(978-512) = 0.04508
    steer_raw_angle = (steer_potentiometer_val - 512) * 0.04313;
  } else {
    // 우회전 (0° ~ -22.33°) // (0°-(-22.33°)/(512-82) = 0.05193
    steer_raw_angle = (steer_potentiometer_val - 512) * 0.04313;
  }

  // 3. steer_raw_angle에 Low-pass filter 적용
  steer_filtered_angle = steer_angle_lpf_alpha * steer_raw_angle + (1.0 - steer_angle_lpf_alpha) * steer_filtered_angle;

  // 4. 오차 계산 (목표 - 현재)
  steer_error = steer_target_angle - steer_filtered_angle;

  // 5. 데드밴드 설정
  if (abs(steer_error) < steer_error_deadband) {
    steer_error = 0.0;
  }

  // 6. PID 계산
  steer_integral += steer_error * dt;
  steer_integral = constrain(steer_integral, -100.0, 100.0);
  steer_derivative = (steer_error - steer_previous_error) / dt;
  steer_derivative = constrain(steer_derivative, -100.0, 100.0);
  steer_pid_output = steer_Kp * steer_error + steer_Ki * steer_integral + steer_Kd * steer_derivative;  
  steer_previous_error = steer_error;

  // 7. PWM 변환, LPF, 제한
  steer_target_PWM = abs(steer_pid_output);
  steer_target_PWM_filtered = steer_pwm_lpf_alpha * steer_target_PWM + (1.0 - steer_pwm_lpf_alpha) * steer_target_PWM_filtered;

  // 8. pwm이 0 근처 범위안으로 들어오면 물리적 브레이크
  if (steer_target_PWM_filtered <= steer_pwm_deadband){
    digitalWrite(STEERING_DIR_PIN, HIGH);
    analogWrite(STEERING_PWM_PIN, 0);
    steer_target_PWM_compensated = 0.0;
    steer_target_PWM_filtered = 0.0;
    steer_cur_compensation = 0.0;
  } else {
    // 9. 최소 구동 보상 (정지 극복용 오프셋)
    steer_target_PWM_compensated = steer_target_PWM_filtered; // 우선 필터링된 값을 할당
    if (steer_target_PWM_compensated < steer_pid_min_pwm_threshold) {
      steer_target_PWM_compensated = steer_pid_min_pwm_threshold; // 최소값보다 작으면, 최소값으로 강제 설정
    }
    steer_target_PWM_compensated = constrain(steer_target_PWM_compensated, 0, 255);

    //10. 방향 및 출력
    if (steer_pid_output > 0) {
      digitalWrite(STEERING_DIR_PIN, LOW);
    } else {
      digitalWrite(STEERING_DIR_PIN, HIGH);
    }
    analogWrite(STEERING_PWM_PIN, (int)steer_target_PWM_compensated);
  }
}

void remoteController() {
  // 채널 값 읽기 (A3, A4, A5)
  m_thro = pulseIn(remote_THRO, HIGH, 25000);
  m_aile = pulseIn(remote_AILE, HIGH, 25000);

  if (m_aile > 500) {
    MODE = MANUAL;

    // --- 1. 요청대로, 스로틀과 조향 모두 유효 범위를 1000 ~ 1900으로 제한합니다. ---
    long clamped_thro = constrain(m_thro, 1000, 2000);
    long clamped_aile = constrain(m_aile, 1000, 2000);

    // --- 2. 새로운 1000~2000 범위를 사용하여 속도를 매핑합니다. ---
    // fromLow는 1000.0, fromHigh는 2000.0으로 변경되었습니다.
    m_remote_speed = (clamped_thro - 1000.0) * (1.3889 - (-1.3889)) / (2000.0 - 1000.0) + (-1.3889);
    
    // 스로틀 중앙 데드밴드
    if (-1.3889 * 0.15 < m_remote_speed && m_remote_speed < 1.3889 * 0.15)
      m_remote_speed = 0.0;

    // --- 3. 조향에도 동일한 1000~2000 범위를 사용하여 매핑합니다. ---
    m_remote_steering_angle = (clamped_aile - 1000.0) * (-20.0 - 20.0) / (2000.0 - 1000.0) + 20.0;
    
    // 조향 중앙 데드밴드
    if (-10.0 < m_remote_steering_angle && m_remote_steering_angle < 10.0)
      m_remote_steering_angle = 0.0;

      if (m_remote_speed > 0.9){ 
      m_remote_speed = 1.3889;
      }
      
    // 속도 LPF 로직
    m_remote_speed_filtered = remote_speed_lpf_alpha * m_remote_speed + (1.0 - remote_speed_lpf_alpha) * m_remote_speed_filtered;

  } else {
    MODE = AUTONOMOUS;
    m_remote_speed = 0;
    m_remote_speed_filtered = 0; 
    m_remote_steering_angle = 0;
  }
}

void updateMatrix(uint8_t mode, uint8_t serial_state) {
  if (mode == 0) {  // 수동 모드
    switch (serial_state) {
      case SERIAL_WAITING:
        matrix.loadFrame(manual_SerialWaiting);
        break;
      case SERIAL_CONNECTED:
        matrix.loadFrame(manual_SerialConnected);
        break;
      case SERIAL_ERROR:
        matrix.loadFrame(manual_SerialError);
        break;
    }
  } else if (mode == 1) {  // 자율주행 모드
    switch (serial_state) {
      case SERIAL_WAITING:
        matrix.loadFrame(automonous_SerialWaiting);
        break;
      case SERIAL_CONNECTED:
        matrix.loadFrame(automonous_SerialConnected);
        break;
      case SERIAL_ERROR:
        matrix.loadFrame(automonous_SerialError);
        break;
    }
  }
}

void handleSerialReceive() {

  if (Serial.available() == 0) {
  serial_state = SERIAL_WAITING;
  return;
  }

  // 시리얼 수신 버퍼 채우기
  while (Serial.available() > 0 && rx_index < RX_BUFFER_SIZE) {
    rx_buffer[rx_index++] = Serial.read();
  }

  // 최소 패킷 단위 존재 여부 확인
  while (rx_index >= 11) {
    // 버퍼 넘쳤는지 확인
    if (rx_index >= RX_BUFFER_SIZE) {
      rx_index = 0;
      serial_state = SERIAL_ERROR;
    }

    // 헤더 탐색
    int header_index = -1;
    for (int i = 0; i <= rx_index - 2; i++) {
      if (rx_buffer[i] == 0xAA && rx_buffer[i + 1] == 0x55) {
        header_index = i;
        break;
      }
    }

    if (header_index == -1) {
      rx_index = 0;
      serial_state = SERIAL_ERROR;
      break;
    }

    if (header_index > 0) {
      for (int i = 0; i < rx_index - header_index; i++) {
        rx_buffer[i] = rx_buffer[i + header_index];
      }
      rx_index -= header_index;
    }

    if (rx_index < 11) break;

    uint8_t crc_received = rx_buffer[10];
    uint8_t crc_calculated = compute_crc(&rx_buffer[2], 8);

    if (crc_received != crc_calculated) {
      for (int i = 0; i < rx_index - 1; i++) {
        rx_buffer[i] = rx_buffer[i + 1];
      }
      rx_index -= 1;
      serial_state = SERIAL_ERROR;
      continue;
    }

    memcpy(&m_auto_speed, &rx_buffer[2], 4);
    memcpy(&m_auto_steering_angle, &rx_buffer[6], 4);
    serial_state = SERIAL_CONNECTED;

    for (int i = 0; i < rx_index - 11; i++) {
      rx_buffer[i] = rx_buffer[i + 11];
    }
    rx_index -= 11;
  }
}

void sendSerialTelemetry() {
    uint8_t packet[31];
    uint8_t offset = 0;
  
    packet[offset++] = 0xAA;
    packet[offset++] = 0x55;
    memcpy(&packet[offset], &steer_filtered_angle, 4);         offset += 4;
    memcpy(&packet[offset], &steer_target_PWM_compensated, 4); offset += 4;    
    memcpy(&packet[offset], &speed_filtered, 4);               offset += 4;
    memcpy(&packet[offset], &speed_target_PWM_compensated, 4); offset += 4;
    memcpy(&packet[offset], &m_auto_speed, 4);                 offset += 4;
    memcpy(&packet[offset], &m_auto_steering_angle, 4);        offset += 4;
    memcpy(&packet[offset], &last_time, 4);                    offset += 4;
    packet[offset++]  = compute_crc(&packet[2], offset - 2);
  
    Serial.write(packet, offset);
  }

// 기존 printDebugInfo() 함수를 아래와 같이 수정하거나 새로 만들어 사용하세요.
void printDebugInfo() {
  // --- 기존 속도 데이터 (8개) ---
  Serial.print(speed_filtered, 4); Serial.print(",");
  Serial.print(speed_target, 4); Serial.print(",");
  Serial.print(speed_pid_output, 4); Serial.print(",");
  Serial.print(speed_ff_output, 4); Serial.print(",");
  Serial.print(speed_total_output, 4); Serial.print(",");
  Serial.print(speed_is_break); Serial.print(",");
  Serial.print(speed_target_PWM_filtered, 4); Serial.print(",");
  Serial.print(speed_target_PWM_compensated, 4); Serial.print(","); // 마지막에 쉼표 추가

  // --- 추가될 조향 데이터 (12개) ---
  Serial.print(steer_potentiometer_val); Serial.print(",");
  Serial.print(steer_raw_angle, 4); Serial.print(",");
  Serial.print(steer_filtered_angle, 4); Serial.print(",");
  Serial.print(steer_target_angle, 4); Serial.print(",");
  Serial.print(steer_error, 4); Serial.print(",");
  Serial.print(steer_integral, 4); Serial.print(",");
  Serial.print(steer_derivative, 4); Serial.print(",");
  Serial.print(steer_pid_output, 4); Serial.print(",");
  Serial.print(steer_target_PWM, 4); Serial.print(",");
  Serial.print(steer_target_PWM_filtered, 4); Serial.print(",");
  Serial.print(steer_cur_compensation, 4); Serial.print(",");
  Serial.println(steer_target_PWM_compensated, 4); // 마지막은 println
}


//---------------------- 하드웨어 초기화 --------------------//
void setup() {
  Serial.begin(115200);
  matrix.begin();
  delay(1000);


  const char* CSV_HEADER = "speed_filtered,speed_target,speed_pid_output,speed_ff_output,speed_total_output,speed_is_break,speed_target_PWM_filtered,speed_target_PWM_compensated";

  // 시리얼 포트가 준비될 때까지 잠시 대기
  while (!Serial);

  Serial.println(CSV_HEADER); // 파이썬에서 사용할 헤더 출력

  // 핀모드 설정
  pinMode(motorDirectionPin_Front, OUTPUT);
  pinMode(motorDirectionPin_Rear,  OUTPUT);
  pinMode(motorSpeedPin_Front,    OUTPUT);
  pinMode(motorSpeedPin_Rear,     OUTPUT);

  pinMode(STEERING_DIR_PIN, OUTPUT);
  pinMode(STEERING_PWM_PIN, OUTPUT);

  pinMode(encoderPin_1, INPUT_PULLUP);
  pinMode(encoderPin_2, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoderPin_1), ISR_EncoderA, CHANGE);

  pinMode(potentiometerPin_, INPUT);

  // pinMode(remote_GEAR, INPUT);
  pinMode(remote_AILE, INPUT);
  pinMode(remote_THRO, INPUT);

  // 초기 구동 설정
  digitalWrite(motorDirectionPin_Front, LOW);
  digitalWrite(motorDirectionPin_Rear,  LOW);
  digitalWrite(STEERING_DIR_PIN,  LOW);

  matrix.loadFrame(FRAME_HAPPY);    // 초기 프레임 출력

  delay(1000);
}

//---------------------- 메인 루프 --------------------//
void loop() {  
  if (millis() - last_time >= dt*1000) {  // 100ms 주기
    last_time = millis();

    // // ===== 1. PC ← Arduino: 31바이트 패킷 전송 =====
    // sendSerialTelemetry();  // 상태 전송

    // // ===== 2. PC → Arduino: 11바이트 패킷 수신 =====
    // handleSerialReceive();  // 수신 처리

    // ===== 3. 디버깅용 출력 =====
    printDebugInfo();

    // ===== 4. 조종기  =====
    remoteController(); // RC 리모컨 값 읽기

    // ===== 5. 제어 =====
    if(MODE == AUTONOMOUS) {  // AUTONOMOUS
      speed_control(0.5);
      steering_control(m_auto_steering_angle);
    } else {          // MANUAL
      speed_control(m_remote_speed_filtered);
      steering_control(m_remote_steering_angle);
    }

    // ===== 6. LED update =====
    updateMatrix(MODE, serial_state);
  }
}
