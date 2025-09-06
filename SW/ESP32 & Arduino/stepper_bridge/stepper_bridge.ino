#include <Arduino.h>
#include <ESP32Servo.h>

// ===== 서보 설정 =====
const int SERVO_PIN = 26;             // 서보 제어 핀
const int SERVO_OPEN_ANGLE = 70;     // 열릴 때 각도
const int SERVO_CLOSE_ANGLE = 180;    // 닫힌 각도
const int SERVO_STEP_DELAY = 15;      // 서보 부드럽게 이동 딜레이 (ms)

// ===== 스텝모터 설정 =====
const int B_MOTOR_PINS[4] = {19, 18, 5, 17};
constexpr float B_ROTATION_ANGLE = 36.0f;     // 회전 각도
constexpr int STEPS_PER_REVOLUTION = 4096;    // 28BYJ-48 기준
constexpr float STEPS_PER_DEGREE = STEPS_PER_REVOLUTION / 360.0f;
const int B_MOTOR_US_START = 5000, B_MOTOR_US_END = 2000;

// ===== DC 모터 핀 =====
const int DC_EN  = 13;
const int DC_IN1 = 12;
const int DC_IN2 = 14;

// ===== DC 모터 설정 =====
const int DC_POWER_PERCENT   = 50;    // 속도 (0~100%)
const int DC_RUN_DURATION_MS = 1000;  // 동작 시간
const int DC_PWM_CHANNEL     = 6;     // PWM 채널 (0~15)
const int DC_PWM_FREQ        = 5000;  // PWM 주파수 5kHz
const int DC_PWM_RES         = 8;     // 해상도 8비트 (0~255)

// ===== 서보 객체 =====
Servo servoA;
int currentAngle = SERVO_CLOSE_ANGLE;
int targetAngle = SERVO_CLOSE_ANGLE;
unsigned long lastServoUpdate = 0;

// ===== 스텝 모터 제어 클래스 =====
enum StepMode { HALF_STEP, FULL_STEP_2PHASE };

class RampedStepper {
private:
    const int* _pins;
    int _phase = 0;
    long _steps_total = 0;
    long _steps_done = 0;
    int _direction = 0;
    int _us_start, _us_end;
    uint32_t _last_step_us = 0;
    StepMode _mode = HALF_STEP;

    const uint8_t HALF_STEP_SEQ[8][4] = {
        {1,0,0,0}, {1,1,0,0}, {0,1,0,0}, {0,1,1,0},
        {0,0,1,0}, {0,0,1,1}, {0,0,0,1}, {1,0,0,1}
    };
    const uint8_t FULL2_SEQ[4][4] = {
        {1,1,0,0}, {0,1,1,0}, {0,0,1,1}, {1,0,0,1}
    };

    void applyPhase() {
        if (_mode == HALF_STEP) {
            _phase &= 7;
            for (int i=0; i<4; ++i) digitalWrite(_pins[i], HALF_STEP_SEQ[_phase][i]);
        } else {
            _phase &= 3;
            for (int i=0; i<4; ++i) digitalWrite(_pins[i], FULL2_SEQ[_phase][i]);
        }
    }

    int calculateDelay() {
        if (_steps_total <= 1) return _us_end;
        float progress = (float)_steps_done / (float)(_steps_total - 1);
        float k;
        if (progress < 0.3f) k = progress / 0.3f;
        else if (progress > 0.7f) k = (1.0f - (progress - 0.7f) / 0.3f);
        else k = 1.0f;
        int us = (int)lroundf(_us_start - k * (_us_start - _us_end));
        return (us < _us_end) ? _us_end : us;
    }

public:
    RampedStepper(const int pins[4], int us_start, int us_end)
        : _pins(pins), _us_start(us_start), _us_end(us_end) {}

    void begin() { for (int i=0; i<4; ++i) pinMode(_pins[i], OUTPUT); }
    void setStepMode(StepMode mode) { _mode = mode; }
    void move(long steps) {
        _steps_total = abs(steps);
        _steps_done = 0;
        _direction = (steps > 0) ? 1 : -1;
    }

    bool update() {
        if (!isRunning()) return false;
        if (micros() - _last_step_us >= calculateDelay()) {
            _last_step_us = micros();
            int phase_max = (_mode == HALF_STEP) ? 7 : 3;
            _phase += _direction;
            if (_phase < 0) _phase = phase_max;
            if (_phase > phase_max) _phase = 0;
            applyPhase();
            _steps_done++;
        }
        return true;
    }

    void release() { for (int i=0; i<4; ++i) digitalWrite(_pins[i], LOW); }
    bool isRunning() { return _steps_done < _steps_total; }
};

// ===== 전역 객체 =====
RampedStepper motorB(B_MOTOR_PINS, B_MOTOR_US_START, B_MOTOR_US_END);

enum State { STATE_IDLE, STATE_RUNNING, STATE_WAITING };
State currentState = STATE_IDLE;
unsigned long stateStartTime = 0;

// ===== DC 모터 상태 =====
bool dcRunning = false;
unsigned long dcStartMs = 0;
int dcDurationMs = 0;

// ===== 논블로킹 서보 업데이트 =====
void setServoTarget(int angle) {
  targetAngle = angle;
}

void updateServo() {
  if (currentAngle == targetAngle) return;
  if (millis() - lastServoUpdate >= SERVO_STEP_DELAY) {
    lastServoUpdate = millis();
    if (currentAngle < targetAngle) currentAngle++;
    else if (currentAngle > targetAngle) currentAngle--;
    servoA.write(currentAngle);
  }
}

// ===== TRIG 동작 (서보 + 스텝모터 동시에) =====
void startTriggerAction() {
  if (currentState != STATE_IDLE) return;

  Serial.println("STATE -> RUNNING (Servo + Stepper)");

  // 서보 목표 각도 설정
  setServoTarget(SERVO_OPEN_ANGLE);

  // 스텝모터 회전 시작
  const int b_steps = lroundf(B_ROTATION_ANGLE * STEPS_PER_DEGREE);
  motorB.move(b_steps);

  stateStartTime = millis();
  currentState = STATE_RUNNING;
}

// ===== DC 모터 제어 =====
void startDCMotor(int powerPercent, int durationMs) {
  int pwmVal = map(powerPercent, 0, 100, 0, 255);
  digitalWrite(DC_IN1, HIGH);
  digitalWrite(DC_IN2, LOW);
  ledcWrite(DC_PWM_CHANNEL, pwmVal);

  dcStartMs = millis();
  dcDurationMs = durationMs;
  dcRunning = true;
  Serial.println("DC motor started");
}

void updateDCMotor() {
  if (dcRunning && (millis() - dcStartMs >= dcDurationMs)) {
    ledcWrite(DC_PWM_CHANNEL, 0); // 정지
    dcRunning = false;
    Serial.println("DC motor stopped");
  }
}

// ===== 시리얼 명령 처리 =====
// ===== 시리얼 명령 처리 =====
void handleSerial() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();

    if (line.equalsIgnoreCase("TRIG") && currentState == STATE_IDLE) {
      startTriggerAction();
    }
    else if (line.equalsIgnoreCase("WATER")) {
      startDCMotor(DC_POWER_PERCENT, DC_RUN_DURATION_MS);
    }
    else if (line.equalsIgnoreCase("DONE") && currentState == STATE_RUNNING) {
      Serial.println("STATE -> RETURNING (Servo on DONE)");
      setServoTarget(SERVO_CLOSE_ANGLE);   // 닫기
      currentState = STATE_WAITING;
    }
  }
}

// ===== 상태 머신 =====
void updateStateMachine() {
  switch (currentState) {
    case STATE_IDLE:
      break;

    case STATE_RUNNING: {
      motorB.update();
      updateServo();
      break;
    }

    case STATE_WAITING: {
      motorB.update();
      updateServo();
      if (currentAngle == SERVO_CLOSE_ANGLE && !motorB.isRunning()) {
        motorB.release();
        currentState = STATE_IDLE;
        Serial.println("STATE -> IDLE (Complete)");
      }
      break;
    }
  }
}

// ===== 표준 함수 =====
void setup() {
  Serial.begin(115200);

  // 서보 초기화
  servoA.attach(SERVO_PIN, 1000, 2000);
  servoA.write(SERVO_CLOSE_ANGLE);
  currentAngle = SERVO_CLOSE_ANGLE;

  // 스텝모터 초기화
  motorB.begin();

  // DC모터 초기화
  pinMode(DC_IN1, OUTPUT);
  pinMode(DC_IN2, OUTPUT);
  ledcAttachPin(DC_EN, DC_PWM_CHANNEL);
  ledcSetup(DC_PWM_CHANNEL, DC_PWM_FREQ, DC_PWM_RES);
  ledcWrite(DC_PWM_CHANNEL, 0);

  Serial.println("Ready. Send 'TRIG' or 'WATER'.");
}

void loop() {
  handleSerial();
  updateStateMachine();
  updateDCMotor();
}
