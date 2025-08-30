#include <Arduino.h>

const int A_OPEN_STEPS = 220; 

const int A_MOTOR_PINS[4] = {16, 4, 2, 15}; //쓰레기
const int B_MOTOR_PINS[4] = {19, 18, 5, 17}; //걸레면

constexpr float B_ROTATION_ANGLE = 36.0f;
constexpr int STEPS_PER_REVOLUTION = 4096;
constexpr float STEPS_PER_DEGREE = STEPS_PER_REVOLUTION / 360.0f;
constexpr uint32_t LID_OPEN_DURATION_MS = 33000;
const int A_MOTOR_US_START = 6000, A_MOTOR_US_END = 2500;
const int B_MOTOR_US_START = 5000, B_MOTOR_US_END = 2000;
const uint32_t TRIG_DEBOUNCE_MS = 300;

// 스텝 모드 정의
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
    StepMode _mode = HALF_STEP; // 현재 스텝 모드

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
        } else { // FULL_STEP_2PHASE
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

    void hold() { applyPhase(); }
    bool isRunning() { return _steps_done < _steps_total; }
    void release() { for (int i=0; i<4; ++i) digitalWrite(_pins[i], LOW); }
};

RampedStepper motorA(A_MOTOR_PINS, A_MOTOR_US_START, A_MOTOR_US_END);
RampedStepper motorB(B_MOTOR_PINS, B_MOTOR_US_START, B_MOTOR_US_END);
enum State { STATE_IDLE, STATE_OPENING_AND_ROTATING, STATE_WAITING_TO_CLOSE, STATE_CLOSING };
State currentState = STATE_IDLE;
uint32_t last_trig_ms = 0;
uint32_t close_timer_start_ms = 0;
int last_second_printed = -1;

// ===== 동작 함수 =====
void startTriggerAction() {
    if (millis() - last_trig_ms < TRIG_DEBOUNCE_MS) { return; }
    last_trig_ms = millis();
    
    // 열 때는 부드러운 하프 스텝
    motorA.setStepMode(HALF_STEP);
    motorA.move(A_OPEN_STEPS);
    
    // B 모터는 항상 하프 스텝
    const int b_steps = lroundf(B_ROTATION_ANGLE * STEPS_PER_DEGREE);
    motorB.move(b_steps);
    
    currentState = STATE_OPENING_AND_ROTATING;
    Serial.println("STATE -> OPENING_AND_ROTATING");
}

void handleSerial() {
    if (Serial.available()) {
        String line = Serial.readStringUntil('\n');
        line.trim();
        if (line.equalsIgnoreCase("TRIG") && currentState == STATE_IDLE) {
            startTriggerAction();
        }
    }
}

// ===== 상태 머신 =====
void updateStateMachine() {
    switch (currentState) {
        case STATE_IDLE: break;
        case STATE_OPENING_AND_ROTATING: {
            bool a_is_running = motorA.update();
            bool b_is_running = motorB.update();
            if (!a_is_running && !b_is_running) {
                currentState = STATE_WAITING_TO_CLOSE;
                close_timer_start_ms = millis();
                last_second_printed = -1;
                Serial.println("STATE -> WAITING_TO_CLOSE");
            }
            break;
        }
        case STATE_WAITING_TO_CLOSE: {
            motorA.hold();
            uint32_t elapsed_ms = millis() - close_timer_start_ms;
            int remaining_seconds = (LID_OPEN_DURATION_MS > elapsed_ms) ? (LID_OPEN_DURATION_MS - elapsed_ms + 999) / 1000 : 0;
            if (remaining_seconds != last_second_printed) {
                Serial.print(remaining_seconds); Serial.println("s remaining...");
                last_second_printed = remaining_seconds;
            }
            if (elapsed_ms >= LID_OPEN_DURATION_MS) {
                // 닫을 때는 강력한 풀 스텝으로 변경!
                motorA.setStepMode(FULL_STEP_2PHASE);
                // 풀스텝은 스텝수가 절반이므로 /2
                motorA.move(-(A_OPEN_STEPS)*2.5); 
                currentState = STATE_CLOSING;
                Serial.println("STATE -> CLOSING (Full-Step Mode)");
            }
            break;
        }
        case STATE_CLOSING:
            if (!motorA.update()) {
                motorA.release();
                currentState = STATE_IDLE;
                Serial.println("STATE -> IDLE (Sequence complete)");
            }
            break;
    }
}

// ===== 표준 함수 =====
void setup() {
    Serial.begin(115200);
    motorA.begin(); motorB.begin();
    Serial.println("Ready (Calibrated & Torque-Boosted). Send 'TRIG'.");
}

void loop() {
    handleSerial();
    updateStateMachine();
}
