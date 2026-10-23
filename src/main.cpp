#include <Arduino.h>
#include <AccelStepper.h>
#include <Servo.h>
#include <Wire.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <util/atomic.h>

// =====================================================
// PIN DEFINITIONS
// =====================================================
#define Z_DIR_PIN 9
#define Z_STEP_PIN 10
#define Z_EN_PIN 8
#define J1_DIR_PIN 6
#define J1_STEP_PIN 7
#define J1_EN_PIN 5
#define J2_DIR_PIN 12
#define J2_STEP_PIN 13
#define J2_EN_PIN 11

#define YAW_SERVO_PIN 46
#define PITCH_SERVO_PIN 48
#define GRIP_SERVO_PIN 44

#define PITCH_SWITCH A5
#define Z_MIN_SWITCH A2
#define J1_MIN_SWITCH A3
#define J2_MIN_SWITCH A4

#define DISTANCE_SENSOR_PIN A1

// =====================================================
// TIMING
// =====================================================
const unsigned long SWITCH_SCAN_US = 2000;  // switch sampling period
const unsigned long SLOW_PERIOD_MS = 20;    // 50 Hz: servos, IMU, distance
const unsigned long POS_REPORT_MS  = 100;   // 10 Hz: POS line

unsigned long lastSwitchScanUs = 0;
unsigned long lastSlowLoop     = 0;
unsigned long lastPosReport    = 0;

// =====================================================
// DISTANCE SENSOR
// =====================================================
// Moving average over DIST_SAMPLES readings (6 s at 50 Hz). Samples are
// stored as integer centi-cm with a running sum, so each report is O(1)
// and the sum never drifts the way a float accumulator would.
const uint16_t DIST_SAMPLES = 300;
uint16_t distBuf[DIST_SAMPLES];
uint32_t distSum = 0;
uint16_t distIdx = 0;
float calculateDistanceCM(int rawAdc);

// =====================================================
// PITCH (continuous-rotation servo + IMU closed loop)
// =====================================================
// PITCH_SET <deg> drives the servo until current_pitch_deg is within
// PITCH_DEADBAND_DEG of <deg>. NaN means "no active setpoint".
const int   PITCH_NEUTRAL      = 90;     // servo value that means "stop"
const int   PITCH_MANUAL_SPEED = 30;     // PITCH_UP / PITCH_DOWN -> 120 / 60
const int   PITCH_SPEED        = 12;     // closed-loop drive speed
const float PITCH_DEADBAND_DEG = 2.0f;   // arrival tolerance (IMU degrees)
const int   PITCH_SERVO_SIGN   = -1;     // flip if the servo drives the wrong way

float pitch_target_deg = NAN;
int   pitchDirection   = 0;              // 0=stop, 1=up (servo > 90), -1=down

// =====================================================
// MMA8452Q ACCELEROMETER (direct register access)
// =====================================================
// Chip found at I2C 0x1C, WHO_AM_I = 0x2A (MMA8452Q).
// The Adafruit_MMA8451 library refuses 0x2A, so we talk to it directly.
#define MMA8452_ADDR              0x1C

#define MMA8452_REG_OUT_X_MSB     0x01
#define MMA8452_REG_WHO_AM_I      0x0D
#define MMA8452_REG_XYZ_DATA_CFG  0x0E
#define MMA8452_REG_CTRL_REG1     0x2A

// 400 kHz cuts each IMU read from ~0.9 ms to ~0.25 ms of blocked stepping.
// Drop to 100000 if the IMU cable is long and reads start failing.
const uint32_t I2C_CLOCK_HZ    = 400000;
const uint32_t I2C_TIMEOUT_US  = 3000;

bool  mma_ok = false;
float pitch_zero_offset = 0.0f;   // set via PITCH_ZERO
float current_pitch_deg = 0.0f;   // reported in POS line

// =====================================================
// PITCH FILTERING (median-of-3 + EMA)
// =====================================================
// Small EMA coefficient = smoother but slower.
//   0.40 -> ~50% noise reduction, ~30 ms lag   (very snappy)
//   0.25 -> ~62% noise reduction, ~60 ms lag   (balanced)
//   0.15 -> ~72% noise reduction, ~110 ms lag  (recommended start)
//   0.10 -> ~77% noise reduction, ~180 ms lag  (smooth, mild lag)
//   0.05 -> ~84% noise reduction, ~380 ms lag  (very smooth, sluggish)
const float PITCH_EMA_ALPHA = 0.15f;

float pitch_filtered_raw = 0.0f;         // filtered pitch before zero offset
bool  pitch_filter_initialized = false;

// =====================================================
// Z AXIS: HARDWARE-TIMER STEP GENERATOR
// =====================================================
// AccelStepper makes pulses in software from loop(), which caps a 16 MHz
// AVR at roughly 4-5 k steps/s and makes the pulse train jitter whenever
// loop() is busy. Z instead gets its pulses from a Timer3 interrupt, so
// its rate doesn't depend on loop() and can go much higher.
//
// The ISR only emits steps and stops exactly on the target. The trapezoidal
// speed ramp is recalculated every Z_RAMP_US by run(), called from loop().
//
// Timer3 runs in CTC mode with TOP = ICR3 and uses the input-capture vector,
// because the Servo library claims TIMER3_COMPA_vect (with 3 servos it only
// actually uses Timer5, so Timer3 is free).

// SPEED <n> sets J1/J2 to n and Z to n * Z_SPEED_SCALE; SPEED_Z <n> sets Z alone.
const float Z_SPEED_SCALE     = 2.0f;
const float Z_ACCEL           = 10000.0f;  // steps/s^2
const float Z_MAX_SPEED_LIMIT = 20000.0f;  // hard ceiling, steps/s
const float Z_MIN_SPEED       = 100.0f;    // start / final creep speed, steps/s
const unsigned long Z_RAMP_US = 1000;      // ramp update period

static volatile uint8_t *zStepPort;
static uint8_t          zStepMask;
static volatile long    zPos = 0;
static volatile long    zTarget = 0;
static volatile int8_t  zDir = 1;
static volatile bool    zRunning = false;  // ISR emits steps while true
static volatile bool    zPinHigh = false;

// Runs at twice the step rate: one call raises STEP, the next lowers it,
// so the pulse is half a period wide (long enough for any driver).
ISR(TIMER3_CAPT_vect) {
    if (zPinHigh) {
        *zStepPort &= ~zStepMask;
        zPinHigh = false;
        if (!zRunning) TIMSK3 &= ~_BV(ICIE3);
        return;
    }
    if (!zRunning) {
        TIMSK3 &= ~_BV(ICIE3);
        return;
    }
    *zStepPort |= zStepMask;
    zPinHigh = true;
    zPos += zDir;
    if (zPos == zTarget) zRunning = false;
}

// Mirrors the subset of the AccelStepper API this firmware uses.
class ZStepper {
public:
    ZStepper(uint8_t stepPin, uint8_t dirPin) : _stepPin(stepPin), _dirPin(dirPin) {}

    void begin() {
        pinMode(_stepPin, OUTPUT);
        pinMode(_dirPin, OUTPUT);
        digitalWrite(_stepPin, LOW);
        zStepPort = portOutputRegister(digitalPinToPort(_stepPin));
        zStepMask = digitalPinToBitMask(_stepPin);

        TIMSK3 = 0;
        TCCR3A = 0;
        TCCR3B = _BV(WGM33) | _BV(WGM32) | _BV(CS31);  // CTC, TOP=ICR3, clk/8 = 2 MHz
        ICR3   = 0xFFFF;
    }

    void setMaxSpeed(float s)     { _maxSpeed = constrain(s, Z_MIN_SPEED, Z_MAX_SPEED_LIMIT); }
    float maxSpeed() const        { return _maxSpeed; }
    void setAcceleration(float a) { if (a > 0) _accel = a; }

    void moveTo(long target) {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { zTarget = target; }
    }

    long currentPosition() {
        long p;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { p = zPos; }
        return p;
    }

    long distanceToGo() {
        long d;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { d = zTarget - zPos; }
        return d;
    }

    bool isRunning() { return _speed > 0 || distanceToGo() != 0; }

    // Like AccelStepper: sets the position and halts instantly.
    void setCurrentPosition(long p) {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            zRunning = false;
            zPos = p;
            zTarget = p;
        }
        _speed = 0;
    }

    // Decelerate to a stop as quickly as the acceleration allows.
    void stop() {
        if (_speed <= 0) {
            moveTo(currentPosition());
            return;
        }
        long stepsToStop = (long)(_speed * _speed / (2.0f * _accel)) + 1;
        moveTo(currentPosition() + zDir * stepsToStop);
    }

    void run() {
        unsigned long now = micros();
        unsigned long elapsed = now - _lastUpdate;
        if (elapsed < Z_RAMP_US) return;
        _lastUpdate = now;
        // Clamp so a long blocking section doesn't cause a huge speed jump.
        float dt = (elapsed > 5000UL ? 5000UL : elapsed) * 1e-6f;

        bool running;
        long dtg;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            running = zRunning;
            dtg = zTarget - zPos;
        }

        if (!running) {
            _speed = 0;
            if (dtg == 0) return;

            // Start a new move from rest.
            zDir = (dtg > 0) ? 1 : -1;
            digitalWrite(_dirPin, zDir > 0 ? HIGH : LOW);
            _speed = Z_MIN_SPEED;
            setRate(_speed);
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
                TCNT3 = 0;
                TIFR3 = _BV(ICF3);
                zRunning = true;
                TIMSK3 |= _BV(ICIE3);
            }
            return;
        }

        bool toward = (dtg > 0 && zDir > 0) || (dtg < 0 && zDir < 0);
        float stopDist = _speed * _speed / (2.0f * _accel);

        if (!toward || labs(dtg) <= stopDist) {
            _speed -= _accel * dt;
        } else if (_speed < _maxSpeed) {
            _speed = fminf(_speed + _accel * dt, _maxSpeed);
        } else {
            _speed = fmaxf(_speed - _accel * dt, _maxSpeed);
        }

        if (_speed < Z_MIN_SPEED) {
            if (!toward) {
                // Target is now behind us: stop; the next run() restarts
                // in the new direction.
                ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { zRunning = false; }
                _speed = 0;
                return;
            }
            _speed = Z_MIN_SPEED;   // creep the last steps; ISR stops on target
        }
        setRate(_speed);
    }

private:
    // ISR fires at 2x the step rate from a 2 MHz clock: TOP = 1e6 / rate - 1.
    static void setRate(float stepsPerSec) {
        uint16_t top = (uint16_t)(1000000.0f / stepsPerSec) - 1;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            ICR3 = top;
            if (TCNT3 > top) TCNT3 = 0;   // ICR3 isn't double-buffered
        }
    }

    uint8_t _stepPin, _dirPin;
    float _maxSpeed = 1000.0f;
    float _accel    = 1000.0f;
    float _speed    = 0.0f;               // current speed magnitude, steps/s
    unsigned long _lastUpdate = 0;
};

// =====================================================
// MOTORS
// =====================================================
ZStepper     stepperZ(Z_STEP_PIN, Z_DIR_PIN);
AccelStepper stepperJ1(AccelStepper::DRIVER, J1_STEP_PIN, J1_DIR_PIN);
AccelStepper stepperJ2(AccelStepper::DRIVER, J2_STEP_PIN, J2_DIR_PIN);

Servo yawServo;
Servo gripServo;
Servo pitchServo;

#define J1_HOME_ANGLE_DEG  -91
#define J2_HOME_ANGLE_DEG  175

#define HOMING_BACKOFF_J2 4000
#define HOMING_BACKOFF 500

const float STEPS_PER_DEG_J1 = 177.78;
const float STEPS_PER_DEG_J2 = 133.33;

// Direction (+1 / -1) J1 moves to reach its homing switch. Approach, backoff
// and the J1 software limit all follow this.
const int8_t J1_HOMING_DIR = 1;

// Homing gives up if the switch isn't found within this much travel
// (disconnected / broken switch) instead of spinning forever.
const float HOMING_MAX_TRAVEL_DEG = 360.0;

// Z homing: fast approach, back off, slow second touch. The slow touch
// position becomes Z_HOME_STEPS, then Z parks Z_HOMING_PARK steps clear.
const int8_t Z_HOMING_DIR         = 1;        // Z+ moves toward the switch
const float  Z_HOMING_FAST_SPEED  = 1500.0;   // steps/s
const float  Z_HOMING_SLOW_SPEED  = 300.0;    // steps/s
const long   Z_HOMING_BACKOFF     = 1500;      // steps between the two touches
const long   Z_HOMING_PARK        = 1500;      // steps clear of the switch at the end
const long   Z_HOMING_MAX_STEPS   = 150000;   // give up after this much travel
const long   Z_HOME_STEPS         = 112000;        // position assigned at the switch

// Direction (+1 / -1) that drives each axis INTO its min switch; 0 disables
// the software limit. All three match their homing direction.
const int8_t Z_MIN_DIR  = Z_HOMING_DIR;
const int8_t J1_MIN_DIR = J1_HOMING_DIR;
const int8_t J2_MIN_DIR = 1;

// A limit switch must read pressed this long before motion is cut. Much
// shorter than DEBOUNCE_MS so a fast jog doesn't overrun the switch.
const unsigned long LIMIT_CONFIRM_MS = 4;

const int GRIP_OPEN_ANGLE  = 130;
const int GRIP_CLOSE_ANGLE = 40;

float jogSpeed = 6000.0;

// Servo targets; servos are only re-written when these change.
int targetYawAngle   = 90;
int targetPitchAngle = PITCH_NEUTRAL;
int targetGripAngle  = 90;
int writtenYaw = -1, writtenPitch = -1, writtenGrip = -1;

// =====================================================
// SWITCH STATE (debounced, active-low: stable == LOW means pressed)
// =====================================================
struct SwitchState {
    int stable = HIGH;
    int lastRaw = HIGH;
    unsigned long lastChange = 0;
};

const unsigned long DEBOUNCE_MS = 25;

SwitchState swPitch, swZ, swJ1, swJ2;
int swStablePrev[4] = {HIGH, HIGH, HIGH, HIGH};

bool emergencyStop = false;

// =====================================================
// SERIAL
// =====================================================
const uint8_t RX_BUF_SIZE = 64;
char    rxBuf[RX_BUF_SIZE];
uint8_t rxLen = 0;
bool    rxOverflow = false;

void processCommand(char *msg);
void homeAllAxes();

// =====================================================
// SWITCHES
// =====================================================
// Active-low convention: LOW = pressed.
int readSwitch(int pin) {
    return !digitalRead(pin);
}

void updateSwitch(SwitchState &s, int pin) {
    int reading = readSwitch(pin);
    unsigned long now = millis();

    if (reading != s.lastRaw) {
        s.lastRaw = reading;
        s.lastChange = now;
    }
    if ((now - s.lastChange) > DEBOUNCE_MS) {
        s.stable = reading;
    }
}

void initSwitch(SwitchState &s, int pin) {
    s.stable = s.lastRaw = readSwitch(pin);
    s.lastChange = millis();
}

bool limitTriggered(const SwitchState &s) {
    return s.lastRaw == LOW && (millis() - s.lastChange) >= LIMIT_CONFIRM_MS;
}

// Hard-stop an axis that is heading into its pressed min switch. Moving away
// from the switch is still allowed. setCurrentPosition() halts instantly and
// keeps the step count; decelerating would overrun the switch.
template <class Stepper>
void enforceLimit(Stepper &s, const SwitchState &sw, int8_t dirToSwitch) {
    if (dirToSwitch == 0 || !limitTriggered(sw)) return;
    long dtg = s.distanceToGo();
    if (dtg != 0 && ((dtg > 0) == (dirToSwitch > 0))) {
        s.setCurrentPosition(s.currentPosition());
    }
}

// =====================================================
// SERVOS
// =====================================================
void writeServos() {
    if (targetYawAngle != writtenYaw)     { yawServo.write(targetYawAngle);     writtenYaw = targetYawAngle; }
    if (targetPitchAngle != writtenPitch) { pitchServo.write(targetPitchAngle); writtenPitch = targetPitchAngle; }
    if (targetGripAngle != writtenGrip)   { gripServo.write(targetGripAngle);   writtenGrip = targetGripAngle; }
}

void setPitchDrive(int dir, int speed) {
    pitchDirection   = dir;
    targetPitchAngle = PITCH_NEUTRAL + dir * speed;
}

void stopPitch() {
    pitch_target_deg = NAN;
    setPitchDrive(0, 0);
}

// Pitch limit switch only guards the "up" direction.
void enforcePitchLimit() {
    if (pitchDirection == 1 && swPitch.stable == LOW) {
        stopPitch();
        writeServos();
    }
}

// =====================================================
// MMA8452Q LOW-LEVEL HELPERS
// =====================================================
void mma_write_reg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(MMA8452_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

uint8_t mma_read_reg(uint8_t reg) {
    Wire.beginTransmission(MMA8452_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return 0;
    if (Wire.requestFrom((uint8_t)MMA8452_ADDR, (uint8_t)1) != 1) return 0;
    return Wire.read();
}

// Put chip in standby -> configure -> active
bool mma_init() {
    uint8_t id = mma_read_reg(MMA8452_REG_WHO_AM_I);
    if (id != 0x1A && id != 0x2A && id != 0x3A) {
        Serial.print(F("MMA8452 bad WHO_AM_I: 0x"));
        Serial.println(id, HEX);
        return false;
    }

    mma_write_reg(MMA8452_REG_CTRL_REG1, 0x00);     // standby
    delay(10);
    mma_write_reg(MMA8452_REG_XYZ_DATA_CFG, 0x00);  // +/-2g
    delay(10);
    // CTRL_REG1: DR = 011 (100 Hz), ACTIVE = 1 -> 0x19
    mma_write_reg(MMA8452_REG_CTRL_REG1, 0x19);
    delay(10);
    return true;
}

// Read XYZ accelerations in g. Returns false on I2C failure.
bool mma_read_accel(float &ax, float &ay, float &az) {
    Wire.beginTransmission(MMA8452_ADDR);
    Wire.write(MMA8452_REG_OUT_X_MSB);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((uint8_t)MMA8452_ADDR, (uint8_t)6) != 6) return false;

    // 12-bit data, left-justified in 16-bit registers
    int16_t x = ((int16_t)Wire.read() << 8) | Wire.read();
    int16_t y = ((int16_t)Wire.read() << 8) | Wire.read();
    int16_t z = ((int16_t)Wire.read() << 8) | Wire.read();

    // At +/-2g, 1024 counts per g after the >>4
    const float scale = 1.0f / 1024.0f;
    ax = (x >> 4) * scale;
    ay = (y >> 4) * scale;
    az = (z >> 4) * scale;
    return true;
}

float pitch_from_accel(float ay, float az) {
    return atan2f(az, ay) * (float)RAD_TO_DEG;
}

// =====================================================
// PITCH FILTER
// =====================================================
// 3-sample median (kills spikes) followed by an EMA seeded with the first
// sample. On an I2C error the previous filtered value is kept.
void update_pitch_filter() {
    float ax, ay, az;
    if (!mma_read_accel(ax, ay, az)) return;
    float raw = pitch_from_accel(ay, az);

    static float m_a = 0.0f, m_b = 0.0f, m_c = 0.0f;
    static uint8_t m_count = 0;

    m_c = m_b;
    m_b = m_a;
    m_a = raw;
    if (m_count < 3) m_count++;

    float med = (m_count < 3)
        ? raw
        : fmaxf(fminf(m_a, m_b), fminf(fmaxf(m_a, m_b), m_c));

    if (!pitch_filter_initialized) {
        pitch_filtered_raw = med;
        pitch_filter_initialized = true;
    } else {
        pitch_filtered_raw = PITCH_EMA_ALPHA * med
                           + (1.0f - PITCH_EMA_ALPHA) * pitch_filtered_raw;
    }
}

// =====================================================
// DISTANCE
// =====================================================
uint16_t readDistanceCenti() {
    return (uint16_t)(calculateDistanceCM(analogRead(DISTANCE_SENSOR_PIN)) * 100.0f + 0.5f);
}

// Seed the whole window with one reading so the average is valid at boot
// instead of ramping up from zero.
void distanceInit() {
    uint16_t v = readDistanceCenti();
    for (uint16_t i = 0; i < DIST_SAMPLES; i++) distBuf[i] = v;
    distSum = (uint32_t)v * DIST_SAMPLES;
}

void distanceSample() {
    uint16_t v = readDistanceCenti();
    distSum -= distBuf[distIdx];
    distSum += v;
    distBuf[distIdx] = v;
    if (++distIdx >= DIST_SAMPLES) distIdx = 0;
}

float distanceAverageCM() {
    return (float)distSum / (DIST_SAMPLES * 100.0f);
}

// =====================================================
// STEPPERS
// =====================================================
inline void runSteppers() {
    stepperZ.run();
    stepperJ1.run();
    stepperJ2.run();
}

void haltAllSteppers() {
    stepperZ.setCurrentPosition(stepperZ.currentPosition());
    stepperJ1.setCurrentPosition(stepperJ1.currentPosition());
    stepperJ2.setCurrentPosition(stepperJ2.currentPosition());
}

// Jog far in one direction; JOG_STOP decelerates it.
bool jogAxis(char axis, long sign) {
    const long far = sign * 1000000L;
    switch (axis) {
        case 'Z': stepperZ.moveTo(stepperZ.currentPosition() + far);   return true;
        case 'A': stepperJ1.moveTo(stepperJ1.currentPosition() + far); return true;
        case 'B': stepperJ2.moveTo(stepperJ2.currentPosition() + far); return true;
    }
    return false;
}

bool stopAxis(char axis) {
    switch (axis) {
        case 'Z': stepperZ.stop();  return true;
        case 'A': stepperJ1.stop(); return true;
        case 'B': stepperJ2.stop(); return true;
    }
    return false;
}

// =====================================================
// SETUP
// =====================================================
void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println(F("BOOT: serial up"));

    pinMode(PITCH_SWITCH, INPUT_PULLUP);
    pinMode(Z_MIN_SWITCH, INPUT_PULLUP);
    pinMode(J1_MIN_SWITCH, INPUT_PULLUP);
    pinMode(J2_MIN_SWITCH, INPUT_PULLUP);
    // NOTE: the pull-up loads the distance sensor output. The calibration
    // table below was taken with it enabled, so it is left as-is.
    pinMode(DISTANCE_SENSOR_PIN, INPUT_PULLUP);
    Serial.println(F("BOOT: pins configured"));

    initSwitch(swPitch, PITCH_SWITCH);
    initSwitch(swZ,     Z_MIN_SWITCH);
    initSwitch(swJ1,    J1_MIN_SWITCH);
    initSwitch(swJ2,    J2_MIN_SWITCH);
    swStablePrev[0] = swPitch.stable;
    swStablePrev[1] = swZ.stable;
    swStablePrev[2] = swJ1.stable;
    swStablePrev[3] = swJ2.stable;
    Serial.println(F("BOOT: switches initialised"));

    yawServo.attach(YAW_SERVO_PIN);
    gripServo.attach(GRIP_SERVO_PIN);
    pitchServo.attach(PITCH_SERVO_PIN);
    writeServos();
    Serial.println(F("BOOT: servos attached"));

    pinMode(Z_EN_PIN, OUTPUT);
    digitalWrite(Z_EN_PIN, LOW);
    pinMode(J1_EN_PIN, OUTPUT);
    digitalWrite(J1_EN_PIN, LOW);
    pinMode(J2_EN_PIN, OUTPUT);
    digitalWrite(J2_EN_PIN, LOW);

    stepperZ.begin();
    stepperZ.setMaxSpeed(jogSpeed * Z_SPEED_SCALE);
    stepperZ.setAcceleration(Z_ACCEL);
    stepperJ1.setMaxSpeed(jogSpeed);
    stepperJ1.setAcceleration(5000);
    stepperJ2.setMaxSpeed(jogSpeed);
    stepperJ2.setAcceleration(5000);
    stepperJ2.setPinsInverted(true, false, false);   // reverse physical DIR
    Serial.println(F("BOOT: steppers configured"));

    Serial.println(F("BOOT: calling Wire.begin()"));
    Wire.begin();
    Wire.setClock(I2C_CLOCK_HZ);
    // Without this a glitch on the bus (motor noise) can hang the AVR
    // Wire library forever, freezing the whole arm.
    Wire.setWireTimeout(I2C_TIMEOUT_US, true);
    Serial.println(F("BOOT: Wire ready"));

    Serial.println(F("BOOT: initialising MMA8452Q"));
    mma_ok = mma_init();
    Serial.println(mma_ok ? F("MMA8452_INIT_OK") : F("MMA8452_INIT_FAIL"));

    distanceInit();

    Serial.println(F("SYSTEM_READY"));
}

// =====================================================
// SERIAL INPUT
// =====================================================
// Collects bytes into rxBuf without blocking. Returns true once a complete,
// non-empty line is ready; any remaining bytes are left for the next call
// so one loop() iteration never handles more than one command.
bool readSerialLine() {
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\r') continue;
        if (c == '\n') {
            bool ready = rxLen > 0 && !rxOverflow;
            if (rxOverflow) Serial.println(F("ERR"));
            rxBuf[rxLen] = '\0';
            rxLen = 0;
            rxOverflow = false;
            if (ready) return true;
            continue;
        }
        if (rxLen < RX_BUF_SIZE - 1) rxBuf[rxLen++] = c;
        else rxOverflow = true;
    }
    return false;
}

// Used inside blocking routines (homing): only ESTOP is honoured, any other
// input is discarded. ESTOP is recognised as soon as its last byte arrives,
// with or without a trailing newline.
bool checkForEmergencyStop() {
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            rxLen = 0;
            rxOverflow = false;
            continue;
        }
        if (rxLen < RX_BUF_SIZE - 1) rxBuf[rxLen++] = c;
        if (rxLen >= 5 && memcmp(rxBuf + rxLen - 5, "ESTOP", 5) == 0) {
            rxLen = 0;
            emergencyStop = true;
        }
    }
    return emergencyStop;
}

// =====================================================
// REPORTS
// =====================================================
void sendSwitchUpdate() {
    Serial.print(F("SW:"));
    Serial.print(swPitch.stable == LOW);
    Serial.print(',');
    Serial.print(swZ.stable == LOW);
    Serial.print(',');
    Serial.print(swJ1.stable == LOW);
    Serial.print(',');
    Serial.println(swJ2.stable == LOW);
}

void reportSwitchChanges() {
    int current[4] = {swPitch.stable, swZ.stable, swJ1.stable, swJ2.stable};
    bool changed = false;
    for (int i = 0; i < 4; i++) {
        if (current[i] != swStablePrev[i]) {
            swStablePrev[i] = current[i];
            changed = true;
        }
    }
    if (changed) sendSwitchUpdate();
}

static char *appendLong(char *p, long v) {
    ltoa(v, p, 10);
    return p + strlen(p);
}

static char *appendFloat(char *p, float v) {
    dtostrf(v, 0, 2, p);
    return p + strlen(p);
}

// POS:<z>,<j1>,<j2>,<yaw>,<distance>,<pitch|nan>
// Built in RAM and written in one go, and only when the TX buffer has room,
// so a slow host never stalls the stepper loop. Returns false if skipped.
bool sendPositionUpdate() {
    char line[80];
    char *p = line;

    memcpy(p, "POS:", 4); p += 4;
    p = appendLong(p, stepperZ.currentPosition());  *p++ = ',';
    p = appendLong(p, stepperJ1.currentPosition()); *p++ = ',';
    p = appendLong(p, stepperJ2.currentPosition()); *p++ = ',';
    p = appendLong(p, writtenYaw);                  *p++ = ',';
    p = appendFloat(p, distanceAverageCM());        *p++ = ',';
    if (mma_ok) p = appendFloat(p, current_pitch_deg);
    else { memcpy(p, "nan", 3); p += 3; }
    *p++ = '\r';
    *p++ = '\n';

    size_t len = p - line;
    if ((size_t)Serial.availableForWrite() < len) return false;
    Serial.write((const uint8_t *)line, len);
    return true;
}

// =====================================================
// PITCH CLOSED LOOP (PITCH_SET)
// =====================================================
void updatePitchControl() {
    if (isnan(pitch_target_deg) || !mma_ok) return;

    float err = pitch_target_deg - current_pitch_deg;
    if (fabsf(err) < PITCH_DEADBAND_DEG) {
        stopPitch();
        Serial.println(F("PITCH_AT_TARGET"));
    } else {
        int dir = ((err > 0) ? 1 : -1) * PITCH_SERVO_SIGN;
        setPitchDrive(dir, PITCH_SPEED);
    }
}

// =====================================================
// SLOW LOOP (50 Hz)
// =====================================================
// Work is interleaved with runSteppers() so no single chunk delays a step
// by more than a few hundred microseconds.
void slowLoop() {
    if (mma_ok) {
        update_pitch_filter();
        current_pitch_deg = pitch_filtered_raw - pitch_zero_offset;
    }
    runSteppers();

    updatePitchControl();
    enforcePitchLimit();
    writeServos();
    runSteppers();

    reportSwitchChanges();
    runSteppers();

    if (millis() - lastPosReport >= POS_REPORT_MS && sendPositionUpdate()) {
        lastPosReport = millis();
    }
    runSteppers();

    distanceSample();
}

// =====================================================
// MAIN LOOP
// =====================================================
void loop() {
    runSteppers();

    if (readSerialLine()) {
        processCommand(rxBuf);
        runSteppers();
    }

    unsigned long nowUs = micros();
    if (nowUs - lastSwitchScanUs >= SWITCH_SCAN_US) {
        lastSwitchScanUs = nowUs;
        updateSwitch(swPitch, PITCH_SWITCH);
        updateSwitch(swZ,     Z_MIN_SWITCH);
        updateSwitch(swJ1,    J1_MIN_SWITCH);
        updateSwitch(swJ2,    J2_MIN_SWITCH);

        enforceLimit(stepperZ,  swZ,  Z_MIN_DIR);
        enforceLimit(stepperJ1, swJ1, J1_MIN_DIR);
        enforceLimit(stepperJ2, swJ2, J2_MIN_DIR);
        enforcePitchLimit();
    }

    unsigned long now = millis();
    if (now - lastSlowLoop >= SLOW_PERIOD_MS) {
        lastSlowLoop = now;
        slowLoop();
    }
}

// =====================================================
// COMMAND PROCESSOR
// =====================================================
static char *trim(char *s) {
    while (isspace((unsigned char)*s)) s++;
    char *e = s + strlen(s);
    while (e > s && isspace((unsigned char)e[-1])) *--e = '\0';
    return s;
}

static const char *skipSpaces(const char *s) {
    while (*s == ' ') s++;
    return s;
}

static bool startsWith(const char *s, const char *prefix) {
    return strncmp(s, prefix, strlen(prefix)) == 0;
}

void processCommand(char *msg) {
    msg = trim(msg);
    if (*msg == '\0') return;

    if (startsWith(msg, "G0")) {
        const char *z = strchr(msg + 2, 'Z');
        const char *a = strchr(msg + 2, 'A');
        const char *b = strchr(msg + 2, 'B');
        const char *y = strchr(msg + 2, 'Y');

        if (z) stepperZ.moveTo(atol(z + 1));
        if (a) stepperJ1.moveTo(atol(a + 1));
        if (b) stepperJ2.moveTo(atol(b + 1));
        if (y) targetYawAngle = constrain(atoi(y + 1), 0, 180);

        Serial.println(F("OK"));
    }

    // JOG_START <Z|A|B> <+|->
    else if (startsWith(msg, "JOG_START")) {
        const char *p = skipSpaces(msg + 9);
        char axis = *p;
        char dir = axis ? *skipSpaces(p + 1) : '\0';

        if ((dir == '+' || dir == '-') && jogAxis(axis, (dir == '+') ? 1 : -1)) {
            Serial.println(F("OK"));
        } else {
            Serial.println(F("ERR"));
        }
    }

    // JOG_STOP <Z|A|B>
    else if (startsWith(msg, "JOG_STOP")) {
        Serial.println(stopAxis(*skipSpaces(msg + 8)) ? F("OK") : F("ERR"));
    }

    // SPEED_Z <steps/s>: Z only (must be checked before SPEED)
    else if (startsWith(msg, "SPEED_Z")) {
        long val = atol(msg + 7);
        if (val > 0) {
            stepperZ.setMaxSpeed(val);
            Serial.println(F("OK"));
        } else {
            Serial.println(F("ERR"));
        }
    }

    else if (startsWith(msg, "SPEED")) {
        long val = atol(msg + 5);
        if (val > 0) {
            jogSpeed = val;
            stepperZ.setMaxSpeed(jogSpeed * Z_SPEED_SCALE);
            stepperJ1.setMaxSpeed(jogSpeed);
            stepperJ2.setMaxSpeed(jogSpeed);
            Serial.println(F("OK"));
        } else {
            Serial.println(F("ERR"));
        }
    }

    else if (strcmp(msg, "ESTOP") == 0) {
        haltAllSteppers();
        stopPitch();
        writeServos();
        Serial.println(F("OK"));
    }

    else if (strcmp(msg, "GRIP_OPEN") == 0) {
        targetGripAngle = GRIP_OPEN_ANGLE;
        Serial.println(F("OK"));
    }
    else if (strcmp(msg, "GRIP_CLOSE") == 0) {
        targetGripAngle = GRIP_CLOSE_ANGLE;
        Serial.println(F("OK"));
    }
    else if (strcmp(msg, "GRIP_STOP") == 0) {
        Serial.println(F("OK"));
    }

    else if (strcmp(msg, "PITCH_UP") == 0) {
        if (swPitch.stable == HIGH) {
            pitch_target_deg = NAN;
            setPitchDrive(1, PITCH_MANUAL_SPEED);
        }
        Serial.println(F("OK"));
    }
    else if (strcmp(msg, "PITCH_DOWN") == 0) {
        pitch_target_deg = NAN;
        setPitchDrive(-1, PITCH_MANUAL_SPEED);
        Serial.println(F("OK"));
    }
    else if (strcmp(msg, "PITCH_STOP") == 0) {
        stopPitch();
        Serial.println(F("OK"));
    }
    else if (strcmp(msg, "PITCH_GET") == 0) {
        if (mma_ok) {
            Serial.print(F("PITCH:"));
            Serial.println(current_pitch_deg, 2);
        } else {
            Serial.println(F("PITCH:nan"));
        }
    }
    else if (strcmp(msg, "PITCH_ZERO") == 0) {
        if (mma_ok) {
            pitch_zero_offset = pitch_filtered_raw;
            current_pitch_deg = 0.0f;
            pitch_target_deg = NAN;
            Serial.println(F("OK"));
        } else {
            Serial.println(F("ERR"));
        }
    }
    else if (startsWith(msg, "PITCH_SET ")) {
        if (mma_ok) {
            pitch_target_deg = atof(msg + 10);
            Serial.println(F("OK"));
        } else {
            Serial.println(F("ERR"));   // no IMU -> no closed loop
        }
    }
    else if (strcmp(msg, "PITCH_RAW") == 0) {
        // Debug: raw accel and un-offset, unfiltered pitch from one sample
        float ax, ay, az;
        if (!mma_ok) {
            Serial.println(F("ACC:nan"));
        } else if (mma_read_accel(ax, ay, az)) {
            Serial.print(F("ACC:"));
            Serial.print(ax, 3);
            Serial.print(',');
            Serial.print(ay, 3);
            Serial.print(',');
            Serial.print(az, 3);
            Serial.print(',');
            Serial.println(pitch_from_accel(ay, az), 2);
        } else {
            Serial.println(F("ACC:read_failed"));
        }
    }

    else if (strcmp(msg, "G28") == 0) {
        homeAllAxes();
    }

    else {
        Serial.println(F("UNKNOWN"));
    }
}

// =====================================================
// HOMING HELPERS
// =====================================================
// Homing switch detection. The normal 25 ms "no change" debounce can't be
// used here: sampled in this tight loop while the stepper is driving, every
// noise spike on the switch line restarts it, so it never settles. Instead
// the switch is sampled every HOMING_SAMPLE_US and fed to an integrator
// (+1 pressed, -1 released); it counts as pressed at HOMING_SWITCH_HITS.
const unsigned long HOMING_SAMPLE_US  = 1000;
const uint8_t       HOMING_SWITCH_HITS = 5;

struct SwitchIntegrator {
    uint8_t hits = 0;
    unsigned long lastSample = micros();

    bool pressed(int pin) {
        unsigned long nowUs = micros();
        if (nowUs - lastSample >= HOMING_SAMPLE_US) {
            lastSample = nowUs;
            if (readSwitch(pin) == LOW) hits++;
            else if (hits > 0) hits--;
        }
        return hits >= HOMING_SWITCH_HITS;
    }
};

// Keep the debounced state in sync for the backoff / release logic.
static void markPressed(SwitchState &sw) {
    sw.stable = sw.lastRaw = LOW;
    sw.lastChange = millis();
}

// After backing off, give the switch up to 1 s to read released.
static bool waitForRelease(SwitchState &sw, int pin) {
    unsigned long release_start = millis();
    while (sw.stable == LOW && (millis() - release_start) < 1000UL) {
        if (checkForEmergencyStop()) return false;
        updateSwitch(sw, pin);
        delay(5);
    }
    return true;
}

// Drive at constant speed until the switch reads pressed. Returns false on
// ESTOP or when the switch isn't found within maxSteps.
bool seekSwitch(AccelStepper &stepper, SwitchState &sw, int pin,
                float speed, long maxSteps) {
    long start = stepper.currentPosition();
    SwitchIntegrator det;

    stepper.setSpeed(speed);
    while (!det.pressed(pin)) {
        if (checkForEmergencyStop()) return false;
        if (labs(stepper.currentPosition() - start) > maxSteps) return false;
        stepper.runSpeed();
    }
    markPressed(sw);
    return true;
}

// Z version: Z is timer-driven, so run it as a normal ramped move toward a
// far target at a reduced max speed and halt the instant the switch closes.
bool seekSwitchZ(float speed, long maxSteps) {
    float savedMax = stepperZ.maxSpeed();
    stepperZ.setMaxSpeed(speed);
    stepperZ.moveTo(stepperZ.currentPosition() + Z_HOMING_DIR * maxSteps);

    SwitchIntegrator det;
    bool found = false;
    while (!checkForEmergencyStop()) {
        if (det.pressed(Z_MIN_SWITCH)) { found = true; break; }
        if (stepperZ.distanceToGo() == 0) break;   // travel exhausted
        stepperZ.run();
    }
    stepperZ.setCurrentPosition(stepperZ.currentPosition());   // hard halt
    stepperZ.setMaxSpeed(savedMax);

    if (found) markPressed(swZ);
    return found;
}

template <class Stepper>
bool moveWithEstop(Stepper &stepper, long target) {
    stepper.moveTo(target);
    while (stepper.distanceToGo() != 0) {
        if (checkForEmergencyStop()) return false;
        stepper.run();
    }
    return true;
}

// Touch the switch `repeats` times (fast first, then slow) and return the
// median switch position in medianOut. The stepper is left at the last touch.
bool homeAxisWithRepeat(AccelStepper &stepper, SwitchState &sw, int pin,
                        int repeats, float fast_speed, float slow_speed,
                        long backoff_steps, long maxSteps, long &medianOut) {
    long positions[8];
    if (repeats > 8) repeats = 8;
    if (repeats < 1) repeats = 1;

    for (int i = 0; i < repeats; i++) {
        float approach = (i == 0) ? fast_speed : slow_speed;
        if (!seekSwitch(stepper, sw, pin, approach, maxSteps)) return false;
        positions[i] = stepper.currentPosition();

        if (i < repeats - 1) {
            stepper.move(backoff_steps);
            while (stepper.distanceToGo() != 0) {
                if (checkForEmergencyStop()) return false;
                stepper.run();
                updateSwitch(sw, pin);
            }

            if (!waitForRelease(sw, pin)) return false;
        }
    }

    // insertion sort, then take the median
    for (int i = 1; i < repeats; i++) {
        long key = positions[i];
        int j = i - 1;
        while (j >= 0 && positions[j] > key) {
            positions[j + 1] = positions[j];
            j--;
        }
        positions[j + 1] = key;
    }
    medianOut = positions[repeats / 2];
    return true;
}

static void homingFailed(const __FlashStringHelper *reason) {
    haltAllSteppers();
    if (!emergencyStop && reason) {
        Serial.print(F("HOMING_ERR: "));
        Serial.println(reason);
    }
    Serial.println(F("HOMING_ABORTED"));
}

// =====================================================
// HOMING
// =====================================================
void homeAllAxes() {
    emergencyStop = false;
    Serial.println(F("HOMING_START"));

    stopPitch();
    writeServos();

    // Bring anything still moving to a controlled stop first; otherwise an
    // axis frozen mid-move would resume at full speed after homing.
    stepperZ.stop();
    stepperJ1.stop();
    stepperJ2.stop();
    while (stepperZ.isRunning() || stepperJ1.isRunning() || stepperJ2.isRunning()) {
        if (checkForEmergencyStop()) { homingFailed(nullptr); return; }
        runSteppers();
    }

    // --- 1. Home Pitch: disabled for now ---

    // --- 2. Home Z (fast touch, back off, slow touch) ---
    if (!seekSwitchZ(Z_HOMING_FAST_SPEED, Z_HOMING_MAX_STEPS)) {
        homingFailed(F("Z switch not found"));
        return;
    }
    if (!moveWithEstop(stepperZ, stepperZ.currentPosition() - Z_HOMING_DIR * Z_HOMING_BACKOFF)
        || !waitForRelease(swZ, Z_MIN_SWITCH)) {
        homingFailed(nullptr);
        return;
    }
    if (!seekSwitchZ(Z_HOMING_SLOW_SPEED, 2 * Z_HOMING_BACKOFF)) {
        homingFailed(F("Z switch not found on second touch"));
        return;
    }
    stepperZ.setCurrentPosition(Z_HOME_STEPS);
    if (!moveWithEstop(stepperZ, Z_HOME_STEPS - Z_HOMING_DIR * Z_HOMING_PARK)) {
        homingFailed(nullptr);
        return;
    }

    // --- 3. Home J2 ---
    const long j2_max = (long)(HOMING_MAX_TRAVEL_DEG * STEPS_PER_DEG_J2);
    if (!seekSwitch(stepperJ2, swJ2, J2_MIN_SWITCH, 400, j2_max)) {
        homingFailed(F("J2 switch not found"));
        return;
    }

    long j2_switch_steps = (long)(J2_HOME_ANGLE_DEG * STEPS_PER_DEG_J2);
    stepperJ2.setCurrentPosition(j2_switch_steps);

    if (!moveWithEstop(stepperJ2, j2_switch_steps - HOMING_BACKOFF_J2)) {
        homingFailed(nullptr);
        return;
    }

    // --- 4. Home J1 (multi-touch) ---
    const long j1_max = (long)(HOMING_MAX_TRAVEL_DEG * STEPS_PER_DEG_J1);
    long j1_median = 0;
    if (!homeAxisWithRepeat(stepperJ1, swJ1, J1_MIN_SWITCH,
                            /*repeats=*/    3,
                            /*fast_speed=*/ J1_HOMING_DIR * 400.0,
                            /*slow_speed=*/ J1_HOMING_DIR * 200.0,
                            /*backoff=*/    -J1_HOMING_DIR * 600L,
                            j1_max, j1_median)) {
        homingFailed(F("J1 switch not found"));
        return;
    }

    Serial.print(F("j1_avg (raw switch counter) = "));
    Serial.println(j1_median);

    // The stepper sits at the last touch; reference the median touch to the
    // home angle so the repeated touches actually improve accuracy.
    long j1_switch_steps = (long)(J1_HOME_ANGLE_DEG * STEPS_PER_DEG_J1);
    stepperJ1.setCurrentPosition(j1_switch_steps + (stepperJ1.currentPosition() - j1_median));

    if (!moveWithEstop(stepperJ1, j1_switch_steps - J1_HOMING_DIR * HOMING_BACKOFF)) {
        homingFailed(nullptr);
        return;
    }

    Serial.println(F("HOMING_COMPLETE"));
}

// =====================================================
// DISTANCE SENSOR CALIBRATION
// =====================================================
float calculateDistanceCM(int rawAdc) {
    const int NUM_POINTS = 9;
    static const int rawADC[] = {366, 375, 386, 400, 407, 420, 427, 432, 437};
    static const int distCM[] = {320, 311, 300, 290, 280, 270, 265, 259, 255};

    if (rawAdc <= rawADC[0]) return distCM[0];
    if (rawAdc >= rawADC[NUM_POINTS - 1]) return distCM[NUM_POINTS - 1];

    for (int i = 0; i < NUM_POINTS - 1; i++) {
        if (rawAdc <= rawADC[i + 1]) {
            float fraction = (float)(rawAdc - rawADC[i]) / (rawADC[i + 1] - rawADC[i]);
            return distCM[i] + fraction * (distCM[i + 1] - distCM[i]);
        }
    }
    return distCM[NUM_POINTS - 1];
}
