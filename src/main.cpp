#include <Arduino.h>
#include <AccelStepper.h>
#include <Servo.h>

// =====================================================
// PIN DEFINITIONS
// =====================================================
#define Z_DIR_PIN 8
#define Z_STEP_PIN 9
#define J1_DIR_PIN 10
#define J1_STEP_PIN 11
#define J2_DIR_PIN 6
#define J2_STEP_PIN 7

#define YAW_SERVO_PIN 3
#define PITCH_SERVO_PIN 2
#define GRIP_SERVO_PIN 4

#define PITCH_SWITCH 25
#define Z_MIN_SWITCH 22
#define J1_MIN_SWITCH 23
#define J2_MIN_SWITCH 24

// =====================================================
// MOTORS
// =====================================================
AccelStepper stepperZ(AccelStepper::DRIVER, Z_STEP_PIN, Z_DIR_PIN);
AccelStepper stepperJ1(AccelStepper::DRIVER, J1_STEP_PIN, J1_DIR_PIN);
AccelStepper stepperJ2(AccelStepper::DRIVER, J2_STEP_PIN, J2_DIR_PIN);

Servo yawServo;
Servo gripServo;
Servo pitchServo;
void homeAllAxes();

// =====================================================
// STATE (motion)
// =====================================================
int currentYawAngle = 90;
int pitchDirection = 0;       // 0=stop, 1=up, -1=down

int targetYawAngle = 90;
int targetPitchAngle = 90;
int targetGripAngle = 90;

// Jogging flags: for each stepper axis (0=Z, 1=A, 2=B)
bool jogActive[3] = {false, false, false};
int jogDirection[3] = {0, 0, 0};  // +1 or -1

// Default jog speed (steps/sec)
float jogSpeed = 1000.0;

// =====================================================
// SWITCH STATE (debounced)
// =====================================================
struct SwitchState {
    int raw = HIGH;
    int stable = HIGH;
    int lastRaw = HIGH;
    unsigned long lastChange = 0;
};

const unsigned long DEBOUNCE_MS = 25;

SwitchState swPitch, swZ, swJ1, swJ2;
bool swStablePrev[4] = {HIGH, HIGH, HIGH, HIGH}; // stores previous stable state for change detection

// =====================================================
// SERIAL
// =====================================================
String inputBuffer = "";
void processCommand(String msg);
void sendSwitchUpdate();
void sendPositionUpdate();

// =====================================================
// TIMING CONTROL
// =====================================================
unsigned long lastFastLoop = 0;
unsigned long lastSlowLoop = 0;
unsigned long lastPosReport = 0;

const unsigned long FAST_PERIOD_US = 1000;   // 1 kHz
const unsigned long SLOW_PERIOD_MS = 20;     // 50 Hz (for serial RX & servo refresh)
const unsigned long POS_REPORT_MS = 100;     // 10 Hz position updates

// =====================================================
// SWITCH UPDATE FUNCTION (fast‑loop only)
// =====================================================
void updateSwitch(SwitchState &s, int pin) {
    int r = digitalRead(pin);
    if (r != s.lastRaw) {
        s.lastRaw = r;
        s.lastChange = millis();
    }
    if ((millis() - s.lastChange) > DEBOUNCE_MS) {
        s.stable = r;
    }
}

// =====================================================
// SETUP
// =====================================================
void setup() {
    Serial.begin(115200);

    pinMode(PITCH_SWITCH, INPUT_PULLUP);
    pinMode(Z_MIN_SWITCH, INPUT_PULLUP);
    pinMode(J1_MIN_SWITCH, INPUT_PULLUP);
    pinMode(J2_MIN_SWITCH, INPUT_PULLUP);

    // initialise switch states
    swPitch.raw = swPitch.stable = swPitch.lastRaw = digitalRead(PITCH_SWITCH);
    swZ.raw     = swZ.stable     = swZ.lastRaw     = digitalRead(Z_MIN_SWITCH);
    swJ1.raw    = swJ1.stable    = swJ1.lastRaw    = digitalRead(J1_MIN_SWITCH);
    swJ2.raw    = swJ2.stable    = swJ2.lastRaw    = digitalRead(J2_MIN_SWITCH);

    // store initial stables for comparison
    swStablePrev[0] = swPitch.stable;
    swStablePrev[1] = swZ.stable;
    swStablePrev[2] = swJ1.stable;
    swStablePrev[3] = swJ2.stable;

    yawServo.attach(YAW_SERVO_PIN);
    gripServo.attach(GRIP_SERVO_PIN);
    pitchServo.attach(PITCH_SERVO_PIN);

    yawServo.write(currentYawAngle);
    gripServo.write(90);
    pitchServo.write(90);

    stepperZ.setMaxSpeed(jogSpeed);
    stepperZ.setAcceleration(3000);
    stepperJ1.setMaxSpeed(jogSpeed);
    stepperJ1.setAcceleration(3000);
    stepperJ2.setMaxSpeed(jogSpeed);
    stepperJ2.setAcceleration(3000);

    Serial.println("SYSTEM_READY");
}

// =====================================================
// FAST LOOP (1 kHz) – motion + safety state only
// =====================================================
void fastLoop() {
    // ---- steppers (always run) ----
    stepperZ.run();
    stepperJ1.run();
    stepperJ2.run();

    // ---- update switches ----
    updateSwitch(swPitch, PITCH_SWITCH);
    updateSwitch(swZ, Z_MIN_SWITCH);
    updateSwitch(swJ1, J1_MIN_SWITCH);
    updateSwitch(swJ2, J2_MIN_SWITCH);

    // ---- check for switch state changes (for event sending in slow loop) ----
    // This flag is read in slowLoop()
    // (handled later)

    // ---- pitch safety (override target, never actuator) ----
    if (pitchDirection == 1 && swPitch.stable == LOW) {
        targetPitchAngle = 90;
        pitchDirection = 0;
    }
    if (pitchDirection == -1 && swPitch.stable == LOW) { // if we ever add pitch down
        targetPitchAngle = 90;
        pitchDirection = 0;
    }
}

// =====================================================
// SLOW LOOP (50 Hz) – serial, servo refresh, event sending
// =====================================================
void slowLoop() {
    // ---- serial RX ----
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n') {
            inputBuffer.trim();
            if (inputBuffer.length() > 0)
                processCommand(inputBuffer);
            inputBuffer = "";
        } else if (c != '\r') {
            inputBuffer += c;
        }
    }

    // ---- servo writes (50 Hz is perfect) ----
    yawServo.write(targetYawAngle);
    pitchServo.write(targetPitchAngle);
    gripServo.write(targetGripAngle);
    currentYawAngle = targetYawAngle;

    // ---- event: switch change detection ----
    bool switchChanged = false;
    int currentStable[4] = {swPitch.stable, swZ.stable, swJ1.stable, swJ2.stable};
    for (int i = 0; i < 4; i++) {
        if (currentStable[i] != swStablePrev[i]) {
            switchChanged = true;
            swStablePrev[i] = currentStable[i];
        }
    }
    if (switchChanged) {
        sendSwitchUpdate();
    }

    // ---- periodic position update (every 100 ms) ----
    if (millis() - lastPosReport >= POS_REPORT_MS) {
        lastPosReport = millis();
        sendPositionUpdate();
    }
}

// =====================================================
// COMMAND PROCESSOR
// =====================================================
void processCommand(String msg) {
    // Absolute move: G0 Z<pos> A<pos> B<pos> Y<angle>
    if (msg.startsWith("G0")) {
        int zIdx = msg.indexOf('Z');
        int aIdx = msg.indexOf('A');
        int bIdx = msg.indexOf('B');
        int yIdx = msg.indexOf('Y');

        if (zIdx != -1) stepperZ.moveTo(msg.substring(zIdx + 1).toInt());
        if (aIdx != -1) stepperJ1.moveTo(msg.substring(aIdx + 1).toInt());
        if (bIdx != -1) stepperJ2.moveTo(msg.substring(bIdx + 1).toInt());
        if (yIdx != -1) targetYawAngle = msg.substring(yIdx + 1).toInt();

        // Cancel any jog on that axis
        if (zIdx != -1) jogActive[0] = false;
        if (aIdx != -1) jogActive[1] = false;
        if (bIdx != -1) jogActive[2] = false;

        Serial.println("OK");
    }

    // Jog start: JOG_START <axis> <direction>
    else if (msg.startsWith("JOG_START")) {
        // format: "JOG_START Z +" or "JOG_START A -"
        char axis = msg.charAt(10);   // after "JOG_START "
        char dir  = msg.charAt(12);   // after space

        int idx = -1;
        AccelStepper *stepper = nullptr;

        switch (axis) {
            case 'Z': idx = 0; stepper = &stepperZ; break;
            case 'A': idx = 1; stepper = &stepperJ1; break;
            case 'B': idx = 2; stepper = &stepperJ2; break;
        }

        if (stepper && (dir == '+' || dir == '-')) {
            int sign = (dir == '+') ? 1 : -1;
            jogActive[idx] = true;
            jogDirection[idx] = sign;

            // Set a target far away to keep moving
            long target = stepper->currentPosition() + sign * 1000000L;
            stepper->moveTo(target);
            Serial.println("OK");
        } else {
            Serial.println("ERR");
        }
    }

    // Jog stop: JOG_STOP <axis>
    else if (msg.startsWith("JOG_STOP")) {
        char axis = msg.charAt(9);   // after "JOG_STOP "
        int idx = -1;
        AccelStepper *stepper = nullptr;

        switch (axis) {
            case 'Z': idx = 0; stepper = &stepperZ; break;
            case 'A': idx = 1; stepper = &stepperJ1; break;
            case 'B': idx = 2; stepper = &stepperJ2; break;
        }

        if (stepper) {
            jogActive[idx] = false;
            stepper->stop();   // immediate deceleration to a stop, no reversal
            Serial.println("OK");
        } else {
            Serial.println("ERR");
        }
    }

    // Set max speed for jogging: SPEED <value>
    else if (msg.startsWith("SPEED")) {
        int val = msg.substring(6).toInt();
        if (val > 0) {
            jogSpeed = val;
            stepperZ.setMaxSpeed(jogSpeed);
            stepperJ1.setMaxSpeed(jogSpeed);
            stepperJ2.setMaxSpeed(jogSpeed);
            Serial.println("OK");
        }
    }

    // Gripper & pitch commands
    else if (msg == "GRIP_OPEN") {
        targetGripAngle = 120;
        Serial.println("OK");
    }
    else if (msg == "GRIP_CLOSE") {
        targetGripAngle = 60;
        Serial.println("OK");
    }
    else if (msg == "GRIP_STOP") {
        targetGripAngle = 90;    // neutral = stop
        Serial.println("OK");
    }
    else if (msg == "PITCH_UP") {
        if (swPitch.stable == HIGH) {
            targetPitchAngle = 120;
            pitchDirection = 1;
        }
        Serial.println("OK");
    }
    else if (msg == "PITCH_DOWN") {
        if (swPitch.stable == HIGH) {
            targetPitchAngle = 60;    // example
            pitchDirection = -1;
        }
        Serial.println("OK");
    }
    else if (msg == "PITCH_STOP") {
        targetPitchAngle = 90;
        pitchDirection = 0;
        Serial.println("OK");
    }

    // Homing (blocking – kept for now)
    else if (msg == "G28") {
        homeAllAxes();
    }

    else {
        Serial.println("UNKNOWN");
    }
}

// =====================================================
// EVENT HELPERS (only called from slow loop)
// =====================================================
void sendSwitchUpdate() {
    Serial.print("SW:");
    Serial.print(swPitch.stable == LOW);
    Serial.print(",");
    Serial.print(swZ.stable == LOW);
    Serial.print(",");
    Serial.print(swJ1.stable == LOW);
    Serial.print(",");
    Serial.print(swJ2.stable == LOW);
    Serial.println();
}

void sendPositionUpdate() {
    Serial.print("POS:");
    Serial.print(stepperZ.currentPosition());
    Serial.print(",");
    Serial.print(stepperJ1.currentPosition());
    Serial.print(",");
    Serial.print(stepperJ2.currentPosition());
    Serial.print(",");
    Serial.println(currentYawAngle);
}

// =====================================================
// MAIN LOOP
// =====================================================
void loop() {
    unsigned long now = micros();

    if (now - lastFastLoop >= FAST_PERIOD_US) {
        lastFastLoop = now;
        fastLoop();
    }

    if (millis() - lastSlowLoop >= SLOW_PERIOD_MS) {
        lastSlowLoop = millis();
        slowLoop();
    }
}

// =====================================================
// HOMING (blocking but uses debounced switches)
// =====================================================
void homeAllAxes() {
    Serial.println("HOMING_START");

    // Helper lambda (C++11) – polls switch until condition met
    auto waitForSwitch = [](SwitchState &sw, int pin, bool stopWhenLow) {
        if (stopWhenLow) {
            while (sw.stable == HIGH) {
                updateSwitch(sw, pin);
            }
        } else {
            while (sw.stable == LOW) {
                updateSwitch(sw, pin);
            }
        }
    };

    // 1. Home Pitch (continuous servo)
    pitchServo.write(115);
    waitForSwitch(swPitch, PITCH_SWITCH, true);
    pitchServo.write(90);
    pitchDirection = 0;

    // 2. Home Z
    stepperZ.setSpeed(-400);
    while (swZ.stable == HIGH) {
        stepperZ.runSpeed();
        updateSwitch(swZ, Z_MIN_SWITCH);
    }
    stepperZ.setCurrentPosition(0);
    stepperZ.moveTo(0);

    // 3. Home J1
    stepperJ1.setSpeed(-400);
    while (swJ1.stable == HIGH) {
        stepperJ1.runSpeed();
        updateSwitch(swJ1, J1_MIN_SWITCH);
    }
    stepperJ1.setCurrentPosition(0);
    stepperJ1.moveTo(0);

    // 4. Home J2
    stepperJ2.setSpeed(-400);
    while (swJ2.stable == HIGH) {
        stepperJ2.runSpeed();
        updateSwitch(swJ2, J2_MIN_SWITCH);
    }
    stepperJ2.setCurrentPosition(0);
    stepperJ2.moveTo(0);

    Serial.println("HOMING_COMPLETE");
}