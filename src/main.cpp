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

#define DISTANCE_SENSOR_PIN A1


// =====================================================
// Distance Sensor
// =====================================================
const int number_distance_samples = 300;
float distance_readings[number_distance_samples] = {0};
int current_distance_reading_index = 0;
float calculateDistanceCM(int rawAdc);


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
// Physical joint angles at the home limit switch (degrees)
#define J1_HOME_ANGLE_DEG  -105.0
#define J2_HOME_ANGLE_DEG  -150.0

#define HOMING_BACKOFF_J2 14000   // steps to move away from the endstop
#define HOMING_BACKOFF 500       // steps to move away from the endstop

// Steps per degree (must match your Python kinematics exactly)
const float STEPS_PER_DEG_J1 = 139.31;
const float STEPS_PER_DEG_J2 = 63.83;

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

bool emergencyStop = false;   // set by fast‑loop when ESTOP received
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
const unsigned long SLOW_PERIOD_MS = 20;     // 50 Hz
const unsigned long POS_REPORT_MS = 100;     // 10 Hz

// =====================================================
// SWITCH UPDATE FUNCTION
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

    pinMode(DISTANCE_SENSOR_PIN, INPUT);

    swPitch.raw = swPitch.stable = swPitch.lastRaw = digitalRead(PITCH_SWITCH);
    swZ.raw     = swZ.stable     = swZ.lastRaw     = digitalRead(Z_MIN_SWITCH);
    swJ1.raw    = swJ1.stable    = swJ1.lastRaw    = digitalRead(J1_MIN_SWITCH);
    swJ2.raw    = swJ2.stable    = swJ2.lastRaw    = digitalRead(J2_MIN_SWITCH);

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
// FAST LOOP (1 kHz)
// =====================================================
void fastLoop() {
    stepperZ.run();
    stepperJ1.run();
    stepperJ2.run();

    updateSwitch(swPitch, PITCH_SWITCH);
    updateSwitch(swZ, Z_MIN_SWITCH);
    updateSwitch(swJ1, J1_MIN_SWITCH);
    updateSwitch(swJ2, J2_MIN_SWITCH);

    if (pitchDirection == 1 && swPitch.stable == LOW) {
        targetPitchAngle = 90;
        pitchDirection = 0;
    }
}

// =====================================================
// SLOW LOOP (50 Hz)
// =====================================================
void slowLoop() {
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

    yawServo.write(targetYawAngle);
    pitchServo.write(targetPitchAngle);
    gripServo.write(targetGripAngle);
    currentYawAngle = targetYawAngle;

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

    if (millis() - lastPosReport >= POS_REPORT_MS) {
        lastPosReport = millis();
        sendPositionUpdate();
    }

    int current_distance_reading = analogRead(DISTANCE_SENSOR_PIN);
    distance_readings[current_distance_reading_index] = calculateDistanceCM(current_distance_reading);
    current_distance_reading_index++;
    current_distance_reading_index %= number_distance_samples;
}

// =====================================================
// COMMAND PROCESSOR
// =====================================================
void processCommand(String msg) {
    if (msg.startsWith("G0")) {
        int zIdx = msg.indexOf('Z');
        int aIdx = msg.indexOf('A');
        int bIdx = msg.indexOf('B');
        int yIdx = msg.indexOf('Y');

        if (zIdx != -1) stepperZ.moveTo(msg.substring(zIdx + 1).toInt());
        if (aIdx != -1) stepperJ1.moveTo(msg.substring(aIdx + 1).toInt());
        if (bIdx != -1) stepperJ2.moveTo(msg.substring(bIdx + 1).toInt());
        if (yIdx != -1) targetYawAngle = msg.substring(yIdx + 1).toInt();

        if (zIdx != -1) jogActive[0] = false;
        if (aIdx != -1) jogActive[1] = false;
        if (bIdx != -1) jogActive[2] = false;

        Serial.println("OK");
    }

    else if (msg.startsWith("JOG_START")) {
        char axis = msg.charAt(10);
        char dir  = msg.charAt(12);

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

            long target = stepper->currentPosition() + sign * 1000000L;
            stepper->moveTo(target);
            Serial.println("OK");
        } else {
            Serial.println("ERR");
        }
    }

    else if (msg.startsWith("JOG_STOP")) {
        char axis = msg.charAt(9);
        int idx = -1;
        AccelStepper *stepper = nullptr;

        switch (axis) {
            case 'Z': idx = 0; stepper = &stepperZ; break;
            case 'A': idx = 1; stepper = &stepperJ1; break;
            case 'B': idx = 2; stepper = &stepperJ2; break;
        }

        if (stepper) {
            jogActive[idx] = false;
            stepper->stop();
            Serial.println("OK");
        } else {
            Serial.println("ERR");
        }
    }

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

    else if (msg == "GRIP_OPEN") {
        targetGripAngle = 120;
        Serial.println("OK");
    }
    else if (msg == "GRIP_CLOSE") {
        targetGripAngle = 60;
        Serial.println("OK");
    }
    else if (msg == "GRIP_STOP") {
        targetGripAngle = 90;
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
        targetPitchAngle = 60;
        pitchDirection = -1;
        Serial.println("OK");
    }
    else if (msg == "PITCH_STOP") {
        targetPitchAngle = 90;
        pitchDirection = 0;
        Serial.println("OK");
    }

    else if (msg == "G28") {
        homeAllAxes();
    }

    else {
        Serial.println("UNKNOWN");
    }
}

// =====================================================
// EVENT HELPERS
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
    Serial.print(currentYawAngle);
    Serial.print(",");

    float average_distance = 0;
    for (int i = 0; i < number_distance_samples; i++) {
        average_distance += distance_readings[i];
    }
    average_distance /= number_distance_samples;
    Serial.println(average_distance);
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
// EMERGENCY STOP CHECK
// =====================================================
bool checkForEmergencyStop() {
    while (Serial.available()) {
        char c = Serial.read();
        if (c == 'E') {
            delayMicroseconds(500);
            if (Serial.available() >= 4) {
                char buf[5];
                Serial.readBytes(buf, 4);
                buf[4] = '\0';
                if (strcmp(buf, "STOP") == 0) {
                    while (Serial.available() && Serial.read() != '\n');
                    return true;
                }
            }
        }
    }
    return false;
}

// =====================================================
// MULTI-TOUCH HOMING HELPER
// =====================================================
// Returns the median switch counter value at the moment of trigger.
// Used ONLY as a repeatable stopping point; the counter is still
// re-labelled to (HOME_ANGLE_DEG * STEPS_PER_DEG) afterwards, exactly
// like the single-touch version.
long homeAxisWithRepeat(AccelStepper &stepper, SwitchState &sw, int pin,
                        int repeats,
                        float fast_speed,
                        float slow_speed,
                        long backoff_steps)
{
    long positions[8];
    if (repeats > 8) repeats = 8;
    int successful = 0;

    for (int i = 0; i < repeats; i++) {
        if (emergencyStop) break;

        float approach = (i == 0) ? fast_speed : slow_speed;
        stepper.setSpeed(approach);

        while (sw.stable == HIGH && !emergencyStop) {
            stepper.runSpeed();
            updateSwitch(sw, pin);
            if (checkForEmergencyStop()) emergencyStop = true;
        }
        if (emergencyStop) break;

        positions[successful++] = stepper.currentPosition();

        if (i < repeats - 1) {
            stepper.setSpeed(-fast_speed);
            stepper.move(backoff_steps);
            while (stepper.distanceToGo() != 0 && !emergencyStop) {
                stepper.run();
                updateSwitch(sw, pin);
                if (checkForEmergencyStop()) emergencyStop = true;
            }

            unsigned long release_start = millis();
            while (sw.stable == LOW && !emergencyStop
                   && (millis() - release_start) < 1000UL) {
                updateSwitch(sw, pin);
                delay(5);
            }
        }
    }

    if (emergencyStop || successful == 0) return 0;
    if (successful == 1) return positions[0];

    // Median
    for (int i = 1; i < successful; i++) {
        long key = positions[i];
        int j = i - 1;
        while (j >= 0 && positions[j] > key) {
            positions[j + 1] = positions[j];
            j--;
        }
        positions[j + 1] = key;
    }
    return positions[successful / 2];
}

// =====================================================
// HOMING
// =====================================================
void homeAllAxes() {
    emergencyStop = false;
    Serial.println("HOMING_START");

    auto waitForSwitch = [](SwitchState &sw, int pin, bool stopWhenLow) {
        if (stopWhenLow) {
            while (sw.stable == HIGH && !emergencyStop) {
                updateSwitch(sw, pin);
                if (checkForEmergencyStop()) emergencyStop = true;
            }
        } else {
            while (sw.stable == LOW && !emergencyStop) {
                updateSwitch(sw, pin);
                if (checkForEmergencyStop()) emergencyStop = true;
            }
        }
    };

    auto moveWithEstop = [](AccelStepper &stepper, long target) {
        stepper.moveTo(target);
        while (stepper.distanceToGo() != 0 && !emergencyStop) {
            stepper.run();
            if (checkForEmergencyStop()) emergencyStop = true;
        }
    };

    // --- 1. Home Pitch Disabled for now ---
    /*pitchServo.write(115);
    waitForSwitch(swPitch, PITCH_SWITCH, true);
    pitchServo.write(90);
    pitchDirection = 0;
    if (emergencyStop) { Serial.println("HOMING_ABORTED"); return; }*/


    // --- 2. Home J2 (unchanged, single touch) ---
    stepperJ2.setSpeed(-400);
    while (swJ2.stable == HIGH && !emergencyStop) {
        stepperJ2.runSpeed();
        updateSwitch(swJ2, J2_MIN_SWITCH);
        if (checkForEmergencyStop()) emergencyStop = true;
    }
    if (emergencyStop) { stepperJ2.stop(); Serial.println("HOMING_ABORTED"); return; }

    long j2_switch_steps = (long)(J2_HOME_ANGLE_DEG * STEPS_PER_DEG_J2);
    stepperJ2.setCurrentPosition(j2_switch_steps);

    moveWithEstop(stepperJ2, j2_switch_steps + HOMING_BACKOFF_J2);
    if (emergencyStop) { stepperJ2.stop(); Serial.println("HOMING_ABORTED"); return; }


    // --- 3. Home J1 (multi-touch for better repeatability) ---
    // The multi-touch only affects WHERE the arm comes to rest physically.
    // The counter is re-labelled to (J1_HOME_ANGLE_DEG * STEPS_PER_DEG_J1)
    // exactly like the single-touch version, so the post-homing reset
    // is identical to your reference.
    long j1_avg = homeAxisWithRepeat(stepperJ1, swJ1, J1_MIN_SWITCH,
                                     /*repeats=*/     3,       // 1 fast + 3 slow
                                     /*fast_speed=*/ -400.0,
                                     /*slow_speed=*/ -200.0,
                                     /*backoff=*/     600);
    if (emergencyStop) { stepperJ1.stop(); Serial.println("HOMING_ABORTED"); return; }

    // Diagnostic only — raw counter where the switch triggered.
    Serial.print("j1_avg (raw switch counter) = ");
    Serial.println(j1_avg);

    long j1_switch_steps = (long)(J1_HOME_ANGLE_DEG * STEPS_PER_DEG_J1);
    stepperJ1.setCurrentPosition(j1_switch_steps);

    moveWithEstop(stepperJ1, j1_switch_steps + HOMING_BACKOFF);
    if (emergencyStop) { stepperJ1.stop(); Serial.println("HOMING_ABORTED"); return; }

    Serial.println("HOMING_COMPLETE");
}

// =====================================================
// DISTANCE SENSOR
// =====================================================
float calculateDistanceCM(int rawAdc) {
    const int NUM_POINTS = 9;
    const int rawADC[]  = {366,  375,  386, 400, 407, 420, 427, 432, 437};
    const int distCM[]  = {320,  311,  300, 290, 280,  270,  265,  259, 255};

    if (rawAdc <= rawADC[0]) return distCM[0];
    if (rawAdc >= rawADC[NUM_POINTS - 1]) return distCM[NUM_POINTS - 1];

    for (int i = 0; i < NUM_POINTS - 1; i++) {
        if (rawAdc >= rawADC[i] && rawAdc <= rawADC[i + 1]) {
            float fraction = (float)(rawAdc - rawADC[i]) / (rawADC[i + 1] - rawADC[i]);
            return distCM[i] + fraction * (distCM[i + 1] - distCM[i]);
        }
    }
    return 150.0;
}