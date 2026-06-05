#include <Arduino.h>
#include <AccelStepper.h>
#include <Servo.h>

// --- Pin Definitions ---
#define Z_DIR_PIN 8
#define Z_STEP_PIN 9
#define J1_DIR_PIN 10
#define J1_STEP_PIN 11
#define J2_DIR_PIN 6
#define J2_STEP_PIN 7

#define YAW_SERVO_PIN 3
#define PITCH_SERVO_PIN 2
#define GRIP_SERVO_PIN 4

// --- Endswitch Pin Definitions ---
#define PITCH_SWITCH 25
#define Z_MIN_SWITCH 22
#define J1_MIN_SWITCH 23
#define J2_MIN_SWITCH 24

// --- Object Instantiation ---
AccelStepper stepperZ(AccelStepper::DRIVER, Z_STEP_PIN, Z_DIR_PIN);
AccelStepper stepperJ1(AccelStepper::DRIVER, J1_STEP_PIN, J1_DIR_PIN);
AccelStepper stepperJ2(AccelStepper::DRIVER, J2_STEP_PIN, J2_DIR_PIN);

Servo yawServo;
Servo gripServo;
Servo pitchServo;

// --- System State Variables ---
int currentYawAngle = 90;
int pitchDirection = 0;  // 0 = Stopped, 1 = Up, 2 = Down
String inputBuffer = ""; // Non-blocking serial handler

// --- Switch State Tracking (For Instant Console Prints) ---
int lastPitchState = HIGH;
int lastZState = HIGH;
int lastJ1State = HIGH;
int lastJ2State = HIGH;

// --- Non-Blocking Gripper Variables ---
bool gripperRunning = false;
unsigned long gripperStartTime = 0;
const unsigned long GRIPPER_RUN_TIME = 600;

// --- Jog Mode ---
bool jogActiveZ = false;
bool jogActiveJ1 = false;
bool jogActiveJ2 = false;

int jogDirZ = 0;
int jogDirJ1 = 0;
int jogDirJ2 = 0;

const float JOG_SPEED = 800;

// --- Function Declarations ---
void homeAllAxes();
void processCommand(String msg);
void updateGripper();
void checkHardwareLimits();
void readSerialNonBlocking();
void monitorSwitchChanges();

void setup()
{
    Serial.begin(115200);

    // Configure ALL Endswitches with Internal Pullups
    pinMode(PITCH_SWITCH, INPUT_PULLUP);
    pinMode(Z_MIN_SWITCH, INPUT_PULLUP);
    pinMode(J1_MIN_SWITCH, INPUT_PULLUP);
    pinMode(J2_MIN_SWITCH, INPUT_PULLUP);

    // Read initial states
    lastPitchState = digitalRead(PITCH_SWITCH);
    lastZState = digitalRead(Z_MIN_SWITCH);
    lastJ1State = digitalRead(J1_MIN_SWITCH);
    lastJ2State = digitalRead(J2_MIN_SWITCH);

    // Attach Servos
    yawServo.attach(YAW_SERVO_PIN);
    gripServo.attach(GRIP_SERVO_PIN);
    pitchServo.attach(PITCH_SERVO_PIN);

    yawServo.write(currentYawAngle);
    gripServo.write(90);
    pitchServo.write(90);

    // Configure Stepper Max Speeds & Accelerations
    stepperZ.setMaxSpeed(1200);
    stepperZ.setAcceleration(600);
    stepperJ1.setMaxSpeed(1200);
    stepperJ1.setAcceleration(600);
    stepperJ2.setMaxSpeed(1200);
    stepperJ2.setAcceleration(600);

    Serial.println("SYSTEM_READY");
}

void loop()
{
    // 1. Core Motion Engine
    if (jogActiveZ) {
        stepperZ.runSpeed();
    } else {
        stepperZ.run();
    }

    if (jogActiveJ1) {
        stepperJ1.runSpeed();
    } else {
        stepperJ1.run();
    }

    if (jogActiveJ2) {
        stepperJ2.runSpeed();
    } else {
        stepperJ2.run();
    }

    // 2. Hardware Monitoring Logic
    checkHardwareLimits();
    monitorSwitchChanges(); // Instantly prints changes to console
    updateGripper();

    // 3. Asynchronous Serial Reader
    readSerialNonBlocking();
}

void readSerialNonBlocking()
{
    while (Serial.available() > 0)
    {
        char inChar = (char)Serial.read();
        if (inChar == '\n')
        {
            inputBuffer.trim();
            if (inputBuffer.length() > 0)
            {
                processCommand(inputBuffer);
            }
            inputBuffer = "";
        }
        else if (inChar != '\r')
        {
            inputBuffer += inChar;
        }
    }
}

void monitorSwitchChanges()
{
    int currentPitch = digitalRead(PITCH_SWITCH);
    int currentZ = digitalRead(Z_MIN_SWITCH);
    int currentJ1 = digitalRead(J1_MIN_SWITCH);
    int currentJ2 = digitalRead(J2_MIN_SWITCH);

    if (currentPitch != lastPitchState)
    {
        Serial.print("[SWITCH_CHANGED] Pitch -> ");
        Serial.println(currentPitch == LOW ? "TRIGGERED" : "OPEN");
        lastPitchState = currentPitch;
    }
    if (currentZ != lastZState)
    {
        Serial.print("[SWITCH_CHANGED] Z-Axis -> ");
        Serial.println(currentZ == LOW ? "TRIGGERED" : "OPEN");
        lastZState = currentZ;
    }
    if (currentJ1 != lastJ1State)
    {
        Serial.print("[SWITCH_CHANGED] Joint 1 -> ");
        Serial.println(currentJ1 == LOW ? "TRIGGERED" : "OPEN");
        lastJ1State = currentJ1;
    }
    if (currentJ2 != lastJ2State)
    {
        Serial.print("[SWITCH_CHANGED] Joint 2 -> ");
        Serial.println(currentJ2 == LOW ? "TRIGGERED" : "OPEN");
        lastJ2State = currentJ2;
    }
}

void homeAllAxes()
{
    Serial.println("HOMING_START");

    // 1. Home Continuous Servo Pitch Axis
    while (digitalRead(PITCH_SWITCH) == HIGH)
    {
        pitchServo.write(115);
        delay(5);
    }
    pitchServo.write(90);
    pitchDirection = 0;

    // 2. Home Z Axis (Stepper)
    stepperZ.setSpeed(-400);
    while (digitalRead(Z_MIN_SWITCH) == HIGH)
    {
        stepperZ.runSpeed();
    }
    stepperZ.setCurrentPosition(0);
    stepperZ.moveTo(0);

    // 3. Home Joint 1 (Base)
    stepperJ1.setSpeed(-400);
    while (digitalRead(J1_MIN_SWITCH) == HIGH)
    {
        stepperJ1.runSpeed();
    }
    stepperJ1.setCurrentPosition(0);
    stepperJ1.moveTo(0);

    // 4. Home Joint 2 (Elbow)
    stepperJ2.setSpeed(-400);
    while (digitalRead(J2_MIN_SWITCH) == HIGH)
    {
        stepperJ2.runSpeed();
    }
    stepperJ2.setCurrentPosition(0);
    stepperJ2.moveTo(0);

    Serial.println("HOMING_COMPLETE");
}

void checkHardwareLimits()
{
    if (pitchDirection == 1 && digitalRead(PITCH_SWITCH) == LOW)
    {
        pitchServo.write(90);
        pitchDirection = 0;
        Serial.println("PITCH_LIMIT_SAFETY_STOP");
    }
}

void updateGripper()
{
    if (gripperRunning && (millis() - gripperStartTime >= GRIPPER_RUN_TIME))
    {
        gripServo.write(90);
        gripperRunning = false;
        Serial.println("gripper_done");
    }
}

void processCommand(String msg)
{
    if (msg == "G28")
    {
        homeAllAxes();
        return;
    }

    if (msg.startsWith("G0"))
    {
        int zIndex = msg.indexOf('Z');
        int j1Index = msg.indexOf('A');
        int j2Index = msg.indexOf('B');
        int yawIndex = msg.indexOf('Y');

        if (zIndex != -1)
            stepperZ.moveTo(msg.substring(zIndex + 1).toInt());
        if (j1Index != -1)
            stepperJ1.moveTo(msg.substring(j1Index + 1).toInt());
        if (j2Index != -1)
            stepperJ2.moveTo(msg.substring(j2Index + 1).toInt());
        if (yawIndex != -1)
        {
            currentYawAngle = msg.substring(yawIndex + 1).toInt();
            yawServo.write(currentYawAngle);
        }
        Serial.println("ok");
    }

    else if (msg.startsWith("G1"))
    {
        int zIndex = msg.indexOf('Z');
        int j1Index = msg.indexOf('A');
        int j2Index = msg.indexOf('B');
        int yawIndex = msg.indexOf('Y');

        if (zIndex != -1)
            stepperZ.move(msg.substring(zIndex + 1).toInt());
        if (j1Index != -1)
            stepperJ1.move(msg.substring(j1Index + 1).toInt());
        if (j2Index != -1)
            stepperJ2.move(msg.substring(j2Index + 1).toInt());
        if (yawIndex != -1)
        {
            currentYawAngle += msg.substring(yawIndex + 1).toInt();
            currentYawAngle = constrain(currentYawAngle, 0, 180);
            yawServo.write(currentYawAngle);
        }
        Serial.println("ok");
    }

    else if (msg.startsWith("V"))
    {
        int newSpeed = msg.substring(1).toInt();
        stepperZ.setMaxSpeed(newSpeed);
        stepperJ1.setMaxSpeed(newSpeed);
        stepperJ2.setMaxSpeed(newSpeed);
        Serial.println("ok");
    }

    else if (msg == "PITCH_UP")
    {
        if (digitalRead(PITCH_SWITCH) == HIGH)
        {
            pitchServo.write(120);
            pitchDirection = 1;
            Serial.println("ok");
        }
        else
        {
            Serial.println("error: limit active");
        }
    }
    else if (msg == "PITCH_DOWN")
    {
        pitchServo.write(60);
        pitchDirection = 2;
        Serial.println("ok");
    }
    else if (msg == "PITCH_STOP")
    {
        pitchServo.write(90);
        pitchDirection = 0;
        Serial.println("ok");
    }

    else if (msg == "M119")
    {
        int p_sw = digitalRead(PITCH_SWITCH) == LOW ? 1 : 0;
        int z_sw = digitalRead(Z_MIN_SWITCH) == LOW ? 1 : 0;
        int j1_sw = digitalRead(J1_MIN_SWITCH) == LOW ? 1 : 0;
        int j2_sw = digitalRead(J2_MIN_SWITCH) == LOW ? 1 : 0;

        Serial.print("SW:");
        Serial.print(p_sw);
        Serial.print(",");
        Serial.print(z_sw);
        Serial.print(",");
        Serial.print(j1_sw);
        Serial.print(",");
        Serial.println(j2_sw);
    }

    // Reports target positions so UI updates responsively
    else if (msg == "M114")
    {
        Serial.print("POS:");
        Serial.print(stepperZ.currentPosition());
        Serial.print(",");
        Serial.print(stepperJ1.currentPosition());
        Serial.print(",");
        Serial.print(stepperJ2.currentPosition());
        Serial.print(",");
        Serial.println(currentYawAngle);
    }

    else if (msg == "GRIP_OPEN")
    {
        gripServo.write(180);
        gripperStartTime = millis();
        gripperRunning = true;
        Serial.println("ok");
    }
    else if (msg == "GRIP_CLOSE")
    {
        gripServo.write(0);
        gripperStartTime = millis();
        gripperRunning = true;
        Serial.println("ok");
    }

    else if (msg == "JOG_START Z +") {
        jogActiveZ = true;
        stepperZ.setSpeed(JOG_SPEED);
    }
    else if (msg == "JOG_START Z -") {
        jogActiveZ = true;
        stepperZ.setSpeed(-JOG_SPEED);
    }

    else if (msg == "JOG_START A +") {
        jogActiveJ1 = true;
        stepperJ1.setSpeed(JOG_SPEED);
    }
    else if (msg == "JOG_START A -") {
        jogActiveJ1 = true;
        stepperJ1.setSpeed(-JOG_SPEED);
    }

    else if (msg == "JOG_START B +") {
        jogActiveJ2 = true;
        stepperJ2.setSpeed(JOG_SPEED);
    }
    else if (msg == "JOG_START B -") {
        jogActiveJ2 = true;
        stepperJ2.setSpeed(-JOG_SPEED);
    }
    else if (msg == "JOG_STOP Z") {
        jogActiveZ = false;
        stepperZ.moveTo(stepperZ.currentPosition());
        Serial.println("ok");
    }
    else if (msg == "JOG_STOP A") {
        jogActiveJ1 = false;
        stepperJ1.moveTo(stepperJ1.currentPosition());
        Serial.println("ok");
    }
    else if (msg == "JOG_STOP B") {
        jogActiveJ2 = false;
        stepperJ2.moveTo(stepperJ2.currentPosition());
        Serial.println("ok");
    }
}