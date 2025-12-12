/**************************************************************
 * SAFRS AGV - 4WD Motor Controller (Arduino Mega 2560)
 * -----------------------------------------------------------
 * Controls:
 *   - 4 × TB6612FNG Motor Drivers (LF, RF, LR, RR)
 *   - Quadrature Encoders (per wheel)
 *   - PID-based straight driving
 *   - Turn balancing using encoder feedback
 *
 * Works with:
 *   motor_serial_bridge.py on Motor Raspberry Pi
 *
 * Serial Commands:
 *   w → Forward
 *   s → Backward
 *   a → Turn Left
 *   d → Turn Right
 *   x → Stop
 *
 * Status Output:
 *   Prints encoder data every loop:
 *     LF:<val> RF:<val> LR:<val> RR:<val>
 *
 *************************************************************/

#include "tb6612_motor.h"

// ============================================================
//  Motor Definitions (Remapped to LF / RF / LR / RR)
// ============================================================

// LF = original M1
SPDMotor *LF = new SPDMotor(18, 31, true, 12, 35, 34);

// RF = original M2
SPDMotor *RF = new SPDMotor(19, 38, false, 8, 37, 36);

// LR = original M3
SPDMotor *LR = new SPDMotor(3, 49, true, 6, 42, 43);

// RR = original M4
SPDMotor *RR = new SPDMotor(2, A1, false, 5, A4, A5);

#define STBY 9

// ============================================================
//  Speed Configuration
// ============================================================
int speed = 127;
const int maxSpeed = 255;

// Base driving scale (tuned per wheel)
float scale_LF = 0.99f;
float scale_RF = 1.05f;
float scale_LR = 1.10f;
float scale_RR = 1.03f;

// Turning scale
float turn_LF = 1.10f;
float turn_RF = 1.05f;
float turn_LR = 1.15f;
float turn_RR = 1.05f;

// Motor direction correction
int dir_LF = -1;
int dir_RF = -1;
int dir_LR = -1;
int dir_RR = -1;

// Encoder sign
int sign_LF = 1;
int sign_RF = 1;
int sign_LR = -1;
int sign_RR = -1;

// ============================================================
// PID Controller
// ============================================================
float Kp = 0.8f, Ki = 0.0f, Kd = 0.1f;
float error = 0, prev_error = 0, integral = 0;

float computePID(float leftEnc, float rightEnc) {

    float e = fabs(leftEnc) - fabs(rightEnc);
    error = e;

    integral += e;
    integral = constrain(integral, -300, 300);

    float derivative = e - prev_error;
    prev_error = e;

    float out = Kp * e + Ki * integral + Kd * derivative;
    return constrain(out, -80, 80);
}

void resetPID() {
    error = prev_error = integral = 0;
}

// ============================================================
// Encoder Offset
// ============================================================
long offsetL = 0, offsetR = 0;

void resetEncoders() {
    long lf = sign_LF * LF->getEncoderPosition();
    long rf = sign_RF * RF->getEncoderPosition();
    long lr = sign_LR * LR->getEncoderPosition();
    long rr = sign_RR * RR->getEncoderPosition();

    offsetL = (lf + lr) / 2;
    offsetR = (rf + rr) / 2;
}

// ============================================================
// Command Enum
// ============================================================
enum Command { STOP, FORWARD, BACKWARD, TURN_LEFT, TURN_RIGHT };
Command current_cmd = STOP;

// ============================================================
// Deadzone Compensation
// ============================================================
inline int with_deadzone(int pwm) {
    if (pwm == 0) return 0;
    int s = (pwm > 0) ? 1 : -1;
    if (abs(pwm) < 20) return s * 20;
    return pwm;
}

// ============================================================
// Motor Stop
// ============================================================
void stop_all() {
    LF->hardStop();
    RF->hardStop();
    LR->hardStop();
    RR->hardStop();
}

// ============================================================
// Setup
// ============================================================
void setup() {

    Serial.begin(115200);
    pinMode(STBY, OUTPUT);
    digitalWrite(STBY, HIGH);

    stop_all();
    resetEncoders();
    resetPID();

    Serial.println("=== SAFRS 4WD Motor Controller Ready ===");
}

// ============================================================
// Main Loop
// ============================================================
void loop() {

    // =======================
    // Read Serial Command
    // =======================
    while (Serial.available() > 0) {

        String cmd = Serial.readStringUntil('\n');
        cmd.trim();

        bool changed = false;

        if (cmd == "w") { current_cmd = FORWARD; changed = true; }
        else if (cmd == "s") { current_cmd = BACKWARD; changed = true; }
        else if (cmd == "a") { current_cmd = TURN_LEFT; changed = true; }
        else if (cmd == "d") { current_cmd = TURN_RIGHT; changed = true; }
        else if (cmd == "x") { current_cmd = STOP; changed = true; }

        if (changed) {
            resetEncoders();
            resetPID();
        }
    }

    // ===================================================
    // Print Encoder Values
    // ===================================================
    long lf = sign_LF * LF->getEncoderPosition();
    long rf = sign_RF * RF->getEncoderPosition();
    long lr = sign_LR * LR->getEncoderPosition();
    long rr = sign_RR * RR->getEncoderPosition();

    Serial.print("LF:"); Serial.print(lf);
    Serial.print(" RF:"); Serial.print(rf);
    Serial.print(" LR:"); Serial.print(lr);
    Serial.print(" RR:"); Serial.println(rr);

    // ===================================================
    // TURN MODE
    // ===================================================
    if (current_cmd == TURN_LEFT || current_cmd == TURN_RIGHT) {

        int baseTurn = speed;

        long leftEnc  = (lf + lr) / 2 - offsetL;
        long rightEnc = (rf + rr) / 2 - offsetR;

        float correction = constrain(computePID(leftEnc, rightEnc), -20, 20);

        int Lcmd, Rcmd;

        if (current_cmd == TURN_LEFT) {
            Lcmd = -(baseTurn - correction);
            Rcmd =  (baseTurn - correction);
        } else {
            Lcmd =  (baseTurn - correction);
            Rcmd = -(baseTurn - correction);
        }

        auto turn_deadzone = [](int pwm) {
            if (pwm == 0) return 0;
            int s = (pwm > 0) ? 1 : -1;
            if (abs(pwm) < 15) return s * 15;
            return pwm;
        };

        Lcmd = turn_deadzone(Lcmd);
        Rcmd = turn_deadzone(Rcmd);

        LF->speed(constrain(Lcmd * scale_LF * turn_LF, -255, 255) * dir_LF);
        LR->speed(constrain(Lcmd * scale_LR * turn_LR, -255, 255) * dir_LR);
        RF->speed(constrain(Rcmd * scale_RF * turn_RF, -255, 255) * dir_RF);
        RR->speed(constrain(Rcmd * scale_RR * turn_RR, -255, 255) * dir_RR);

        delay(50);
        return;
    }

    // ===================================================
    // FORWARD / BACKWARD
    // ===================================================
    if (current_cmd == STOP) {
        stop_all();
        delay(50);
        return;
    }

    long leftEnc  = (lf + lr) / 2 - offsetL;
    long rightEnc = (rf + rr) / 2 - offsetR;

    float correction = computePID(leftEnc, rightEnc);
    int baseSpeed = (current_cmd == FORWARD) ? speed : -speed;

    int leftPID  = with_deadzone(constrain(baseSpeed - correction, -maxSpeed, maxSpeed));
    int rightPID = with_deadzone(constrain(baseSpeed + correction, -maxSpeed, maxSpeed));

    LF->speed(constrain(leftPID  * scale_LF, -255, 255) * dir_LF);
    LR->speed(constrain(leftPID  * scale_LR, -255, 255) * dir_LR);
    RF->speed(constrain(rightPID * scale_RF, -255, 255) * dir_RF);
    RR->speed(constrain(rightPID * scale_RR, -255, 255) * dir_RR);

    delay(50);
}
