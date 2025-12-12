/**************************************************************
 * TB6612 Motor Driver Class for SAFRS AGV
 * Requires: Paul Stoffregen Encoder Library
 *************************************************************/

#include "tb6612_motor.h"

SPDMotor::SPDMotor(int encoderA, int encoderB, bool encoderReversed,
                   int motorPWM, int motorDir1, int motorDir2) {

    _encoder = new Encoder(encoderA, encoderB);
    _encoderReversed = encoderReversed;

    _motorPWM = motorPWM;
    pinMode(_motorPWM, OUTPUT);

    _motorDir1 = motorDir1;
    pinMode(_motorDir1, OUTPUT);

    _motorDir2 = motorDir2;
    pinMode(_motorDir2, OUTPUT);
}

void SPDMotor::speed(int speedPWM) {

    _speed = speedPWM;

    if (speedPWM == 0) {
        digitalWrite(_motorDir1, LOW);
        digitalWrite(_motorDir2, LOW);
        analogWrite(_motorPWM, 255);
        return;
    }

    if (speedPWM > 0) {
        digitalWrite(_motorDir1, LOW);
        digitalWrite(_motorDir2, HIGH);
        analogWrite(_motorPWM, min(speedPWM, 255));
    } else {
        digitalWrite(_motorDir1, HIGH);
        digitalWrite(_motorDir2, LOW);
        analogWrite(_motorPWM, min(-speedPWM, 255));
    }
}

void SPDMotor::hardStop() {
    _speed = 0;
    digitalWrite(_motorDir1, HIGH);
    digitalWrite(_motorDir2, HIGH);
    analogWrite(_motorPWM, 0);
}

int SPDMotor::getSpeed() {
    return _speed;
}

long SPDMotor::getEncoderPosition() {
    long pos = _encoder->read();
    return _encoderReversed ? -pos : pos;
}
