//
// Abstraction of a single motor. Templated.
//
// author: alekd
//

#pragma once

enum class MotorDirection {
    Stopped,
    Forward,
    Backward,
};

// This is static as we don't want multiple motors on the same pins.
template<
    char Name,
    byte PinPwm,
    byte PinDir1,
    byte PinDir2,
    bool DontMove>
class Motor {
public:
    static void Setup() {
        pinMode(PinPwm, OUTPUT);
        pinMode(PinDir1, OUTPUT);
        pinMode(PinDir2, OUTPUT);
    }

    static void SetDirectedPwm(int val) {
        if(val < 0) {
            direction = MotorDirection::Backward;
            pwm = static_cast<byte>(-val);
        } else {
            direction = MotorDirection::Forward;
            pwm = static_cast<byte>(val);
        }
    }

    static void SetBoundedDirectedPwm(int val, int bound) {
        if(val < 0) {
            direction = MotorDirection::Backward;
            pwm = min(-val, bound);
        } else {
            direction = MotorDirection::Forward;
            pwm = min(val, bound);
        }
    }

    static void SetStopped() {
        direction = MotorDirection::Stopped;
        pwm = 0;
    }

    static void Apply() {
        if(direction == MotorDirection::Forward) {
            Forward(pwm);
        } else if(direction == MotorDirection::Backward) {
            Backward(pwm);
        } else {
            Stop();
        }
    }

    static byte GetPwm() {
        return pwm;
    }

    static MotorDirection GetDirection() {
        return direction;
    }

    static bool IsStopped() {
        return direction == MotorDirection::Stopped;
    }

private:
    static void Forward(int pwm) {
        if(DontMove) {
            Serial.print(Name);
            Serial.print(" forward ");
            Serial.println(pwm, DEC);
        } else {
            digitalWrite(PinDir1, LOW);
            digitalWrite(PinDir2, HIGH);
            analogWrite(PinPwm, pwm);
        }
    }

    static void Backward(int pwm) {
        if(DontMove) {
            Serial.print(Name);
            Serial.print(" backward ");
            Serial.println(pwm, DEC);
        } else {
            digitalWrite(PinDir1, HIGH);
            digitalWrite(PinDir2, LOW);
            analogWrite(PinPwm, pwm);
        }
    }

    static void Stop() {
        if(DontMove) {
            Serial.print(Name);
            Serial.println(" stopped");
        } else {
            digitalWrite(PinDir1, LOW);
            digitalWrite(PinDir2, LOW);
            analogWrite(PinPwm, 0);
        }
    }

    static MotorDirection direction;
    static byte pwm;
};

template<char Name, byte PinPwm, byte PinDir1, byte PinDir2, bool DontMove>
MotorDirection Motor<Name, PinPwm, PinDir1, PinDir2, DontMove>::direction;

template<char Name, byte PinPwm, byte PinDir1, byte PinDir2, bool DontMove>
byte Motor<Name, PinPwm, PinDir1, PinDir2, DontMove>::pwm;
