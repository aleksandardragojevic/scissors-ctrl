//
// Abstraction of a single motor. Templated.
//
// author: alekd
//

#pragma once

enum class MotorDirection {
    Forward,
    Backward,
    Stopped,
};

struct MotorState {
    MotorDirection direction;
    int pwm;

    int RealPwm() const {
        if(direction == MotorDirection::Stopped) {
            return 0;
        }

        return pwm;
    }

    void SetDirectedPwm(int val) {
        if(val < 0) {
            direction = MotorDirection::Backward;
            pwm = -val;
        } else {
            direction = MotorDirection::Forward;
            pwm = val;
        }
    }

    void SetDirectedPwm(int val, int max_val) {
        if(val < 0) {
            direction = MotorDirection::Backward;
            pwm = min(-val, max_val);
        } else {
            direction = MotorDirection::Forward;
            pwm = min(val, max_val);
        }
    }

    void SetStopped() {
        direction = MotorDirection::Stopped;
        pwm = 0;
    }
};

template<
    char Name,
    byte PinPwm,
    byte PinDir1,
    byte PinDir2,
    bool DontMove>
struct Motor {
    static void Setup() {
        pinMode(PinPwm, OUTPUT);
        pinMode(PinDir1, OUTPUT);
        pinMode(PinDir2, OUTPUT);
    }

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

    static void ApplyState(const MotorState &state) {
        if(state.direction == MotorDirection::Forward) {
            Forward(state.pwm);
        } else if(state.direction == MotorDirection::Backward) {
            Backward(state.pwm);
        } else {
            Stop();
        }
    }
};
