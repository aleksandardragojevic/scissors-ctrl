//
// Abstraction of a single motor. Templated.
//
// author: alekd
//

#pragma once

// This is static as we don't want multiple motors on the same pins.
template<
    char Name,
    byte PinPwm,
    byte PinDir1,
    byte PinDir2,
    bool DontMove,
    int MaxAbsPwm = 240>
class Motor {
public:
    static constexpr int MaxAbsolutePwm = MaxAbsPwm;

    static Motor &Instance() {
        return instance;
    }

    void Setup() {
        pinMode(PinPwm, OUTPUT);
        pinMode(PinDir1, OUTPUT);
        pinMode(PinDir2, OUTPUT);

        ApplyStop();

        pwm = last_pwm = 0;
    }

    void SetPwm(int val) {
        if(val < 0) {
            pwm = max(val, -MaxAbsPwm);
        } else {
            pwm = min(val, MaxAbsPwm);
        }
    }

    void SetStopped() {
        pwm = 0;
    }

    void Apply() {
        if(last_pwm == pwm) {
            return;
        }

        if(IsGoingForward()) {
            ApplyForward();
        } else if(IsGoingBackward()) {
            ApplyBackward();
        } else {
            ApplyStop();
        }

        last_pwm = pwm;
    }

    int GetPwm() {
        return pwm;
    }

    bool IsGoingBackward() const {
        return pwm < 0;
    }

    bool IsGoingForward() const {
        return pwm > 0;
    }

    bool IsStopped() const {
        return pwm == 0;
    }

private:
    Motor()
            : pwm(0) {
        // empty
    }

    void ApplyForward() {
        if(DontMove) {
            Serial.print("Motor ");
            Serial.print(Name);
            Serial.print(" ");
            Serial.println(pwm, DEC);
        } else {
            digitalWrite(PinDir1, LOW);
            digitalWrite(PinDir2, HIGH);
            analogWrite(PinPwm, pwm);
        }
    }

    void ApplyBackward() {
        if(DontMove) {
            Serial.print("Motor ");
            Serial.print(Name);
            Serial.print(" ");
            Serial.println(pwm, DEC);
        } else {
            digitalWrite(PinDir1, HIGH);
            digitalWrite(PinDir2, LOW);
            analogWrite(PinPwm, -pwm);
        }
    }

    void ApplyStop() {
        if(DontMove) {
            Serial.print(Name);
            Serial.println(" stopped");
        } else {
            digitalWrite(PinDir1, LOW);
            digitalWrite(PinDir2, LOW);
            analogWrite(PinPwm, 0);
        }
    }

    int last_pwm;
    int pwm;

    static Motor instance;
};

template<char Name, byte PinPwm, byte PinDir1, byte PinDir2, bool DontMove, int MaxAbsPwm>
Motor<Name, PinPwm, PinDir1, PinDir2, DontMove, MaxAbsPwm>
Motor<Name, PinPwm, PinDir1, PinDir2, DontMove, MaxAbsPwm>::instance;
