//
// A servo that is moved gradually to the new position for smoother movement.
//
// author: alekd
//

#pragma once

#include <Servo.h>

template<
    char Name,
    int Pin,
    int MinPosition,
    int MaxPosition,
    bool DontMove,
    int StartPosition = (MinPosition + MaxPosition) / 2,
    int PwmCycleUs = 20000>
class SmoothServo {
public:
    static constexpr int PwmCyclceUs = PwmCycleUs;
    static constexpr byte DefaultStep = 2;

    static SmoothServo &Instance() {
        return instance;
    }

    void Setup() {
        pinMode(Pin, OUTPUT);
        servo.attach(Pin);

        current = StartPosition;
        target = StartPosition;

        MoveServo();
    }

    // move towards the target using step
    void Move() {
        if(current == target) {
            return;
        }

        if(target < current) {
            current = max(target, current - step);
        } else if(target > current) {
            current = min(target, current + step);
        }

        MoveServo();
    }

    void SetTarget(int new_target, byte new_step = DefaultStep) {
        target = new_target;

        if(target < MinPosition) {
            target = MinPosition;
        } else if(target > MaxPosition) {
            target = MaxPosition;
        }

        step = new_step;
    }

    void Down(byte new_step = DefaultStep) {
        target = MinPosition;
        step = new_step;
    }

    void Up(byte new_step = DefaultStep) {
        target = MaxPosition;
        step = new_step;
    }

    void Stop() {
        target = current;
    }

private:
    SmoothServo() = default;

    void MoveServo() {
        if(DontMove) {
            Serial.print("Smooth servo ");
            Serial.print(Name);
            Serial.print(" position ");
            Serial.println(current, DEC);
        } else {
            servo.write(current);
        }
    }

    Servo servo;
    int current;
    int target;
    byte step;

    static SmoothServo instance;
};

template<
    char Name,
    int Pin,
    int MinPosition,
    int MaxPosition,
    bool DontMove,
    int StartPosition,
    int PwmCycleUs>
SmoothServo<Name, Pin, MinPosition, MaxPosition, DontMove, StartPosition, PwmCycleUs>
SmoothServo<Name, Pin, MinPosition, MaxPosition, DontMove, StartPosition, PwmCycleUs>::instance;
