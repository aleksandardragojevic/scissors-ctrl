//
// Control of the motors.
//
// author: alekd
//

#pragma once

#include "motor_timed.h"
#include "scissors_time.h"

template<bool DontMove>
struct WheelMotors {
    using MotorA = MotorTimed<'A', 12, 34, 35, DontMove>;
    using MotorB = MotorTimed<'B', 8, 36, 37, DontMove>;
    using MotorC = MotorTimed<'C', 9, 43, 42, DontMove>;
    using MotorD = MotorTimed<'D', 5, 27, 26, DontMove>;

    static WheelMotors &Instance() {
        return instance;
    }

    void Setup() {
        a.Setup();
        b.Setup();
        c.Setup();
        d.Setup();
    }

    MotorA &A() {
        return a;
    }

    MotorB &B() {
        return b;
    }

    MotorC &C() {
        return c;
    }

    MotorD &D() {
        return d;
    }

    // static void Off() {
    //     A::SetStopped();
    //     B::SetStopped();
    //     C::SetStopped();
    //     D::SetStopped();

    //     Apply();

    //     motors_on = false;
    // }

    void On() {
        a.Apply();
        b.Apply();
        c.Apply();
        d.Apply();
    }

    bool IsAnyMotorOn() {
        return a.IsOn() || b.IsOn() || c.IsOn() || d.IsOn();
    }

    int TotalAbsolutePwm() {
        int pwm =
            abs(a.GetPwm()) +
            abs(b.GetPwm()) +
            abs(c.GetPwm()) +
            abs(d.GetPwm());

        if(pwm > MaxByteVal) {
            pwm = MaxByteVal;
        }

        return pwm;
    }

    TimeUnits LoopProcess(TimeUnits now) {
        now = a.LoopProcess(now);
        now = b.LoopProcess(now);
        now = c.LoopProcess(now);
        now = d.LoopProcess(now);

        return now;
    }

private:
    WheelMotors()
            : a(MotorA::Instance()),
            b(MotorB::Instance()),
            c(MotorC::Instance()),
            d(MotorD::Instance()) {
        // empty
    }

    MotorA &a;
    MotorB &b;
    MotorC &c;
    MotorD &d;

    static WheelMotors instance;
};

template<bool DontMove>
WheelMotors<DontMove> WheelMotors<DontMove>::instance;
