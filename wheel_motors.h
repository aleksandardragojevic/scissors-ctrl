//
// Control of the motors.
//
// author: alekd
//

#pragma once

#include "motor_direct_ctrl.h"
#include "motor_speed_ctrl.h"
#include "scissors_time.h"

template<bool DontMove>
struct WheelMotors {
#ifdef USE_DIRECT_CTRL
    using MotorA = MotorDirectCtrl<'A', 12, 34, 35, DontMove>;
    using MotorB = MotorDirectCtrl<'B', 8, 36, 37, DontMove>;
    using MotorC = MotorDirectCtrl<'C', 9, 43, 42, DontMove>;
    using MotorD = MotorDirectCtrl<'D', 5, 27, 26, DontMove>;
#else
    using MotorA = MotorSpeedCtrl<'A', 12, 34, 35, 18, 31, DontMove, 720>;
    using MotorB = MotorSpeedCtrl<'B', 8, 36, 37, 19, 38, DontMove, 720, true>;
    using MotorC = MotorSpeedCtrl<'C', 9, 43, 42, 3, 49, DontMove, 720>;
    using MotorD = MotorSpeedCtrl<'D', 5, 27, 26, 2, 23, DontMove, 720, true>;
#endif /* USE_DIRECT_CTRL */

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
