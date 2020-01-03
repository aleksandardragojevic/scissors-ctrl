//
// Motor that is controlled directly by setting its pwm.
//
// author: aleksandar
//

#pragma once

#include "motor.h"
#include "scissors_time.h"

template<
    char Name,
    byte PinPwm,
    byte PinDir1,
    byte PinDir2,
    bool DontMove,
    int MaxAbsPwm = 240>
class MotorDirectCtrl {
public:
    static constexpr TimeUnits MotorChangeMinPeriod = 20 * UsInMs;

    static MotorDirectCtrl &Instance() {
        return instance;
    }

    void Setup() {
        motor.Setup();

        change_time = ReadTime();
    }

    void SetTarget(int val) {
        motor.SetPwm(val);
    }

    int GetPwm() const {
        return motor.GetPwm();
    }

    bool IsOn() const {
        return !motor.IsStopped();
    }

    TimeUnits LoopProcess(TimeUnits now) {
        if(!IsTimeAfter(now, change_time)) {
            return;
        }

        motor.Apply();

        now = ReadTime();
        change_time = now + MotorChangeMinPeriod;

        return now;
    }

private:
    //
    // Types.
    //
    using WrappedMotor = Motor<
        Name,
        PinPwm,
        PinDir1,
        PinDir2,
        DontMove,
        MaxAbsPwm>;

    //
    // Functions.
    //
    MotorDirectCtrl()
            : motor(WrappedMotor::Instance()) {
        // empty
    }

    //
    // Data.
    //
    WrappedMotor &motor;

    TimeUnits change_time;

    static MotorDirectCtrl instance;
};

template<char Name, byte PinPwm, byte PinDir1, byte PinDir2, bool DontMove, int MaxAbsPwm>
MotorDirectCtrl<Name, PinPwm, PinDir1, PinDir2, DontMove, MaxAbsPwm>
MotorDirectCtrl<Name, PinPwm, PinDir1, PinDir2, DontMove, MaxAbsPwm>::instance;

