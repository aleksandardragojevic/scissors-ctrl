//
// Motor integrated in our event loop.
//
// author: aleksandar
//

#include "motor.h"
#include "scissors_time.h"

template<
    char Name,
    byte PinPwm,
    byte PinDir1,
    byte PinDir2,
    bool DontMove,
    int MaxAbsPwm = 240>
class MotorTimed {
public:
    static constexpr TimeUnits MotorChangeMinPeriod = 20 * UsInMs;
    static constexpr TimeUnits MotorsOffDelay = 50 * UsInMs;

    static MotorTimed &Instance() {
        return instance;
    }

    void Setup() {
        motor.Setup();

        change_time = ReadTime();
        changed = false;
    }

    void SetPwm(int val) {
        motor.SetPwm(val);
        changed = true;
    }

    void SetBoundedPwm(int val, int abs_bound) {
        motor.SetBoundedPwm(val, abs_bound);
        changed = true;
    }

    int GetPwm() const {
        return motor.GetPwm();
    }

    bool IsOn() const {
        return !motor.IsStopped();
    }

    TimeUnits LoopProcess(TimeUnits now) {
        if(IsTimeAfter(now, change_time)) {
            if(changed) {
                motor.Apply();

                now = ReadTime();
                changed = false;
                change_time = now + MotorChangeMinPeriod;
                off_time = now + MotorsOffDelay;
            } else {
                // keep rolling time forward to avoid
                // very large differences in time
                change_time = now;
            }
        }

        if(!motor.IsStopped() && IsTimeAfter(now, off_time)) {
            motor.SetStopped();
            motor.Apply();
            now = ReadTime();
        }

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
    MotorTimed()
            : motor(WrappedMotor::Instance()) {
        // empty
    }

    //
    // Data.
    //
    WrappedMotor &motor;

    bool changed;
    // next time motor change will happen
    TimeUnits change_time;
    // next time motors will go off
    TimeUnits off_time;

    static MotorTimed instance;
};

template<char Name, byte PinPwm, byte PinDir1, byte PinDir2, bool DontMove, int MaxAbsPwm>
MotorTimed<Name, PinPwm, PinDir1, PinDir2, DontMove, MaxAbsPwm>
MotorTimed<Name, PinPwm, PinDir1, PinDir2, DontMove, MaxAbsPwm>::instance;

