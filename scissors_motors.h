//
// Control of the motors.
//
// author: alekd
//

#pragma once

#include "motor.h"
#include "scissors_time.h"

template<bool DontMove>
struct ScissorsMotorsConfig {
    using A = Motor<'A', 12, 34, 35, DontMove>;
    using B = Motor<'B', 8, 36, 37, DontMove>;
    using C = Motor<'C', 9, 43, 42, DontMove>;
    using D = Motor<'D', 5, 27, 26, DontMove>;

    static constexpr TimeUnits MotorChangeMinPeriod = 20 * UsInMs;
    static constexpr TimeUnits MotorsOffDelay = 50 * UsInMs;

    static void Setup() {
        A::Setup();
        B::Setup();
        C::Setup();
        D::Setup();
    }

    static void Off() {
        A::SetStopped();
        B::SetStopped();
        C::SetStopped();
        D::SetStopped();

        Apply();

        motors_on = false;
    }

    static void On() {
        Apply();

        // TODO: check if any motor is on and set motors_on only if they are

        motors_on = true;
    }

    static bool IsAnyMotorOn() {
        return motors_on;
    }

    static int TotalPwm() {
        int pwm = 
            static_cast<int>(A::GetPwm()) +
            static_cast<int>(B::GetPwm()) +
            static_cast<int>(C::GetPwm()) +
            static_cast<int>(D::GetPwm());

        if(pwm > MaxByteVal) {
            pwm = MaxByteVal;
        }

        return pwm;
    }

    static void SetMotorChange() {
        motor_change = true;
    }

    static TimeUnits LoopProcess(TimeUnits now) {
        if(IsTimeAfter(now, motor_change_time)) {
            if(motor_change) {
                On();

                motor_change = false;

                now = ReadTime();
                motor_change_time = now + MotorChangeMinPeriod;
                motor_off_time = now + MotorsOffDelay;
            } else {
                // keep rolling time forward to avoid
                // very large differences in time
                motor_change_time = now;
            }
        }

        if(motors_on && IsTimeAfter(now, motor_off_time)) {
            Off();
            now = ReadTime();
        }

        return now;
    }

private:
    // Apply state to all motors.
    static void Apply() {
        A::Apply();
        B::Apply();
        C::Apply();
        D::Apply();
    }

    // set if any of the motors is on
    static bool motors_on;
    // set if motor change is pending
    static bool motor_change;
    // next time motor change will happen
    static TimeUnits motor_change_time;
    // next time motors will go off
    static TimeUnits motor_off_time;
};

template<bool DontMove>
bool ScissorsMotorsConfig<DontMove>::motors_on;

template<bool DontMove>
bool ScissorsMotorsConfig<DontMove>::motor_change;

template<bool DontMove>
TimeUnits ScissorsMotorsConfig<DontMove>::motor_change_time;

template<bool DontMove>
TimeUnits ScissorsMotorsConfig<DontMove>::motor_off_time;
