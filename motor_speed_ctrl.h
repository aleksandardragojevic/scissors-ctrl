//
// Motor where the speed is set directly.
//
// author: aleksandar
//

#pragma once

#include "motor.h"
#include "ab_encoder.h"
#include "scissors_time.h"

template<
    char Name,
    byte PinPwm,
    byte PinDir1,
    byte PinDir2,
    byte PinA,
    byte PinB,
    bool DontMove,
    int EncoderResolution,
    bool Opposite = false,
    int MaxAbsPwm = 240>
class MotorSpeedCtrl {
public:
    static constexpr TimeUnits Period = 50 * UsInMs;

    static MotorSpeedCtrl &Instance() {
        return instance;
    }

    void Setup() {
        motor.Setup();
        encoder.Setup();

        target_rpm = 0;
        err_sum = 0;
        err_last = 0;

        SetChangeTimes(ReadTime());
    }

    TimeUnits LoopProcess(TimeUnits now) {
        if(!IsTimeAfter(now, change_time)) {
            return now;
        }

        auto elapsed = now - last_change_time;

        encoder.CalculateRpm(elapsed);

        int pwm;
        int err = 0;
        int pwm_p = 0;
        int pwm_i = 0;
        int pwm_d = 0;

        if(target_rpm) {
            err = target_rpm - encoder.Rpm();
            err_sum += err;

            pwm_p = err * KP;
            pwm_i = err_sum * KI;

            if(err_last) {
                pwm_d = (err - err_last) * KD;
            }

            pwm = pwm_p + pwm_i + pwm_d;

            err_last = err;
        } else {
            pwm = 0;
            err_sum = 0;
            err_last = 0;
        }

        // debug stats
        // Serial.print(CurrentRpm(), DEC);
        // Serial.print(" ");
        // Serial.print(TargetRpm(), DEC);
        // Serial.print(" ");
        // Serial.print(CurrentPwm(), DEC);
        // Serial.print(" ");
        Serial.print(err, DEC);
        Serial.print(" ");
        // Serial.print(GetErrSum(), DEC);
        // Serial.print(" ");
        Serial.print(pwm_p, DEC);
        Serial.print(" ");
        Serial.print(pwm_i, DEC);
        Serial.print(" ");
        Serial.print(pwm_d, DEC);
        Serial.print(" ");
        Serial.print(pwm, DEC);
        Serial.print(" ");
        Serial.println("");
        // debug stats

        motor.SetPwm(pwm);
        motor.Apply();

        SetChangeTimes(now);

        return now;
    }

    void SetTarget(int val) {
        target_rpm = val;
    }

    //
    // Read state.
    //
    int CurrentRpm() const {
        return encoder.Rpm();
    }

    int TargetRpm() const {
        return target_rpm;
    }

    int CurrentPwm() const {
        return motor.GetPwm();
    }

    int GetPwm() const {
        return motor.GetPwm();
    }

    int GetErrSum() const {
        return err_sum;
    }

    static char GetName() {
        return Name;
    }

private:
    //
    // Constants.
    //
    static constexpr float KP = 0.4;
    static constexpr float KI = 0.22;
    static constexpr float KD = 0.3;

    //
    // Types.
    //
    using WrappedMotor = Motor<Name, PinPwm, PinDir1, PinDir2, DontMove, MaxAbsPwm>;
    using WrappedEncoder = AbEncoder<PinA, PinB, EncoderResolution, Opposite>;

    //
    // Functions.
    //
    MotorSpeedCtrl()
            : motor(WrappedMotor::Instance()),
            encoder(WrappedEncoder::Instance()) {
        // empty
    }

    void SetChangeTimes(TimeUnits now) {
        last_change_time = now;
        change_time = now + Period;
    }

    //
    // Data.
    //
    WrappedMotor &motor;
    WrappedEncoder &encoder;

    int target_rpm;
    TimeUnits last_change_time;
    TimeUnits change_time;

    int err_sum;
    int err_last;

    static MotorSpeedCtrl instance;
};

template<
    char Name,
    byte PinPwm,
    byte PinDir1,
    byte PinDir2,
    byte PinA,
    byte PinB,
    bool DontMove,
    int EncoderResolution,
    bool Opposite,
    int MaxAbsPwm>
MotorSpeedCtrl<Name, PinPwm, PinDir1, PinDir2, PinA, PinB, DontMove, EncoderResolution, Opposite, MaxAbsPwm>
MotorSpeedCtrl<Name, PinPwm, PinDir1, PinDir2, PinA, PinB, DontMove, EncoderResolution, Opposite, MaxAbsPwm>::instance;
