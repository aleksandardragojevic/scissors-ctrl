//
// The definition of camera servos on the robot.
//
// author: alekd
//

#pragma once

#include "smooth_servo_timed.h"
#include "scissors_time.h"

template<bool DontMove = false>
class CameraServos {
public:
    static CameraServos &Instance() {
        return instance;
    }

    void Setup() {
        lr.Setup();
        ud.Setup();
    }

    void SetMovePeriod(TimeUnits new_move_period) {
        lr.SetMovePeriod(new_move_period);
        ud.SetMovePeriod(new_move_period);
    }

    void Off() {
        lr.Stop();
        ud.Stop();
    }

    void Left(byte step) {
        lr.Up(step);
    }

    void Right(byte step) {
        lr.Down(step);
    }

    void Down(byte step) {
        ud.Up(step);
    }

    void Up(byte step) {
        ud.Down(step);
    }

    TimeUnits LoopProcess(TimeUnits now) {
        now = lr.LoopProcess(now);
        return ud.LoopProcess(now);
    }

    void StopLeftRight() {
        lr.Stop();
    }

    void StopUpDown() {
        ud.Stop();
    }

private:
    //
    // Types.
    //
    using LeftRight = SmoothServoTimed<'Z', 48, 0, 180, DontMove>;
    using UpDown = SmoothServoTimed<'X', 47, 0, 180, DontMove, 0>;

    //
    // Functions.
    //
    CameraServos()
            : lr(LeftRight::Instance()),
            ud(UpDown::Instance()) {
        // empty
    }

    //
    // Data.
    //
    LeftRight &lr;
    UpDown &ud;

    static CameraServos instance;
};

template<bool DontMove>
CameraServos<DontMove> CameraServos<DontMove>::instance;
