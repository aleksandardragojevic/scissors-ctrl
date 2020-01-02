//
// Smooth servo integrated in our event loop.
//
// author: aleksandar
//

#include "smooth_servo.h"
#include "scissors_time.h"

template<
    char Name,
    int Pin,
    int MinPosition,
    int MaxPosition,
    bool DontMove,
    int StartPosition = (MinPosition + MaxPosition) / 2,
    int PwmCycleUs = 20 * UsInMs>
class SmoothServoTimed {
public:
    static constexpr TimeUnits MoveMinPeriod = 20 * UsInMs;
    // This is used to control servos when they are moved up or down, not towards
    // a specific goal.
    static constexpr TimeUnits OffDelay = 50 * UsInMs;

    static SmoothServoTimed &Instance() {
        return instance;
    }

    void Setup() {
        smooth_servo.Setup();

        move_period = MoveMinPeriod;
        is_off_time_set = false;

        SetMoveTime();
    }

    void SetMovePeriod(TimeUnits new_move_period) {
        move_period = new_move_period;
    }

    void Stop() {
        smooth_servo.Stop();
    }

    void Up(byte step) {
        smooth_servo.Up(step);
        SetOffTimeout();
    }

    void Down(byte step) {
        smooth_servo.Down(step);
        SetOffTimeout();
    }

    TimeUnits LoopProcess(TimeUnits now) {
        if(is_off_time_set && IsTimeAfter(now, off_time)) {
            Stop();
            is_off_time_set = false;
        }

        if(IsTimeAfter(now, move_time)) {
            smooth_servo.Move();

            move_time = now + move_period;

            now = ReadTime();
        }

        return now;
    }

private:
    //
    // Types.
    //
    using WrappedServo = SmoothServo<
        Name,
        Pin,
        MinPosition,
        MaxPosition,
        DontMove,
        StartPosition,
        PwmCycleUs>;

    //
    // Functions.
    //
    SmoothServoTimed()
            : smooth_servo(WrappedServo::Instance()) {
        // empty
    }

    void SetMoveTime() {
        SetMoveTime(ReadTime());
    }

    void SetMoveTime(TimeUnits now) {
        move_time = now + move_period;
    }

    void SetOffTimeout() {
        is_off_time_set = true;
        off_time = ReadTime() + OffDelay;
    }
    
    //
    // Data.
    //
    WrappedServo &smooth_servo;
    TimeUnits move_period;
    TimeUnits move_time;
    bool is_off_time_set;
    TimeUnits off_time;

    static SmoothServoTimed instance;
};

template<
    char Name,
    int Pin,
    int MinPosition,
    int MaxPosition,
    bool DontMove,
    int StartPosition,
    int PwmCycleUs>
SmoothServoTimed<Name, Pin, MinPosition, MaxPosition, DontMove, StartPosition, PwmCycleUs>
SmoothServoTimed<Name, Pin, MinPosition, MaxPosition, DontMove, StartPosition, PwmCycleUs>::instance;