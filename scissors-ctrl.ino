//
// Controller code for ArduinoMega2560 on MecanumRobot.
//
// author: aleksandar
// note:   The code started and heavily borrows from https://github.com/MoebiusTech/MecanumRobot-ArduinoMega2560.
//

static constexpr bool GoSlow = false;

#include "defs.h"
#include "scissors_time.h"
#include "ps2_ctrl.h"
#include "scissors_motors.h"
#include "log.h"

static constexpr bool DontMove = false;
using ScissorsMotors = ScissorsMotorsConfig<DontMove>;

//
// Constants.
//
static constexpr TimeUnits Ps2CheckPeriod = 10 * UsInMs;
static constexpr TimeUnits SerialCheckPeriod = 10 * UsInMs;

static constexpr byte MotorMinPwm = 20;
static constexpr byte MotorStartMaxPwm = 60;
static constexpr byte MotorMaxPwm = 200;
static constexpr byte MotorCount = 4;
static constexpr float MotorMaxOverFactor = 1.2;

static constexpr byte SlowPwm = 70;

static constexpr byte Ps2MidX = 127;
static constexpr byte Ps2MidY = 128;
static constexpr byte Ps2NormalizeFactor = 127;
static constexpr byte Ps2Tolerance = 5;

//
// Data used in the loop. All times are in micro-seconds.
//
// next time to process ps2
TimeUnits next_ps2_process_time;

// next time to process serial
TimeUnits next_serial_process_time;

// current time
TimeUnits now;

static void ProcessPs2();
static void ProcessSerial();

void setup() {
    Serial.begin(115200);
    Serial.println("Starting Scissors controller");

    // give some time for ps2 module to start
    delay(300);

    Ps2Setup();

    ScissorsMotors::Setup();

    ScissorsMotors::Off();
}

void loop() {
    now = ReadTime();

    if(IsTimeAfter(now, next_ps2_process_time)) {
        ProcessPs2();

        now = ReadTime();
        next_ps2_process_time = now + Ps2CheckPeriod;
    }

    if(IsTimeAfter(now, next_serial_process_time)) {
        ProcessSerial();

        now = ReadTime();
        next_serial_process_time = now + SerialCheckPeriod;
    }

    now = ScissorsMotors::LoopProcess(now);

    // no need to sleep - keep servicing IO as fast as we can
    // and use the per-section time keeping to know when to check them
}

//
// Serial.
//
static void ProcessSerial() {
    // empty
}

//
// Ps2.
//
static byte Ps2CalcPwm(byte stick_pos);
static bool Ps2IsStickMoved(byte val);
static void Ps2Readout();
static void ProcessPs2MotorPairs();
static void ProcessPs2ForwardBackward();
static void ProcessPs2Meccanum();

using Ps2ProcessFun = void (*)();

static Ps2ProcessFun ps2_fun[] = {
    ProcessPs2Meccanum,
    ProcessPs2MotorPairs,
    ProcessPs2ForwardBackward,
};

static constexpr size_t Ps2FunCount = sizeof(ps2_fun) / sizeof(ps2_fun[0]);

static byte ps2_fun_idx = 0;

byte ps2_max_pwm = MotorStartMaxPwm;

static void ProcessPs2() {
    Ps2Readout();

    if(Ps2ButtonPressed(PSB_TRIANGLE)) {
        ps2_max_pwm = min(MotorMaxPwm, ps2_max_pwm + 10);
    }

    if(Ps2ButtonPressed(PSB_CROSS)) {
        ps2_max_pwm = max(MotorMinPwm, ps2_max_pwm - 10);
    }

    if(Ps2ButtonPressed(PSB_SELECT)) {
        if(++ps2_fun_idx == Ps2FunCount) {
            ps2_fun_idx = 0;
        }
    }

    ps2_fun[ps2_fun_idx]();
}

static void Ps2Readout() {
    if(ScissorsMotors::IsAnyMotorOn()) {
        auto total_pwm = ScissorsMotors::TotalPwm();
        Ps2SetLargeVibrate(total_pwm);

        if(total_pwm == MaxByteVal) {
            Ps2SetSmallVibrate(true);

        }
    } else {
        Ps2SetSmallVibrate(false);
        Ps2SetLargeVibrate(0);
    }

    Ps2Read();
}

template<bool SlowDrive>
byte Ps2CalcPwm(byte stick_pos) {
    if(SlowDrive) {
        return SlowPwm;
    }

    auto diff = abs(stick_pos - 127);
    auto ret = diff + MotorMinPwm;

    if(ret > ps2_max_pwm) {
        ret = ps2_max_pwm;
    }

    return ret;
}

static bool Ps2IsStickMoved(byte val) {
    // Ps2MidX and Ps2MidY are so close that it doesn't matter
    return
        val < Ps2MidX - Ps2Tolerance ||
        val > Ps2MidX + Ps2Tolerance;
}

static void ProcessPs2MotorPairs() {
    auto left_y = Ps2Analog(PSS_LY);
    auto right_y = Ps2Analog(PSS_RY);

    if(left_y < Ps2MidY - Ps2Tolerance) {
        auto pwm = Ps2CalcPwm<GoSlow>(left_y);

        ScissorsMotors::A::SetDirectedPwm(pwm);
        ScissorsMotors::C::SetDirectedPwm(pwm);
  
        ScissorsMotors::SetMotorChange();
    } else if(left_y > Ps2MidY + Ps2Tolerance) {
        auto pwm = Ps2CalcPwm<GoSlow>(left_y);

        ScissorsMotors::A::SetDirectedPwm(-pwm);
        ScissorsMotors::C::SetDirectedPwm(-pwm);

        ScissorsMotors::SetMotorChange();
    } else {
        if(!ScissorsMotors::A::IsStopped()) {
            ScissorsMotors::A::SetStopped();
            ScissorsMotors::C::SetStopped();

            ScissorsMotors::SetMotorChange();
        }
    }

    if(right_y < Ps2MidY - Ps2Tolerance) {
        auto pwm = Ps2CalcPwm<GoSlow>(right_y);

        ScissorsMotors::B::SetDirectedPwm(pwm);
        ScissorsMotors::D::SetDirectedPwm(pwm);
  
        ScissorsMotors::SetMotorChange();
    } else if(right_y > Ps2MidY + Ps2Tolerance) {
        auto pwm = Ps2CalcPwm<GoSlow>(right_y);

        ScissorsMotors::B::SetDirectedPwm(-pwm);
        ScissorsMotors::D::SetDirectedPwm(-pwm);
    
        ScissorsMotors::SetMotorChange();
    } else {
        if(!ScissorsMotors::B::IsStopped()) {
            ScissorsMotors::B::SetStopped();
            ScissorsMotors::D::SetStopped();

            ScissorsMotors::SetMotorChange();
        }
    }
}

static void ProcessPs2ForwardBackward() {
    auto left_y = Ps2Analog(PSS_LY);

    if(left_y < Ps2MidY - Ps2Tolerance) {
        ScissorsMotors::A::SetDirectedPwm(SlowPwm);
        ScissorsMotors::B::SetDirectedPwm(SlowPwm);
        ScissorsMotors::C::SetDirectedPwm(SlowPwm);
        ScissorsMotors::D::SetDirectedPwm(SlowPwm);
        ScissorsMotors::SetMotorChange();
    } else if(left_y > Ps2MidY + Ps2Tolerance) {
        ScissorsMotors::A::SetDirectedPwm(-SlowPwm);
        ScissorsMotors::B::SetDirectedPwm(-SlowPwm);
        ScissorsMotors::C::SetDirectedPwm(-SlowPwm);
        ScissorsMotors::D::SetDirectedPwm(-SlowPwm);
        ScissorsMotors::SetMotorChange();
    }
}

static constexpr double Ps2MeccanumRotateFactor = 0.3;

static void ProcessPs2Meccanum() {
    int right_x = Ps2Analog(PSS_RX);
    int right_y = Ps2Analog(PSS_RY);
    int left_x = Ps2Analog(PSS_LX);

    // Translate x and y into "ordinary" coordinate space.
    int rx = Ps2IsStickMoved(right_x) ? right_x - Ps2MidX : 0;
    int ry = Ps2IsStickMoved(right_y) ? Ps2MidY - right_y : 0;
    double lx = Ps2IsStickMoved(left_x) ? left_x - Ps2MidX : 0;
    
    if(!rx & !ry && !lx) {
        return;
    }

    // Calculate intensities. They are normalized to (-2, 2).
    auto angle = atan2(ry, rx);

    auto rxn = static_cast<double>(rx) / Ps2NormalizeFactor;
    auto ryn = static_cast<double>(ry) / Ps2NormalizeFactor;
    auto intensity = sqrt(rxn * rxn + ryn * ryn);

    auto lxn = static_cast<double>(lx) / Ps2NormalizeFactor;
    lxn = lxn * Ps2MeccanumRotateFactor;
    
    auto va = intensity * sin(angle + PI / 4) + lxn;
    auto vb = intensity * sin(angle - PI / 4) - lxn;
    auto vc = intensity * sin(angle - PI / 4) + lxn;
    auto vd = intensity * sin(angle + PI / 4) - lxn;

    // Allow running some motors at more than current maximum pwm
    // if some other motors are not running.
    // We should really read the rotations of motors and control
    // the robot by setting target rotations.
    auto total = abs(va) + abs(vb) + abs(vc) + abs(vd);
    auto total_pwm = MotorCount * ps2_max_pwm;
    auto motor_max_pwm = min(MotorMaxPwm, ps2_max_pwm * MotorMaxOverFactor);

    ScissorsMotors::A::SetBoundedDirectedPwm(
        total_pwm * va / total,
        motor_max_pwm);
    ScissorsMotors::B::SetBoundedDirectedPwm(
        total_pwm * vb / total,
        motor_max_pwm);
    ScissorsMotors::C::SetBoundedDirectedPwm(
        total_pwm * vc / total,
        motor_max_pwm);
    ScissorsMotors::D::SetBoundedDirectedPwm(
        total_pwm * vd / total,
        motor_max_pwm);

    ScissorsMotors::SetMotorChange();
}
