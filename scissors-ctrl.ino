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
#include "wheel_motors.h"
#include "camera_servos.h"
#include "ab_encoder.h"
#include "log.h"

static constexpr bool DontMoveMotors = false;
auto &motors = WheelMotors<DontMoveMotors>::Instance();

static constexpr bool DontMoveCameraServos = false;
auto &camera_servos = CameraServos<DontMoveCameraServos>::Instance();

//
// Constants.
//
static constexpr TimeUnits Ps2CheckPeriod = 10 * UsInMs;
static constexpr TimeUnits SerialCheckPeriod = 10 * UsInMs;

static constexpr byte MotorMinPwm = 20;
static constexpr byte MotorStartMaxPwm = 60;
static constexpr byte MotorMaxPwm = 240;
static constexpr byte MotorPwmStepIncrement = 10;
static constexpr byte MotorCount = 4;
static constexpr float MotorMaxOverFactor = 1.2;

static constexpr byte SlowPwm = 70;

static constexpr byte CamerServoMinStep = 1;
static constexpr byte CamerServoMaxStep = 180;
static constexpr byte CamerServoDefaultStep = 2;
static constexpr byte CamerServoStepIncrement = 1;

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

static void PrintStats();
template<typename Motor>
static void PrintMotorStats(const Motor &motor);

void setup() {
    Serial.begin(115200);
    Serial.println("Starting Scissors controller");

    // give some time for ps2 module to start
    delay(300);

    Ps2Setup();

    motors.Setup();

    camera_servos.Setup();



    // Serial.print("current-rpm ");
    // Serial.print("target-rpm ");
    // Serial.print("pwm ");
    Serial.print("err ");
    // Serial.print("err-sum ");
    Serial.print("delta-p ");
    Serial.print("delta-i ");
    Serial.print("delta-d ");
    Serial.print("delta ");
    Serial.println("");
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

    now = motors.LoopProcess(now);

    now = camera_servos.LoopProcess(now);

    PrintStats();

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
static void ProcessPs2CameraServos();
static void ProcessPs2WheelMotors();
static void ProcessPs2MotorPairs();
static void ProcessPs2ForwardBackward();
static void ProcessPs2Meccanum();
static int BoundedValue(int val, int abs_bound);

using Ps2ProcessFun = void (*)();

static Ps2ProcessFun ps2_fun[] = {
    ProcessPs2Meccanum,
    ProcessPs2MotorPairs,
    ProcessPs2ForwardBackward,
};

static constexpr size_t Ps2FunCount = sizeof(ps2_fun) / sizeof(ps2_fun[0]);

static byte ps2_fun_idx = 0;
static byte ps2_max_pwm = MotorStartMaxPwm;

static byte cam_servo_step = CamerServoDefaultStep;

static void ProcessPs2() {
    Ps2Readout();

    ProcessPs2WheelMotors();

    ProcessPs2CameraServos();
}

static void Ps2Readout() {
    auto motor_pwm = motors.TotalAbsolutePwm();

    if(motor_pwm) {
        Ps2SetLargeVibrate(motor_pwm);

        if(motor_pwm == MaxByteVal) {
            Ps2SetSmallVibrate(true);

        }
    } else {
        Ps2SetSmallVibrate(false);
        Ps2SetLargeVibrate(0);
    }

    Ps2Read();
}

static void ProcessPs2CameraServos() {
    if(Ps2ButtonPressed(PSB_L1)) {
        cam_servo_step = min(
            CamerServoMaxStep,
            cam_servo_step + CamerServoStepIncrement);
    }

    if(Ps2ButtonPressed(PSB_L2)) {
        cam_servo_step = max(
            CamerServoMinStep,
            cam_servo_step - CamerServoStepIncrement);
    }

    if(Ps2Button(PSB_PAD_LEFT)) {
        camera_servos.Left(cam_servo_step);
    } else if(Ps2Button(PSB_PAD_RIGHT)) {
        camera_servos.Right(cam_servo_step);
    } else {
        camera_servos.StopLeftRight();
    }

    if(Ps2Button(PSB_PAD_UP)) {
        camera_servos.Up(cam_servo_step);
    } else if(Ps2Button(PSB_PAD_DOWN)) {
        camera_servos.Down(cam_servo_step);
    } else {
        camera_servos.StopUpDown();
    }
}

static void ProcessPs2WheelMotors() {
    if(Ps2ButtonPressed(PSB_R1)) {
        ps2_max_pwm = min(MotorMaxPwm, ps2_max_pwm + MotorPwmStepIncrement);
    }

    if(Ps2ButtonPressed(PSB_R2)) {
        ps2_max_pwm = max(MotorMinPwm, ps2_max_pwm - MotorPwmStepIncrement);
    }

    if(Ps2ButtonPressed(PSB_SELECT)) {
        if(++ps2_fun_idx == Ps2FunCount) {
            ps2_fun_idx = 0;
        }

        Serial.print("Changed ps2_fun index to ");
        Serial.println(ps2_fun_idx, DEC);
    }

    ps2_fun[ps2_fun_idx]();
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

        motors.A().SetTarget(pwm);
        motors.C().SetTarget(pwm);
    } else if(left_y > Ps2MidY + Ps2Tolerance) {
        auto pwm = Ps2CalcPwm<GoSlow>(left_y);

        motors.A().SetTarget(-pwm);
        motors.C().SetTarget(-pwm);
    } else {
        motors.A().SetTarget(0);
        motors.C().SetTarget(0);
    }

    if(right_y < Ps2MidY - Ps2Tolerance) {
        auto pwm = Ps2CalcPwm<GoSlow>(right_y);

        motors.B().SetTarget(pwm);
        motors.D().SetTarget(pwm);
    } else if(right_y > Ps2MidY + Ps2Tolerance) {
        auto pwm = Ps2CalcPwm<GoSlow>(right_y);

        motors.B().SetTarget(-pwm);
        motors.D().SetTarget(-pwm);
    } else {
        motors.B().SetTarget(0);
        motors.D().SetTarget(0);
    }
}

static void ProcessPs2ForwardBackward() {
    auto left_y = Ps2Analog(PSS_RY);

    if(left_y < Ps2MidY - Ps2Tolerance) {
        motors.A().SetTarget(SlowPwm);
        motors.B().SetTarget(SlowPwm);
        motors.C().SetTarget(SlowPwm);
        motors.D().SetTarget(SlowPwm);
    } else if(left_y > Ps2MidY + Ps2Tolerance) {
        motors.A().SetTarget(-SlowPwm);
        motors.B().SetTarget(-SlowPwm);
        motors.C().SetTarget(-SlowPwm);
        motors.D().SetTarget(-SlowPwm);
    } else {
        motors.A().SetTarget(0);
        motors.B().SetTarget(0);
        motors.C().SetTarget(0);
        motors.D().SetTarget(0);
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
    
    // Calculate intensities. They are normalized.
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

    motors.A().SetTarget(BoundedValue(total_pwm * va / total, motor_max_pwm));
    motors.B().SetTarget(BoundedValue(total_pwm * vb / total, motor_max_pwm));
    motors.C().SetTarget(BoundedValue(total_pwm * vc / total, motor_max_pwm));
    motors.D().SetTarget(BoundedValue(total_pwm * vd / total, motor_max_pwm));
}

static int BoundedValue(int val, int abs_bound) {
    if(val < 0) {
        return max(val, -abs_bound);
    }

    return min(val, abs_bound);
}

//
// Stats.
//
static void PrintStats() {
    // PrintMotorStats(motors.A());
    // Serial.print(" ");
    // PrintMotorStats(motors.B());
    // Serial.print(" ");
    // PrintMotorStats(motors.C());
    // Serial.print(" ");
    // PrintMotorStats(motors.D());

    //Serial.println("");
}

template<typename Motor>
static void PrintMotorStats(const Motor &motor) {
    Serial.print(motor.GetName());
    Serial.print(" rpm: ");
    Serial.print(motor.CurrentRpm(), DEC);
    Serial.print(" target: ");
    Serial.print(motor.TargetRpm(), DEC);
    Serial.print(" pwm: ");
    Serial.print(motor.CurrentPwm(), DEC);
}
