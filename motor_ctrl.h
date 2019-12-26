//
// Control of the motors.
//
// author: alekd
//

#pragma once

#include "motor.h"

static constexpr bool DontMove = false;

using MotorA = Motor<'A', 12, 34, 35, DontMove>;
using MotorB = Motor<'B', 8, 36, 37, DontMove>;
using MotorC = Motor<'C', 9, 43, 42, DontMove>;
using MotorD = Motor<'D', 5, 27, 26, DontMove>;

#define MAX_PWM 200
#define MIN_PWM 130
int Motor_PWM = 130;

//    ↑A-----B↑   
//     |  ↑  |
//     |  |  |
//    ↑C-----D↑
void ADVANCE() {
    MotorA::Forward(Motor_PWM);MotorB::Forward(Motor_PWM);
    MotorC::Forward(Motor_PWM);MotorD::Forward(Motor_PWM);
}

//    ↓A-----B↓   
//     |  |  |
//     |  ↓  |
//    ↓C-----D↓
void BACK() {
    MotorA::Backward(Motor_PWM);MotorB::Backward(Motor_PWM);
    MotorC::Backward(Motor_PWM);MotorD::Backward(Motor_PWM);
}

//    =A-----B↑   
//     |   ↖ |
//     | ↖   |
//    ↑C-----D=
void LEFT_1() {
    MotorA::Stop();MotorB::Forward(Motor_PWM);
    MotorC::Forward(Motor_PWM);MotorD::Stop();
}

//    ↓A-----B↑   
//     |  ←  |
//     |  ←  |
//    ↑C-----D↓
void LEFT_2() {
    MotorA::Backward(Motor_PWM);MotorB::Forward(Motor_PWM);
    MotorC::Forward(Motor_PWM);MotorD::Backward(Motor_PWM);
}

//    ↓A-----B=   
//     | ↙   |
//     |   ↙ |
//    =C-----D↓
void LEFT_3() {
    MotorA::Backward(Motor_PWM);MotorB::Stop();
    MotorC::Stop();MotorD::Backward(Motor_PWM);
}

//    ↑A-----B=   
//     | ↗   |
//     |   ↗ |
//    =C-----D↑
void RIGHT_1() {
    MotorA::Forward(Motor_PWM);MotorB::Stop();
    MotorC::Stop();MotorD::Forward(Motor_PWM);
}

//    ↑A-----B↓   
//     |  →  |
//     |  →  |
//    ↓C-----D↑
void RIGHT_2() {
    MotorA::Forward(Motor_PWM);MotorB::Backward(Motor_PWM);
    MotorC::Backward(Motor_PWM);MotorD::Forward(Motor_PWM);
}

//    =A-----B↓   
//     |   ↘ |
//     | ↘   |
//    ↓C-----D=
void RIGHT_3() {
    MotorA::Stop();MotorB::Backward(Motor_PWM);
    MotorC::Backward(Motor_PWM);MotorD::Stop();
}

//    =A-----B=  
//     |  =  |
//     |  =  |
//    =C-----D=
void STOP() {
    MotorA::Stop();MotorB::Stop();
    MotorC::Stop();MotorD::Stop();
}

void MotorSetup() {
    MotorA::Setup();
    MotorB::Setup();
    MotorC::Setup();
    MotorD::Setup();
}

