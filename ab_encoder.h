//
// AB motor encoder. Outputs rotations per minute.
//
// author: alekd
//

#pragma once

#include "scissors_time.h"

template<
    byte PinA,
    byte PinB,
    int Resolution,
    bool Opposite = false>
class AbEncoder {
public:
    static AbEncoder &Instance() {
        return instance;
    }

    void Setup() {
        pinMode(PinA, INPUT);
        pinMode(PinB, INPUT);
        attachInterrupt(digitalPinToInterrupt(PinA), IsrHandler, CHANGE);

        isr_backward = false;
    }

    void CalculateRpm(TimeUnits elapsed) {
        noInterrupts();
        int count = isr_count;
        isr_count = 0;

        auto backward = isr_backward;
        interrupts();

        rpm = count * SecInMin * (UsInS / Resolution) / elapsed;

        if(backward) {
            rpm = -rpm;
        }
    }

    int Rpm() {
        return rpm;
    }

private:
    //
    // Functions.
    //
    AbEncoder() = default;

    static void IsrHandler() {
        Instance().DoIsrHandler();
    }

    void DoIsrHandler() {
        isr_count++;

        auto a_val = digitalRead(PinA);
        auto b_val = digitalRead(PinB);

        if(Opposite) {
            isr_backward = a_val != b_val;
        } else {
            isr_backward = a_val == b_val;
        }
    }

    //
    // Data.
    //
    volatile bool isr_backward;
    volatile int isr_count;

    int rpm;

    static AbEncoder instance;
};

template<byte PinA, byte PinB, int Resolution,  bool Opposite>
AbEncoder<PinA, PinB, Resolution, Opposite>
AbEncoder<PinA, PinB, Resolution, Opposite>::instance;
