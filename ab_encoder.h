//
// AB motor encoder.
//
// author: alekd
//

#pragma once

#include "scissors_time.h"

template<
    char Name,
    int PinA,
    int PinB,
    int Resolution,
    bool Opposite = false,
    int RpmCalcPeriodMs = 50>
class AbEncoder {
public:
    static void Setup() {
        pinMode(PinA, INPUT);
        pinMode(PinB, INPUT);
        attachInterrupt(digitalPinToInterrupt(PinA), IsrHandler, CHANGE);

        last_rpm_calc_time = ReadTime();
        SetNextRmpCalcTime();

        isr_backward = false;
    }

    static TimeUnits LoopProcess(TimeUnits now) {
        if(!IsTimeAfter(now, next_rpm_calc_time)) {
            return now;
        }

        noInterrupts();
        int count = isr_count;
        isr_count = 0;

        auto backward = isr_backward;
        interrupts();

        auto elapsed = now - last_rpm_calc_time;
        rpm = count * SecInMin * (UsInS / Resolution) / elapsed;

        if(backward) {
            rpm = -rpm;
        }

        last_rpm_calc_time = now;
        SetNextRmpCalcTime();

        // Serial.print(Name);
        // Serial.print(" elapsed: ");
        // Serial.print(elapsed, DEC);
        // Serial.print(" count: ");
        // Serial.print(count, DEC);
        // Serial.print(" rpm: ");
        // Serial.print(rpm, DEC);
        // Serial.print(" next_rpm_calc_time: ");
        // Serial.print(next_rpm_calc_time, DEC);
        // Serial.println("");

        return now;
    }

    static int Rpm() {
        return rpm;
    }

    static void WriteStateToSerial() {
        Serial.print(Name);
        Serial.print(": ");
        Serial.print(rpm, DEC);
        Serial.print(" ");
    }

private:
    //
    // Functions.
    //
    static void IsrHandler() {
        isr_count++;

        auto a_val = digitalRead(PinA);
        auto b_val = digitalRead(PinB);

        if(Opposite) {
            isr_backward = a_val != b_val;
        } else {
            isr_backward = a_val == b_val;
        }
    }

    static void SetNextRmpCalcTime() {
        next_rpm_calc_time = last_rpm_calc_time + RpmCalcPeriodMs * UsInMs;
    }

    //
    // Data.
    //
    static volatile bool isr_backward;
    static volatile int isr_count;

    static TimeUnits last_rpm_calc_time;
    static TimeUnits next_rpm_calc_time;
    static int rpm;
};

template<char Name, int PinA, int PinB, int Resolution,  bool Opposite, int RpmCalcPeriodMs>
volatile bool AbEncoder<Name, PinA, PinB, Resolution, Opposite, RpmCalcPeriodMs>::isr_backward;

template<char Name, int PinA, int PinB, int Resolution,  bool Opposite, int RpmCalcPeriodMs>
volatile int AbEncoder<Name, PinA, PinB, Resolution, Opposite, RpmCalcPeriodMs>::isr_count;

template<char Name, int PinA, int PinB, int Resolution,  bool Opposite, int RpmCalcPeriodMs>
TimeUnits AbEncoder<Name, PinA, PinB, Resolution, Opposite, RpmCalcPeriodMs>::last_rpm_calc_time;

template<char Name, int PinA, int PinB, int Resolution,  bool Opposite, int RpmCalcPeriodMs>
TimeUnits AbEncoder<Name, PinA, PinB, Resolution, Opposite, RpmCalcPeriodMs>::next_rpm_calc_time;

template<char Name, int PinA, int PinB, int Resolution,  bool Opposite, int RpmCalcPeriodMs>
int AbEncoder<Name, PinA, PinB, Resolution, Opposite, RpmCalcPeriodMs>::rpm;
