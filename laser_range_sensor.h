//
// Laser range sensor that uses I2C.
//
// author: aleksandar
//

#pragma once

#include <VL53L0X.h>

class LaserRangeSensor {
public:
    static constexpr int Timeout = 500;
    static constexpr unsigned Infinite = 0xffff;

    void Setup() {
        sensor.setTimeout(Timeout);

        inited = sensor.init();

        if(!inited) {
            Serial.println("Failed to detect and initialize larser range sensor");
        }

        //sensor.setMeasurementTimingBudget(50000);

        sensor.startContinuous();
    }

    unsigned ReadMm() {
        if(!inited) {
            return Infinite;
        }

        return sensor.readRangeContinuousMillimeters();
    }

private:
    VL53L0X sensor;
    bool inited;
};