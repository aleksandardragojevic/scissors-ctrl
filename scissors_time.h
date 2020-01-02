//
// Utilities related to time.
//
// author: aleksandar
//

#pragma once

using TimeUnits = unsigned long;

static constexpr TimeUnits UsInS = 1000000ul;
static constexpr TimeUnits UsInMs = 1000ul;
static constexpr TimeUnits SecInMin = 60ul;

static constexpr TimeUnits VeryLongPeriod = 120 * UsInS;

inline TimeUnits ReadTime() {
    return micros();
}

inline bool IsTimeAfter(TimeUnits time, TimeUnits ref) {
    // account for clock wrap around which occurs every ~70 minutes
    return (time > ref) && (time - ref < VeryLongPeriod);
}
