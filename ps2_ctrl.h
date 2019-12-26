//
// A thin wrapper around PS2X.
//
// author: alekd
//

#pragma once

#include <PS2X_lib.h>

void Ps2Setup(
    bool rumble = true,
    bool pressures = false,
    bool small_vibrate = false,
    byte large_vibrate = 0);

bool Ps2IsReady();

void Ps2Read();

bool Ps2Button(uint16_t button);
bool Ps2ButtonPressed(uint16_t button);

byte Ps2Analog(byte button);

void Ps2SetLargeVibrate(byte data);
void Ps2SetSmallVibrate(bool val);
