//
// Moved the ps2 logic to a separate compilation unit.
//
// author: aleksadar
//

#include "Arduino.h"

#include "ps2_ctrl.h"

// pin definitions
#define PS2_DAT 52
#define PS2_CMD 51
#define PS2_SEL 53
#define PS2_CLK 50

// local variables
static PS2X ps2x;

static bool ps2_ready;
static bool ps2_rumble;
static bool ps2_pressures;
static bool ps2_small_vibrate;
static byte ps2_large_vibrate;

// Mystery: I don't understand why declaring and invoking this function in case
//          of an error makes things work. If anything, I would think this would
//          not be great.
static void (*ps2_reset_func) (void) = 0;

// local function declarations
static void SetVariables(
        bool rumble,
        bool pressures,
        bool small_vibrate,
        byte large_vibrate);
static void DoSetup();
static void SerialOutputBool(bool val);

//
// Implementation.
//
void Ps2Setup(
        bool rumble,
        bool pressures,
        bool small_vibrate,
        byte large_vibrate) {
    SetVariables(rumble, pressures, small_vibrate, large_vibrate);
    DoSetup();
}

static void SetVariables(
        bool rumble,
        bool pressures,
        bool small_vibrate,
        byte large_vibrate) {
    ps2_rumble = rumble;
    ps2_pressures = pressures;
    ps2_small_vibrate = small_vibrate;
    ps2_large_vibrate = large_vibrate;
}

static void DoSetup() {
  byte error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, ps2_pressures, ps2_rumble);

  if (error == 0) {
    Serial.print("Found Controller, configured successful ");
    Serial.print("pressures = ");
    if (ps2_pressures)
      Serial.print("true ");
    else
      Serial.print("false ");
    Serial.print("rumble = ");
    if (ps2_rumble)
      Serial.println("true");
    else
      Serial.println("false");
  } else if (error == 1)
  {
    Serial.println("No controller found, check wiring, see readme.txt to enable debug. visit www.billporter.info for troubleshooting tips");   
    ps2_reset_func();
  }

  else if (error == 2)
    Serial.println("Controller found but not accepting commands. see readme.txt to enable debug. Visit www.billporter.info for troubleshooting tips");

  else if (error == 3)
    Serial.println("Controller refusing to enter Pressures mode, may not support it. ");

//  Serial.print(ps2x.Analog(1), HEX);

  byte type = 0;

    if(!error) {
        type = ps2x.readType();
        switch (type) {
        case 0:
            Serial.print("Unknown Controller type found ");
            break;
        case 1:
            Serial.print("DualShock Controller found ");
            break;
        case 2:
            Serial.print("GuitarHero Controller found ");
            break;
        case 3:
            Serial.print("Wireless Sony DualShock Controller found ");
            break;
        }
    }

    ps2_ready = (error == 0) && (type == 1);
}

bool Ps2IsReady() {
    return ps2_ready;
}

void Ps2Read() {
    ps2x.read_gamepad(ps2_small_vibrate, ps2_large_vibrate);
}

bool Ps2Button(uint16_t button) {
    return ps2x.Button(button);
}

bool Ps2ButtonPressed(uint16_t button) {
    return ps2x.ButtonPressed(button);
}

byte Ps2Analog(byte button) {
    return ps2x.Analog(button);
}

void Ps2SetLargeVibrate(byte data) {
    ps2_large_vibrate = data;
}

void Ps2SetSmallVibrate(bool val) {
    ps2_small_vibrate = val;
}
