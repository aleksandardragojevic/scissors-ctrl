//
// Logging functions.
//
// author: alekd
//

#pragma once

#define LOG_VERBOSE
#ifdef LOG_VERBOSE
#define LogVerbose(...) Serial.print(__VA_ARGS__)
#define LogVerboseLn(...) Serial.println(__VA_ARGS__)
#else
#define LogVerbose(...)
#define LogVerboseLn(...)
#endif /* LOG_VERBOSE */

//#define LOG_PS2_CTRL
#ifdef LOG_PS2_CTRL
#define LogPs2(...) Serial.print(__VA_ARGS__)
#define LogPs2Ln(...) Serial.println(__VA_ARGS__)
#else
#define LogPs2(...)
#define LogPs2Ln(...)
#endif /* LOG_PS2_CTRL */

//#define LOG_MOTOR_PID
#ifdef LOG_MOTOR_PID
#define LogMotorPid(...) Serial.print(__VA_ARGS__)
#define LogMotorPidLn(...) Serial.println(__VA_ARGS__)
#else
#define LogMotorPid(...)
#define LogMotorPidLn(...)
#endif /* LOG_MOTOR_PID */
