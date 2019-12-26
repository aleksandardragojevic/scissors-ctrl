//
// Logging functions.
//
// author: alekd
//

#pragma once

#define LOG_VERBOSE
#ifdef LOG_VERBOSE
#define LogVerbose(...) Serial.print(__VA_ARGS__)
#else
#define Verbose(...)
#endif /* LOG_VERBOSE */

#define LOG_PS2_CTRL
#ifdef LOG_PS2_CTRL
#define LogPs2(...) Serial.print(__VA_ARGS__)
#else
#define LogPs2(...)
#endif /* LOG_PS2_CTRL */
