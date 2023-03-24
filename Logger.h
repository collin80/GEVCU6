/*
 * Logger.h
 *
 Copyright (c) 2013 Collin Kidder, Michael Neuweiler, Charles Galpin

 Permission is hereby granted, free of charge, to any person obtaining
 a copy of this software and associated documentation files (the
 "Software"), to deal in the Software without restriction, including
 without limitation the rights to use, copy, modify, merge, publish,
 distribute, sublicense, and/or sell copies of the Software, and to
 permit persons to whom the Software is furnished to do so, subject to
 the following conditions:

 The above copyright notice and this permission notice shall be included
 in all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

 */

#ifndef LOGGER_H_
#define LOGGER_H_

#include <Arduino.h>
#include "config.h"
#include "DeviceTypes.h"
#include "constants.h"

class Logger {
public:
    enum LogLevel {
        Debug = 0, Info = 1
    };
    static void debug(const char *, ...);
    static void debug(DeviceId, const char *, ...);
    static void info(const char *, ...);
    static void info(DeviceId, const char *, ...);
    static void console(const char *, ...);
    static LogLevel getLogLevel();
    static uint32_t getLastLogTime();
    static boolean isDebug();
private:
    static LogLevel logLevel;
    static uint32_t lastLogTime;

    static void log(DeviceId, LogLevel, const char *format, va_list);
    static void logMessage(const char *format, va_list args);
    static void printDeviceName(DeviceId);
};

#endif /* LOGGER_H_ */


