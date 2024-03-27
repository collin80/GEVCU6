/*
 * PWMHeater.h - Control a PWM controlled heater plus an additional output to start a pump for water
 *
 Copyright (c) 2022 Collin Kidder, Michael Neuweiler, Charles Galpin

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

#ifndef PWM_HEAT_H_
#define PWM_HEAT_H_

#include <Arduino.h>
#include "config.h"
#include "Throttle.h"
#include "sys_io.h"
#include "TickHandler.h"
#include "Logger.h"
#include "DeviceManager.h"
#include "FaultHandler.h"
#include "FaultCodes.h"
#include <ArduPID.h>


/*
 * The extended configuration class with additional parameters for PotThrottle
 */
class PWMHeaterConfiguration: public DeviceConfiguration {
public:
    uint8_t desiredWaterTemp; //in degrees celsius
    uint8_t pwmPin;
    uint8_t pumpOutputPin;
    uint8_t analogTempPin;
    uint8_t enablePin;
};

class PWMHeater: public Device {
public:
    PWMHeater();
    void setup();
    void handleTick();
    DeviceId getId();

    void loadConfiguration();
    void saveConfiguration();

private:
    bool active;
    int duty;
    ArduPID pidCtrl;
    double adcIn, pwmOut, setpt;
};

#endif


