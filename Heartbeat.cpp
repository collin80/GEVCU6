/*
 * Heartbeat.c
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

#include "Heartbeat.h"
#include "BatteryManager.h"

Heartbeat::Heartbeat() {
    led = false;
    throttleDebug = false;
}

void Heartbeat::setup() {
    tickHandler.detach(this);

    tickHandler.attach(this, CFG_TICK_INTERVAL_HEARTBEAT);
}

void Heartbeat::setThrottleDebug(bool debug) {
    throttleDebug = debug;
}

bool Heartbeat::getThrottleDebug() {
    return throttleDebug;
}

void Heartbeat::handleTick() {
    // Print a dot if no other output has been made since the last tick
    if (Logger::getLastLogTime() < lastTickTime) {
        SerialUSB.print('.');
        if ((++dotCount % 80) == 0) {
            SerialUSB.println();
        }
    }
    lastTickTime = millis();

    if (led) {
        digitalWrite(BLINK_LED, HIGH);
    } else {
        digitalWrite(BLINK_LED, LOW);
    }
    led = !led;

    if (throttleDebug) {
        MotorController *motorController = deviceManager.getMotorController();
        Throttle *accelerator = deviceManager.getAccelerator();
        Throttle *brake = deviceManager.getBrake();

        Logger::console("");
        if (motorController) {
            Logger::console("Motor Controller Status->       isRunning: %T               isFaulted: %T", motorController->isRunning(), motorController->isFaulted());
        }

        Logger::console("AIN0: %i, AIN1: %i, AIN2: %i, AIN3: %i", 
                        systemIO.getAnalogIn(0), systemIO.getAnalogIn(1), systemIO.getAnalogIn(2), systemIO.getAnalogIn(3));
        Logger::console("DIN0: %d, DIN1: %d, DIN2: %d, DIN3: %d", 
                        systemIO.getDigitalIn(0), systemIO.getDigitalIn(1), systemIO.getDigitalIn(2), systemIO.getDigitalIn(3));
        Logger::console("DOUT0: %d, DOUT1: %d, DOUT2: %d, DOUT3: %d,DOUT4: %d, DOUT5: %d, DOUT6: %d, DOUT7: %d", 
                        systemIO.getDigitalOutput(0), systemIO.getDigitalOutput(1), systemIO.getDigitalOutput(2), systemIO.getDigitalOutput(3),
                        systemIO.getDigitalOutput(4), systemIO.getDigitalOutput(5), systemIO.getDigitalOutput(6), systemIO.getDigitalOutput(7));
        BatteryManager *bms = reinterpret_cast<BatteryManager *>(deviceManager.getDeviceByType(DEVICE_BMS));
        if (!bms) {
            Logger::console("BATTHI: %d, BATTLO: %d, BATTERY: %d, CURRENT: %d", 
                        systemIO.getPackHighReading(), systemIO.getPackLowReading(), systemIO.getPackHighReading()+systemIO.getPackLowReading(),
                        systemIO.getCurrentReading());
        }
        else {
            Logger::console( "HV Batt Voltage: %d, HV Current: %d, SOC: %i",
                        bms->getPackVoltage(), bms->getPackCurrent(), bms->getSOC() );            
        }

        if (accelerator) {
            Logger::console("Throttle is Faulted:%T", accelerator->isFaulted());
            Logger::console("Raw throttle torque level: %i", accelerator->getLevel());            
            RawSignalData *rawSignal = accelerator->acquireRawSignal();
            Logger::console("Throttle rawSignal1: %d, rawSignal2: %d", rawSignal->input1, rawSignal->input2);
        }
        if (brake) {
            Logger::console("Brake Output: %i", brake->getLevel());
            RawSignalData *rawSignal = brake->acquireRawSignal();
            Logger::console("Brake rawSignal1: %d", rawSignal->input1);
        }
    }
}



