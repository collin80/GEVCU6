/*
 * adafruitBLE.h
 *
 * Class to interface with adafruit BLE module
 *
 * Created: 7/15/2016 9:10:00 PM
 *  Author: Collin Kidder
 */

/*
 Copyright (c) 2016 Collin Kidder, Michael Neuweiler, Charles Galpin

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

#ifndef ADAFRUITBLE_H_
#define ADAFRUITBLE_H_

#include <Arduino.h>
#include <SPI.h>
#include "config.h"
#include "constants.h"
#include "DeviceManager.h"
#include "PotThrottle.h"
#include "Sys_Messages.h"
#include "Logger.h"
#include "DeviceTypes.h"
#include "ELM327Processor.h"
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BLEGatt.h"
#include "BluefruitConfig.h"
#include "paramcache.h"
//#include "sys_io.h"


extern PrefHandler *sysPrefs;
/*
 * The extended configuration class with additional parameters for ichip WLAN
 */
class BLEConfiguration : public DeviceConfiguration {
public:
};

struct Characteristic {
    int minSize;
    int maxSize;
    uint8_t properties;
    const char *descript;
    GattPresentationFormat present;
};

class ADAFRUITBLE : public Device {
public:

    ADAFRUITBLE();
    void setup(); //initialization on start up    
    void handleTick(); //periodic processes
    void handleMessage(uint32_t messageType, void* message);
    DeviceType getType();
    DeviceId getId();
    void loop();
    char *getTimeRunning();

    void loadConfiguration();
    void saveConfiguration();
    void loadParameters();


private:
    ELM327Processor *elmProc;
    int tickCounter;
    int32_t ServiceId;
    int32_t MeasureCharId[55]; //Keep track and update this to be large enough.
    int32_t LocationCharId;
    boolean success;
    int counter;
    char buffer[30]; // a buffer for various string conversions
    ParamCache paramCache;
    boolean didParamLoad;

    void setupBLEservice();
    void processParameterChange(char *response);
};

#endif


