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
#include "sys_io.h"
#include "PotThrottle.h"
#include "BatteryManager.h"
#include "Sys_Messages.h"
#include "Logger.h"
#include "DeviceTypes.h"
#include "PrefHandler.h"
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BLEGatt.h"
#include "BluefruitConfig.h"
#include "paramcache.h"
//#include "sys_io.h"


extern PrefHandler *sysPrefs;
/*
 * The extended configuration class with additional parameters for Adafruit BLE
 */
class BLEConfiguration : public DeviceConfiguration {
public:
};

struct Characteristic 
{
    BLEDataType_t dataType;
    int minSize;
    int maxSize;
    uint8_t properties;
    const char *descript;
    GattPresentationFormat present;
};

#pragma pack(push, 1)
struct BLETrqReqAct 
{
    int16_t torqueRequested;
    int16_t torqueActual;
    uint8_t doUpdate; //0 = no need to update this struct to the BLE module 1 = need to update onto BLE module
};

struct BLEThrBrkLevels 
{
    uint16_t throttleRawLevel1;
    uint16_t throttleRawLevel2;
    uint16_t brakeRawLevel;
    int8_t throttlePercentage; // -100 to 100 with regen being negative
    int8_t brakePercentage; //always regen
    uint8_t doUpdate; 
};

struct BLESpeeds
{
    int16_t speedRequested;
    int16_t speedActual;
    uint8_t doUpdate; 
};

struct BLEModes
{
    uint8_t powerMode;
    uint8_t gear;
    uint8_t isRunning;
    uint8_t isFaulted;
    uint8_t isWarning;
    uint8_t logLevel;
    uint8_t doUpdate; 
};
    
struct BLEPowerStatus
{
    uint16_t busVoltage;
    int16_t busCurrent;
    int16_t motorCurrent;
    uint16_t kwHours;
    int16_t mechPower;
    uint8_t SOC;
    uint8_t doUpdate; 
};

struct BLEBitfields
{
    uint32_t bitfield1;
    uint32_t bitfield2;
    uint32_t digitalInputs;
    uint32_t digitalOutputs;
    uint8_t doUpdate; 
};

struct BLETemperatures
{
    int16_t motorTemperature;
    int16_t inverterTemperature;
    int16_t systemTemperature;
    uint8_t doUpdate; 
};


struct BLEDigIO
{
    uint16_t prechargeDuration;
    uint8_t prechargeRelay;
    uint8_t mainContRelay;
    uint8_t coolingRelay;
    int8_t coolOnTemp;
    int8_t coolOffTemp;
    uint8_t brakeLightOut;
    uint8_t reverseLightOut;
    uint8_t enableIn;
    uint8_t reverseIn;
    uint8_t doUpdate; 
};

struct BLEThrottleIO
{
    uint16_t throttle1Min;
    uint16_t throttle2Min;
    uint16_t throttle1Max;
    uint16_t throttle2Max;   
    uint8_t numThrottlePots;
    uint8_t throttleType;
    uint8_t doUpdate; 
};

struct BLEThrottleMap
{
    uint16_t throttleRegenMax; //% of pedal where regen is at max setting
    uint16_t throttleRegenMin; //% of pedal with lowest regen
    uint16_t throttleFwd; //% of pedal where forward motion starts
    uint16_t throttleMap; //% of pedal where 50% power is given
    uint8_t throttleLowestRegen; //% of system max regen to use for lowest regen with throttle
    uint8_t throttleHighestRegen; //% of system max regen to use for highest regen with throttle
    uint8_t throttleCreep; //% of forward torque to output for creeping
    uint8_t doUpdate; 
};
    
struct BLEBrakeParam
{
    uint16_t brakeMin;
    uint16_t brakeMax;
    uint8_t brakeRegenMin;
    uint8_t brakeRegenMax;
    uint8_t doUpdate; 
};

struct BLEMaxParams
{
    uint16_t nomVoltage;
    uint16_t maxRPM;
    uint16_t maxTorque;
    uint8_t doUpdate; 
};

//two 32 bit bitfields so that a total of 64 devices can be enabled or disabled from the app
struct BLEDeviceEnable
{
    uint32_t deviceEnable1;
    uint32_t deviceEnable2;
    uint8_t doUpdate; 
};
#pragma pack(pop)


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
    void gattRX(int32_t chars_id, uint8_t *data, uint16_t len);

    void loadConfiguration();
    void saveConfiguration();
    void loadParameters();    

private:
    int tickCounter;
    int32_t ServiceId;
    int32_t MeasureCharId[29]; //Keep track and update this to be large enough.
    int32_t LocationCharId;
    boolean success;
    int counter;
    char buffer[30]; // a buffer for various string conversions
    ParamCache paramCache;
    boolean didParamLoad;
    BLETrqReqAct bleTrqReqAct;
    BLEThrBrkLevels bleThrBrkLevels;
    BLESpeeds bleSpeeds;
    BLEModes bleModes;
    BLEPowerStatus blePowerStatus;
    BLEBitfields bleBitFields;
    BLETemperatures bleTemperatures;
    BLEDigIO bleDigIO;
    BLEThrottleIO bleThrottleIO;
    BLEThrottleMap bleThrottleMap;
    BLEBrakeParam bleBrakeParam;
    BLEMaxParams bleMaxParams;
    BLEDeviceEnable bleDeviceEnable;
    
    void transferUpdates();
    void dumpRawData(uint8_t *data, int len);    
    void buildEnabledDevices();    
    void checkGattChar(uint8_t charact);
    void setupBLEservice();
};

#endif
