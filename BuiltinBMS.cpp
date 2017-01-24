/*
 * InternalBMS.cpp
 *
 * Provides the appropriate interface to the pack voltage and current monitoring capabilities of
 * GEVCU6 hardware.
 *
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

#include "BuiltinBMS.h"
#include "sys_io.h"

BuiltinBatteryManager::BuiltinBatteryManager() : BatteryManager() {
    prefsHandler = new PrefHandler(BUILTINBMS);
    allowCharge = false;
    allowDischarge = false;
    commonName = "GEVCU Internal BMS";
    
    packCurrentFiltered = new EMAFilter(20); //respond fairly quickly
    packUpperFilteredVoltage = new EMAFilter(5); //very slow change in value
    packLowerFilteredVoltage = new EMAFilter(5);
    
    lastUpdate = micros();
}

void BuiltinBatteryManager::setup() {
    tickHandler.detach(this);

    Logger::info("add device: Internal BMS (id: %X, %X)", BUILTINBMS, this);
    
    loadConfiguration();

    BatteryManager::setup(); // run the Parent class version of this function

    tickHandler.attach(this, CFG_TICK_INTERVAL_BMS_INTERNAL);
}

void BuiltinBatteryManager::handleTick() {
    BatteryManager::handleTick(); //kick the ball up to papa
    
    if (!config) {
        config = (BuiltinBMSConfiguration *)getConfiguration();
    }
    
    int32_t temp;
    
    //every tick we'll grab the voltages and current and pass them through
    //filters to give us a good value to work with.
    temp = systemIO.getPackHighReading();
    Logger::warn("Pack high: %i", temp);
    packVoltage = packUpperFilteredVoltage->calc(temp);
    temp = systemIO.getPackLowReading();
   Logger::warn("Pack Low: %i", temp);
    packVoltage += packLowerFilteredVoltage->calc(temp);
    Logger::warn("Pack voltage: %i", packVoltage);
    
    packCurrent = packCurrentFiltered->calc(systemIO.getCurrentReading());
    Logger::warn("Pack current: %i", packCurrent);
        
    temp = micros();
    int32_t interval = temp - lastUpdate;
    lastUpdate = temp;
    
    //interval in microseconds, current in hundredths of an amp, the AH remaining variable is in millionths of an AH
    //With current in hundredths of an amp you'd need to divide by 100 to get to amps. Then, the interval is in millionths of a second
    //so, (ampreading / 100) * (interval / 1,000,000) / 60 / 60 = Used amp hours this interval. But, the answer should be in millionths of an AH
    //so multiply by 1,000,000 which cancels the divide by a million above giving:
    //(ampreading / 100) * interval / 3600 = Used microAmpHours this interval. But, to maintain precision take the /100 and /3600 and consolidate:
    //(ampReading * interval) / 360,000 = microampHours
    int32_t usage = (packCurrent * interval) / 360000l;
    int32_t totalCapacity = (config->packCapacity * 1000000l);
    
    if (config->packAHRemaining > usage)  config->packAHRemaining -= usage;
    else config->packAHRemaining = 0;
    if (config->packAHRemaining > totalCapacity) config->packAHRemaining = totalCapacity;
    
    if (totalCapacity > 0) SOC = (100 * config->packAHRemaining) / totalCapacity;
    else SOC = 50; //Divide by zero is naughty! But, set SOC to 50 in that case.

     Logger::warn("AH remaining: %i  SOC:%i", config->packAHRemaining,SOC);
    
}

DeviceId BuiltinBatteryManager::getId()
{
    return (BUILTINBMS);
}

bool BuiltinBatteryManager::hasPackVoltage()
{
    return true;
}

bool BuiltinBatteryManager::hasPackCurrent()
{
    return true;
}

bool BuiltinBatteryManager::hasTemperatures()
{
    return false;
}

bool BuiltinBatteryManager::isChargeOK()
{
    return allowCharge;
}

bool BuiltinBatteryManager::isDischargeOK()
{
    return allowDischarge;
}

void BuiltinBatteryManager::loadConfiguration() {
    config = (BuiltinBMSConfiguration *)getConfiguration();
    
    if (!config) {
        config = new BuiltinBMSConfiguration();
        setConfiguration(config);
    }   

    BatteryManager::loadConfiguration(); // call parent

}

void BuiltinBatteryManager::saveConfiguration() {
    BatteryManager::saveConfiguration(); // call parent
}
