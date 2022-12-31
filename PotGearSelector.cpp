/*
 * PotGearSelector.cpp
 
   Monitor an ADC pin and use it to select which gear we should be in.
 
Copyright (c) 2022 Collin Kidder

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

#include "PotGearSelector.h"

PotGearSelector::PotGearSelector() : Device()
{
    prefsHandler = new PrefHandler(POTGEAR);
    commonName = "Potentiometer Gear Selector";
}

void PotGearSelector::setup()
{
    tickHandler.detach(this);

    Logger::info("add device: Pot Gear Selector (id:%X, %X)", POTGEAR, this);

    loadConfiguration();

    Device::setup(); // run the parent class version of this function

    tickHandler.attach(this, CFG_TICK_INTERVAL_DCDC);
}

void PotGearSelector::handleTick() {
    PotGearSelConfiguration *config = (PotGearSelConfiguration *)getConfiguration();
    MotorController* motorController = (MotorController*) deviceManager.getMotorController();

    Device::handleTick(); //kick the ball up to papa

    if (config->adcPin == 255) return;

    int16_t gearSelector = systemIO.getAnalogIn(config->adcPin);

    bool foundGearPos = false;

    if (gearSelector > (config->parkPosition - config->hysteresis) &&  gearSelector < (config->parkPosition + config->hysteresis))
    {
        Logger::debug("Setting gear to Park(neutral)");
        if (motorController) motorController->setSelectedGear(MotorController::NEUTRAL);
        foundGearPos = true;
    }

    if (gearSelector > (config->neutralPosition - config->hysteresis) &&  gearSelector < (config->neutralPosition + config->hysteresis))
    {
        Logger::debug("Setting gear to neutral");
        if (motorController) motorController->setSelectedGear(MotorController::NEUTRAL);
        foundGearPos = true;
    }

    if (gearSelector > (config->drivePosition - config->hysteresis) &&  gearSelector < (config->drivePosition + config->hysteresis))
    {
        Logger::debug("Setting gear to drive");
        if (motorController) motorController->setSelectedGear(MotorController::DRIVE);
        foundGearPos = true;
    }

    if (gearSelector > (config->reversePosition - config->hysteresis) &&  gearSelector < (config->reversePosition + config->hysteresis))
    {
        Logger::debug("Setting gear to reverse");
        if (motorController) motorController->setSelectedGear(MotorController::REVERSE);
        foundGearPos = true;
    }

    if (!foundGearPos)
    {
        Logger::debug("Gear selector ADC out of bounds! Is it misconfigured?");
    }
}

DeviceId PotGearSelector::getId() {
    return (POTGEAR);
}

uint32_t PotGearSelector::getTickInterval()
{
    return CFG_TICK_INTERVAL_DCDC;
}

DeviceType PotGearSelector::getType()
{
    return DEVICE_MISC;
}

void PotGearSelector::loadConfiguration()
{
    PotGearSelConfiguration *config = (PotGearSelConfiguration *)getConfiguration();

    if (!config) {
        config = new PotGearSelConfiguration();
        setConfiguration(config);
    }

    Device::loadConfiguration(); // call parent

    if (prefsHandler->checksumValid()) { //checksum is good, read in the values stored in EEPROM
        Logger::info((char *)Constants::validChecksum);
        prefsHandler->read(EEGEARSEL_ADC, &config->adcPin);
        prefsHandler->read(EEGEARSEL_PARK, &config->parkPosition);
        prefsHandler->read(EEGEARSEL_REVERSE, &config->reversePosition);
        prefsHandler->read(EEGEARSEL_NEUTRAL, &config->neutralPosition);
        prefsHandler->read(EEGEARSEL_DRIVE, &config->drivePosition);
        prefsHandler->read(EEGEARSEL_HYST, &config->hysteresis);
    }
    else { //checksum invalid. Reinitialize values and store to EEPROM
        Logger::info((char *)Constants::invalidChecksum);
        config->adcPin = 3;
        config->parkPosition = 500;
        config->reversePosition = 1500;
        config->neutralPosition = 2500;
        config->drivePosition = 3500;
        config->hysteresis = 100;
        saveConfiguration();
    }
}

void PotGearSelector::saveConfiguration()
{
    PotGearSelConfiguration *config = (PotGearSelConfiguration *)getConfiguration();

    Device::saveConfiguration();

    prefsHandler->write(EEGEARSEL_ADC, config->adcPin);
    prefsHandler->write(EEGEARSEL_PARK, config->parkPosition);
    prefsHandler->write(EEGEARSEL_REVERSE, config->reversePosition);
    prefsHandler->write(EEGEARSEL_NEUTRAL, config->neutralPosition);
    prefsHandler->write(EEGEARSEL_DRIVE, config->drivePosition);
    prefsHandler->write(EEGEARSEL_HYST, config->hysteresis);
    prefsHandler->saveChecksum();
    prefsHandler->forceCacheWrite();
}
