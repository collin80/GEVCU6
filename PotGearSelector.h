#ifndef POT_GEARSEL_H_
#define POT_GEARSEL_H_

#include <Arduino.h>
#include "config.h"
#include "sys_io.h"
#include "TickHandler.h"
#include "Logger.h"
#include "DeviceManager.h"
#include "FaultHandler.h"
#include "FaultCodes.h"

class PotGearSelConfiguration: public DeviceConfiguration
{
public:
    uint8_t adcPin;
    uint16_t hysteresis;
    uint16_t parkPosition;
    uint16_t drivePosition;
    uint16_t reversePosition;
    uint16_t neutralPosition;
};

class PotGearSelector: public Device {
public:
    PotGearSelector();
    void setup();
    void handleTick();
    DeviceId getId();
    DeviceType getType();
    uint32_t getTickInterval();

    void loadConfiguration();
    void saveConfiguration();

protected:

private:
    uint32_t positionAccum;
    int counter;
};

#endif
