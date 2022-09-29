#ifndef OVAR_CHARGE_H_
#define OVAR_CHARGE_H_

#include <Arduino.h>
#include "config.h"
#include "sys_io.h"
#include "TickHandler.h"
#include "Logger.h"
#include "DeviceManager.h"
#include "FaultHandler.h"
#include "FaultCodes.h"
#include "CanHandler.h"

class OvarChargerConfiguration: public DeviceConfiguration
{
public:
    uint16_t maxAllowedVolts;
    uint16_t maxAllowedCurrent;
};

class OvarCharger: public Device, CanObserver {
public:
    OvarCharger();
    void setup();
    void handleTick();
    void handleCanFrame(CAN_FRAME *frame);
    DeviceId getId();
    DeviceType getType();
    uint32_t getTickInterval();

    void loadConfiguration();
    void saveConfiguration();

protected:

private:
};

#endif
