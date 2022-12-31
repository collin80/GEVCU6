/*
 * OvarChargerDCDC.cpp
 
   Driver to turn on an Ovar Charger / DCDC combo unit and set it to reasonable values for the car
 
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

#include "OvarChargerDCDC.h"

OvarCharger::OvarCharger() : Device()
{
    prefsHandler = new PrefHandler(OVARCHARGE);
    commonName = "Ovar Charger and DC/DC";
}

void OvarCharger::setup()
{
    tickHandler.detach(this);

    Logger::info("add device: Ovar Charger (id:%X, %X)", OVARCHARGE, this);

    loadConfiguration();

    Device::setup(); // run the parent class version of this function

    canHandlerEv.attach(this, 0x18000000, 0x18000000, true);
    //0x18FF50E5 for charger
    //0x1806F4D5 for DC/DC

    tickHandler.attach(this, CFG_TICK_INTERVAL_DCDC);
}


void OvarCharger::handleCanFrame(CAN_FRAME *frame)
{
    uint16_t currentVoltage = 0;
	uint16_t currentAmps = 0;
    uint8_t status = 0;
	uint16_t dcdcVoltage = 0;
	uint16_t dcdcAmps = 0;
    int16_t dcdcTemperature = 0;
    uint8_t dcdc_status = 0;
    Logger::debug("Ovar msg: %X   %X   %X   %X   %X   %X   %X   %X  %X", frame->id, frame->data.bytes[0],
                  frame->data.bytes[1],frame->data.bytes[2],frame->data.bytes[3],frame->data.bytes[4],
                  frame->data.bytes[5],frame->data.bytes[6],frame->data.bytes[7]);

    switch (frame->id)
    {
    case 0x18FF50E5: //message from battery charger
	    currentVoltage = (frame->data.bytes[0] << 8) + (frame->data.bytes[1]);
	    currentAmps = (frame->data.bytes[2] << 8) + (frame->data.bytes[3]);
        status = frame->data.bytes[4];
        if (status & 1) Logger::error("Hardware failure of OVAR");
        if (status & 2) Logger::error("Charger over temperature!");
        if (status & 4) Logger::error("Input voltage out of spec!");
        if (status & 8) Logger::error("Charger can't detect proper battery voltage");
        if (status & 16) Logger::error("Comm timeout. Failed!");
        Logger::debug("Charger    V: %f  A: %f   Status: %u", currentVoltage / 10.0f, currentAmps / 10.0f, status);
        break;
    case 0x1806F4D5: //message from DC/DC
	    dcdcVoltage = (frame->data.bytes[0] << 8) + (frame->data.bytes[1]);
	    dcdcAmps = (frame->data.bytes[2] << 8) + (frame->data.bytes[3]);
        dcdcTemperature = frame->data.bytes[4] - 40;
        dcdc_status = frame->data.bytes[5];
        if (dcdc_status & 1) Logger::error("DCDC has failed");
        if (dcdc_status & 2) Logger::error("DCDC temperature abnormal");
        if (dcdc_status & 4) Logger::error("DCDC Input voltage abnormal");
        if (dcdc_status & 8) Logger::error("DCDC output voltage abnormal");
        if (dcdc_status & 16) Logger::error("Comm timeout. Failed!");
        Logger::debug("DCDC    V: %f  A: %f   Status: %u", dcdcVoltage / 10.0f, dcdcAmps / 10.0f, dcdc_status);
        break;
	}
}

void OvarCharger::handleTick() {
    OvarChargerConfiguration *config = (OvarChargerConfiguration *)getConfiguration();

    Device::handleTick(); //kick the ball up to papa

    CAN_FRAME output;
    output.length = 8;
    output.id = 0x1806D5F4;
    output.extended = 1;
    output.rtr = 0;

	output.data.bytes[0] = 0;
	output.data.bytes[1] = 0;
	output.data.bytes[2] = 0;
	output.data.bytes[3] = 0;
	output.data.bytes[4] = 0;
	output.data.bytes[5] = 0;
	output.data.bytes[6] = 0;
	output.data.bytes[7] = 0;
	
    canHandlerEv.sendFrame(output);  //Mail it.

    Logger::debug("DCDC Command Frame: %X  %X  %X  %X  %X  %X  %X  %X",output.id, output.data.bytes[0],
                  output.data.bytes[1],output.data.bytes[2],output.data.bytes[3],output.data.bytes[4],
				  output.data.bytes[5],output.data.bytes[6],output.data.bytes[7]);

    output.length = 8;
    output.id = 0x1806E5F4;
    output.extended = 1;
    output.rtr = 0;

	output.data.bytes[0] = (config->maxAllowedVolts >> 8);
	output.data.bytes[1] = (config->maxAllowedVolts & 0xFF);
	output.data.bytes[2] = (config->maxAllowedCurrent >> 8);
	output.data.bytes[3] = (config->maxAllowedCurrent & 0xFF);
	output.data.bytes[4] = 0; // 0 = start charging
	output.data.bytes[5] = 0;
	output.data.bytes[6] = 0;
	output.data.bytes[7] = 0;
    canHandlerEv.sendFrame(output);  //Mail it.

    Logger::debug("Charger Command Frame: %X  %X  %X  %X  %X  %X  %X  %X",output.id, output.data.bytes[0],
                  output.data.bytes[1],output.data.bytes[2],output.data.bytes[3],output.data.bytes[4],
				  output.data.bytes[5],output.data.bytes[6],output.data.bytes[7]);
}

DeviceId OvarCharger::getId() {
    return (OVARCHARGE);
}

uint32_t OvarCharger::getTickInterval()
{
    return CFG_TICK_INTERVAL_DCDC;
}

DeviceType OvarCharger::getType()
{
    return DEVICE_MISC;
}

void OvarCharger::loadConfiguration()
{
    OvarChargerConfiguration *config = (OvarChargerConfiguration *)getConfiguration();

    if (!config) {
        config = new OvarChargerConfiguration();
        setConfiguration(config);
    }

    Device::loadConfiguration(); // call parent

    if (prefsHandler->checksumValid()) { //checksum is good, read in the values stored in EEPROM
        Logger::info((char *)Constants::validChecksum);
        prefsHandler->read(EECHARGER_MAX_V, &config->maxAllowedVolts);
        prefsHandler->read(EECHARGER_MAX_A, &config->maxAllowedCurrent);
    }
    else { //checksum invalid. Reinitialize values and store to EEPROM
        Logger::info((char *)Constants::invalidChecksum);
        config->maxAllowedVolts = 3600;
        config->maxAllowedCurrent = 200;
        saveConfiguration();
    }
}

void OvarCharger::saveConfiguration()
{
    OvarChargerConfiguration *config = (OvarChargerConfiguration *)getConfiguration();

    Device::saveConfiguration();

    prefsHandler->write(EECHARGER_MAX_V, config->maxAllowedVolts);
    prefsHandler->write(EECHARGER_MAX_A, config->maxAllowedCurrent);
    prefsHandler->saveChecksum();
    prefsHandler->forceCacheWrite();
}
