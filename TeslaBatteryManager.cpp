/*
 * TeslaBatteryManager.cpp
 *
 * Interface to the EVTV BMS that in turn controls and interfaces with Model S modules
 *
Copyright (c) 2023 Collin Kidder

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

#include "TeslaBatteryManager.h"

TeslaBatteryManager::TeslaBatteryManager() : BatteryManager() {
    prefsHandler = new PrefHandler(TESLABMS);
    allowCharge = false;
    allowDischarge = false;
    commonName = "Tesla Mods BMS";
    //the BMS starts up before contactors are closed and it handles our contactors so don't
    //enable discharge or charge until the BMS says the contactors are closed and there are no faults
    allowDischarge = false;
    allowCharge = false;
    trafficLostCounter = 0;
}

void TeslaBatteryManager::setup() {
    tickHandler.detach(this);

    Logger::info("add device: Tesla Module BMS (id: %X, %X)", TESLABMS, this);

    BatteryManager::setup(); // run the parent class version of this function

    //Relevant BMS messages we care about are 0x650 and 0x651. There is 0x68F with
    //voltages for every cell but we probably don't need that. 
    canHandlerEv.attach(this, 0x650, 0x7f0, false);

    tickHandler.attach(this, CFG_TICK_INTERVAL_BMS_THINK);
}

/*For all multibyte integers the format little endian
    bool allowCharge, allowDischarge;
*/
void TeslaBatteryManager::handleCanFrame(CAN_FRAME *frame) {
    int temp;

    trafficLostCounter = 0; //if we got here then there was traffic

    switch (frame->id) {
    case 0x650:
        packVoltage = (frame->data.bytes[1] * 256 + frame->data.bytes[0]);
        packCurrent = (frame->data.bytes[3] * 256 + frame->data.bytes[2]);
        SOC = frame->data.bytes[4] / 2;
        highestCellTemp = (((S8)frame->data.bytes[5]) * 10) - 200;
        lowestCellTemp = (((S8)frame->data.bytes[6]) * 10) - 200;
        //lower bits of byte 7 tell the actual reason
        if (frame->data.bytes[7] != 0) //if BMS registers some sort of fault
        {
            isFaulted = true;
            U8 reason = frame->data.bytes[7] & 0xF;
            if (reason == 1)
            {
                faultHandler.raiseFault(TESLABMS, FAULT_HV_BATT_LOW, true);
                allowDischarge = false;
                allowCharge = false;
            }
            else
            {
                faultHandler.cancelOngoingFault(TESLABMS, FAULT_HV_BATT_LOW);
            }
            if (reason == 2)
            {
                faultHandler.raiseFault(TESLABMS, FAULT_HV_BATT_HIGH, true);
                allowDischarge = false;
                allowCharge = false;
            }
            else
            {
                faultHandler.cancelOngoingFault(TESLABMS, FAULT_HV_BATT_HIGH);
            }
            if (reason == 3)
            {
                faultHandler.raiseFault(TESLABMS, FAULT_HV_BATT_UNDERTEMP, true);
                allowDischarge = false;
                allowCharge = false;
            }
            else
            {
                faultHandler.cancelOngoingFault(TESLABMS, FAULT_HV_BATT_UNDERTEMP);
            }
            if (reason == 4)
            {
                faultHandler.raiseFault(TESLABMS, FAULT_HV_BATT_OVERTEMP, true);
                allowDischarge = false;
                allowCharge = false;
            }
            else
            {
                faultHandler.cancelOngoingFault(TESLABMS, FAULT_HV_BATT_OVERTEMP);
            }
            if (reason == 5)
            {
                faultHandler.raiseFault(TESLABMS, FAULT_HV_BATT_ISOLATION, true);
                allowDischarge = false;
                allowCharge = false;
            }
            else
            {
                faultHandler.cancelOngoingFault(TESLABMS, FAULT_HV_BATT_ISOLATION);
            }
        }
        else 
        {
            isFaulted = false;
            allowDischarge = true;
            allowCharge = true;
        }
        break;
    case 0x651:
        lowestCellV = (frame->data.bytes[1] * 256 + frame->data.bytes[0]);
        highestCellV = (frame->data.bytes[3] * 256 + frame->data.bytes[2]);
        //byte 6 is contactor status. 
        //NEGCONFIRM = bit 0
        //NEGCONTACTOR = bit 1
        //POSCONFIRM = bit 2
        //POSCONTACTOR = bit 3
        if (frame->data.bytes[6] != 0xF)
        {
            allowDischarge = false;
            allowCharge = false;
        }
        else if (!isFaulted) 
        {
            allowDischarge = true;
            allowCharge = true;
        }
        break;
    }
}

void TeslaBatteryManager::handleTick() {
    BatteryManager::handleTick(); //kick the ball up to papa
    trafficLostCounter++;
    if (trafficLostCounter > 4) //it should NEVER be anything other than the 1 we just caused with the above line!
    {
        isFaulted = true;
        allowCharge = false;
        allowDischarge = false;
        faultHandler.raiseFault(TESLABMS, FAULT_BMS_COMM, true);
    }
    else
    {
        faultHandler.cancelOngoingFault(TESLABMS, FAULT_BMS_COMM);
        //we cancel the fault but don't immediately enable charge/discharge
        //they will be re-enabled by the actual comm handling code
    }
    //sendKeepAlive();

}

void TeslaBatteryManager::sendKeepAlive()
{
    //this BMS doesn't really need any comm to it to keep running. It could optionally be done though.
}

DeviceId TeslaBatteryManager::getId()
{
    return (TESLABMS);
}

bool TeslaBatteryManager::hasPackVoltage()
{
    return true;
}

bool TeslaBatteryManager::hasPackCurrent()
{
    return true;
}

bool TeslaBatteryManager::hasTemperatures()
{
    return true;
}

bool TeslaBatteryManager::isChargeOK()
{
    return allowCharge;
}

bool TeslaBatteryManager::isDischargeOK()
{
    return allowDischarge;
}
