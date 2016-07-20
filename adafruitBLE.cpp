/*
 * adafruitBLE.cpp
 *
 * Class to interface with adafruit BLE module
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

#include "adafruitBLE.h"

Adafruit_BluefruitLE_SPI ble(63, 27, 65);  //Instantiate SPI version
Adafruit_BLEGatt gatt(ble);
uint8_t DFU=65;
uint8_t MODE=64;
uint8_t BLETYPE=1;  //Type 1 is SPI type 2 is UART

//formed as a standard C array right now. Sort of dangerous in that there is no good way
//to know the length... well, you can grab the sizeof operator with division
//but instead this array is null terminated sort of like a C string.
Characteristic characteristics[] = 
{
    {4, 4, "TimeRunning", {GATT_PRESENT_FORMAT_UINT32, 0, GATT_PRESENT_UNIT_TIME_SECOND, 1, 0}}, //MeasureCharId[0]
    {2, 2, "TorqueRequest", {GATT_PRESENT_FORMAT_SINT16, -1, GATT_PRESENT_UNIT_MOMENT_OF_FORCE_NEWTON_METRE, 1, 0}}, //1
    {2, 2, "TorqueActual", {GATT_PRESENT_FORMAT_SINT16, -1, GATT_PRESENT_UNIT_MOMENT_OF_FORCE_NEWTON_METRE, 1, 0}}, //2
    {2, 2, "ThrottleLevel", {GATT_PRESENT_FORMAT_UINT16, 0, GATT_PRESENT_UNIT_NONE, 1, 0}}, //3
    {2, 2, "BrakeLevel", {GATT_PRESENT_FORMAT_UINT16, 0, GATT_PRESENT_UNIT_NONE, 1, 0}}, //4
    {2, 2, "SpeedRequested", {GATT_PRESENT_FORMAT_SINT16, 0, GATT_PRESENT_UNIT_ANGULAR_VELOCITY_REVOLUTION_PER_MINUTE, 1, 0}}, //5
    {2, 2, "SpeedActual", {GATT_PRESENT_FORMAT_SINT16, 0, GATT_PRESENT_UNIT_ANGULAR_VELOCITY_REVOLUTION_PER_MINUTE, 1, 0}}, //6
    {1, 1, "PowerMode", {GATT_PRESENT_FORMAT_UINT8, 0, GATT_PRESENT_UNIT_NONE, 1, 0}}, //MeasureCharId[7]
    {1, 1, "Gear", {GATT_PRESENT_FORMAT_UINT8, 0, GATT_PRESENT_UNIT_NONE, 1, 0}}, //8
    {2, 2, "BusVoltage", {GATT_PRESENT_FORMAT_UINT16, -1, GATT_PRESENT_UNIT_ELECTRIC_POTENTIAL_DIFFERENCE_VOLT, 1, 0}}, //9
    {2, 2, "BusCurrent", {GATT_PRESENT_FORMAT_SINT16, -1, GATT_PRESENT_UNIT_ELECTRIC_CURRENT_AMPERE, 1, 0}}, //10
    {2, 2, "MotorCurrent", {GATT_PRESENT_FORMAT_SINT16, -1, GATT_PRESENT_UNIT_ELECTRIC_CURRENT_AMPERE, 1, 0}}, //11
    {2, 2, "NomVoltage", {GATT_PRESENT_FORMAT_UINT16, -1, GATT_PRESENT_UNIT_ELECTRIC_POTENTIAL_DIFFERENCE_VOLT, 1, 0}}, //12
    {2, 2, "KWHours", {GATT_PRESENT_FORMAT_UINT16, -1, GATT_PRESENT_UNIT_ENERGY_KILOWATT_HOUR, 1, 0}}, //13
    {4, 4, "Bitfield1", {GATT_PRESENT_FORMAT_UINT32, 0, GATT_PRESENT_UNIT_NONE, 1, 0}}, //MeasureCharId[14]
    {4, 4, "Bitfield2", {GATT_PRESENT_FORMAT_UINT32, 0, GATT_PRESENT_UNIT_NONE, 1, 0}}, //15
    {4, 4, "Bitfield3", {GATT_PRESENT_FORMAT_UINT32, 0, GATT_PRESENT_UNIT_NONE, 1, 0}}, //16
    {4, 4, "Bitfield4", {GATT_PRESENT_FORMAT_UINT32, 0, GATT_PRESENT_UNIT_NONE, 1, 0}}, //17
    {1, 1, "IsRunning", {GATT_PRESENT_FORMAT_BOOLEAN, 0, GATT_PRESENT_UNIT_NONE, 1, 0}}, //18
    {1, 1, "IsFaulted", {GATT_PRESENT_FORMAT_BOOLEAN, 0, GATT_PRESENT_UNIT_NONE, 1, 0}}, //19
    {1, 1, "IsWarning", {GATT_PRESENT_FORMAT_BOOLEAN, 0, GATT_PRESENT_UNIT_NONE, 1, 0}}, //MeasureCharId[20]
    {2, 2, "MotorTemperature", {GATT_PRESENT_FORMAT_SINT16, -1, GATT_PRESENT_UNIT_THERMODYNAMIC_TEMPERATURE_DEGREE_CELSIUS, 1, 0}}, //21
    {2, 2, "InverterTemp", {GATT_PRESENT_FORMAT_SINT16, -1, GATT_PRESENT_UNIT_THERMODYNAMIC_TEMPERATURE_DEGREE_CELSIUS, 1, 0}}, //22
    {2, 2, "SystemTemp", {GATT_PRESENT_FORMAT_SINT16, -1, GATT_PRESENT_UNIT_THERMODYNAMIC_TEMPERATURE_DEGREE_CELSIUS, 1, 0}}, //23
    {2, 2, "MechPower", {GATT_PRESENT_FORMAT_SINT16, 2, GATT_PRESENT_UNIT_POWER_WATT, 1, 0}}, //24
    {2, 2, "PrechargeResist", {GATT_PRESENT_FORMAT_UINT16, 0, GATT_PRESENT_UNIT_ELECTRIC_RESISTANCE_OHM, 1, 0}}, //25
    {1, 1, "PrechargeRelay", {GATT_PRESENT_FORMAT_UINT8, 0, GATT_PRESENT_UNIT_NONE, 1, 0}}, //26
    {1, 1, "MainContRelay", {GATT_PRESENT_FORMAT_UINT8, 0, GATT_PRESENT_UNIT_NONE, 1, 0}}, //27
    {1, 1, "CoolFanRelay", {GATT_PRESENT_FORMAT_UINT8, 0, GATT_PRESENT_UNIT_NONE, 1, 0}}, //MeasureCharId[28]
    {1, 1, "CoolOnTemp", {GATT_PRESENT_FORMAT_SINT8, 0, GATT_PRESENT_UNIT_THERMODYNAMIC_TEMPERATURE_DEGREE_CELSIUS, 1, 0}}, //29
    {1, 1, "CoolOffTemp", {GATT_PRESENT_FORMAT_SINT8, 0, GATT_PRESENT_UNIT_THERMODYNAMIC_TEMPERATURE_DEGREE_CELSIUS, 1, 0}}, //30
    {1, 1, "BrakeLightOut", {GATT_PRESENT_FORMAT_UINT8, 0, GATT_PRESENT_UNIT_NONE, 1, 0}}, //31
    {1, 1, "ReverseLightOut", {GATT_PRESENT_FORMAT_UINT8, 0, GATT_PRESENT_UNIT_NONE, 1, 0}}, //32
    {1, 1, "EnableInput", {GATT_PRESENT_FORMAT_UINT8, 0, GATT_PRESENT_UNIT_NONE, 1, 0}}, //33
    {1, 1, "ReverseInput", {GATT_PRESENT_FORMAT_UINT8, 0, GATT_PRESENT_UNIT_NONE, 1, 0}}, //34
    {1, 1, "NumThrottlePots", {GATT_PRESENT_FORMAT_UINT8, 0, GATT_PRESENT_UNIT_NONE, 1, 0}}, //35
    {1, 1, "ThrottleSubType", {GATT_PRESENT_FORMAT_UINT8, 0, GATT_PRESENT_UNIT_NONE, 1, 0}}, //36
    {1, 1, "ThrottleMin1", {GATT_PRESENT_FORMAT_UINT8, 0, GATT_PRESENT_UNIT_NONE, 1, 0}}, //MeasureCharId[37]
    {1, 1, "ThrottleMin2", {GATT_PRESENT_FORMAT_UINT8, 0, GATT_PRESENT_UNIT_NONE, 1, 0}}, //38
    {1, 1, "ThrottleMax1", {GATT_PRESENT_FORMAT_UINT8, 0, GATT_PRESENT_UNIT_NONE, 1, 0}}, //39
    {1, 1, "ThrottleMax2", {GATT_PRESENT_FORMAT_UINT8, 0, GATT_PRESENT_UNIT_NONE, 1, 0}}, //40
    {2, 2, "ThrRegenMax", {GATT_PRESENT_FORMAT_UINT16, 0, GATT_PRESENT_UNIT_NONE, 1, 0}}, //41
    {2, 2, "ThrRegenMin", {GATT_PRESENT_FORMAT_UINT16, 0, GATT_PRESENT_UNIT_NONE, 1, 0}}, //42
    {2, 2, "ThrottleFwd", {GATT_PRESENT_FORMAT_UINT16, 0, GATT_PRESENT_UNIT_NONE, 1, 0}}, //43
    {2, 2, "ThrottleMap", {GATT_PRESENT_FORMAT_UINT16, 0, GATT_PRESENT_UNIT_NONE, 1, 0}}, //44
    {1, 1, "TRegenMin", {GATT_PRESENT_FORMAT_UINT8, 0, GATT_PRESENT_UNIT_NONE, 1, 0}}, //45
    {1, 1, "TRegenMax", {GATT_PRESENT_FORMAT_UINT8, 0, GATT_PRESENT_UNIT_NONE, 1, 0}}, //MeasureCharId[46]
    {1, 1, "TCreep", {GATT_PRESENT_FORMAT_UINT8, 0, GATT_PRESENT_UNIT_NONE, 1, 0}}, //47
    {1, 1, "BrakeMin", {GATT_PRESENT_FORMAT_UINT8, 0, GATT_PRESENT_UNIT_NONE, 1, 0}}, //48
    {1, 1, "BrakeMax", {GATT_PRESENT_FORMAT_UINT8, 0, GATT_PRESENT_UNIT_NONE, 1, 0}}, //49
    {1, 1, "BRegenMin", {GATT_PRESENT_FORMAT_UINT8, 0, GATT_PRESENT_UNIT_NONE, 1, 0}}, //50
    {1, 1, "BRegenMax", {GATT_PRESENT_FORMAT_UINT8, 0, GATT_PRESENT_UNIT_NONE, 1, 0}}, //51
    {2, 2, "MaxRPM", {GATT_PRESENT_FORMAT_UINT16, 0, GATT_PRESENT_UNIT_ANGULAR_VELOCITY_REVOLUTION_PER_MINUTE, 1, 0}}, //52 
    {2, 2, "TorqueMax", {GATT_PRESENT_FORMAT_UINT16, -1, GATT_PRESENT_UNIT_MOMENT_OF_FORCE_NEWTON_METRE, 1, 0}}, //53
    {1, 1, "LogLevel", {GATT_PRESENT_FORMAT_UINT8, 0, GATT_PRESENT_UNIT_NONE, 1, 0}}, //54
 
    {0, 0, "END", {GATT_PRESENT_FORMAT_UINT8, 0, GATT_PRESENT_UNIT_NONE, 1, 0}} //leave this at the end - null terminator
};
   
/*
 * Constructor. Assign serial interface to use for ichip communication
 */
ADAFRUITBLE::ADAFRUITBLE() {
    prefsHandler = new PrefHandler(ADABLUE);

    uint8_t sys_type;
    sysPrefs->read(EESYS_SYSTEM_TYPE, &sys_type);

    commonName = "Adafruit BLE";
}

/*
 * Initialization of hardware and parameters
 */
void ADAFRUITBLE::setup() {

    Logger::info(ADABLUE, "add device: AdaFruit BLE (id: %X, %X)", ADABLUE, this);

    TickHandler::getInstance()->detach(this);

    tickCounter = 0;

    paramCache.brakeNotAvailable = true;

    elmProc = new ELM327Processor();
        
    //These apply to UART only
    if(BLETYPE==2)
    {
        pinMode(DFU,OUTPUT);
        digitalWrite(DFU,HIGH); //Set DFU bootloader pin 65 to HIGH -normal operation
        pinMode(MODE,OUTPUT); //Set MODE command pin 64 to HIGH (Command Mode).  Low is DATA mode.
        digitalWrite(MODE,HIGH);    
    }  

    //These apply to both UART and SPI versions
    Logger::debug(ADABLUE, "Initializing ADAFruit BLE bluetooth device...");    
    ble.begin(false);  //True = verbose mode for debuggin.   
    ble.factoryReset();        
    Logger::debug(ADABLUE, "Retrieving Bluetooth module information...");
    ble.info();     
    ble.echo(false);
    setupBLEservice();
    Logger::debug(ADABLUE, "BluefruitLE Test Complete....");

    TickHandler::getInstance()->attach(this, CFG_TICK_INTERVAL_WIFI);
}

void ADAFRUITBLE::setupBLEservice()
{
//Very important.  Will not work with values less than 5.  This slows down transmission to prevent overflowing 
//very small UART buffer in nRF chip.  If errors received, try increasing this value.
//This command only applies to UART version of device.  If SPI selected, comment out

    //UART version only comment out for SPI version
    if (BLETYPE == 2)
    {
        //ble.setInterCharWriteDelay(5);        
    }
    
    if (! gatt.clear() ) 
    {
        Logger::error(ADABLUE, "Could not clear device..");
        return;
    }
  
  
    //Set peripheral transmit power level -40 minimum -20 -16 -12 -8 -4 0 4 maximum
    if (! ble.sendCommandCheckOK(F("AT+BLEPOWERLEVEL=4")) ) 
    {
        Logger::error(ADABLUE, "Could not set device power level...");
        return;
    }
 

    //Set peripheral name
    if (! ble.sendCommandCheckOK(F("AT+GAPDEVNAME=GEVCU 6.2 Test Program")) ) 
    {
        Logger::error(ADABLUE, "Could not set device name...");
        return;
    }
  

    //Set service ID
    ServiceId = gatt.addService(0x3100);
    if (ServiceId == 0) 
    {
        Logger::error(ADABLUE, "Could not add service....");   
        return;
    }

/* 
    PROPERTIES: The 8-bit characteristic properties field, as defined by the Bluetooth SIG. The following values can be used:
      0x02 - Read
      0x04 - Write Without Response
      0x08 - Write
      0x10 - Notify
      0x20 - Indicate

    DATATYPE: This argument indicates the data type stored in the characteristic, and is used to help parse data properly.  
     It can be one of the following values:
      AUTO (0, default)
      STRING (1)
      BYTEARRAY (2)
      INTEGER (3)   
*/
    int charCounter = 0;
    Characteristic charact = characteristics[charCounter];
    while (charact.minSize != 0)
    {
        MeasureCharId[charCounter] = gatt.addCharacteristic(0x3101 + charCounter, 0x10, charact.minSize, charact.maxSize, BLE_DATATYPE_INTEGER, charact.descript, &charact.present);
        if (MeasureCharId[charCounter] == 0) 
        {
            Logger::error(ADABLUE, "Could not add characteristic %x", 0x3101 + charCounter);
            return;            
        }
        charact = characteristics[++charCounter];
    }           
 
    if (!ble.sendCommandWithIntReply( F("AT+GATTADDCHAR=UUID=0x31D0, PROPERTIES=0x10, MIN_LEN=4, MAX_LEN=4, DATATYPE=3, VALUE=3,DESCRIPTION=COUNT"), &LocationCharId))
    {
         Logger::error(ADABLUE, "Could not add characteristic 0x31D0");
    }
    
    /* Add the  Service to the advertising data (needed for Nordic apps to detect the service) */
    ble.sendCommandCheckOK( F("AT+GAPSETADVDATA=02-01-06-05-02-00-31-10-31") );

    /* Reset the device for the new service setting changes to take effect */
    ble.reset();
  
    Logger::info(ADABLUE, "Service ID: %i", ServiceId);
}

/*
 * Periodic updates of parameters to ichip RAM.
 * Also query for changed parameters of the config page.
 */
//TODO: See the processing function below for a more detailed explanation - can't send so many setParam commands in a row
void ADAFRUITBLE::handleTick() {
    MotorController* motorController = DeviceManager::getInstance()->getMotorController();
    Throttle *accelerator = DeviceManager::getInstance()->getAccelerator();
    Throttle *brake = DeviceManager::getInstance()->getBrake();
    static int pollListening = 0;
    static int pollSocket = 0;
    uint32_t ms = millis();
    char buff[6];
    uint8_t brklt;
    tickCounter++;

    if (ms < 1000) return; //wait 10 seconds for things to settle before doing a thing

    // Do a delayed parameter load once about a second after startup
    if (!didParamLoad && ms > 5000) {
        loadParameters();
        Logger::console("Wifi Parameters loaded...");
        paramCache.bitfield1 = motorController->getStatusBitfield1();
        gatt.setChar(MeasureCharId[14], paramCache.bitfield1);
        paramCache.bitfield2 = motorController->getStatusBitfield2();
        gatt.setChar(MeasureCharId[15], paramCache.bitfield1);
        // DeviceManager::getInstance()->updateWifiByID(BRUSA_DMC5);
        didParamLoad = true;
    }

    // make small slices so the main loop is not blocked for too long
    if (tickCounter == 1) {
        if (motorController) {
            //Logger::console("Wifi tick counter 1...");

            paramCache.timeRunning = ms;
            gatt.setChar(MeasureCharId[1], getTimeRunning());

            if ( paramCache.torqueRequested != motorController->getTorqueRequested() ) {
                paramCache.torqueRequested = motorController->getTorqueRequested();
                gatt.setChar(MeasureCharId[1], paramCache.torqueRequested);
            }
            if ( paramCache.torqueActual != motorController->getTorqueActual() ) {
                paramCache.torqueActual = motorController->getTorqueActual();
                gatt.setChar(MeasureCharId[2], paramCache.torqueActual);
            }
        }
        if (accelerator) {
            RawSignalData *rawSignal = accelerator->acquireRawSignal();
            if ( paramCache.throttle !=  rawSignal->input1) {
                paramCache.throttle = rawSignal->input1;
                gatt.setChar(MeasureCharId[3], paramCache.throttle);
            }
        }
        if (brake) {
            RawSignalData *rawSignal = brake->acquireRawSignal();
            if ( paramCache.brake !=  rawSignal->input1) {
                paramCache.brake = rawSignal->input1;
                paramCache.brakeNotAvailable = false;
                gatt.setChar(MeasureCharId[4], paramCache.brake);
            }
        } else {
            if ( paramCache.brakeNotAvailable == true ) {
                paramCache.brakeNotAvailable = false; // no need to keep sending this
                gatt.setChar(MeasureCharId[4], (int16_t)0);                
            }
        }
    } else if (tickCounter == 2) {
        if (motorController) {
            //Logger::console("Wifi tick counter 2...");
            if ( paramCache.speedRequested != motorController->getSpeedRequested() ) {
                paramCache.speedRequested = motorController->getSpeedRequested();
                gatt.setChar(MeasureCharId[5], paramCache.speedRequested);
            }
            if ( paramCache.speedActual != motorController->getSpeedActual() ) {
                paramCache.speedActual = motorController->getSpeedActual();
                if (paramCache.speedActual<0) paramCache.speedActual=0;
                if (paramCache.speedActual>10000) paramCache.speedActual=10000;
                gatt.setChar(MeasureCharId[6], paramCache.speedActual);
            }
            if ( paramCache.dcVoltage != motorController->getDcVoltage() ) {
                paramCache.dcVoltage = motorController->getDcVoltage();
                if(paramCache.dcVoltage<1000) paramCache.dcVoltage=1000;  //Limits of the gage display
                if(paramCache.dcVoltage>4500) paramCache.dcVoltage=4500;
                gatt.setChar(MeasureCharId[9], paramCache.dcVoltage);
            }
            if ( paramCache.dcCurrent != motorController->getDcCurrent() ) {
                paramCache.dcCurrent = motorController->getDcCurrent();
                gatt.setChar(MeasureCharId[10], paramCache.dcCurrent);
            }
            if ( paramCache.prechargeR != motorController->getprechargeR() ) {
                paramCache.prechargeR = motorController->getprechargeR();
                gatt.setChar(MeasureCharId[25], (uint16_t)paramCache.prechargeR);
            }

            if ( paramCache.prechargeRelay != motorController->getprechargeRelay() ) {
                paramCache.prechargeRelay = motorController->getprechargeRelay();
                gatt.setChar(MeasureCharId[26], paramCache.prechargeRelay);
                //Logger::console("Precharge Relay %i", paramCache.prechargeRelay);
                //Logger::console("motorController:prechargeRelay:%d, paramCache.prechargeRelay:%d, Constants:prechargeRelay:%s", motorController->getprechargeRelay(),paramCache.prechargeRelay, Constants::prechargeRelay);
            }

            if ( paramCache.mainContactorRelay != motorController->getmainContactorRelay() ) {
                paramCache.mainContactorRelay = motorController->getmainContactorRelay();
                gatt.setChar(MeasureCharId[27], paramCache.mainContactorRelay);
            }
            //DeviceManager::getInstance()->updateWifi();
        }
    } else if (tickCounter == 3) {
        if (motorController) {
            //Logger::console("Wifi tick counter 2...");
            if ( paramCache.acCurrent != motorController->getAcCurrent() ) {
                paramCache.acCurrent = motorController->getAcCurrent();
                gatt.setChar(MeasureCharId[11], paramCache.acCurrent);
            }

            //if ( paramCache.kiloWattHours != motorController->getkiloWattHours()/3600000 ) {
            paramCache.kiloWattHours = motorController->getKiloWattHours()/3600000;
            if(paramCache.kiloWattHours<0)paramCache.kiloWattHours = 0;
            if(paramCache.kiloWattHours>300)paramCache.kiloWattHours = 300;
            gatt.setChar(MeasureCharId[13], paramCache.kiloWattHours);
            //}

            if ( paramCache.nominalVolt != motorController->getnominalVolt() ) {
                paramCache.nominalVolt = motorController->getnominalVolt();
                gatt.setChar(MeasureCharId[12], paramCache.nominalVolt);
            }

            if ( paramCache.bitfield1 != motorController->getStatusBitfield1() ) {
                paramCache.bitfield1 = motorController->getStatusBitfield1();
                gatt.setChar(MeasureCharId[14], paramCache.bitfield1);
            }
            if ( paramCache.bitfield2 != motorController->getStatusBitfield2() ) {
                paramCache.bitfield2 = motorController->getStatusBitfield2();
                gatt.setChar(MeasureCharId[15], paramCache.bitfield2);
            }
            if ( paramCache.bitfield3 != motorController->getStatusBitfield3() ) {
                paramCache.bitfield3 = motorController->getStatusBitfield3();
                gatt.setChar(MeasureCharId[16], paramCache.bitfield3);
            }
            if ( paramCache.bitfield4 != motorController->getStatusBitfield4() ) {
                paramCache.bitfield4 = motorController->getStatusBitfield4();
                gatt.setChar(MeasureCharId[17], paramCache.bitfield4);
            }
        }
    } else if (tickCounter == 4) {
        if (motorController) {
            // Logger::console("Wifi tick counter 4...");
            if ( paramCache.running != motorController->isRunning() ) {
                paramCache.running = motorController->isRunning();
                gatt.setChar(MeasureCharId[18], (uint8_t)(paramCache.running ? 1 : 0));
            }
            if ( paramCache.faulted != motorController->isFaulted() ) {
                paramCache.faulted = motorController->isFaulted();
                gatt.setChar(MeasureCharId[19], (uint8_t)(paramCache.faulted ? 1 : 0));
            }
            if ( paramCache.warning != motorController->isWarning() ) {
                paramCache.warning = motorController->isWarning();
                gatt.setChar(MeasureCharId[20], (uint8_t)(paramCache.warning ? 1 : 0));
            }
            if ( paramCache.gear != motorController->getSelectedGear() ) {
                paramCache.gear = motorController->getSelectedGear();
                gatt.setChar(MeasureCharId[8], (uint8_t)paramCache.gear);
            }

            if ( paramCache.coolFan != motorController->getCoolFan() ) {
                paramCache.coolFan = motorController->getCoolFan();
                gatt.setChar(MeasureCharId[28], (uint8_t)paramCache.coolFan);
            }

            if ( paramCache.coolOn != motorController->getCoolOn() ) {
                paramCache.coolOn = motorController->getCoolOn();
                gatt.setChar(MeasureCharId[29], (uint8_t)paramCache.coolOn);
            }

            if ( paramCache.coolOff != motorController->getCoolOff() ) {
                paramCache.coolOff = motorController->getCoolOff();
                gatt.setChar(MeasureCharId[30], (uint8_t)paramCache.coolOff);
            }

            if ( paramCache.brakeLight != motorController->getBrakeLight() ) {
                paramCache.brakeLight = motorController->getBrakeLight();
                gatt.setChar(MeasureCharId[31], (uint8_t) paramCache.brakeLight);
            }

            if ( paramCache.revLight != motorController->getRevLight() ) {
                paramCache.revLight = motorController->getRevLight();
                gatt.setChar(MeasureCharId[32], (uint8_t)paramCache.revLight);
            }

            if ( paramCache.enableIn != motorController->getEnableIn() ) {
                paramCache.enableIn = motorController->getEnableIn();
                gatt.setChar(MeasureCharId[33], (uint8_t)paramCache.enableIn);
            }
            if ( paramCache.reverseIn != motorController->getReverseIn() ) {
                paramCache.reverseIn = motorController->getReverseIn();
                gatt.setChar(MeasureCharId[34], (uint8_t)paramCache.reverseIn);
            }
        }
    } else if (tickCounter > 4) {
        if (motorController) {
            // Logger::console("Wifi tick counter 5...");
            if ( paramCache.tempMotor != motorController->getTemperatureMotor() ) {
                paramCache.tempMotor = motorController->getTemperatureMotor();
                gatt.setChar(MeasureCharId[21], paramCache.tempMotor);
            }
            if ( paramCache.tempInverter != motorController->getTemperatureInverter() ) {
                paramCache.tempInverter = motorController->getTemperatureInverter();
                gatt.setChar(MeasureCharId[22], paramCache.tempInverter);
            }
            if ( paramCache.tempSystem != motorController->getTemperatureSystem() ) {
                paramCache.tempSystem = motorController->getTemperatureSystem();
                gatt.setChar(MeasureCharId[23], paramCache.tempSystem);
            }

            if (paramCache.powerMode != motorController->getPowerMode() ) {
                paramCache.powerMode = motorController->getPowerMode();
                gatt.setChar(MeasureCharId[7], (uint8_t)paramCache.powerMode);
            }

            //if ( paramCache.mechPower != motorController->getMechanicalPower() ) {
            paramCache.mechPower = motorController->getMechanicalPower();
            if (paramCache.mechPower<-250)paramCache.mechPower=-250;
            if (paramCache.mechPower>1500)paramCache.mechPower=1500;
            gatt.setChar(MeasureCharId[24], paramCache.mechPower);
            //}
        }
        tickCounter = 0;
    }
}

/*
 * Calculate the runtime in hh:mm:ss
   This runtime calculation is good for about 50 days of uptime.
   Of course, the sprintf is only good to 99 hours so that's a bit less time.
 */
char *ADAFRUITBLE::getTimeRunning() {
    uint32_t ms = millis();
    int seconds = (int) (ms / 1000) % 60;
    int minutes = (int) ((ms / (1000 * 60)) % 60);
    int hours = (int) ((ms / (1000 * 3600)) % 24);
    sprintf(buffer, "%02d:%02d:%02d", hours, minutes, seconds);
    return buffer;
}

/*
 * Handle a message sent by the DeviceManager.
 * Currently MSG_SET_PARAM is supported. The message should be a two element char pointer array
 * containing the addresses of a two element char array. char *paramPtr[2] = { &param[0][0], &param[1][0] };
 * Element 0 of the base array (char param [2][20]; )should contain the name of the parameter to be changed
 * Element 1 of the base array should contain the new value to be set.
 *
 *  sendMessage(DEVICE_WIFI, ICHIP2128, MSG_SET_PARAM,  paramPtr);
 *
 */
void ADAFRUITBLE::handleMessage(uint32_t messageType, void* message) {
    Device::handleMessage(messageType, message);  //Only matters if message is MSG_STARTUP

    switch (messageType) {

    case MSG_SET_PARAM: {  //Sets a single parameter to a single value
        char **params = (char **)message;  //recast message as a two element array (params)
        // Logger::console("Received Device: %s value %s",params[0], params[1]);
        //setParam((char *)params[0], (char *)params[1]);
        break;
    }
    case MSG_CONFIG_CHANGE: { //Loads all characteristics to BLE module
        loadParameters();
        break;
    }
    case MSG_COMMAND:  //Sends a message to the BLE module in the form of AT command
        //sendCmd((char *)message);
        break;
    }

}

/*
 * Called in the main loop (hopefully) in order to process serial input waiting for us
 * from the wifi module. It should always terminate its answers with 13 so buffer
 * until we get 13 (CR) and then process it.
 *
 */

void ADAFRUITBLE::loop() {
}

/*
 * Process the parameter update from ichip we received as a response to AT+iWNXT.
 * The response usually looks like this : key="value", so the key can be isolated
 * by looking for the '=' sign and the leading/trailing '"' have to be ignored.
 */
void ADAFRUITBLE::processParameterChange(char *key) {
    PotThrottleConfiguration *acceleratorConfig = NULL;
    PotThrottleConfiguration *brakeConfig = NULL;
    MotorControllerConfiguration *motorConfig = NULL;
    bool parameterFound = true;

    char *value = strchr(key, '=');
    if (!value)
        return;

    Throttle *accelerator = DeviceManager::getInstance()->getAccelerator();
    Throttle *brake = DeviceManager::getInstance()->getBrake();
    MotorController *motorController = DeviceManager::getInstance()->getMotorController();

    if (accelerator)
        acceleratorConfig = (PotThrottleConfiguration *)accelerator->getConfiguration();
    if (brake)
        brakeConfig = (PotThrottleConfiguration *)brake->getConfiguration();
    if(motorController)
        motorConfig = (MotorControllerConfiguration *)motorController->getConfiguration();

    value[0] = 0; // replace the '=' sign with a 0
    value++;
    if (value[0] == '"')
        value++; // if the value starts with a '"', advance one character
    if (value[strlen(value) - 1] == '"')
        value[strlen(value) - 1] = 0; // if the value ends with a '"' character, replace it with 0

    if (!strcmp(key, Constants::numThrottlePots) && acceleratorConfig) {
        acceleratorConfig->numberPotMeters = atol(value);
        accelerator->saveConfiguration();
    } else if (!strcmp(key, Constants::throttleSubType) && acceleratorConfig) {
        acceleratorConfig->throttleSubType = atol(value);
        accelerator->saveConfiguration();
    } else if (!strcmp(key, Constants::throttleMin1) && acceleratorConfig) {
        acceleratorConfig->minimumLevel1 = atol(value);
        accelerator->saveConfiguration();
    } else if (!strcmp(key, Constants::throttleMin2) && acceleratorConfig) {
        acceleratorConfig->minimumLevel2 = atol(value);
        accelerator->saveConfiguration();
    } else if (!strcmp(key, Constants::throttleMax1) && acceleratorConfig) {
        acceleratorConfig->maximumLevel1 = atol(value);
        accelerator->saveConfiguration();
    } else if (!strcmp(key, Constants::throttleMax2) && acceleratorConfig) {
        acceleratorConfig->maximumLevel2 = atol(value);
        accelerator->saveConfiguration();
    } else if (!strcmp(key, Constants::throttleRegenMax) && acceleratorConfig) {
        acceleratorConfig->positionRegenMaximum = atol(value) * 10;
    } else if (!strcmp(key, Constants::throttleRegenMin) && acceleratorConfig) {
        acceleratorConfig->positionRegenMinimum = atol(value) * 10;
        accelerator->saveConfiguration();
    } else if (!strcmp(key, Constants::throttleFwd) && acceleratorConfig) {
        acceleratorConfig->positionForwardMotionStart = atol(value) * 10;
        accelerator->saveConfiguration();
    } else if (!strcmp(key, Constants::throttleMap) && acceleratorConfig) {
        acceleratorConfig->positionHalfPower = atol(value) * 10;
        accelerator->saveConfiguration();
    } else if (!strcmp(key, Constants::throttleMinRegen) && acceleratorConfig) {
        acceleratorConfig->minimumRegen = atol(value);
        accelerator->saveConfiguration();
    } else if (!strcmp(key, Constants::throttleMaxRegen) && acceleratorConfig) {
        acceleratorConfig->maximumRegen = atol(value);
        accelerator->saveConfiguration();
    } else if (!strcmp(key, Constants::throttleCreep) && acceleratorConfig) {
        acceleratorConfig->creep = atol(value);
        accelerator->saveConfiguration();
    } else if (!strcmp(key, Constants::brakeMin) && brakeConfig) {
        brakeConfig->minimumLevel1 = atol(value);
        brake->saveConfiguration();
    } else if (!strcmp(key, Constants::brakeMax) && brakeConfig) {
        brakeConfig->maximumLevel1 = atol(value);
        brake->saveConfiguration();
    } else if (!strcmp(key, Constants::brakeMinRegen) && brakeConfig) {
        brakeConfig->minimumRegen = atol(value);
        brake->saveConfiguration();
    } else if (!strcmp(key, Constants::brakeMaxRegen) && brakeConfig) {
        brakeConfig->maximumRegen = atol(value);
        brake->saveConfiguration();
    } else if (!strcmp(key, Constants::speedMax) && motorConfig) {
        motorConfig->speedMax = atol(value);
        motorController->saveConfiguration();
    } else if (!strcmp(key, Constants::torqueMax) && motorConfig) {
        motorConfig->torqueMax = atol(value) * 10;
        motorController->saveConfiguration();
    } else if (!strcmp(key, Constants::coolFan) && motorConfig) {
        motorConfig->coolFan = atol(value);
        motorController->saveConfiguration();
    } else if (!strcmp(key, Constants::coolOn) && motorConfig) {
        motorConfig->coolOn = (atol(value));
        motorController->saveConfiguration();
    } else if (!strcmp(key, Constants::coolOff) && motorConfig) {
        motorConfig->coolOff = (atol(value));
        motorController->saveConfiguration();
    } else if (!strcmp(key, Constants::prechargeR) && motorConfig) {
        motorConfig->prechargeR = atol(value);
        motorController->saveConfiguration();
    } else if (!strcmp(key, Constants::prechargeRelay) && motorConfig) {
        motorConfig->prechargeRelay = atol(value);
        motorController->saveConfiguration();
    } else if (!strcmp(key, Constants::nominalVolt) && motorConfig) {
        motorConfig->nominalVolt = (atol(value))*10;
        motorController->saveConfiguration();

    } else if (!strcmp(key, Constants::mainContactorRelay) && motorConfig) {
        motorConfig->mainContactorRelay = atol(value);
        motorController->saveConfiguration();
    } else if (!strcmp(key, Constants::brakeLight) && motorConfig) {
        motorConfig->brakeLight = atol(value);
        motorController->saveConfiguration();
    } else if (!strcmp(key, Constants::revLight) && motorConfig) {
        motorConfig->revLight = atol(value);
        motorController->saveConfiguration();
    } else if (!strcmp(key, Constants::enableIn) && motorConfig) {
        motorConfig->enableIn = atol(value);
        motorController->saveConfiguration();
    } else if (!strcmp(key, Constants::reverseIn) && motorConfig) {
        motorConfig->reverseIn = atol(value);
        motorController->saveConfiguration();
        /*  } else if (!strcmp(key, Constants::motorMode) && motorConfig) {
        motorConfig->motorMode = (MotorController::PowerMode)atoi(value);
        motorController->saveConfiguration();
        */



    } else if (!strcmp(key, "x1000")) {
        if (255==atol(value)) {
            sysPrefs->setDeviceStatus(strtol(key+1, 0, 16),true);
        }
        else {
            sysPrefs->setDeviceStatus(strtol(key+1, 0, 16), false);
        }
        //sysPrefs->forceCacheWrite();
    } else if (!strcmp(key, "x1001")) {
        if (255==atol(value)) {
            sysPrefs->setDeviceStatus(strtol(key+1, 0, 16),true);
        }
        else {
            sysPrefs->setDeviceStatus(strtol(key+1, 0, 16), false);
        }
        //sysPrefs->forceCacheWrite();
    } else if (!strcmp(key, "x1002")) {
        if (255==atol(value)) {
            sysPrefs->setDeviceStatus(strtol(key+1, 0, 16),true);
        }
        else {
            sysPrefs->setDeviceStatus(strtol(key+1, 0, 16), false);
        }
        // sysPrefs->forceCacheWrite();
    } else if (!strcmp(key, "x1031")) {
        if (255==atol(value)) {
            sysPrefs->setDeviceStatus(strtol(key+1, 0, 16), true);
        }
        else {
            sysPrefs->setDeviceStatus(strtol(key+1, 0, 16), false);
        }
        //sysPrefs->forceCacheWrite();
    } else if (!strcmp(key, "x1032")) {
        if (255==atol(value)) {
            sysPrefs->setDeviceStatus(strtol(key+1, 0, 16), true);
        }
        else {
            sysPrefs->setDeviceStatus(strtol(key+1, 0, 16), false);
        }
        //sysPrefs->forceCacheWrite();
    } else if (!strcmp(key, "x1033")) {
        if (255==atol(value)) {
            sysPrefs->setDeviceStatus(strtol(key+1, 0, 16), true);
        }
        else {
            sysPrefs->setDeviceStatus(strtol(key+1, 0, 16), false);
        }
        //sysPrefs->forceCacheWrite();
    } else if (!strcmp(key, "x1034")) {
        if (255==atol(value)) {
            sysPrefs->setDeviceStatus(strtol(key+1, 0, 16), true);
        }
        else {
            sysPrefs->setDeviceStatus(strtol(key+1, 0, 16), false);
        }
        // sysPrefs->forceCacheWrite();
    } else if (!strcmp(key, "x1010")) {
        if (255==atol(value)) {
            sysPrefs->setDeviceStatus(strtol(key+1, 0, 16), true);
        }
        else {
            sysPrefs->setDeviceStatus(strtol(key+1, 0, 16), false);
        }
        // sysPrefs->forceCacheWrite();
    } else if (!strcmp(key, "x1011")) {
        if (255==atol(value)) {
            sysPrefs->setDeviceStatus(strtol(key+1, 0, 16), true);
        }
        else {
            sysPrefs->setDeviceStatus(strtol(key+1, 0, 16), false);
        }
        //sysPrefs->forceCacheWrite();
    } else if (!strcmp(key, "x1012")) {
        if (255==atol(value)) {
            sysPrefs->setDeviceStatus(strtol(key+1, 0, 16), true);
        }
        else {
            sysPrefs->setDeviceStatus(strtol(key+1, 0, 16), false);
        }
        //sysPrefs->forceCacheWrite();
    } else if (!strcmp(key, "x1020")) {
        if (255==atol(value)) {
            sysPrefs->setDeviceStatus(strtol(key+1, 0, 16), true);
        }
        else {
            sysPrefs->setDeviceStatus(strtol(key+1, 0, 16), false);
        }
        sysPrefs->forceCacheWrite();
    } else if (!strcmp(key, "x1040")) {
        if (255==atol(value)) {
            sysPrefs->setDeviceStatus(strtol(key+1, 0, 16), true);
        }
        else {
            sysPrefs->setDeviceStatus(strtol(key+1, 0, 16), false);
        }
        // sysPrefs->forceCacheWrite();
    } else if (!strcmp(key, "x1050")) {
        if (255==atol(value)) {
            sysPrefs->setDeviceStatus(strtol(key+1, 0, 16), true);
        }
        else {
            sysPrefs->setDeviceStatus(strtol(key+1, 0, 16), false);
        }
        // sysPrefs->forceCacheWrite();
    } else if (!strcmp(key, "x2000")) {
        if (255==atol(value)) {
            sysPrefs->setDeviceStatus(strtol(key+1, 0, 16), true);
        }
        else {
            sysPrefs->setDeviceStatus(strtol(key+1, 0, 16), false);
        }
        sysPrefs->forceCacheWrite();
    } else if (!strcmp(key, "x4400")) {
        if (255==atol(value)) {
            sysPrefs->setDeviceStatus(strtol(key+1, 0, 16), true);
        }
        else {
            sysPrefs->setDeviceStatus(strtol(key+1, 0, 16), false);
        }
        // sysPrefs->forceCacheWrite();
        sysPrefs->forceCacheWrite();
    } else if (!strcmp(key, "x6000")) {
        if (255==atol(value)) {
            sysPrefs->setDeviceStatus(strtol(key+1, 0, 16), true);
        }
        else {
            sysPrefs->setDeviceStatus(strtol(key+1, 0, 16), false);
        }
        //   sysPrefs->forceCacheWrite();
    } else if (!strcmp(key, "x650")) {
        if (255==atol(value)) {
            sysPrefs->setDeviceStatus(strtol(key+1, 0, 16), true);
        }
        else {
            sysPrefs->setDeviceStatus(strtol(key+1, 0, 16), false);
        }
        // sysPrefs->forceCacheWrite();

    } else if (!strcmp(key, Constants::logLevel)) {
        extern PrefHandler *sysPrefs;
        uint8_t loglevel = atoi(value);
        Logger::setLoglevel((Logger::LogLevel)loglevel);
        sysPrefs->write(EESYS_LOG_LEVEL, loglevel);
    } else {
        parameterFound = false;
    }
    if (parameterFound) {
        Logger::info(ADABLUE, "parameter change: %s", key);
    }
    else {
        sysPrefs->forceCacheWrite();
        DeviceManager::getInstance()->updateWifi();
    }
}

/*
 * Get parameters from devices and forward them to ichip.
 * This is required to initially set-up the ichip
 */
void ADAFRUITBLE::loadParameters() {
    MotorController *motorController = DeviceManager::getInstance()->getMotorController();
    Throttle *accelerator = DeviceManager::getInstance()->getAccelerator();
    Throttle *brake = DeviceManager::getInstance()->getBrake();
    PotThrottleConfiguration *acceleratorConfig = NULL;
    PotThrottleConfiguration *brakeConfig = NULL;
    MotorControllerConfiguration *motorConfig = NULL;

    Logger::info("loading config params to adafruit ble");

    //DeviceManager::getInstance()->updateWifi();

    if (accelerator)
        acceleratorConfig = (PotThrottleConfiguration *)accelerator->getConfiguration();
    if (brake)
        brakeConfig = (PotThrottleConfiguration *)brake->getConfiguration();
    if (motorController)
        motorConfig = (MotorControllerConfiguration *)motorController->getConfiguration();    
    
    if (acceleratorConfig) {
        gatt.setChar(MeasureCharId[35], acceleratorConfig->numberPotMeters);
        gatt.setChar(MeasureCharId[36], acceleratorConfig->throttleSubType);
        gatt.setChar(MeasureCharId[37], acceleratorConfig->minimumLevel1);
        gatt.setChar(MeasureCharId[38], acceleratorConfig->minimumLevel2);
        gatt.setChar(MeasureCharId[39], acceleratorConfig->maximumLevel1);
        gatt.setChar(MeasureCharId[40], acceleratorConfig->maximumLevel2);
        gatt.setChar(MeasureCharId[41], (uint16_t)(acceleratorConfig->positionRegenMaximum / 10));
        gatt.setChar(MeasureCharId[42], (uint16_t)(acceleratorConfig->positionRegenMinimum / 10));
        gatt.setChar(MeasureCharId[43], (uint16_t)(acceleratorConfig->positionForwardMotionStart / 10));
        gatt.setChar(MeasureCharId[44], (uint16_t)(acceleratorConfig->positionHalfPower / 10));
        gatt.setChar(MeasureCharId[45], acceleratorConfig->minimumRegen);
        gatt.setChar(MeasureCharId[46], acceleratorConfig->maximumRegen);
        gatt.setChar(MeasureCharId[47], acceleratorConfig->creep);                
    }
    if (brakeConfig) {
        gatt.setChar(MeasureCharId[48], brakeConfig->minimumLevel1);
        gatt.setChar(MeasureCharId[49], brakeConfig->maximumLevel1);
        gatt.setChar(MeasureCharId[50], brakeConfig->minimumRegen);
        gatt.setChar(MeasureCharId[51], brakeConfig->maximumRegen);
    }
    if (motorConfig) {
        gatt.setChar(MeasureCharId[52], motorConfig->speedMax);
        gatt.setChar(MeasureCharId[28], motorConfig->coolFan);
        gatt.setChar(MeasureCharId[29], motorConfig->coolOn);
        gatt.setChar(MeasureCharId[30], motorConfig->coolOff);
        gatt.setChar(MeasureCharId[31], motorConfig->brakeLight);
        gatt.setChar(MeasureCharId[32], motorConfig->revLight);
        gatt.setChar(MeasureCharId[33], motorConfig->enableIn);
        gatt.setChar(MeasureCharId[34], motorConfig->reverseIn);
        gatt.setChar(MeasureCharId[25], motorConfig->prechargeR);
        gatt.setChar(MeasureCharId[26], motorConfig->prechargeRelay);
        gatt.setChar(MeasureCharId[27], motorConfig->mainContactorRelay);
        gatt.setChar(MeasureCharId[12], (uint16_t)(motorConfig->nominalVolt));
        gatt.setChar(MeasureCharId[53], (uint16_t)(motorConfig->torqueMax));         
    }
    gatt.setChar(MeasureCharId[54], (uint8_t)Logger::getLogLevel());
}

DeviceType ADAFRUITBLE::getType() {
    return DEVICE_WIFI;
}

DeviceId ADAFRUITBLE::getId() {
    return (ADABLUE);
}

void ADAFRUITBLE::loadConfiguration() {
    BLEConfiguration *config = (BLEConfiguration *)getConfiguration();

    if (prefsHandler->checksumValid()) { //checksum is good, read in the values stored in EEPROM
        Logger::debug(ADABLUE, "Valid checksum so using stored BLE config values");
//		prefsHandler->read(EESYS_WIFI0_SSID, &config->ssid);
    }
}

void ADAFRUITBLE::saveConfiguration() {
    BLEConfiguration *config = (BLEConfiguration *) getConfiguration();

//	prefsHandler->write(EESYS_WIFI0_SSID, config->ssid);
//	prefsHandler->saveChecksum();
}


