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
    {4, 4, "TimeRunning"},
    {2, 2, "TorqueRequest"},
    {2, 2, "TorqueActual"},
    {2, 2, "ThrottleLevel"},
    {2, 2, "BrakeLevel"},
    {2, 2, "SpeedRequested"},
    {2, 2, "SpeedActual"},
    {1, 1, "PowerMode"},
    {1, 1, "Gear"},
    {2, 2, "BusVoltage"},
    {2, 2, "BusCurrent"},
    {2, 2, "MotorCurrent"},
    {2, 2, "NomVoltage"},
    {2, 2, "KWHours"},
    {4, 4, "Bitfield1"},
    {4, 4, "Bitfield2"},
    {4, 4, "Bitfield3"},
    {4, 4, "Bitfield4"},
    {1, 1, "IsRunning"},
    {1, 1, "IsFaulted"},
    {1, 1, "IsWarning"},
    {1, 1, "MotorTemperature"},
    {1, 1, "InverterTemp"},
    {1, 1, "SystemTemp"},
    {2, 2, "MechPower"},
    {2, 2, "PrechargeResist"},
    {1, 1, "PrechargeRelay"},
    {1, 1, "MainContRelay"},
    {1, 1, "CoolFanRelay"},
    {1, 1, "CoolOnTemp"},
    {1, 1, "CoolOffTemp"},
    {1, 1, "BrakeLightOut"},
    {1, 1, "ReverseLightOut"},
    {1, 1, "EnableInput"},
    {1, 1, "ReverseInput"},    
    
    {0, 0, "END"} //leave this at the end - null terminator
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
        MeasureCharId[charCounter] = gatt.addCharacteristic(0x3101 + charCounter, 0x10, charact.minSize, charact.maxSize, BLE_DATATYPE_INTEGER, charact.descript);
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

void ADAFRUITBLE::updateBLE(int CharacteristicID, int MeasuredValue)
{
  /* Command is sent when \n (\r) or println is called */
  /* AT+GATTCHAR=CharacteristicID,value */
  
  ble.print( F("AT+GATTCHAR=") );
  ble.print(CharacteristicID);
  ble.print( F(",") );
  ble.println(MeasuredValue);
  
  /* Check if command executed OK */
  if (!ble.waitForOK()) Logger::error(ADABLUE, "Failed to get response....");
  
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
        //setParam(Constants::bitfield1, paramCache.bitfield1);
        paramCache.bitfield2 = motorController->getStatusBitfield2();
        //setParam(Constants::bitfield2, paramCache.bitfield2);
        // DeviceManager::getInstance()->updateWifiByID(BRUSA_DMC5);

        didParamLoad = true;
    }

    // make small slices so the main loop is not blocked for too long
    if (tickCounter == 1) {
        if (motorController) {
            //Logger::console("Wifi tick counter 1...");

            paramCache.timeRunning = ms;
            //setParam(Constants::timeRunning, getTimeRunning());

            if ( paramCache.torqueRequested != motorController->getTorqueRequested() ) {
                paramCache.torqueRequested = motorController->getTorqueRequested();
                //setParam(Constants::torqueRequested, paramCache.torqueRequested / 10.0f, 1);
            }
            if ( paramCache.torqueActual != motorController->getTorqueActual() ) {
                paramCache.torqueActual = motorController->getTorqueActual();
                //setParam(Constants::torqueActual, paramCache.torqueActual / 10.0f, 1);
            }
        }
        if (accelerator) {
            RawSignalData *rawSignal = accelerator->acquireRawSignal();
            if ( paramCache.throttle !=  rawSignal->input1) {
                paramCache.throttle = rawSignal->input1;
                //setParam(Constants::throttle, paramCache.throttle);

                /*if ( paramCache.throttle != accelerator->getLevel() ) {
                	paramCache.throttle = accelerator->getLevel();
                                    if (paramCache.throttle<-600){paramCache.throttle=-600;}
                	setParam(Constants::throttle, paramCache.throttle / 10.0f, 1);*/
            }
        }
        if (brake) {
            RawSignalData *rawSignal = brake->acquireRawSignal();
            if ( paramCache.brake !=  rawSignal->input1) {
                paramCache.brake = rawSignal->input1;
                paramCache.brakeNotAvailable = false;
                //setParam(Constants::brake, paramCache.brake);

                /*if ( paramCache.brake != brake->getLevel() ) {
                	paramCache.brake = brake->getLevel();
                	paramCache.brakeNotAvailable = false;
                	setParam(Constants::brake, paramCache.brake / 10.0f, 1);*/
            }
        } else {
            if ( paramCache.brakeNotAvailable == true ) {
                paramCache.brakeNotAvailable = false; // no need to keep sending this
                //setParam(Constants::brake, Constants::notAvailable);
            }
        }
    } else if (tickCounter == 2) {
        if (motorController) {
            //Logger::console("Wifi tick counter 2...");
            if ( paramCache.speedRequested != motorController->getSpeedRequested() ) {
                paramCache.speedRequested = motorController->getSpeedRequested();
                //setParam(Constants::speedRequested, paramCache.speedRequested);
            }
            if ( paramCache.speedActual != motorController->getSpeedActual() ) {
                paramCache.speedActual = motorController->getSpeedActual();
                if (paramCache.speedActual<0) paramCache.speedActual=0;
                if (paramCache.speedActual>10000) paramCache.speedActual=10000;
                //setParam(Constants::speedActual, paramCache.speedActual);
            }
            if ( paramCache.dcVoltage != motorController->getDcVoltage() ) {
                paramCache.dcVoltage = motorController->getDcVoltage();
                if(paramCache.dcVoltage<1000) paramCache.dcVoltage=1000;  //Limits of the gage display
                if(paramCache.dcVoltage>4500) paramCache.dcVoltage=4500;

                //setParam(Constants::dcVoltage, paramCache.dcVoltage / 10.0f, 1);
            }
            if ( paramCache.dcCurrent != motorController->getDcCurrent() ) {
                paramCache.dcCurrent = motorController->getDcCurrent();
                //setParam(Constants::dcCurrent, paramCache.dcCurrent / 10.0f, 1);
            }
            if ( paramCache.prechargeR != motorController->getprechargeR() ) {
                paramCache.prechargeR = motorController->getprechargeR();
                //setParam(Constants::prechargeR, (uint16_t)paramCache.prechargeR);
            }

            if ( paramCache.prechargeRelay != motorController->getprechargeRelay() ) {
                paramCache.prechargeRelay = motorController->getprechargeRelay();
                //setParam(Constants::prechargeRelay, (uint8_t) paramCache.prechargeRelay);
                //Logger::console("Precharge Relay %i", paramCache.prechargeRelay);
                //Logger::console("motorController:prechargeRelay:%d, paramCache.prechargeRelay:%d, Constants:prechargeRelay:%s", motorController->getprechargeRelay(),paramCache.prechargeRelay, Constants::prechargeRelay);
            }

            if ( paramCache.mainContactorRelay != motorController->getmainContactorRelay() ) {
                paramCache.mainContactorRelay = motorController->getmainContactorRelay();
                //setParam(Constants::mainContactorRelay, (uint8_t) paramCache.mainContactorRelay);
            }
            //DeviceManager::getInstance()->updateWifi();
        }
    } else if (tickCounter == 3) {
        if (motorController) {
            //Logger::console("Wifi tick counter 2...");
            if ( paramCache.acCurrent != motorController->getAcCurrent() ) {
                paramCache.acCurrent = motorController->getAcCurrent();
                //setParam(Constants::acCurrent, paramCache.acCurrent / 10.0f, 1);
            }

            //if ( paramCache.kiloWattHours != motorController->getkiloWattHours()/3600000 ) {
            paramCache.kiloWattHours = motorController->getKiloWattHours()/3600000;
            if(paramCache.kiloWattHours<0)paramCache.kiloWattHours = 0;
            if(paramCache.kiloWattHours>300)paramCache.kiloWattHours = 300;
            //setParam(Constants::kiloWattHours, paramCache.kiloWattHours / 10.0f, 1);
            //}

            if ( paramCache.nominalVolt != motorController->getnominalVolt()/10 ) {
                paramCache.nominalVolt = motorController->getnominalVolt()/10;
                //setParam(Constants::nominalVolt, paramCache.nominalVolt);
            }

            if ( paramCache.bitfield1 != motorController->getStatusBitfield1() ) {
                paramCache.bitfield1 = motorController->getStatusBitfield1();
                //setParam(Constants::bitfield1, paramCache.bitfield1);
            }
            if ( paramCache.bitfield2 != motorController->getStatusBitfield2() ) {
                paramCache.bitfield2 = motorController->getStatusBitfield2();
                //setParam(Constants::bitfield2, paramCache.bitfield2);
            }
            if ( paramCache.bitfield3 != motorController->getStatusBitfield3() ) {
                paramCache.bitfield3 = motorController->getStatusBitfield3();
                //setParam(Constants::bitfield3, paramCache.bitfield3);
            }
            if ( paramCache.bitfield4 != motorController->getStatusBitfield4() ) {
                paramCache.bitfield4 = motorController->getStatusBitfield4();
                //setParam(Constants::bitfield4, paramCache.bitfield4);
            }

        }
    } else if (tickCounter == 4) {
        if (motorController) {
            // Logger::console("Wifi tick counter 4...");
            if ( paramCache.running != motorController->isRunning() ) {
                paramCache.running = motorController->isRunning();
                //setParam(Constants::running, (paramCache.running ? Constants::trueStr : Constants::falseStr));
            }
            if ( paramCache.faulted != motorController->isFaulted() ) {
                paramCache.faulted = motorController->isFaulted();
                //setParam(Constants::faulted, (paramCache.faulted ? Constants::trueStr : Constants::falseStr));
            }
            if ( paramCache.warning != motorController->isWarning() ) {
                paramCache.warning = motorController->isWarning();
                //setParam(Constants::warning, (paramCache.warning ? Constants::trueStr : Constants::falseStr));
            }
            if ( paramCache.gear != motorController->getSelectedGear() ) {
                paramCache.gear = motorController->getSelectedGear();
                //setParam(Constants::gear, (uint16_t)paramCache.gear);
            }

            if ( paramCache.coolFan != motorController->getCoolFan() ) {
                paramCache.coolFan = motorController->getCoolFan();
                //setParam(Constants::coolFan, (uint8_t) paramCache.coolFan);
            }

            if ( paramCache.coolOn != motorController->getCoolOn() ) {
                paramCache.coolOn = motorController->getCoolOn();
                //setParam(Constants::coolOn, (uint8_t) paramCache.coolOn);
            }

            if ( paramCache.coolOff != motorController->getCoolOff() ) {
                paramCache.coolOff = motorController->getCoolOff();
                //setParam(Constants::coolOff, (uint8_t) paramCache.coolOff);
            }

            if ( paramCache.brakeLight != motorController->getBrakeLight() ) {
                paramCache.brakeLight = motorController->getBrakeLight();
               // setParam(Constants::brakeLight, (uint8_t) paramCache.brakeLight);
            }

            if ( paramCache.revLight != motorController->getRevLight() ) {
                paramCache.revLight = motorController->getRevLight();
                //setParam(Constants::revLight, (uint8_t) paramCache.revLight);
            }

            if ( paramCache.enableIn != motorController->getEnableIn() ) {
                paramCache.enableIn = motorController->getEnableIn();
                //setParam(Constants::enableIn, (uint8_t) paramCache.enableIn);
            }
            if ( paramCache.reverseIn != motorController->getReverseIn() ) {
                paramCache.reverseIn = motorController->getReverseIn();
                //setParam(Constants::reverseIn, (uint8_t) paramCache.reverseIn);
            }

        }
    } else if (tickCounter > 4) {
        if (motorController) {
            // Logger::console("Wifi tick counter 5...");
            if ( paramCache.tempMotor != motorController->getTemperatureMotor() ) {
                paramCache.tempMotor = motorController->getTemperatureMotor();
                //setParam(Constants::tempMotor, paramCache.tempMotor / 10.0f, 1);
            }
            if ( paramCache.tempInverter != motorController->getTemperatureInverter() ) {
                paramCache.tempInverter = motorController->getTemperatureInverter();
                //setParam(Constants::tempInverter, paramCache.tempInverter / 10.0f, 1);
            }
            if ( paramCache.tempSystem != motorController->getTemperatureSystem() ) {
                paramCache.tempSystem = motorController->getTemperatureSystem();
                //setParam(Constants::tempSystem, paramCache.tempSystem / 10.0f, 1);
            }

            if (paramCache.powerMode != motorController->getPowerMode() ) {
                paramCache.powerMode = motorController->getPowerMode();
                //setParam(Constants::motorMode, (uint8_t)paramCache.powerMode);
            }

            //if ( paramCache.mechPower != motorController->getMechanicalPower() ) {
            paramCache.mechPower = motorController->getMechanicalPower();
            if (paramCache.mechPower<-250)paramCache.mechPower=-250;
            if (paramCache.mechPower>1500)paramCache.mechPower=1500;
            //setParam(Constants::mechPower, paramCache.mechPower / 10.0f, 1);
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
    case MSG_CONFIG_CHANGE: { //Loads all parameters to web site
        loadParameters();
        break;
    }
    case MSG_COMMAND:  //Sends a message to the WiReach module in the form of AT+imessage
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
        /*
        setParam(Constants::numThrottlePots, acceleratorConfig->numberPotMeters);
        setParam(Constants::throttleSubType, acceleratorConfig->throttleSubType);
        setParam(Constants::throttleMin1, acceleratorConfig->minimumLevel1);
        setParam(Constants::throttleMin2, acceleratorConfig->minimumLevel2);
        setParam(Constants::throttleMax1, acceleratorConfig->maximumLevel1);
        setParam(Constants::throttleMax2, acceleratorConfig->maximumLevel2);
        setParam(Constants::throttleRegenMax, (uint16_t)(acceleratorConfig->positionRegenMaximum / 10));
        setParam(Constants::throttleRegenMin, (uint16_t)(acceleratorConfig->positionRegenMinimum / 10));
        setParam(Constants::throttleFwd, (uint16_t)(acceleratorConfig->positionForwardMotionStart / 10));
        setParam(Constants::throttleMap, (uint16_t)(acceleratorConfig->positionHalfPower / 10));
        setParam(Constants::throttleMinRegen, acceleratorConfig->minimumRegen);
        setParam(Constants::throttleMaxRegen, acceleratorConfig->maximumRegen);
        setParam(Constants::throttleCreep, acceleratorConfig->creep);
        */
    }
    if (brakeConfig) {
        /*
        setParam(Constants::brakeMin, brakeConfig->minimumLevel1);
        setParam(Constants::brakeMax, brakeConfig->maximumLevel1);
        setParam(Constants::brakeMinRegen, brakeConfig->minimumRegen);
        setParam(Constants::brakeMaxRegen, brakeConfig->maximumRegen);
        */
    }
    if (motorConfig) {
        /*
        setParam(Constants::speedMax, motorConfig->speedMax);
        setParam(Constants::coolFan, motorConfig->coolFan);
        setParam(Constants::coolOn, motorConfig->coolOn);
        setParam(Constants::coolOff, motorConfig->coolOff);
        setParam(Constants::brakeLight, motorConfig->brakeLight);
        setParam(Constants::revLight, motorConfig->revLight);
        setParam(Constants::enableIn, motorConfig->enableIn);
        setParam(Constants::reverseIn, motorConfig->reverseIn);
        setParam(Constants::prechargeR, motorConfig->prechargeR);
        setParam(Constants::prechargeRelay, motorConfig->prechargeRelay);
        setParam(Constants::mainContactorRelay, motorConfig->mainContactorRelay);
        uint16_t nmvlt = motorConfig->nominalVolt/10;
        setParam(Constants::nominalVolt, nmvlt);
        setParam(Constants::torqueMax, (uint16_t)(motorConfig->torqueMax / 10)); // skip the tenth's
        */
    }
    //setParam(Constants::logLevel, (uint8_t)Logger::getLogLevel());


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
        Logger::debug(ADABLUE, "Valid checksum so using stored wifi config values");
        //TODO: implement processing of config params for WIFI
//		prefsHandler->read(EESYS_WIFI0_SSID, &config->ssid);
    }
}

void ADAFRUITBLE::saveConfiguration() {
    BLEConfiguration *config = (BLEConfiguration *) getConfiguration();

    //TODO: implement processing of config params for WIFI
//	prefsHandler->write(EESYS_WIFI0_SSID, config->ssid);
//	prefsHandler->saveChecksum();
}


