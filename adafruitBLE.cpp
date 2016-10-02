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

ADAFRUITBLE *adaRef;

/*
 * Extra things that should be sent but perhaps aren't:
 * Enabled/Disabled device drivers
 */

/*
 * formed as a standard C array right now. Sort of dangerous in that there is no good way
 * to know the length... well, you can grab the sizeof operator with division
 * but instead this array is null terminated sort of like a C string.
 * Characteristics can be up to 20 bytes so stay under that.
*/
Characteristic characteristics[] = 
{
    {BLE_DATATYPE_BYTEARRAY, 4, 4, 0x12, "TimeRunning", {GATT_PRESENT_FORMAT_UINT32, 0, GATT_PRESENT_UNIT_TIME_SECOND, 1, 0}}, //MeasureCharId[0]
    
    //Requested torque followed by actual torque, both signed 16 bit
    {BLE_DATATYPE_BYTEARRAY, sizeof(BLETrqReqAct) - 1, sizeof(BLETrqReqAct) - 1, 0x12,  "TrqReqAct", {GATT_PRESENT_FORMAT_STRUCT, -1, GATT_PRESENT_UNIT_MOMENT_OF_FORCE_NEWTON_METRE, 1, 0}}, //1
    
    //First the throttle level, then the brake level, both 16 bit values
    {BLE_DATATYPE_BYTEARRAY, sizeof(BLEThrBrkLevels) - 1, sizeof(BLEThrBrkLevels) - 1, 0x12,  "ThrBrkLevels", {GATT_PRESENT_FORMAT_STRUCT, 0, GATT_PRESENT_UNIT_NONE, 1, 0}}, //2
    
    //First requested speed then actual speed, both 16 bit
    {BLE_DATATYPE_BYTEARRAY, sizeof(BLESpeeds) - 1, sizeof(BLESpeeds) - 1, 0x12,  "Speed", {GATT_PRESENT_FORMAT_STRUCT, 0, GATT_PRESENT_UNIT_ANGULAR_VELOCITY_REVOLUTION_PER_MINUTE, 1, 0}}, //3
    
    //PowerMode, then Gear, then IsRunning then isFaulted, then IsWarning, lastly LogLevel all 1 byte each
    {BLE_DATATYPE_BYTEARRAY, sizeof(BLEModes) - 1, sizeof(BLEModes) - 1, 0x1A,  "Modes", {GATT_PRESENT_FORMAT_STRUCT, 0, GATT_PRESENT_UNIT_NONE, 1, 0}}, //MeasureCharId[4]

    //Bus Voltage then Bus Current, then motor current, then kwhours, then mechanical power all 16 bit with currents being signed.
    {BLE_DATATYPE_BYTEARRAY, sizeof(BLEPowerStatus) - 1, sizeof(BLEPowerStatus) - 1, 0x12,  "PowerStatus", {GATT_PRESENT_FORMAT_STRUCT, -1, GATT_PRESENT_UNIT_NONE, 1, 0}}, //5
        
    //Bitfield 1 through 4 all in the same characteristic. All 32 bit
    {BLE_DATATYPE_BYTEARRAY, sizeof(BLEBitfields) - 1, sizeof(BLEBitfields) - 1, 0x12,  "Bitfields", {GATT_PRESENT_FORMAT_STRUCT, 0, GATT_PRESENT_UNIT_NONE, 1, 0}}, //MeasureCharId[6]

    //Temperatures - Motor, Inverter, System. All 16 bit
    {BLE_DATATYPE_BYTEARRAY, sizeof(BLETemperatures) - 1, sizeof(BLETemperatures) - 1, 0x12,  "Temperatures", {GATT_PRESENT_FORMAT_STRUCT, -1, GATT_PRESENT_UNIT_THERMODYNAMIC_TEMPERATURE_DEGREE_CELSIUS, 1, 0}}, //7
    
    //PrechargeResist(2), PrechargeRelay(1), MainContRelay(1), CoolFanRelay(1), CoolOnTemp(1), coolOffTemp(1), BrakeLightOut(1), ReverseLightOut(1), EnableIn(1), ReverseIn(1)
    {BLE_DATATYPE_BYTEARRAY, sizeof(BLEDigIO) - 1, sizeof(BLEDigIO) - 1, 0x1A,  "DigIO", {GATT_PRESENT_FORMAT_STRUCT, 0, GATT_PRESENT_UNIT_NONE, 1, 0}}, //8
    
    //NumThrPots(1), ThrottleType(1), T1Min(2), T2Min(2), T1Max(2), T2Max(2)   
    {BLE_DATATYPE_BYTEARRAY, sizeof(BLEThrottleIO) - 1, sizeof(BLEThrottleIO) - 1, 0x1A,  "ThrottleIO", {GATT_PRESENT_FORMAT_STRUCT, 0, GATT_PRESENT_UNIT_NONE, 1, 0}}, //9
     
    //ThrottleRegenMax(2), ThrRegenMin(2), ThrottleFwd(2), ThrottleMap(2), TRegenMin(1), TRegenMax(1), TCreep(1)
    {BLE_DATATYPE_BYTEARRAY, sizeof(BLEThrottleMap) - 1, sizeof(BLEThrottleMap) - 1, 0x1A,  "ThrottleMap", {GATT_PRESENT_FORMAT_STRUCT, 0, GATT_PRESENT_UNIT_NONE, 1, 0}}, //10

    //BrakeMin(2), BrakeMax(2), BRegenMin(1), BRegenMax(1)
    {BLE_DATATYPE_BYTEARRAY, sizeof(BLEBrakeParam) - 1, sizeof(BLEBrakeParam) - 1, 0x1A,  "BrakeParam", {GATT_PRESENT_FORMAT_STRUCT, 0, GATT_PRESENT_UNIT_NONE, 1, 0}}, //11
        
    //NominalVoltage(2), MaxRPM(2), MaxTorque(2)
    {BLE_DATATYPE_BYTEARRAY, sizeof(BLEMaxParams) - 1, sizeof(BLEMaxParams) - 1, 0x1A,  "MaxParams", {GATT_PRESENT_FORMAT_STRUCT, 0, GATT_PRESENT_UNIT_NONE, 1, 0}}, //12
    
    //Two 32 bit bitfields that contain the current enabled/disabled status of all the possible devices.
    {BLE_DATATYPE_BYTEARRAY, sizeof(BLEDeviceEnable) - 1, sizeof(BLEDeviceEnable) - 1, 0x1A, "EnabledDevices", {GATT_PRESENT_FORMAT_STRUCT, 0, GATT_PRESENT_UNIT_NONE, 1, 0}}, //13
             
    {BLE_DATATYPE_INTEGER, 0, 0, 0, "END", {GATT_PRESENT_FORMAT_UINT8, 0, GATT_PRESENT_UNIT_NONE, 1, 0}} //leave this at the end - null terminator
};

/*
 * This next array allows for a static ordering of devices. It is used to form the bitfields for the enabled vs disabled devices. If
 * a bit is set then the device is enabled. Otherwise disabled.
 * DO NOT REORDER THIS LIST OR REMOVE ANY! If you have new devices add them to the end. But, never remove or reorder!
 * Obviously though, leave the 0 at the end. That's the extent of reordering - add before the 0 at the end
 */
uint16_t deviceTable[] = 
{
    DMOC645,        //0
    BRUSA_DMC5,     //1
    CODAUQM,        //2
    CKINVERTER,     //3
    TESTINVERTER,   //4
    BRUSACHARGE,    //5
    TCCHCHARGE,     //6
    LEAR,           //7
    POTACCELPEDAL,  //8
    POTBRAKEPEDAL,  //9
    CANACCELPEDAL,  //10
    CANBRAKEPEDAL,  //11
    TESTACCEL,      //12
    EVICTUS,        //13
    ADABLUE,        //14
    THINKBMS,       //15
    PIDLISTENER,    //16
    ELM327EMU,      //17
    0               //NULL terminator
};

void BleGattRX(int32_t chars_id, uint8_t data[], uint16_t len)
{
    adaRef->gattRX(chars_id, data, len);
    Logger::debug(ADABLUE, "Got GATT update for char: %i", chars_id);
}
   
/*
 * Constructor. Assign serial interface to use for ichip communication
 */
ADAFRUITBLE::ADAFRUITBLE() {
    prefsHandler = new PrefHandler(ADABLUE);

    uint8_t sys_type;
    sysPrefs->read(EESYS_SYSTEM_TYPE, &sys_type);

    commonName = "Adafruit BLE";
    needParamReload = false;
    bOkToWrite = false;
    okWriteCounter = 0;
    
    memset(&bleTrqReqAct, 0, sizeof(bleTrqReqAct));
    memset(&bleThrBrkLevels, 0, sizeof(bleThrBrkLevels));
    memset(&bleSpeeds, 0, sizeof(bleSpeeds));
    memset(&bleModes, 0, sizeof(bleModes));
    memset(&blePowerStatus, 0, sizeof(blePowerStatus));
    memset(&bleBitFields, 0, sizeof(bleBitFields));
    memset(&bleTemperatures, 0, sizeof(bleTemperatures));
    memset(&bleDigIO, 0, sizeof(bleDigIO));
    memset(&bleThrottleIO, 0, sizeof(bleThrottleIO));
    memset(&bleThrottleMap, 0, sizeof(bleThrottleMap));
    memset(&bleBrakeParam, 0, sizeof(bleBrakeParam));
    memset(&bleMaxParams, 0, sizeof(bleMaxParams));
    memset(&bleDeviceEnable, 0, sizeof(bleDeviceEnable));
    
    //Ok, this is sort of like a singleton pattern but technically it'd be possible to
    //instantiate more than once. So, don't do it. But, GEVCU doesn't do it so we're OK
    //even though this is really kludgey. Ignore that. Nothing to see here.
    adaRef = this;
}

/*
 * Initialization of hardware and parameters
 */
void ADAFRUITBLE::setup() {

    Logger::info(ADABLUE, "add device: AdaFruit BLE (id: %X, %X)", ADABLUE, this);

    tickHandler.detach(this);

    tickCounter = 0;

    paramCache.brakeNotAvailable = true;
        
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
    
    //need to wait 1 second after begin, done via polling instead of a hard delay.
    resetTime = millis();
    didResetInit = false;
    
    tickHandler.attach(this, CFG_TICK_INTERVAL_BLE);
    
    attachInterrupt(digitalPinToInterrupt(27), BLEInterrupt, RISING);
}

void BLEInterrupt()
{
    //SerialUSB.println("Y!");
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
    
    Logger::debug("Factory Resetting BLE");
    ble.factoryReset();
  
    //Set peripheral transmit power level -40 minimum -20 -16 -12 -8 -4 0 4 maximum
    if (! ble.sendCommandCheckOK(F("AT+BLEPOWERLEVEL=4")) ) 
    {
        Logger::error(ADABLUE, "Could not set device power level...");
        return;
    }

    //Set peripheral name
    if (! ble.sendCommandCheckOK(F("AT+GAPDEVNAME=GEVCU 6.2 ECU")) ) 
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
    else Logger::debug(ADABLUE, "Service ID: %i", ServiceId);

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
        MeasureCharId[charCounter] = gatt.addCharacteristic(0x3101 + charCounter, charact.properties, charact.minSize, charact.maxSize, charact.dataType, charact.descript, &charact.present);
        if (MeasureCharId[charCounter] == 0) 
        {
            Logger::error(ADABLUE, "Could not add characteristic %x", 0x3101 + charCounter);
            //return;            
        }
        else Logger::debug(ADABLUE, "Added characteristic %x id %x", 0x3101 + charCounter, MeasureCharId[charCounter]);
        charact = characteristics[++charCounter];
    }           
    
    /* Add the  Service to the advertising data (needed for Nordic apps to detect the service) */
    ble.sendCommandCheckOK( F("AT+GAPSETADVDATA=02-01-06-05-02-00-31-10-31") );

    Logger::debug("Resetting BLE to enable new settings.");
    /* Reset the device for the new service setting changes to take effect */
    ble.reset();
}

void ADAFRUITBLE::dumpRawData(uint8_t* data, int len)
{
    for (int i = 0; i < len; i++)
    {
        Logger::debug(ADABLUE, "Byte %i = %x", i, data[i]);
    }
}

/*
 * Transfers any changed characteristics over SPI to the BLE module. transCounter is used
 * to periodically transfer each characteristic even if it isn't set as changed. This is because
 * sometimes updates get lost either on our side or the BLE side and so rarely changing values
 * can sometimes get lost if you don't occassionally update each characteristic to make sure it's
 * still fresh.
 */
void ADAFRUITBLE::transferUpdates()
{
    static int transCounter = 0;
    
    if (needParamReload) transCounter++;
    
    if (bleTrqReqAct.doUpdate != 0 || transCounter == 2)
    {
        bleTrqReqAct.doUpdate = 0;
        if (!gatt.setChar(2, (uint8_t *)&bleTrqReqAct, sizeof(bleTrqReqAct) - 1))
        {
            Logger::error("Could not update bleTrqReqAct 1");
        }
        else Logger::debug(ADABLUE, "Updated bleTrqReqAct 1");
        if (!bOkToWrite) okWriteCounter++;
        //dumpRawData((uint8_t *)&bleTrqReqAct, sizeof(bleTrqReqAct) - 1);
    }
    
    if (bleThrBrkLevels.doUpdate != 0 || transCounter == 4)
    {
        bleThrBrkLevels.doUpdate = 0;
        if (!gatt.setChar(3, (uint8_t *)&bleThrBrkLevels, sizeof(bleThrBrkLevels) - 1))
        {
            Logger::error("Could not update bleThrBrkLevels 2");
        }

        else Logger::debug(ADABLUE, "Updated bleThrBrkLevels 2");
        if (!bOkToWrite) okWriteCounter++;
        //dumpRawData((uint8_t *)&bleThrBrkLevels, sizeof(bleThrBrkLevels) - 1);
    }
    
    if (bleSpeeds.doUpdate != 0 || transCounter == 6)
    {
        bleSpeeds.doUpdate = 0;
        if (!gatt.setChar(4, (uint8_t *)&bleSpeeds, sizeof(bleSpeeds) - 1))
        {
            Logger::error("Could not update bleSpeeds 3");
        }            
        else Logger::debug(ADABLUE, "Updated bleSpeeds 3");
        if (!bOkToWrite) okWriteCounter++;
        //dumpRawData((uint8_t *)&bleSpeeds, sizeof(bleSpeeds) - 1);
    }
    
    if (bleModes.doUpdate != 0  || transCounter == 8)
    {
        bleModes.doUpdate = 0;
        if (!gatt.setChar(5, (uint8_t *)&bleModes, sizeof(bleModes) - 1))
        {
            Logger::error("Could not update bleModes 4");
        }            
        else Logger::debug(ADABLUE, "Updated bleModes 4");
        if (!bOkToWrite) okWriteCounter++;
        //dumpRawData((uint8_t *)&bleModes, sizeof(bleModes) - 1);
    }
    
    if (blePowerStatus.doUpdate != 0  || transCounter == 10)
    {
        blePowerStatus.doUpdate = 0;
        if (!gatt.setChar(6, (uint8_t *)&blePowerStatus, sizeof(blePowerStatus) - 1))
        {
            Logger::error("Could not update blePowerStatus 5");
        }            
        else Logger::debug(ADABLUE, "Updated blePowerStatus 5");
        if (!bOkToWrite) okWriteCounter++;
        //dumpRawData((uint8_t *)&blePowerStatus, sizeof(blePowerStatus) - 1);
    }
    
    if (bleBitFields.doUpdate != 0  || transCounter == 12)
    {
        bleBitFields.doUpdate = 0;
        if (!gatt.setChar(7, (uint8_t *)&bleBitFields, sizeof(bleBitFields) - 1))
        {
            Logger::error("Could not update bleBitFields 6");
        }            
        else Logger::debug(ADABLUE, "Updated bleBitFields 6");
        if (!bOkToWrite) okWriteCounter++;
        //dumpRawData((uint8_t *)&bleBitFields, sizeof(bleBitFields) - 1);
    }
    
    if (bleTemperatures.doUpdate != 0  || transCounter == 14)
    {
        bleTemperatures.doUpdate = 0;
        if (!gatt.setChar(8, (uint8_t *)&bleTemperatures, sizeof(bleTemperatures) - 1))
        {
            Logger::error("Could not update bleTemperatures 7");
        }            
        else Logger::debug(ADABLUE, "Updated bleTemperatures 7");
        if (!bOkToWrite) okWriteCounter++;
        //dumpRawData((uint8_t *)&bleTemperatures, sizeof(bleTemperatures) - 1);
    }
     
    if (bleDigIO.doUpdate != 0  || transCounter == 16)
    {
        bleDigIO.doUpdate = 0;
        if (!gatt.setChar(9, (uint8_t *)&bleDigIO, sizeof(bleDigIO) - 1))
        {
            Logger::error("Could not update bleDigIO 8");
        }            
        else Logger::debug(ADABLUE, "Updated bleDigIO 8");
        if (!bOkToWrite) okWriteCounter++;
        //dumpRawData((uint8_t *)&bleDigIO, sizeof(bleDigIO) - 1);
    }
        
    if (bleThrottleIO.doUpdate != 0  || transCounter == 18)
    {
        bleThrottleIO.doUpdate = 0;
        if (!gatt.setChar(10, (uint8_t *)&bleThrottleIO, sizeof(bleThrottleIO) - 1))
        {
            Logger::error("Could not update bleThrottleIO 9");
        }            
        else Logger::debug(ADABLUE, "Updated bleThrottleIO 9");
        if (!bOkToWrite) okWriteCounter++;
        //dumpRawData((uint8_t *)&bleThrottleIO, sizeof(bleThrottleIO) - 1);
    }
    
    if (bleThrottleMap.doUpdate != 0  || transCounter == 20)
    {
        bleThrottleMap.doUpdate = 0;
        if (!gatt.setChar(11, (uint8_t *)&bleThrottleMap, sizeof(bleThrottleMap) - 1))
        {
            Logger::error("Could not update bleThrottleMap 10");
        }            
        else Logger::debug(ADABLUE, "Updated bleThrottleMap 10");
        if (!bOkToWrite) okWriteCounter++;
        //dumpRawData((uint8_t *)&bleThrottleMap, sizeof(bleThrottleMap) - 1);
    }    

    if (bleBrakeParam.doUpdate != 0  || transCounter == 22)
    {
        bleBrakeParam.doUpdate = 0;
        if (!gatt.setChar(12, (uint8_t *)&bleBrakeParam, sizeof(bleBrakeParam) - 1))
        {
            Logger::error("Could not update bleBrakeParam 11");
        }            
        else Logger::debug(ADABLUE, "Updated bleBrakeParam 11");
        if (!bOkToWrite) okWriteCounter++;
        //dumpRawData((uint8_t *)&bleBrakeParam, sizeof(bleBrakeParam) - 1);
    }
    
    if (bleMaxParams.doUpdate != 0  || transCounter == 24)
    {
        bleMaxParams.doUpdate = 0;
        if (!gatt.setChar(13, (uint8_t *)&bleMaxParams, sizeof(bleMaxParams) - 1))
        {
            Logger::error("Could not update bleMaxParams 12");
        }            
        else Logger::debug(ADABLUE, "Updated bleMaxParams 12");
        if (!bOkToWrite) okWriteCounter++;
        //dumpRawData((uint8_t *)&bleMaxParams, sizeof(bleMaxParams) - 1);
    }
    
    if (bleDeviceEnable.doUpdate != 0  || transCounter > 26)
    {
        bleDeviceEnable.doUpdate = 0;
        transCounter = 0; //have to reset it at the last characteristic which is currently this one.
        if (!gatt.setChar(14, (uint8_t *)&bleDeviceEnable, sizeof(bleDeviceEnable) - 1))
        {
            Logger::error("Could not update bleDeviceEnable 13");
        }            
        else Logger::debug(ADABLUE, "Updated bleDeviceEnable 13");
        if (!bOkToWrite) okWriteCounter++;
        //dumpRawData((uint8_t *)&bleDeviceEnable, sizeof(bleDeviceEnable) - 1);        
        needParamReload = false;
    }
}

/*
 * Periodic updates of parameters to ichip RAM.
 * Also query for changed parameters of the config page.
 */
//TODO: See the processing function below for a more detailed explanation - can't send so many setParam commands in a row
void ADAFRUITBLE::handleTick() {
    uint32_t ms = millis();
    
    if ( (ms > (resetTime + 1100)) && !didResetInit )
    {
        ble.echo(false);
    
        Logger::debug("Setting GATT callback 4");
        ble.setBleGattRxCallback(5, BleGattRX);
    
        //1  2  3  4  5     6     7     8     9      10     11     12     13
        //1, 2, 4, 8, 0x10, 0x20, 0x40, 0x80, 0x100  0x200  0x400  0x800  0x1000
        Logger::debug("Setting GATT callback 8");
        if (!ble.sendCommandCheckOK("AT+EVENTENABLE=0x0,0x100")) //MeasureCharId[8]
        {
            Logger::error("FAILED!");
        }

        Logger::debug("Setting GATT callback 9");
        if (!ble.sendCommandCheckOK("AT+EVENTENABLE=0x0,0x200")) //MeasureCharId[9]
        {
            Logger::error("FAILED!");
        }
    
        Logger::debug("Setting GATT callback 10");
        if (!ble.sendCommandCheckOK("AT+EVENTENABLE=0x0,0x400")) //MeasureCharId[10]
        {
            Logger::error("FAILED!");
        }
    
        Logger::debug("Setting GATT callback 11");
        if (!ble.sendCommandCheckOK("AT+EVENTENABLE=0x0,0x800")) //MeasureCharId[11]
        {
            Logger::error("FAILED!");
        }
    
        Logger::debug("Setting GATT callback 12");
        if (!ble.sendCommandCheckOK("AT+EVENTENABLE=0x0,0x1000")) //MeasureCharId[12]
        {
            Logger::error("FAILED!");
        } 
    
        Logger::debug("Setting GATT callback 13");
        if (!ble.sendCommandCheckOK("AT+EVENTENABLE=0x0,0x2000")) //MeasureCharId[13]
        {
            Logger::error("FAILED!");
        } 
        Logger::debug("Done with callbacks");
        didResetInit = true;
    }    
    
    if (ms < (resetTime + 2000)) return;
    
    MotorController* motorController = deviceManager.getMotorController();
    Throttle *accelerator = deviceManager.getAccelerator();
    Throttle *brake = deviceManager.getBrake();
    BatteryManager *bms = reinterpret_cast<BatteryManager *>(deviceManager.getDeviceByType(DEVICE_BMS));
    
    PotThrottleConfiguration *acceleratorConfig = NULL;
    PotThrottleConfiguration *brakeConfig = NULL;
    MotorControllerConfiguration *motorConfig = NULL;
    
    uint32_t IOTemp = 0;
    uint8_t brklt;
    tickCounter++;
    
    if (accelerator)
        acceleratorConfig = (PotThrottleConfiguration *)accelerator->getConfiguration();
    if (brake)
        brakeConfig = (PotThrottleConfiguration *)brake->getConfiguration();
    if (motorController)
        motorConfig = (MotorControllerConfiguration *)motorController->getConfiguration();  
        
    ble.update(200); //check for updates every 200ms. Does nothing until 200ms has passed since last time.

    if (paramCache.timeRunning != (ms / 1000))
    {
        paramCache.timeRunning = ms / 1000;
        if (!gatt.setChar(1, (uint8_t*)&paramCache.timeRunning, 4))
        {
            Logger::error("Could not update timeRunning");
        }
        else Logger::debug(ADABLUE, "Updated timeRunning");
        //dumpRawData((uint8_t *)&paramCache.timeRunning, 4);
    }
    
    if (okWriteCounter > 40 && !bOkToWrite) bOkToWrite = true;
    
    //every other time - 80ms by default
    if (tickCounter & 1)
    {
        if (motorController) {
            if ( bleTrqReqAct.torqueRequested  != motorController->getTorqueRequested() ) {
                bleTrqReqAct.torqueRequested = motorController->getTorqueRequested();
                bleTrqReqAct.doUpdate = 1;
            }
            if ( bleTrqReqAct.torqueActual != motorController->getTorqueActual() ) {
                bleTrqReqAct.torqueActual = motorController->getTorqueActual();
                bleTrqReqAct.doUpdate = 1;
            }
            if ( bleSpeeds.speedRequested != motorController->getSpeedRequested() ) {
                bleSpeeds.speedRequested = motorController->getSpeedRequested();
                bleSpeeds.doUpdate = 1;
            }
            
            if ( bleSpeeds.speedActual != motorController->getSpeedActual() ) {
                bleSpeeds.speedActual = motorController->getSpeedActual();
                bleSpeeds.doUpdate = 1;
            }
        }
    
        if (accelerator) {
            RawSignalData *rawSignal = accelerator->acquireRawSignal();
            if ( bleThrBrkLevels.throttleRawLevel1 !=  rawSignal->input1) 
            {
                bleThrBrkLevels.throttleRawLevel1 = rawSignal->input1;
                bleThrBrkLevels.doUpdate = 1;
            }
            if ( bleThrBrkLevels.throttleRawLevel2 !=  rawSignal->input2) 
            {
                bleThrBrkLevels.throttleRawLevel2 = rawSignal->input2;
                bleThrBrkLevels.doUpdate = 1;
            }
            if (bleThrBrkLevels.throttlePercentage != accelerator->getLevel() / 10)
            {
                bleThrBrkLevels.throttlePercentage = accelerator->getLevel() / 10;
                bleThrBrkLevels.doUpdate = 1;
            }
        }
        if (brake) {
            RawSignalData *rawSignal = brake->acquireRawSignal();
            if (bleThrBrkLevels.brakeRawLevel !=  rawSignal->input1) {
                bleThrBrkLevels.brakeRawLevel = rawSignal->input1;
                paramCache.brakeNotAvailable = false;
                bleThrBrkLevels.doUpdate = 1;
            }
            if (bleThrBrkLevels.brakePercentage !=  brake->getLevel() / 10) {
                bleThrBrkLevels.brakePercentage = brake->getLevel() / 10;
                bleThrBrkLevels.doUpdate = 1;
            }            
        } else {
            if ( paramCache.brakeNotAvailable == true ) {
                paramCache.brakeNotAvailable = false; // no need to keep sending this
                bleThrBrkLevels.brakeRawLevel = 0;
                bleThrBrkLevels.doUpdate = 1;
            }
        }
    }
    
    if (tickCounter == 2) {        
        if (bms)
        {
            if (blePowerStatus.SOC != bms->getSOC())
            {
                blePowerStatus.SOC = bms->getSOC();
                blePowerStatus.doUpdate = 1;
            }
        }
        if (motorController) {
            //Logger::console("Wifi tick counter 2...");            
            if ( blePowerStatus.busVoltage != motorController->getDcVoltage() ) {
                blePowerStatus.busVoltage = motorController->getDcVoltage();
                if(blePowerStatus.busVoltage<0) blePowerStatus.busVoltage=0; 
                //if(blePowerStatus.busVoltage>4500) blePowerStatus.busVoltage=4500;
                blePowerStatus.doUpdate = 1;
            }
            
            if ( blePowerStatus.busCurrent != motorController->getDcCurrent() ) {
                blePowerStatus.busCurrent = motorController->getDcCurrent();
                blePowerStatus.doUpdate = 1;
            }
            
            if ( blePowerStatus.motorCurrent != motorController->getAcCurrent() ) {
                blePowerStatus.motorCurrent = motorController->getAcCurrent();
                blePowerStatus.doUpdate = 1;
            }

            if ( blePowerStatus.kwHours != motorController->getKiloWattHours()/3600000 ) {
                blePowerStatus.kwHours = motorController->getKiloWattHours()/3600000;
                if(blePowerStatus.kwHours<0)blePowerStatus.kwHours = 0;
                if(blePowerStatus.kwHours>300)blePowerStatus.kwHours = 300;
                blePowerStatus.doUpdate = 1;
            }
            
            if ( blePowerStatus.mechPower != motorController->getMechanicalPower() ) {
                blePowerStatus.mechPower = motorController->getMechanicalPower();
                if (blePowerStatus.mechPower<-250)blePowerStatus.mechPower=-250;
                if (blePowerStatus.mechPower>1500)blePowerStatus.mechPower=1500;
                blePowerStatus.doUpdate = 1;
            }
        }

    } else if (tickCounter == 3) {
        if (motorController) {
            //Logger::console("Wifi tick counter 3...");

            if ( bleMaxParams.nomVoltage != motorController->getnominalVolt() ) {
                bleMaxParams.nomVoltage = motorController->getnominalVolt();
                bleMaxParams.doUpdate = 1;
            }
            
            if ( bleMaxParams.maxTorque != motorConfig->torqueMax ) {
                bleMaxParams.maxTorque = motorConfig->torqueMax;
                bleMaxParams.doUpdate = 1;
            }
            
            if ( bleMaxParams.maxRPM != motorConfig->speedMax ) {
                bleMaxParams.maxRPM = motorConfig->speedMax;
                bleMaxParams.doUpdate = 1;
            }

            if ( bleBitFields.bitfield1 != motorController->getStatusBitfield1() ) {
                bleBitFields.bitfield1 = motorController->getStatusBitfield1();
                bleBitFields.doUpdate = 1;
            }
            if ( bleBitFields.bitfield2 != motorController->getStatusBitfield2() ) {
                bleBitFields.bitfield2 = motorController->getStatusBitfield2();
                bleBitFields.doUpdate = 1;
            }
            
            IOTemp = 0;
            for (int i = 0; i < 4; i++)
            {
                if (systemIO.getDigitalIn(i)) bleBitFields.digitalInputs |= 1 << i;
            }            
                        
            if ( bleBitFields.digitalInputs != IOTemp ) {
                bleBitFields.digitalInputs = IOTemp;
                bleBitFields.doUpdate = 1;
            }
            
            IOTemp = 0;
            for (int i = 0; i < 8; i++)
            {
                if (systemIO.getDigitalOutput(i)) bleBitFields.digitalOutputs |= 1 << i;
            }
            
            if ( bleBitFields.digitalOutputs != IOTemp ) {
                bleBitFields.digitalOutputs = IOTemp;
                bleBitFields.doUpdate = 1;
            }
            
            if ( bleModes.isRunning != motorController->isRunning() ) {
                bleModes.isRunning = motorController->isRunning();
                bleModes.doUpdate = 1;
            }
            if ( bleModes.isFaulted != motorController->isFaulted() ) {
                bleModes.isFaulted = motorController->isFaulted();
                bleModes.doUpdate = 1;
            }
            if ( bleModes.isWarning != motorController->isWarning() ) {
                bleModes.isWarning = motorController->isWarning();
                bleModes.doUpdate = 1;
            }
            if ( bleModes.gear != motorController->getSelectedGear() ) {
                bleModes.gear = motorController->getSelectedGear();
                bleModes.doUpdate = 1;
            }
            
            if ( bleModes.powerMode != motorController->getPowerMode() ) {
                bleModes.powerMode = motorController->getPowerMode();
                bleModes.doUpdate = 1;
            }
            
            if ( bleModes.logLevel != (uint8_t)Logger::getLogLevel() ) {
                bleModes.logLevel = (uint8_t)Logger::getLogLevel();
                bleModes.doUpdate = 1;
            }
        }

    } else if (tickCounter == 4) {        
        if (motorController) {
            //Logger::console("Wifi tick counter 4...");
            
            if ( bleDigIO.prechargeDuration != motorController->getprechargeR() ) {
                bleDigIO.prechargeDuration = motorController->getprechargeR();
                bleDigIO.doUpdate = 1;
            }

            if ( bleDigIO.prechargeRelay != motorController->getprechargeRelay() ) {
                bleDigIO.prechargeRelay = motorController->getprechargeRelay();
                bleDigIO.doUpdate = 1;
                //Logger::console("Precharge Relay %i", paramCache.prechargeRelay);
                //Logger::console("motorController:prechargeRelay:%d, paramCache.prechargeRelay:%d, Constants:prechargeRelay:%s", motorController->getprechargeRelay(),paramCache.prechargeRelay, Constants::prechargeRelay);
            }

            if ( bleDigIO.mainContRelay != motorController->getmainContactorRelay() ) {
                bleDigIO.mainContRelay = motorController->getmainContactorRelay();
                bleDigIO.doUpdate = 1;
            }


            if ( bleDigIO.coolingRelay != motorController->getCoolFan() ) {
                bleDigIO.coolingRelay = motorController->getCoolFan();
                bleDigIO.doUpdate = 1;
            }

            if ( bleDigIO.coolOnTemp != motorController->getCoolOn() ) {
                bleDigIO.coolOnTemp = motorController->getCoolOn();
                bleDigIO.doUpdate = 1;
            }

            if ( bleDigIO.coolOffTemp != motorController->getCoolOff() ) {
                bleDigIO.coolOffTemp = motorController->getCoolOff();
                bleDigIO.doUpdate = 1;
            }

            if ( bleDigIO.brakeLightOut != motorController->getBrakeLight() ) {
                bleDigIO.brakeLightOut = motorController->getBrakeLight();
                bleDigIO.doUpdate = 1;
            }

            if ( bleDigIO.reverseLightOut != motorController->getRevLight() ) {
                bleDigIO.reverseLightOut = motorController->getRevLight();
                bleDigIO.doUpdate = 1;
            }

            if ( bleDigIO.enableIn != motorController->getEnableIn() ) {
                bleDigIO.enableIn = motorController->getEnableIn();
                bleDigIO.doUpdate = 1;
            }
            if ( bleDigIO.reverseIn != motorController->getReverseIn() ) {
                bleDigIO.reverseIn = motorController->getReverseIn();
                bleDigIO.doUpdate = 1;
            }
        }
    } else if (tickCounter == 5) {     
        if (acceleratorConfig) {
            if ( bleThrottleIO.numThrottlePots != acceleratorConfig->numberPotMeters ) {
                bleThrottleIO.numThrottlePots = acceleratorConfig->numberPotMeters;
                bleThrottleIO.doUpdate = 1;    
            }
            if ( bleThrottleIO.throttleType != acceleratorConfig->throttleSubType ) {
                bleThrottleIO.throttleType = acceleratorConfig->throttleSubType;
                bleThrottleIO.doUpdate = 1;    
            }
            
            if ( bleThrottleIO.throttle1Min != acceleratorConfig->minimumLevel1 ) {
                bleThrottleIO.throttle1Min = acceleratorConfig->minimumLevel1;
                bleThrottleIO.doUpdate = 1;    
            }
            
            if ( bleThrottleIO.throttle2Min != acceleratorConfig->minimumLevel2 ) {
                bleThrottleIO.throttle2Min = acceleratorConfig->minimumLevel2;
                bleThrottleIO.doUpdate = 1;    
            }
            
            if ( bleThrottleIO.throttle1Max != acceleratorConfig->maximumLevel1 ) {
                bleThrottleIO.throttle1Max = acceleratorConfig->maximumLevel1;
                bleThrottleIO.doUpdate = 1;    
            }
            
            if ( bleThrottleIO.throttle2Max != acceleratorConfig->maximumLevel2 ) {
                bleThrottleIO.throttle2Max = acceleratorConfig->maximumLevel2;
                bleThrottleIO.doUpdate = 1;    
            }
            
            if ( bleThrottleMap.throttleRegenMax != acceleratorConfig->positionRegenMaximum ) {
                bleThrottleMap.throttleRegenMax = acceleratorConfig->positionRegenMaximum;
                bleThrottleMap.doUpdate = 1;
            }
            
            if ( bleThrottleMap.throttleRegenMin != acceleratorConfig->positionRegenMinimum ) {
                bleThrottleMap.throttleRegenMin = acceleratorConfig->positionRegenMinimum;
                bleThrottleMap.doUpdate = 1;
            }
            
            if ( bleThrottleMap.throttleFwd != acceleratorConfig->positionForwardMotionStart ) {
                bleThrottleMap.throttleFwd = acceleratorConfig->positionForwardMotionStart;
                bleThrottleMap.doUpdate = 1;
            }
            
            if ( bleThrottleMap.throttleMap != acceleratorConfig->positionHalfPower ) {
                bleThrottleMap.throttleMap = acceleratorConfig->positionHalfPower;
                bleThrottleMap.doUpdate = 1;
            }
            
            if ( bleThrottleMap.throttleLowestRegen != acceleratorConfig->minimumRegen ) {
                bleThrottleMap.throttleLowestRegen = acceleratorConfig->minimumRegen;
                bleThrottleMap.doUpdate = 1;
            }
            
            if ( bleThrottleMap.throttleHighestRegen != acceleratorConfig->maximumRegen ) {
                bleThrottleMap.throttleHighestRegen = acceleratorConfig->maximumRegen;
                bleThrottleMap.doUpdate = 1;
            }
            
            if ( bleThrottleMap.throttleCreep != acceleratorConfig->creep ) {
                bleThrottleMap.throttleCreep = acceleratorConfig->creep;
                bleThrottleMap.doUpdate = 1;
            }
        }
     } else if (tickCounter == 6) {     
        if (brakeConfig) {
            if ( bleBrakeParam.brakeMin != brakeConfig->minimumLevel1 ) {
                bleBrakeParam.brakeMin = brakeConfig->minimumLevel1;
                bleBrakeParam.doUpdate = 1;
            }
            if ( bleBrakeParam.brakeMax != brakeConfig->maximumLevel1 ) {
                bleBrakeParam.brakeMax = brakeConfig->maximumLevel1;
                bleBrakeParam.doUpdate = 1;
            }
            if ( bleBrakeParam.brakeRegenMin != brakeConfig->minimumRegen ) {
                bleBrakeParam.brakeRegenMin = brakeConfig->minimumRegen;
                bleBrakeParam.doUpdate = 1;
            }
            if ( bleBrakeParam.brakeRegenMax != brakeConfig->maximumRegen ) {
                bleBrakeParam.brakeRegenMax = brakeConfig->maximumRegen;
                bleBrakeParam.doUpdate = 1;
            }
        }

        if (motorConfig) {
            if ( bleDigIO.coolingRelay != motorConfig->coolFan ) {
                bleDigIO.coolingRelay = motorConfig->coolFan;
                bleDigIO.doUpdate = 1;
            }
            
            if ( bleDigIO.coolOnTemp != motorConfig->coolOn ) {
                bleDigIO.coolOnTemp = motorConfig->coolOn;
                bleDigIO.doUpdate = 1;
            }
            
            if ( bleDigIO.coolOffTemp != motorConfig->coolOff ) {
                bleDigIO.coolOffTemp = motorConfig->coolOff;
                bleDigIO.doUpdate = 1;
            }
            
            if ( bleDigIO.brakeLightOut != motorConfig->brakeLight ) {
                bleDigIO.brakeLightOut = motorConfig->brakeLight;
                bleDigIO.doUpdate = 1;
            }
            
            if ( bleDigIO.reverseLightOut != motorConfig->revLight ) {
                bleDigIO.reverseLightOut = motorConfig->revLight;
                bleDigIO.doUpdate = 1;
            }
            
            if ( bleDigIO.enableIn != motorConfig->enableIn ) {
                bleDigIO.enableIn = motorConfig->enableIn;
                bleDigIO.doUpdate = 1;
            }
           
            if ( bleDigIO.reverseIn != motorConfig->reverseIn ) {
                bleDigIO.reverseIn = motorConfig->reverseIn;
                bleDigIO.doUpdate = 1;
            }
            
            if ( bleDigIO.prechargeDuration != motorConfig->prechargeR ) {
                bleDigIO.prechargeDuration = motorConfig->prechargeR;
                bleDigIO.doUpdate = 1;
            }
            
            if ( bleDigIO.prechargeRelay != motorConfig->prechargeRelay ) {
                bleDigIO.prechargeRelay = motorConfig->prechargeRelay;
                bleDigIO.doUpdate = 1;
            }
            
            if ( bleDigIO.mainContRelay != motorConfig->mainContactorRelay ) {
                bleDigIO.mainContRelay = motorConfig->mainContactorRelay;
                bleDigIO.doUpdate = 1;
            }
        } 
    } else if (tickCounter > 6) {
        if (motorController) {
            //Logger::console("Wifi tick counter 5...");
            if ( bleTemperatures.motorTemperature != motorController->getTemperatureMotor() ) {
                bleTemperatures.motorTemperature = motorController->getTemperatureMotor();
                bleTemperatures.doUpdate = 1;
            }
            if ( bleTemperatures.inverterTemperature != motorController->getTemperatureInverter() ) {
                bleTemperatures.inverterTemperature = motorController->getTemperatureInverter();
                bleTemperatures.doUpdate = 1;
            }
            if ( bleTemperatures.systemTemperature != motorController->getTemperatureSystem() ) {
                bleTemperatures.systemTemperature = motorController->getTemperatureSystem();
                bleTemperatures.doUpdate = 1;
            }
            
            if ( bleBitFields.bitfield1 != motorController->getStatusBitfield1() )
            {
                bleBitFields.bitfield1 = motorController->getStatusBitfield1();
                bleBitFields.doUpdate = 1;                  
            }
            
            if ( bleBitFields.bitfield2 != motorController->getStatusBitfield2() )
            {
                bleBitFields.bitfield2 = motorController->getStatusBitfield2();
                bleBitFields.doUpdate = 1;                  
            }            
        }
        
        buildEnabledDevices(); //build list of enabled devices to report over BLE
        
        tickCounter = 0;
    }
    transferUpdates(); //send out any updates required
}

void ADAFRUITBLE::checkGattChar(uint8_t charact)
{
    uint16_t len;
    uint8_t buff[40];
    
    if (!bOkToWrite) return; // don't check for parameter writes if it isn't OK yet.
    
    Logger::debug("Checking %i", charact);
    ble.print("AT+GATTCHARRAW="); // use RAW command version
    ble.println(charact);
    len = ble.readraw(); // readraw swallow OK/ERROR already
    memcpy(buff, ble.buffer, len);
    gattRX(charact, buff, len);
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
        needParamReload = true;
        break;
    }
    case MSG_COMMAND:  //Sends a message to the BLE module in the form of AT command
        //sendCmd((char *)message);
        break;
    case 0xDEADBEEF:
        setupBLEservice();
        break;        
    }
}

void ADAFRUITBLE::loop() {
}

void ADAFRUITBLE::buildEnabledDevices()
{
    uint32_t bitfield = 0;
    int idx = 0 ;
    Device *dev;
    while (deviceTable[idx] != 0)
    {
        dev = deviceManager.getDeviceByID((DeviceId)deviceTable[idx]);
        if (dev != 0 && dev->isEnabled())
        {
            bitfield |= 1 << idx;
            Logger::debug(ADABLUE, "Found enabled device: %x", deviceTable[idx]);
        }
        idx++;
        if (idx > 31) break; //force not even supporting the second bitfield yet. TODO: Fix this.
    }
    if (bleDeviceEnable.deviceEnable1 != bitfield)
    {
        bleDeviceEnable.deviceEnable1 = bitfield;
        bleDeviceEnable.deviceEnable2 = 0;
        bleDeviceEnable.doUpdate = 1;
    }
}

//Handle it when the connected device updates a GATT characteristic.
void ADAFRUITBLE::gattRX(int32_t chars_id, uint8_t data[], uint16_t len)
{
    Logger::debug("Entered gattRX with charid: %i, datalen = %i", chars_id, len);
    MotorController *motorController = deviceManager.getMotorController();
    Throttle *accelerator = deviceManager.getAccelerator();
    Throttle *brake = deviceManager.getBrake();
    PotThrottleConfiguration *acceleratorConfig = NULL;
    PotThrottleConfiguration *brakeConfig = NULL;
    MotorControllerConfiguration *motorConfig = NULL;
    
    uint16_t uint16;
    int16_t int16;
    bool needUpdate = false;
    
    if (accelerator)
        acceleratorConfig = (PotThrottleConfiguration *)accelerator->getConfiguration();
    if (brake)
        brakeConfig = (PotThrottleConfiguration *)brake->getConfiguration();
    if (motorController)
        motorConfig = (MotorControllerConfiguration *)motorController->getConfiguration(); 
    
    if (chars_id == 5) //0x3105
    {
        if (bleModes.powerMode != data[0])
        {
            Logger::info("Updating power mode to %i from %i", data[0], bleModes.powerMode);
            bleModes.powerMode = data[0];
            if (motorController)
            {
                motorController->setPowerMode((MotorController::PowerMode)bleModes.powerMode);    
                motorController->saveConfiguration();
            }
        }
        
        if (bleModes.logLevel != data[5])
        {
            Logger::info("Updating log level to %i from %i", data[5], bleModes.logLevel);
            bleModes.logLevel = data[5];
            Logger::setLoglevel((Logger::LogLevel)bleModes.logLevel);
            sysPrefs->write(EESYS_LOG_LEVEL, bleModes.logLevel);
            sysPrefs->calcChecksum();
        }
    } 
    else if (chars_id == 9) //0x3109
    {        
        uint16 = data[0] + data[1] * 256ul;
        if (bleDigIO.prechargeDuration != uint16)
        {
            Logger::info("Updating precharge duration to %i from %i", uint16, bleDigIO.prechargeDuration);
            bleDigIO.prechargeDuration = uint16;
            if (motorConfig)
            {
                motorConfig->prechargeR = bleDigIO.prechargeDuration;
                needUpdate = true;
            }
        }
        
        if (bleDigIO.prechargeRelay != data[2])
        {
            Logger::info("Updating precharge output to %i from %i", data[2], bleDigIO.prechargeRelay);
            bleDigIO.prechargeRelay = data[2];
            if (motorConfig)
            {            
                motorConfig->prechargeRelay = bleDigIO.prechargeRelay;
                needUpdate = true;
            }
        }
        
        if (bleDigIO.mainContRelay != data[3])
        {    
            Logger::info("Updating main contactor output to %i from %i", data[3], bleDigIO.mainContRelay);
            bleDigIO.mainContRelay = data[3];
            if (motorConfig)
            {
                motorConfig->mainContactorRelay = bleDigIO.mainContRelay;
                needUpdate = true;
            }
        }
        
        if (bleDigIO.coolingRelay != data[4])
        {
            Logger::info("Updating cooling output to %i from %i", data[4], bleDigIO.coolingRelay);
            bleDigIO.coolingRelay = data[4];
            if (motorConfig)
            {
                motorConfig->coolFan = bleDigIO.coolingRelay;
                needUpdate = true;
            }
        }
        
        if (bleDigIO.coolOnTemp != (int8_t)data[5])
        {
            Logger::info("Updating cooling on temperature to %i from %i", data[5], bleDigIO.coolOnTemp);
            bleDigIO.coolOnTemp = (int8_t)data[5];
            if (motorConfig)
            {            
                motorConfig->coolOn = bleDigIO.coolOnTemp;
                needUpdate = true;
            }
        }
        
        if (bleDigIO.coolOffTemp != (int8_t)data[6])
        {
            Logger::info("Updating cooling off temperature to %i from %i", data[6], bleDigIO.coolOffTemp);
            bleDigIO.coolOffTemp = (int8_t)data[6];
            if (motorConfig)
            {            
                motorConfig->coolOff = bleDigIO.coolOffTemp;
                needUpdate = true;
            }
        }
        
        if (bleDigIO.brakeLightOut != data[7])
        {
            Logger::info("Updating brake light output to %i from %i", data[7], bleDigIO.brakeLightOut);
            bleDigIO.brakeLightOut = data[7];
            if (motorConfig)
            {            
                motorConfig->brakeLight = bleDigIO.brakeLightOut;
                needUpdate = true;
            }
        }
        
        if (bleDigIO.reverseLightOut != data[8])
        {
            Logger::info("Updating reverse light output to %i from %i", data[8], bleDigIO.reverseLightOut);
            bleDigIO.reverseLightOut = data[8];
            if (motorConfig)
            {            
                motorConfig->revLight = bleDigIO.reverseLightOut;
                needUpdate = true;
            }
        }
        
        if (bleDigIO.enableIn != data[9])
        {
            Logger::info("Updating enable input to %i from %i", data[9], bleDigIO.enableIn);
            bleDigIO.enableIn = data[9];
            if (motorConfig)
            {            
                motorConfig->enableIn = bleDigIO.enableIn;
                needUpdate = true;
            }
        }
        
        if (bleDigIO.reverseIn != data[10])
        {        
            Logger::info("Updating reverse input to %i from %i", data[10], bleDigIO.reverseIn);
            bleDigIO.reverseIn = data[10];                                
            if (motorConfig)
            {            
                motorConfig->reverseIn = bleDigIO.reverseIn;
                needUpdate = true;
            }
        }
        
        if (needUpdate)
        {
            motorController->saveConfiguration();
            needUpdate = false;
        }
    }    
    else if (chars_id == 10) //0x310A
    {
        uint16 = data[0] + data[1] * 256ul;
        if (bleThrottleIO.throttle1Min != uint16)
        {
            Logger::info("Throttle1Min updated to %i from %i", uint16, bleThrottleIO.throttle1Min);
            bleThrottleIO.throttle1Min = uint16;
            if (acceleratorConfig)
            {            
                acceleratorConfig->minimumLevel1 = bleThrottleIO.throttle1Min;            
                needUpdate = true;
            }
        }
        
        uint16 = data[2] + data[3] * 256ul;
        if (bleThrottleIO.throttle2Min != uint16)
        {
            Logger::info("Throttle2Min updated to %i from %i", uint16, bleThrottleIO.throttle2Min);
            bleThrottleIO.throttle2Min = uint16;
            if (acceleratorConfig)
            {                        
                acceleratorConfig->minimumLevel2 = bleThrottleIO.throttle2Min;
                needUpdate = true;
            }
        }
        
        
        uint16 = data[4] + data[5] * 256ul;
        if (bleThrottleIO.throttle1Max != uint16)
        {
            Logger::info("Throttle1Max updated to %i from %i", uint16, bleThrottleIO.throttle1Max);
            bleThrottleIO.throttle1Max = uint16;
            if (acceleratorConfig)
            {                        
                acceleratorConfig->maximumLevel1 = bleThrottleIO.throttle1Max;
                needUpdate = true;
            }
        }
        
        uint16 = data[6] + data[7] * 256ul;
        if (bleThrottleIO.throttle2Max != uint16)
        {
            Logger::info("Throttle2Max updated to %i from %i", uint16, bleThrottleIO.throttle2Max);
            bleThrottleIO.throttle2Max = uint16;
            if (acceleratorConfig)
            {                        
                acceleratorConfig->maximumLevel2 = bleThrottleIO.throttle2Max;    
                needUpdate = true;
            }
        }
        
        if (bleThrottleIO.numThrottlePots != data[8])
        {
            Logger::info("numPots updated to %i from %i", data[8], bleThrottleIO.numThrottlePots);
            bleThrottleIO.numThrottlePots = data[8];
            if (acceleratorConfig)
            {                        
                acceleratorConfig->numberPotMeters = bleThrottleIO.numThrottlePots;
                needUpdate = true;
            }
        }
         
        if (bleThrottleIO.throttleType != data[9])
        {
            Logger::info("Throttle type updated to %i from %i", data[9], bleThrottleIO.throttleType);
            bleThrottleIO.throttleType = data[9];
            if (acceleratorConfig)
            {                        
                acceleratorConfig->throttleSubType = bleThrottleIO.throttleType;
                needUpdate = true;
            }
        }
                
        if (needUpdate)
        {
            accelerator->saveConfiguration();
            needUpdate = false;
        }
    }
    else if (chars_id == 11) //0x310B
    {
        uint16 = data[0] + data[1] * 256ul;
        if (bleThrottleMap.throttleRegenMax != uint16)
        {
            Logger::info("ThrottleRegenMax updated to %i from %i", uint16, bleThrottleMap.throttleRegenMax);
            bleThrottleMap.throttleRegenMax = uint16;
            if (acceleratorConfig)
            {                        
                acceleratorConfig->positionRegenMaximum = bleThrottleMap.throttleRegenMax;
                needUpdate = true;
            }
        }
        
        uint16 = data[2] + data[3] * 256ul;
        if (bleThrottleMap.throttleRegenMin != uint16)
        {
            Logger::info("ThrottleRegenMin updated to %i from %i", uint16, bleThrottleMap.throttleRegenMin);
            bleThrottleMap.throttleRegenMin = uint16;
            if (acceleratorConfig)
            {                        
                acceleratorConfig->positionRegenMinimum = bleThrottleMap.throttleRegenMin;
                needUpdate = true;
            }
        }
        
        uint16 = data[4] + data[5] * 256ul;
        if (bleThrottleMap.throttleFwd != uint16)
        {
            Logger::info("ThrottleFwd updated to %i from %i", uint16, bleThrottleMap.throttleFwd);
            bleThrottleMap.throttleFwd = uint16;
            if (acceleratorConfig)
            {                        
                acceleratorConfig->positionForwardMotionStart = bleThrottleMap.throttleFwd;
                needUpdate = true;
            }
        }
                
        uint16 = data[6] + data[7] * 256ul;
        if (bleThrottleMap.throttleMap != uint16)
        {
            Logger::info("ThrottleMap updated to %i from %i", uint16, bleThrottleMap.throttleMap);
            bleThrottleMap.throttleMap = uint16;
            if (acceleratorConfig)
            {                        
                acceleratorConfig->positionHalfPower = bleThrottleMap.throttleMap;
                needUpdate = true;
            }
        }
        
        if (bleThrottleMap.throttleLowestRegen != data[8])
        {
            Logger::info("ThrottleLowestRegen updated to %i from %i", data[8], bleThrottleMap.throttleLowestRegen);
            bleThrottleMap.throttleLowestRegen = data[8];
            if (acceleratorConfig)
            {                        
                acceleratorConfig->minimumRegen = bleThrottleMap.throttleLowestRegen;
                needUpdate = true;
            }
        }
        
        if (bleThrottleMap.throttleHighestRegen != data[9])
        {        
            Logger::info("ThrottleHighestRegen updated to %i from %i", data[9], bleThrottleMap.throttleHighestRegen);
            bleThrottleMap.throttleHighestRegen = data[9];
            if (acceleratorConfig)
            {            
                acceleratorConfig->maximumRegen = bleThrottleMap.throttleHighestRegen;
                needUpdate = true;
            }
        }
         
        if (bleThrottleMap.throttleCreep != data[10])
        {
            Logger::info("ThrottleCreep updated to %i from %i", data[10], bleThrottleMap.throttleCreep);
            bleThrottleMap.throttleCreep = data[10];
            if (acceleratorConfig)
            {                        
                acceleratorConfig->creep = bleThrottleMap.throttleCreep;
                needUpdate = true;
            }
        }
        
        if (needUpdate)
        {
            accelerator->saveConfiguration();
            needUpdate = false;
        }
    }    
    else if (chars_id == 12) //0x310C
    {
        uint16 = data[0] + data[1] * 256ul;
        if (bleBrakeParam.brakeMin != uint16)
        {
            Logger::info("Updating brakemin to %i from %i", uint16, bleBrakeParam.brakeMin);
            bleBrakeParam.brakeMin = uint16;
            if (brakeConfig)
            {                        
                brakeConfig->minimumLevel1 = bleBrakeParam.brakeMin;
                needUpdate = true;
            }
        }
        
        uint16 = data[2] + data[3] * 256ul;
        if (bleBrakeParam.brakeMax != uint16)
        {
            Logger::info("Updating brakemax to %i from %i", uint16, bleBrakeParam.brakeMax);
            bleBrakeParam.brakeMax = uint16;
            if (brakeConfig)
            {                                    
                brakeConfig->maximumLevel1 = bleBrakeParam.brakeMax;
                needUpdate = true;
            }
        }
            
        if (bleBrakeParam.brakeRegenMin != data[4])
        {
            Logger::info("Updating brakeregenmin to %i from %i", data[4], bleBrakeParam.brakeRegenMin);
            bleBrakeParam.brakeRegenMin = data[4];
            if (brakeConfig)
            {                                    
                brakeConfig->minimumRegen = bleBrakeParam.brakeRegenMin;
                needUpdate = true;
            }
        }
        
        if (bleBrakeParam.brakeRegenMax != data[5])
        {
            Logger::info("Updating brakeregenmax to %i from %i", data[5], bleBrakeParam.brakeRegenMax);
            bleBrakeParam.brakeRegenMax = data[5];
            if (brakeConfig)
            {                                    
                brakeConfig->maximumRegen = bleBrakeParam.brakeRegenMax;
                needUpdate = true;
            }
        }

        if (needUpdate)
        {
            brake->saveConfiguration();
            needUpdate = false;
        } 
    }    
    else if (chars_id == 13) //0x310D
    {
        uint16 = data[0] + data[1] * 256ul;
        if (bleMaxParams.nomVoltage != uint16)
        {
            Logger::info("Updating nominal voltage to %i from %i", uint16, bleMaxParams.nomVoltage);
            bleMaxParams.nomVoltage = uint16;
            if (motorConfig)
            {                                    
                motorConfig->nominalVolt = bleMaxParams.nomVoltage;
                motorController->nominalVolts = bleMaxParams.nomVoltage;
                needUpdate = true;
            }
        }
        
        uint16 = data[2] + data[3] * 256ul;
        if (bleMaxParams.maxRPM != uint16)
        {
            Logger::info("Updating max RPM to %i from %i", uint16, bleMaxParams.maxRPM);
            bleMaxParams.maxRPM = uint16;
            if (motorConfig)
            {                                                
                motorConfig->speedMax = bleMaxParams.maxRPM;
                needUpdate = true;
            }
        }
        
        uint16 = data[4] + data[5] * 256ul;
        if (bleMaxParams.maxTorque != uint16)
        {
            Logger::info("Updating max torque to %i from %i", uint16, bleMaxParams.maxTorque);
            bleMaxParams.maxTorque = uint16;
            if (motorConfig)
            {                                                
                motorConfig->torqueMax = bleMaxParams.maxTorque;
                needUpdate = true;
            }
        }        
        
        if (needUpdate)
        {
            motorController->saveConfiguration();
            needUpdate = false;
        }
    }    
    else if (chars_id == 14) //0x310E
    {
        bleDeviceEnable.deviceEnable1 = data[0] + data[1] * 256ul + data[2] * 65536ul + data[3] * 16777216ul;
        int idx;
        bool enStatus;
        idx = 0;
        enStatus = false;
        while (deviceTable[idx] != 0)
        {
            if (bleDeviceEnable.deviceEnable1 & 1 << idx) enStatus = true;
                else enStatus = false;
                
            //Logger::info("Device %x setting enable status to %T", deviceTable[idx], enStatus);
            
            //PrefHandler::setDeviceStatus(deviceTable[idx], enStatus);
            
            idx++;
        }
    }        
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
    else
    {       
        setupBLEservice();
        Logger::debug(ADABLUE, "BluefruitLE Initialization Complete....");
        prefsHandler->saveChecksum();
    }
}

void ADAFRUITBLE::saveConfiguration() {
    BLEConfiguration *config = (BLEConfiguration *) getConfiguration();

//	prefsHandler->write(EESYS_WIFI0_SSID, config->ssid);
//	prefsHandler->saveChecksum();
}


