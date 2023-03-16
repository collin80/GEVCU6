/*
 GEVCU6.ino
 New hardware version 9/20/2016 for Hardware 6.23c
 Created: 1/4/2013 1:34:14 PM
 Author: Collin Kidder
 
 New, new plan: Allow for an arbitrary # of devices that can have both tick and canbus handlers. These devices register themselves
 into the handler framework and specify which sort of device they are. They can have custom tick intervals and custom can filters.
 The system automatically manages when to call the tick handlers and automatically filters canbus and sends frames to the devices.
 There is a facility to send data between devices by targetting a certain type of device. For instance, a motor controller
 can ask for any throttles and then retrieve the current throttle position from them.

Copyright (c) 2013 Collin Kidder, Michael Neuweiler, Charles Galpin

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
 
/*Changelog removed. All changes are logged to GIT */

/*
Random comments on things that should be coded up soon:
4. It is a possibility that there should be support for actually controlling the power to some of the devices.
	For instance, power could be controlled to the +12V connection at the DMOC so that it can be power cycled
	in software. But, that uses up an input and people can just cycle the key (though that resets the GEVCU too)
5. Some people (like me, Collin) have a terrible habit of mixing several coding styles. It would be beneficial to
	continue to harmonize the source code - Perhaps use a tool to do this.
6. It should be possible to limit speed and/or torque in reverse so someone doesn't kill themselves or someone else
	while gunning it in reverse - The configuration variable is there and settable now. Just need to integrate it.
7. The DMOC code duplicates a bunch of functionality that the base class also used to implement. We've got to figure
	out where the overlaps are and fix it up so that as much as possible is done generically at the base MotorController
	class and not directly in the Dmoc class.
*/

#include "GEVCU.h"

// The following includes are required in the .ino file by the Arduino IDE in order to properly
// identify the required libraries for the build.
#include <due_can.h>
#include <BLE.h>
#include <FirmwareReceiver.h>
#include <due_wire.h>
#include <DueTimer.h>
#include <SPI.h>
#include <microsmooth.h>

#define DEBUG_STARTUP_DELAY         //if this is defined there is a large start up delay so you can see the start up messages. NOT for production!

//Evil, global variables
PrefHandler *sysPrefs;
MemCache *memCache;
Heartbeat *heartbeat;
SerialConsole *serialConsole;
ADAFRUITBLE *btDevice;
template<class T> inline Print &operator <<(Print &obj, T arg) { obj.print(arg); return obj; } //Lets us stream SerialUSB

byte i = 0;


void watchdogSetup(void)
{
  watchdogEnable(250);
}

//initializes all the system EEPROM values. Chances are this should be broken out a bit but
//there is only one checksum check for all of them so it's simple to do it all here.

void initSysEEPROM() {
	//three temporary storage places to make saving to EEPROM easy
	uint8_t eight;
	uint16_t sixteen;
	uint32_t thirtytwo;

	eight = 6; //GEVCU 6.2 board
	sysPrefs->write(EESYS_SYSTEM_TYPE, eight);

	sixteen = 1024; //no gain
	sysPrefs->write(EESYS_ADC0_GAIN, sixteen);
	sysPrefs->write(EESYS_ADC1_GAIN, sixteen);
	sysPrefs->write(EESYS_ADC2_GAIN, sixteen);
	sysPrefs->write(EESYS_ADC3_GAIN, sixteen);
    sysPrefs->write(EESYS_ADC_PACKH_GAIN, sixteen);
    sysPrefs->write(EESYS_ADC_PACKL_GAIN, sixteen);
    sysPrefs->write(EESYS_ADC_PACKC_GAIN, sixteen);


	sixteen = 0; //no offset
	sysPrefs->write(EESYS_ADC0_OFFSET, sixteen);
	sysPrefs->write(EESYS_ADC1_OFFSET, sixteen);
	sysPrefs->write(EESYS_ADC2_OFFSET, sixteen);
	sysPrefs->write(EESYS_ADC3_OFFSET, sixteen);
    sysPrefs->write(EESYS_ADC_PACKH_OFFSET, sixteen);
    sysPrefs->write(EESYS_ADC_PACKL_OFFSET, sixteen);
    sysPrefs->write(EESYS_ADC_PACKC_OFFSET, sixteen);


	sixteen = CFG_CAN0_SPEED;
	sysPrefs->write(EESYS_CAN0_BAUD, sixteen);
    sixteen = CFG_CAN1_SPEED;
	sysPrefs->write(EESYS_CAN1_BAUD, sixteen);

	eight = 2;  //0=debug, 1=info,2=warn,3=error,4=off
	sysPrefs->write(EESYS_LOG_LEVEL, eight);

	sysPrefs->saveChecksum();
    sysPrefs->forceCacheWrite();
}

/*
Creating objects here is all you need to do to register them. The pointer
reference will obviously expire at the end of the function but the object
lives on and is hereafter controlled by the system. 
*/
void createObjects() {
	PotThrottle *paccelerator = new PotThrottle();
	PotBrake *pbrake = new PotBrake();
    TestThrottle *testAccel = new TestThrottle();
	DmocMotorController *dmotorController = new DmocMotorController();
    CodaMotorController *cmotorController = new CodaMotorController();
	CKMotorController *ckMotorController = new CKMotorController();
	RMSMotorController *rmsMotorController = new RMSMotorController();
    TestMotorController *testMotorController = new TestMotorController();
    DCDCController *dcdcController = new DCDCController();
	BrusaMotorController *bmotorController = new BrusaMotorController();
    C300MotorController *c300MotorController = new C300MotorController();
	ThinkBatteryManager *thinkBMS = new ThinkBatteryManager();
    BuiltinBatteryManager *builtinBMS = new BuiltinBatteryManager();
	ELM327Emu *emu = new ELM327Emu();
    ADAFRUITBLE *ble = new ADAFRUITBLE();
    EVIC *eVIC = new EVIC();
    PowerkeyPad *powerKey = new PowerkeyPad();
    VehicleSpecific *vehicleSpecific = new VehicleSpecific();
}

void initializeDevices() {
	//heartbeat is always enabled now
	heartbeat = new Heartbeat();
	Logger::info("add: Heartbeat (id: %X, %X)", HEARTBEAT, heartbeat);
	heartbeat->setup();

	//fault handler is always enabled too - its also statically allocated so no using -> here
	//This is initialized before the other devices so that they can go ahead and use it if they fault upon start up
	faultHandler.setup();

	/*
	We used to instantiate all the objects here along with other code. To simplify things this is done somewhat
	automatically now. Just instantiate your new device object in createObjects above. This takes care of the details
	so long as you follow the template of how other devices were coded.
	*/
	createObjects(); 

	/*
	 *	We defer setting up the devices until here. This allows all objects to be instantiated
	 *	before any of them set up. That in turn allows the devices to inspect what else is
	 *	out there as they initialize. For instance, a motor controller could see if a BMS
	 *	exists and supports a function that the motor controller wants to access.
	 */
    sysPrefs->forceCacheWrite();
	deviceManager.sendMessage(DEVICE_ANY, INVALID, MSG_STARTUP, NULL);

}

void setup() {
    //Most boards pre 6.2 have outputs on 2-9 and a problem where their outputs can trigger on for just a quick moment
    //upon start up. So, try to pull the outputs low as soon as we can just to be sure.
    //This project is now GEVCU6.2 specific but still, doesn't hurt to set your outputs properly.
    for (int i = 2; i < 10; i++) {
        pinMode(i, OUTPUT);
        digitalWrite(i, LOW);
    }
    
    pinMode(64, OUTPUT); //DFU for BLE Module
    digitalWrite(64, HIGH);
    pinMode(65, OUTPUT); //reset for BLE module
    digitalWrite(65, HIGH);
    
    //Activate the supply monitor at 2.8v and make it hold the CPU in reset under that point
    //That first value is 1.9V + the value for the supply threshold. So, 9 would be 2.8v, A would be 2.9v, etc
    
    //               Thresh  Enable   Force Reset
    SUPC->SUPC_SMMR = 0xA | (1<<8) | (1<<12);
	
#ifdef DEBUG_STARTUP_DELAY
    for (int c = 0; c < 200; c++) {
        delay(25);  //This delay lets you see startup.  But it breaks DMOC645 really badly.  You have to have comm quickly upon start up
        watchdogReset();
    }
#endif
       
	pinMode(BLINK_LED, OUTPUT);
	digitalWrite(BLINK_LED, LOW);
    SerialUSB.begin(CFG_SERIAL_SPEED);
	SerialUSB.println(CFG_VERSION);
	SerialUSB.print("Build number: ");
	SerialUSB.println(CFG_BUILD_NUM);
	Wire.begin((uint32_t)1000000);
	Logger::info("TWI init ok");
	memCache = new MemCache();
	Logger::info("add MemCache (id: %X, %X)", MEMCACHE, memCache);
	memCache->setup();
	sysPrefs = new PrefHandler(SYSTEM);
	if (!sysPrefs->checksumValid()) 
        {
	      Logger::info("Initializing system EEPROM settings");
	      initSysEEPROM();
	    } 
        else {Logger::info("Using existing EEPROM system values");}//checksum is good, read in the values stored in EEPROM

	uint8_t loglevel;
	sysPrefs->read(EESYS_LOG_LEVEL, &loglevel);
	loglevel = 0; //force debugging log level
    Logger::console("LogLevel: %i", loglevel);
	Logger::setLoglevel((Logger::LogLevel)loglevel);
    //Logger::setLoglevel((Logger::LogLevel)0);
	systemIO.setup();  
	canHandlerEv.setup();
	canHandlerCar.setup();
	Logger::info("SYSIO init ok");	

	initializeDevices();
    serialConsole = new SerialConsole(memCache, heartbeat);
	serialConsole->printMenu();
	btDevice = static_cast<ADAFRUITBLE *>(deviceManager.getDeviceByID(ADABLUE));
    deviceManager.sendMessage(DEVICE_WIFI, ADABLUE, MSG_CONFIG_CHANGE, NULL); //Load config into BLE interface

   
	Logger::info("System Ready");	
}

void loop() {
#ifdef CFG_TIMER_USE_QUEUING
	tickHandler.process();
#endif

	// check if incoming frames are available in the can buffer and process them
	canHandlerEv.process();
	canHandlerCar.process();

	serialConsole->loop();

    systemIO.pollInitialization();
    
    if (btDevice) btDevice->loop();
    
    watchdogReset();
}
