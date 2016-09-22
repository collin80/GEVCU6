/*
 GEVCU.ino
 
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
#include <due_rtc.h>
#include <due_can.h>
#include <FirmwareReceiver.h>
#include <due_wire.h>
#include <DueTimer.h>
#include <SPI.h>

//RTC_clock rtc_clock(XTAL); //init RTC with the external 32k crystal as a reference

//Evil, global variables
PrefHandler *sysPrefs;
MemCache *memCache;
Heartbeat *heartbeat;
SerialConsole *serialConsole;
Device *wifiDevice;
Device *btDevice;

byte i = 0;

void sendWiReach(char* message)
{
    Serial2.println(message);
    delay(700);
    while (Serial2.available()) {SerialUSB.write(Serial2.read());}
}

void initWiReach()
{
SerialUSB.begin(115200); // use SerialUSB only as the programming port doesn't work
Serial2.begin(115200); // use Serial3 for GEVCU2, use Serial2 for GEVCU3+4

//sendWiReach("AT+iFD");//Host connection set to serial port
//delay(5000);
sendWiReach("AT+iHIF=1");//Host connection set to serial port
sendWiReach("AT+iBDRF=9");//Automatic baud rate on host serial port
sendWiReach("AT+iRPG=secret"); //Password for iChip wbsite
sendWiReach("AT+iWPWD=secret");//Password for our website
sendWiReach("AT+iWST0=0");//Connection security wap/wep/wap2 to no security
sendWiReach("AT+iWLCH=4");  //Wireless channel
sendWiReach("AT+iWLSI=GEVCU");//SSID
sendWiReach("AT+iWSEC=1");//IF security is used, set for WPA2-AES
sendWiReach("AT+iSTAP=1");//Act as AP
sendWiReach("AT+iDIP=192.168.3.10");//default ip - must be 10.x.x.x
sendWiReach("AT+iDPSZ=8");//DHCP pool size
sendWiReach("AT+iAWS=1");//Website on
sendWiReach("AT+iDOWN");//Powercycle reset
delay(5000);
SerialUSB.println("WiReach Wireless Module Initialized....");
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

	sixteen = 0; //no offset
	sysPrefs->write(EESYS_ADC0_OFFSET, sixteen);
	sysPrefs->write(EESYS_ADC1_OFFSET, sixteen);
	sysPrefs->write(EESYS_ADC2_OFFSET, sixteen);
	sysPrefs->write(EESYS_ADC3_OFFSET, sixteen);

	sixteen = 500; //multiplied by 1000 so 500k baud
	sysPrefs->write(EESYS_CAN0_BAUD, sixteen);
	sysPrefs->write(EESYS_CAN1_BAUD, sixteen);

	eight = 2;  //0=debug, 1=info,2=warn,3=error,4=off
	sysPrefs->write(EESYS_LOG_LEVEL, eight);

	sysPrefs->saveChecksum();
}

void createObjects() {
	PotThrottle *paccelerator = new PotThrottle();
	CanThrottle *caccelerator = new CanThrottle();
	PotBrake *pbrake = new PotBrake();
    TestThrottle *testAccel = new TestThrottle();
	CanBrake *cbrake = new CanBrake();
	DmocMotorController *dmotorController = new DmocMotorController();
    CodaMotorController *cmotorController = new CodaMotorController();
	CKMotorController *ckMotorController = new CKMotorController();
    TestMotorController *testMotorController = new TestMotorController();
    DCDCController *dcdcController = new DCDCController();
	BrusaMotorController *bmotorController = new BrusaMotorController();
	ThinkBatteryManager *BMS = new ThinkBatteryManager();
	ELM327Emu *emu = new ELM327Emu();
	ICHIPWIFI *iChip = new ICHIPWIFI();
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
	deviceManager.sendMessage(DEVICE_ANY, INVALID, MSG_STARTUP, NULL);

}

void setup() {
    //Most boards pre 6.2 have outputs on 2-9 and a problem where their outputs can trigger on for just a quick moment
    //upon start up. So, try to pull the outputs low as soon as we can just to be sure.
    for (int i = 2; i < 10; i++) {
        pinMode(i, OUTPUT);
        digitalWrite(i, LOW);
    }
    
    pinMode(64, OUTPUT); //DFU for BLE Module
    digitalWrite(64, HIGH);
    pinMode(65, OUTPUT); //reset for BLE module
    digitalWrite(65, HIGH);
	
    //delay(5000);  //This delay lets you see startup.  But it breaks DMOC645 really badly.  You have to have comm way before 5 seconds.
       
    //initWiReach();
	pinMode(BLINK_LED, OUTPUT);
	digitalWrite(BLINK_LED, LOW);
    SerialUSB.begin(CFG_SERIAL_SPEED);
	SerialUSB.println(CFG_VERSION);
	SerialUSB.print("Build number: ");
	SerialUSB.println(CFG_BUILD_NUM);
	Wire.begin();
	Logger::info("TWI init ok");
	memCache = new MemCache();
	Logger::info("add MemCache (id: %X, %X)", MEMCACHE, memCache);
	memCache->setup();
	sysPrefs = new PrefHandler(SYSTEM);
	if (!sysPrefs->checksumValid()) 
        {
	      Logger::info("Initializing EEPROM");
	      initSysEEPROM();
          // initWiReach();
	    } 
        else {Logger::info("Using existing EEPROM values");}//checksum is good, read in the values stored in EEPROM

	uint8_t loglevel;
	sysPrefs->read(EESYS_LOG_LEVEL, &loglevel);
    Logger::console("LogLevel: %i", loglevel);
	Logger::setLoglevel((Logger::LogLevel)loglevel);    
	systemIO.setup();  
	canHandlerEv.setup();
	canHandlerCar.setup();
	Logger::info("SYSIO init ok");	

	initializeDevices();
    serialConsole = new SerialConsole(memCache, heartbeat);
	serialConsole->printMenu();
	wifiDevice = deviceManager.getDeviceByID(ICHIP2128);
	btDevice = deviceManager.getDeviceByID(ELM327EMU);
    deviceManager.sendMessage(DEVICE_WIFI, ICHIP2128, MSG_CONFIG_CHANGE, NULL); //Load configuration variables into WiFi Web Configuration screen
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
	//TODO: this is dumb... shouldn't have to manually do this. Devices should be able to register loop functions
	if ( wifiDevice != NULL ) {
		((ICHIPWIFI*)wifiDevice)->loop();
	}

	//if (btDevice != NULL) {
	//	((ELM327Emu*)btDevice)->loop();
	//}

	//this should still be here. It checks for a flag set during an interrupt
	systemIO.adcPoll();
}




