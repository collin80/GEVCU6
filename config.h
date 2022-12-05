/*
 * config.h
 *
 * Defines the components to be used in the GEVCU and allows the user to configure
 * static parameters.
 *
 * Note: Make sure with all pin defintions of your hardware that each pin number is
 *       only defined once.

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
 *      Author: Michael Neuweiler
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#include <due_can.h>

#define CFG_BUILD_NUM	1058      //increment this every time a git commit is done. 
#define CFG_VERSION "GEVCU 2016-10-13"

/*
 * SERIAL CONFIGURATION
 */
#define CFG_SERIAL_SPEED 115200
//#define SerialUSB Serial // re-route serial-usb output to programming port ;) comment if output should go to std usb


//The defines that used to be here to configure devices are gone now.
//The EEPROM stores which devices to bring up at start up and all
//devices are programmed into the firware at the same time.

/*
 * TIMER INTERVALS
 *
 * specify the intervals (microseconds) at which each device type should be "ticked"
 * try to use the same numbers for several devices because then they will share
 * the same timer (out of a limited number of 9 timers).
 */
#define CFG_TICK_INTERVAL_HEARTBEAT                 2000000
#define CFG_TICK_INTERVAL_POT_THROTTLE              40000
#define CFG_TICK_INTERVAL_CAN_THROTTLE              40000
#define CFG_TICK_INTERVAL_MOTOR_CONTROLLER          40000
#define CFG_TICK_INTERVAL_MOTOR_CONTROLLER_DMOC     40000
#define CFG_TICK_INTERVAL_MOTOR_CONTROLLER_CODAUQM  10000
#define CFG_TICK_INTERVAL_MOTOR_CONTROLLER_BRUSA    20000
#define CFG_TICK_INTERVAL_MEM_CACHE                 40000
#define CFG_TICK_INTERVAL_BMS_THINK                 500000
#define CFG_TICK_INTERVAL_WIFI                      200000
#define CFG_TICK_INTERVAL_BLE                       40000
#define CFG_TICK_INTERVAL_DCDC                      200000
#define CFG_TICK_INTERVAL_EVIC                      100000
#define CFG_TICK_INTERVAL_VEHICLE                   100000

/*
 * CAN BUS CONFIGURATION
 */
#define CFG_CAN0_SPEED CAN_BPS_500K // specify the speed of the CAN0 bus (EV)
#define CFG_CAN1_SPEED CAN_BPS_500K // specify the speed of the CAN1 bus (Car)
#define CFG_CAN0_NUM_RX_MAILBOXES 9 // amount of CAN bus receive mailboxes for CAN0
#define CFG_CAN1_NUM_RX_MAILBOXES 9 // amount of CAN bus receive mailboxes for CAN1
#define CFG_CANTHROTTLE_MAX_NUM_LOST_MSG 3 // maximum number of lost messages allowed

/*
 * MISCELLANEOUS
 *
 */
#define CFG_THROTTLE_TOLERANCE  150 //the max that things can go over or under the min/max without fault - 1/10% each #


/*
 * HARD CODED PARAMETERS
 *
 * If USE_HARD_CODED is defined or the checksum of the parameters stored in EEPROM,
 * the parameter values defined here are used instead of those stored in the EEPROM.
 */
//#define USE_HARD_CODED
#define ThrottleNumPots			1		//# of pots to use by default
#define ThrottleADC1			0		//Which pin to use
#define ThrottleADC2			1		//Which pin to use
#define ThrottleSubtype			1		//subtype 1 is a standard linear pot throttle
#define ThrottleRegenMinValue	270		//where does Regen stop (1/10 of percent)
#define ThrottleRegenMaxValue	30		//where Regen is at maximum (1/10 of percent)
#define ThrottleFwdValue		300		//where does forward motion start
#define ThrottleMapValue		750		//Where is the 1/2 way point for throttle
#define ThrottleMinRegenValue	0		//how many percent of full power to use at minimal regen
#define ThrottleMaxRegenValue	50		//how many percent of full power to use at maximum regen
#define ThrottleCreepValue		0		//how many percent of full power to use at creep
#define ThrottleMaxErrValue		150		//tenths of percentage allowable deviation between pedals
#define Throttle1MinValue		95		//Value ADC reads when pedal is up
#define Throttle1MaxValue		3150		//Value ADC reads when pedal fully depressed
#define Throttle2MinValue		0		//Value ADC reads when pedal is up
#define Throttle2MaxValue		0	//Value ADC reads when pedal fully depressed
#define BrakeMinValue			100		//Value ADC reads when brake is not pressed
#define BrakeMaxValue			3200		//Value ADC reads when brake is pushed all of the way down
#define BrakeMinRegenValue		0		//percent of full power to use for brake regen (min)
#define BrakeMaxRegenValue		50		//percent of full power to use for brake regen (max)
#define BrakeADC				2       //which ADC pin to use


#define MaxTorqueValue      3000 //in tenths of a Nm
#define	MaxRPMValue         6000 //DMOC will ignore this but we can use it ourselves for limiting
#define RPMSlewRateValue    10000 // rpm/sec the requested speed should change (speed mode)
#define TorqueSlewRateValue 6000 // 0.1Nm/sec the requested torque output should change (torque mode)
#define KilowattHrs         11000 //not currently used
#define PrechargeR          3000 //millliseconds precharge
#define NominalVolt         3300 //a reasonable figure for a lithium cell pack driving the DMOC (in tenths of a volt)
#define PrechargeRelay      4 // precharge relay pin digital out
#define MainContactorRelay  5 // main contactor realy pin - digital out
#define ReversePercent      50
#define CoolFan             255  //output to use for cooling fan
#define CoolOn              40 //temperature (in C) to turn on cooling fan
#define BrakeLight          255 //temperature to turn it off
#define CoolOff             35 //temperature to turn it off
#define RevLight            255 //temperature to turn it off
#define EnableIn            0 //enable  digital input pin
#define ReverseIn           1 //reverse digital input pin
#define MaxRegenWatts       40000 //in actual watts, there is no scale here
#define MaxAccelWatts       150000
#define BatteryCapacity     100
#define RegenTaperUpper     500
#define RegenTaperLower     75

/*
 * ARRAY SIZE
 *
 * Define the maximum number of various object lists.
 * These values should normally not be changed.
 */
#define CFG_DEV_MGR_MAX_DEVICES 30 // the maximum number of devices supported by the DeviceManager
#define CFG_CAN_NUM_OBSERVERS	7 // maximum number of device subscriptions per CAN bus
#define CFG_TIMER_NUM_OBSERVERS	7 // the maximum number of supported observers per timer
#define CFG_TIMER_USE_QUEUING	// if defined, TickHandler uses a queuing buffer instead of direct calls from interrupts
#define CFG_TIMER_BUFFER_SIZE	100 // the size of the queuing buffer for TickHandler
#define CFG_FAULT_HISTORY_SIZE	50 //number of faults to store in eeprom. A circular buffer so the last 50 faults are always stored.

/*
 * PIN ASSIGNMENT
 */
#define CFG_THROTTLE_NONE	255
#define BLINK_LED          73 //13 is L, 73 is TX, 72 is RX

#define NUM_ANALOG	7
#define NUM_DIGITAL	4
#define NUM_OUTPUT	8
#define NUM_EXT_IO  24

//CAN message ID ASSIGNMENTS FOR I/0 MANAGEMENT
#define CAN_SWITCH 0x606
#define CAN_OUTPUTS 0x607
#define CAN_ANALOG_INPUTS 0x608
#define CAN_DIGITAL_INPUTS 0x609

//These allow the code to automatically configure up to 6 devices when the device table is initialized
//Set to 0xFFFF to not set a device. Device numbers used here are found in DeviceTypes.h
#define AUTO_ENABLE_DEV1    0x1000 //DMOC645
#define AUTO_ENABLE_DEV2    0x1031 //pot throttle
#define AUTO_ENABLE_DEV3    0x1041 //Adafruit BLE
#define AUTO_ENABLE_DEV4    0xFFFF
#define AUTO_ENABLE_DEV5    0xFFFF
#define AUTO_ENABLE_DEV6    0xFFFF

#endif /* CONFIG_H_ */


