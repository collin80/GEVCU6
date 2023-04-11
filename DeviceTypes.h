/*
 * Device.h
 *
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

#ifndef DEVICE_TYPES_H_
#define DEVICE_TYPES_H_

enum DeviceType {
    DEVICE_ANY,
    DEVICE_MOTORCTRL,
    DEVICE_BMS,
    DEVICE_CHARGER,
    DEVICE_DISPLAY,
    DEVICE_THROTTLE,
    DEVICE_BRAKE,
    DEVICE_MISC,
    DEVICE_WIFI,
    DEVICE_IO,
    DEVICE_NONE
};

enum DeviceId { //unique device ID for every piece of hardware possible
    ELM327EMU = 0x650,
    POWERKEYPRO = 0x700,
    DMOC645 = 0x1000,
    BRUSA_DMC5 = 0x1001,
    CODAUQM = 0x1002,
    CKINVERTER = 0x1003,
    RINEHARTINV = 0x1004,
    C300INV     = 0x1005,
    TESTINVERTER =0x100F,
    BRUSACHARGE = 0x1010,
    TCCHCHARGE = 0x1011,
    LEAR=0x1012,
    OVARCHARGE = 0x1013,
    THROTTLE = 0x1030,
    POTACCELPEDAL = 0x1031,
    POTBRAKEPEDAL = 0x1032,
    CANACCELPEDAL = 0x1033,
    CANBRAKEPEDAL = 0x1034,
    ICHIP2128 = 0x1040,
    ADABLUE = 0x1041,
    TESTACCEL = 0x104F,
    DCDC = 0x1050,
    POTGEAR = 0x1100,
    THINKBMS = 0x2000,
    TESLABMS = 0x2050,
    BUILTINBMS = 0x2100,
    VEHICLESPECIFIC = 0x3000,
    FAULTSYS = 0x4000,
    EVICTUS = 0x4400,
    SYSTEM = 0x5000,
    HEARTBEAT = 0x5001,
    MEMCACHE = 0x5002,
    PIDLISTENER = 0x6000,
    INVALID = 0xFFFF
};

namespace LatchModes
{
    enum LATCHMODE
    {
        NO_LATCHING, //always reads the actual current input state
        LATCHING, //input state sticks "ON" until it is actually read then is automatically cleared
        TOGGLING, //Pushing a button toggles it on/off
        LOCKING //input locks as ON until it is explicitly set off by request
    };
}

#endif /* DEVICE_TYPES_H_ */


