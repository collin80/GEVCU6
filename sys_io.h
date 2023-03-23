/*
 * sys_io.h
 *
 * Handles raw interaction with system I/O
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


#ifndef SYS_IO_H_
#define SYS_IO_H_

#include <Arduino.h>
#include <SPI.h>
#include "config.h"
#include "Logger.h"
#include <avr/wdt.h>

class CANIODevice;

#define CS1	26
#define CS2	28
#define CS3	30

#define ADE7913_WRITE	0
#define ADE7913_READ	4

#define ADE7913_AMP_READING	0
#define ADE7913_ADC1_READING	1 << 3
#define ADE7913_ADC2_READING	2 << 3
#define ADE7913_ADC_CRC		4 << 3
#define ADE7913_CTRL_CRC	5 << 3
#define ADE7913_CNT_SNAP	7 << 3
#define ADE7913_CONFIG		8 << 3
#define ADE7913_STATUS0		9 << 3
#define ADE7913_LOCK		0xA << 3
#define ADE7913_SYNC_SNAP	0xB << 3
#define ADE7913_COUNTER0	0xC << 3
#define ADE7913_COUNTER1	0xD << 3
#define ADE7913_EMI_CTRL	0xE << 3
#define ADE7913_STATUS1		0xF << 3
#define ADE7913_TEMPOS		0x18 << 3

typedef struct {
    uint16_t offset;
    uint16_t gain;
} ADC_COMP;

enum SystemType {
    GEVCU1 = 1,
    GEVCU2 = 2,
    GEVCU3 = 3,
    GEVCU4 = 4,
    GEVCU5 = 5,
    GEVCU6 = 6
};

enum SYSIO_STATE {
    SYSSTATE_UNINIT,
    SYSSTATE_ADC1OK,
    SYSSTATE_INITIALIZED
};

class ExtendedIODev
{
public:
    CANIODevice *device;
    uint8_t localOffset;
};

class SystemIO
{
public:
    SystemIO();
    
    void setup();
    void setup_ADC_params();

    int16_t getAnalogIn(uint8_t which); //get value of one of the 4 analog inputs
    boolean setAnalogOut(uint8_t which, int32_t level);
    int32_t getAnalogOut(uint8_t which);
    boolean getDigitalIn(uint8_t which); //get value of one of the 4 digital inputs
    void setDigitalOutput(uint8_t which, boolean active); //set output high or not
    boolean getDigitalOutput(uint8_t which); //get current value of output state (high?)    
    
    void setDigitalInLatchMode(int which, LatchModes::LATCHMODE mode);
    void unlockDigitalInLatch(int which);

    int32_t getCurrentReading();
    int32_t getPackHighReading();
    int32_t getPackLowReading();
    
    void installExtendedIO(CANIODevice *device);
    
    int numDigitalInputs();
    int numDigitalOutputs();
    int numAnalogInputs();
    int numAnalogOutputs();
    
    void setSystemType(SystemType);
    SystemType getSystemType();
    bool calibrateADCOffset(int, bool);
    bool calibrateADCGain(int, int32_t, bool);
    bool isInitialized();
    void pollInitialization();

private:
    int32_t getSPIADCReading(int CS, int sensor);
    int16_t getRawADC(uint8_t which);
    bool setupSPIADC();

    uint8_t dig[NUM_DIGITAL];
    uint8_t adc[NUM_ANALOG][2];
    uint8_t out[NUM_OUTPUT];

    SystemType sysType;

    ADC_COMP adc_comp[NUM_ANALOG]; //GEVCU 6.2 has 7 adc inputs but three are special

    bool useSPIADC;
    bool adc2Initialized;
    bool adc3Initialized;
    SYSIO_STATE sysioState;
    uint32_t lastInitAttempt;
    
    int numDigIn;
    int numDigOut;
    int numAnaIn;
    int numAnaOut;
    
    ExtendedIODev extendedDigitalOut[NUM_EXT_IO];
    ExtendedIODev extendedDigitalIn[NUM_EXT_IO];
    ExtendedIODev extendedAnalogOut[NUM_EXT_IO];
    ExtendedIODev extendedAnalogIn[NUM_EXT_IO];    
};

extern SystemIO systemIO;

#endif
