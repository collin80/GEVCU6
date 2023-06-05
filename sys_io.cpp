/*
 * sys_io.cpp
 *
 * Handles the low level details of system I/O
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

some portions based on code credited as:
Arduino Due ADC->DMA->USB 1MSPS
by stimmer

*/

#include "sys_io.h"
#include "CANIODevice.h"

#undef HID_ENABLED

SPISettings spi_settings(1000000, MSBFIRST, SPI_MODE3);    

SystemIO::SystemIO()
{
    useSPIADC = true;
    
    for (int i = 0; i < NUM_EXT_IO; i++)
    {
        extendedDigitalOut[i].device = NULL;
        extendedDigitalIn[i].device = NULL;
        extendedAnalogOut[i].device = NULL;
        extendedAnalogIn[i].device = NULL;
    }
    
    numDigIn = NUM_DIGITAL;
    numDigOut = NUM_OUTPUT;
    numAnaIn = NUM_ANALOG;
    numAnaOut = 0;
    
    sysioState = SYSSTATE_UNINIT;
    adc2Initialized = false;
    adc3Initialized = false;
    lastInitAttempt = 0;
}

bool SystemIO::isInitialized()
{
    if (sysioState == SYSSTATE_INITIALIZED) return true;
    return false;
}

void SystemIO::pollInitialization()
{
    if (isInitialized()) return;
    if (millis() > (lastInitAttempt + 10))
    {
        lastInitAttempt = millis();
    }
}

void SystemIO::setSystemType(SystemType systemType) {
        sysType = GEVCU6;
}

SystemType SystemIO::getSystemType() {
    return sysType;
}

void SystemIO::setup() {
    int i;
    
    //the first order of business is to figure out what hardware we are running on and fill in
    //the pin tables.

    // TODO check pins
    Logger::info("Running on Feather M0 hardware");

    // PreCharge Relay // pref 17
    pinMode(PrechargeRelay, OUTPUT);
    SystemIO::setDigitalOutput(PrechargeRelay,0);

    // Main Relay  // pref 18
    pinMode(MainContactorRelay, OUTPUT);
    SystemIO::setDigitalOutput(MainContactorRelay, 0);

    // brake pedal (Analog) // pref A0/14/ BrakeADC
    pinMode(BrakeADC, INPUT);

    //gas pedal 1 (Analog) // pref A1/15 // ThrottleADC1
    pinMode(ThrottleADC1, INPUT);

    // gas pedal 2 (Analog) // pref A2/16 // ThrottleADC2
    pinMode(ThrottleADC2, INPUT);

    // enable (Digital) // pref 17// EnablePin
    if (EnableIn < 24 ) {
        pinMode(EnableIn, INPUT);
    }

    // reverse 2 (Digital) // pref 18 // ReverseIn
    if (ReverseIn < 24 ) {
        pinMode(ReverseIn, INPUT);
    }

    //Digital // CoolFan
    if (CoolFan < 24 ) {
        pinMode(CoolFan, OUTPUT);
    }

    //Digital // BrakeLight
    if (BrakeLight < 24 ) {
        pinMode(BrakeLight, OUTPUT);
    }

    //Digital // RevLight
    if (RevLight < 24 ) {
        pinMode(RevLight, OUTPUT);
    }

    // adjust ADC
    // analogReference(AR_INTERNAL_3_0);
    analogReadResolution(12);
}

void SystemIO::installExtendedIO(CANIODevice *device)
{
    bool found = false;
    int counter;
    
    //Logger::debug("Before adding extended IO counts are DI:%i DO:%i AI:%i AO:%i", numDigIn, numDigOut, numAnaIn, numAnaOut);
    //Logger::debug("Num Analog Inputs: %i", device->getAnalogInputCount());
    //Logger::debug("Num Analog Outputs: %i", device->getAnalogOutputCount());
    //Logger::debug("Num Digital Inputs: %i", device->getDigitalInputCount());
    //Logger::debug("Num Digital Outputs: %i", device->getDigitalOutputCount());
   
    if (device->getAnalogInputCount() > 0)
    {
        for (counter = 0; counter < NUM_EXT_IO; counter++)
        {
            if (extendedAnalogIn[counter].device == NULL)
            {
                for (int i = 0; i < device->getAnalogInputCount(); i++)
                {
                    if ((counter + i) == NUM_EXT_IO) break;
                    extendedAnalogIn[counter + i].device = device;
                    extendedAnalogIn[counter + i].localOffset = i;
                }
                break;
            }
        }
    }
    
    if (device->getAnalogOutputCount() > 0)
    {
        for (counter = 0; counter < NUM_EXT_IO; counter++)
        {
            if (extendedAnalogOut[counter].device == NULL)
            {
                for (int i = 0; i < device->getAnalogOutputCount(); i++)
                {
                    if ((counter + i) == NUM_EXT_IO) break;
                    extendedAnalogOut[counter + i].device = device;
                    extendedAnalogOut[counter + i].localOffset = i;
                }
                break;
            }
        }
    }

    if (device->getDigitalOutputCount() > 0)
    {
        for (counter = 0; counter < NUM_EXT_IO; counter++)
        {
            if (extendedDigitalOut[counter].device == NULL)
            {
                for (int i = 0; i < device->getDigitalOutputCount(); i++)
                {
                    if ((counter + i) == NUM_EXT_IO) break;
                    extendedDigitalOut[counter + i].device = device;
                    extendedDigitalOut[counter + i].localOffset = i;
                }
                break;
            }
        }
    }

    if (device->getDigitalInputCount() > 0)
    {
        for (counter = 0; counter < NUM_EXT_IO; counter++)
        {
            if (extendedDigitalIn[counter].device == NULL)
            {
                for (int i = 0; i < device->getDigitalInputCount(); i++)
                {
                    if ((counter + i) == NUM_EXT_IO) break;
                    extendedDigitalIn[counter + i].device = device;
                    extendedDigitalIn[counter + i].localOffset = i;
                }
                break;
            }
        }
    }
    
    int numDI = NUM_DIGITAL; 
    for (int i = 0; i < NUM_EXT_IO; i++)
    {
        if (extendedDigitalIn[i].device != NULL) numDI++;
        else break;
    }
    numDigIn = numDI;
    
    int numDO = NUM_OUTPUT;
    for (int i = 0; i < NUM_EXT_IO; i++)
    {
        if (extendedDigitalOut[i].device != NULL) numDO++;
        else break;
    }
    numDigOut = numDO;

    int numAI = NUM_ANALOG; 
    for (int i = 0; i < NUM_EXT_IO; i++)
    {
        if (extendedAnalogIn[i].device != NULL) numAI++;
        else break;
    }
    numAnaIn = numAI;

    int numAO = 0; //GEVCU has no real analog outputs - there are PWM but they're on the digital outputs
    for (int i = 0; i < NUM_EXT_IO; i++)
    {
        if (extendedDigitalIn[i].device != NULL) numAO++;
        else break;
    }
    numAnaOut = numAO;
    Logger::debug("After added extended IO the counts are DI:%i DO:%i AI:%i AO:%i", numDigIn, numDigOut, numAnaIn, numAnaOut);
}

int SystemIO::numDigitalInputs()
{
    return numDigIn;
}

int SystemIO::numDigitalOutputs()
{
    return numDigOut;
}

int SystemIO::numAnalogInputs()
{
    return numAnaIn;
}

int SystemIO::numAnalogOutputs()
{
    return numAnaOut;
}

/*
get value of one of the analog inputs
On GEVCU6.2 or higher
Gets reading over SPI which is still pretty fast. The SPI connected chip is 24 bit
but too much of the code for GEVCU uses 16 bit integers for storage so the 24 bit values returned
are knocked down to 16 bit values before being passed along.
*/
int16_t SystemIO::getAnalogIn(uint8_t which) {
    int base;
    if (which > numAnaIn) {
        return 0;
    }
    
    if (!isInitialized()) return 0;
    
    if (which < NUM_ANALOG) {
        //if (getSystemType() == GEVCU6)
        //{
            int32_t valu;
            //first 4 analog readings must match old methods

            if (which < 2)
            {
                valu = getSPIADCReading(CS1, (which & 1) + 1);
            }
            else if (which < 4) valu = getSPIADCReading(CS2, (which & 1) + 1);
            //the next three are new though. 4 = current sensor, 5 = pack high (ref to mid), 6 = pack low (ref to mid)
            else if (which == 4) valu = getSPIADCReading(CS1, 0);
            else if (which == 5) valu = getSPIADCReading(CS3, 1);
            else if (which == 6) valu = getSPIADCReading(CS3, 2);
            valu = valu / 2048;
            valu -= adc_comp[which].offset;
            valu = (valu * adc_comp[which].gain) / 1024;
            return valu;
        //}
    }
    else //the return makes this superfluous...
    {        
        //handle an extended I/O call
        CANIODevice *dev = extendedAnalogIn[which - NUM_ANALOG].device;
        if (dev) return dev->getAnalogInput(extendedAnalogIn[which - NUM_ANALOG].localOffset);
        return 0;
    }
    return 0; //if it falls through and nothing could provide the answer then return 0
}

boolean SystemIO::setAnalogOut(uint8_t which, int32_t level)
{
    if (which >= numAnaOut) return false;
    CANIODevice *dev;
    dev = extendedAnalogOut[which].device;
    if (dev) dev->setAnalogOutput(extendedAnalogOut[which].localOffset, level);    
    return false;
}

int32_t SystemIO::getAnalogOut(uint8_t which)
{
    if (which >= numAnaOut) return 0;
    CANIODevice *dev;
    dev = extendedAnalogOut[which].device;
    if (dev) return dev->getAnalogOutput(extendedAnalogOut[which].localOffset);    
    return false;
}


//the new pack voltage and current functions however, being new, don't have legacy problems so they're 24 bit ADC.
int32_t SystemIO::getCurrentReading()
{
    int32_t valu;
    int64_t gainTemp;
    valu = getSPIADCReading(CS1, 0);
    valu -= (adc_comp[4].offset * 32);
    valu = valu >> 3;
    gainTemp = (int64_t)((int64_t)valu * adc_comp[4].gain) / 16384ll;
    valu = (int32_t) gainTemp;
    return valu;
}

int32_t SystemIO::getPackHighReading()
{
    int32_t valu;
    int64_t gainTemp;
    valu = getSPIADCReading(CS3, 1);
    valu -= (adc_comp[5].offset * 32);
    valu = valu >> 3; //divide by 8
    gainTemp = (int64_t)((int64_t)valu * adc_comp[5].gain) / 16384ll;
    valu = (int32_t) gainTemp;
    return valu;
}

int32_t SystemIO::getPackLowReading()
{
    int32_t valu;
    int64_t gainTemp;
    valu = getSPIADCReading(CS3, 2);
    valu -= (adc_comp[6].offset * 32);
    valu >>= 3;

    gainTemp = (int64_t)((int64_t)valu * adc_comp[6].gain) / 16384ll;
    valu = (int32_t) -gainTemp;    
    return valu;
}

//get value of one of the 4 digital inputs
boolean SystemIO::getDigitalIn(uint8_t pin) {
    
    if (pin < MAX_PORT) return !(digitalRead(pin));
    else
    {
        CANIODevice *dev;
        dev = extendedDigitalIn[pin - NUM_DIGITAL].device;
        if (dev) return dev->getDigitalInput(extendedDigitalIn[pin - NUM_DIGITAL].localOffset);
    }
    return false;
}

//set output high or not
void SystemIO::setDigitalOutput(uint8_t pin, boolean active) {
    Logger::info("SET DIGITAL:  pin %d  numDigOut %d MAX_PORT %d active %s", pin, numDigOut, MAX_PORT, active);
    
    if (pin < MAX_PORT)
    {
        if (active)
            digitalWrite(pin, LOW);
        else digitalWrite(pin, HIGH);
    }
    else
    {
        CANIODevice *dev;
        dev = extendedDigitalOut[pin - NUM_OUTPUT].device;
        if (dev) return dev->setDigitalOutput(extendedDigitalOut[pin - NUM_OUTPUT].localOffset, active);
    }
}

//get current value of output state (high?)
boolean SystemIO::getDigitalOutput(uint8_t pin) {
    
    if (pin < NUM_OUTPUT)
    {
        if (pin == 255) return false;
        return digitalRead(pin);
    }
    else
    {
        CANIODevice *dev;
        dev = extendedDigitalOut[pin - NUM_OUTPUT].device;
        if (dev) return dev->getDigitalOutput(extendedDigitalOut[pin - NUM_OUTPUT].localOffset);
    }
    return false;
}

int32_t SystemIO::getSPIADCReading(int CS, int sensor)
{
    int32_t result;
    int32_t byt;
    
    if (!isInitialized()) return 0;
    
    //Logger::debug("SPI Read CS: %i Sensor: %i", CS, sensor);
    SPI.beginTransaction(spi_settings);
    digitalWrite(CS, LOW);
    if (sensor == 0) SPI.transfer(ADE7913_READ | ADE7913_AMP_READING);
    if (sensor == 1) SPI.transfer(ADE7913_READ | ADE7913_ADC1_READING);
    if (sensor == 2) SPI.transfer(ADE7913_READ | ADE7913_ADC2_READING);
    byt = SPI.transfer(0);
    result = (byt << 16);
    byt = SPI.transfer(0);
    result = result + (byt << 8);
    byt = SPI.transfer(0);
    digitalWrite(CS, HIGH);
    SPI.endTransaction();
    
    result = result + byt;
    //now we've got the whole 24 bit value but it is a signed 24 bit value so we must sign extend
    if (result & (1 << 23)) result |= (255 << 24);
    return result;
}

/*
 * adc is the adc port to calibrate, update if true will write the new value to EEPROM automatically
 */
bool SystemIO::calibrateADCOffset(int adc, bool update)
{
    int32_t accum = 0;
    
    if (adc < 0 || adc > 6) return false;
    
    for (int j = 0; j < 500; j++)
    {
        if (adc < 2)
        {
            accum += getSPIADCReading(CS1, (adc & 1) + 1);
        }
        else if (adc < 4) accum += getSPIADCReading(CS2, (adc & 1) + 1);
        //4 = current sensor, 5 = pack high (ref to mid), 6 = pack low (ref to mid)
        else if (adc == 4) accum += getSPIADCReading(CS1, 0);
        else if (adc == 5) accum += getSPIADCReading(CS3, 1);
        else if (adc == 6) accum += getSPIADCReading(CS3, 2);

        //normally one shouldn't call watchdog reset in multiple
        //places but this is a special case.

        Watchdog.reset();
        delay(2);
    }
    accum /= 500;
    if (adc < 4) accum >>= 11;
    else accum >>= 5;
    //if (accum > 2) accum -= 2; 
    Logger::console("ADC %i offset is now %i", adc, accum);
    return true;
}


//much like the above function but now we use the calculated offset and take readings, average them
//and figure out how to set the gain such that the average reading turns up to be the target value
bool SystemIO::calibrateADCGain(int adc, int32_t target, bool update)
{
    int32_t accum = 0;
    
    if (adc < 0 || adc > 6) return false;
    
    for (int j = 0; j < 500; j++)
    {
        if (adc < 2)
        {
            accum += getSPIADCReading(CS1, (adc & 1) + 1);
        }
        else if (adc < 4) accum += getSPIADCReading(CS2, (adc & 1) + 1);
        //the next three are new though. 4 = current sensor, 5 = pack high (ref to mid), 6 = pack low (ref to mid)
        else if (adc == 4) accum += getSPIADCReading(CS1, 0);
        else if (adc == 5) accum += getSPIADCReading(CS3, 1);
        else if (adc == 6) accum += getSPIADCReading(CS3, 2);

        //normally one shouldn't call watchdog reset in multiple
        //places but this is a special case.

        Watchdog.reset();
        delay(2);
    }
    accum /= 500;
    Logger::console("Unprocessed accum: %i", accum);
    
    //now apply the proper offset we've got set.
    if (adc < 4) {
        accum /= 2048;
        accum -= adc_comp[adc].offset;
    }
    else {
        accum -= adc_comp[adc].offset * 32;
        accum >>= 3;
    }
    
    if ((target / accum) > 20) {
        Logger::console("Calibration not possible. Check your target value.");
        return false;
    }
    
    if (accum < 1000 && accum > -1000) {
        Logger::console("Readings are too low. Try applying more voltage/current");
        return false;
    }
    
    //1024 is one to one so all gains are multiplied by that much to bring them into fixed point math.
    //we've got a reading accum and a target. The rational gain is target/accum    
    adc_comp[adc].gain = (int16_t)((16384ull * target) / accum);
    
    Logger::console("Accum: %i    Target: %i", accum, target);
    Logger::console("ADC %i gain is now %i", adc, adc_comp[adc].gain);
    return true;

}

SystemIO systemIO;


