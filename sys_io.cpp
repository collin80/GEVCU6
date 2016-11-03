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
        setupSPIADC();
    }
}

void SystemIO::setup_ADC_params()
{
    int i;
    //requires the value to be contiguous in memory
    for (i = 0; i < 7; i++) {
        sysPrefs->read(EESYS_ADC0_GAIN + 4*i, &adc_comp[i].gain);
        sysPrefs->read(EESYS_ADC0_OFFSET + 4*i, &adc_comp[i].offset);
        if (adc_comp[i].gain == 0xFFFF) adc_comp[i].gain = 1024;
        Logger::debug("ADC:%d GAIN: %d Offset: %d", i, adc_comp[i].gain, adc_comp[i].offset);
    }
}

void SystemIO::setSystemType(SystemType systemType) {
    if (systemType >= GEVCU1 && systemType <= GEVCU6)
    {
        sysType = systemType;
        sysPrefs->write(EESYS_SYSTEM_TYPE, (uint8_t)sysType);
    }
}

SystemType SystemIO::getSystemType() {
    return sysType;
}

void SystemIO::setup() {
    int i;
    
    //the first order of business is to figure out what hardware we are running on and fill in
    //the pin tables.

    sysPrefs->read(EESYS_SYSTEM_TYPE, (uint8_t *) &sysType);
    if (sysType == GEVCU6) {
        Logger::info("Running on GEVCU 6.2 hardware");
        dig[0]=48;
        dig[1]=49;
        dig[2]=50;
        dig[3]=51;
        adc[0][0] = 255;
        adc[0][1] = 255; //doesn't use SAM3X analog
        adc[1][0] = 255;
        adc[1][1] = 255;
        adc[2][0] = 255;
        adc[2][1] = 255;
        adc[3][0] = 255;
        adc[3][1] = 255;
        out[0] = 4;
        out[1] = 5;
        out[2] = 6;
        out[3] = 7;
        out[4] = 2;
        out[5] = 3;
        out[6] = 8;
        out[7] = 9;
        useSPIADC = true;
        pinMode(26, OUTPUT); //Chip Select for first ADC chip
        pinMode(28, OUTPUT); //Chip select for second ADC chip
        pinMode(30, OUTPUT); //chip select for third ADC chip
        digitalWrite(26, HIGH);
        digitalWrite(28, HIGH);
        digitalWrite(30, HIGH);
        SPI.begin();
        pinMode(32, INPUT); //Data Ready indicator
        SPI.begin(); //sets up with default 4Mhz, MSB first
    } else {
        Logger::info("Running on unknown hardware");
        dig[0]=48;
        dig[1]=49;
        dig[2]=50;
        dig[3]=51;
        adc[0][0] = 255;
        adc[0][1] = 255; //doesn't use SAM3X analog
        adc[1][0] = 255;
        adc[1][1] = 255;
        adc[2][0] = 255;
        adc[2][1] = 255;
        adc[3][0] = 255;
        adc[3][1] = 255;
        out[0] = 4;
        out[1] = 5;
        out[2] = 6;
        out[3] = 7;
        out[4] = 2;
        out[5] = 3;
        out[6] = 8;
        out[7] = 9;
        useSPIADC = true;
        pinMode(26, OUTPUT); //Chip Select for first ADC chip
        pinMode(28, OUTPUT); //Chip select for second ADC chip
        pinMode(30, OUTPUT); //chip select for third ADC chip
        digitalWrite(26, HIGH);
        digitalWrite(28, HIGH);
        digitalWrite(30, HIGH);
        SPI.begin();
        pinMode(32, INPUT); //Data Ready indicator
        SPI.begin(); //sets up with default 4Mhz, MSB first        
    }

    for (i = 0; i < NUM_DIGITAL; i++) pinMode(dig[i], INPUT);
    for (i = 0; i < NUM_OUTPUT; i++) {
        if (out[i] != 255) {
            pinMode(out[i], OUTPUT);
            digitalWrite(out[i], LOW);
        }
    }    

    setup_ADC_params();

    setupSPIADC();
}

bool SystemIO::setupSPIADC()
{
    byte result;
    //ADC chips use this format for SPI command byte: 0-1 = reserved set as 0, 2 = read en (0=write), 3-7 = register address

    switch (sysioState)
    {
    case SYSSTATE_UNINIT:
        SPI.beginTransaction(spi_settings);
        SPI.transfer(0);
        SPI.endTransaction();
        Logger::info("Trying to wait ADC1 as ready");
        SPI.beginTransaction(spi_settings);
        digitalWrite(CS1, LOW); //select first ADC chip
        SPI.transfer(ADE7913_READ | ADE7913_STATUS0);
        result = SPI.transfer(0);
        digitalWrite(CS1, HIGH);
        SPI.endTransaction();
        if (result & 1)  //not ready yet
        {
            return false;
        }
        else
        {
            sysioState = SYSSTATE_ADC1OK;
            Logger::info("ADC1 is ready. Trying to enable clock out");
            //Now enable the CLKOUT function on first unit so that the other two will wake up
            SPI.beginTransaction(spi_settings);
            digitalWrite(CS1, LOW);
            SPI.transfer(ADE7913_WRITE |  ADE7913_CONFIG);
            SPI.transfer(1 | 2 << 4); //Set clock out enable and ADC_FREQ to 2khz
            digitalWrite(CS1, HIGH);
            SPI.endTransaction();
            break;
        }
        break;
    case SYSSTATE_ADC1OK:
        if (!adc2Initialized)
        {
            SPI.beginTransaction(spi_settings);
            digitalWrite(CS2, LOW); //select second ADC chip
            SPI.transfer(ADE7913_READ | ADE7913_STATUS0);
            result = SPI.transfer(0);
            digitalWrite(CS2, HIGH);
            SPI.endTransaction();
            if (result & 1)  //not ready yet
            {
               
            }
            else
            {
                adc2Initialized = true;
            }
        }
        if (!adc3Initialized)
        {
            SPI.beginTransaction(spi_settings);
            digitalWrite(CS3, LOW); //select third ADC chip
            SPI.transfer(ADE7913_READ | ADE7913_STATUS0);
            result = SPI.transfer(0);
            digitalWrite(CS3, HIGH);
            SPI.endTransaction();
            if (result & 1)  //not ready yet
            {
            }
            else
            {
                adc3Initialized = true;
            }
        }
        if (adc2Initialized && adc3Initialized)
        {
            SPI.beginTransaction(spi_settings);
            digitalWrite(CS2, LOW);
            SPI.transfer(ADE7913_WRITE |  ADE7913_CONFIG);
            SPI.transfer(3 << 4 | 1 << 7); //Set ADC_FREQ to 1khz and lower bandwidth to 2khz
            digitalWrite(CS2, HIGH);
            SPI.endTransaction();
            SPI.beginTransaction(spi_settings);
            digitalWrite(CS3, LOW);
            SPI.transfer(ADE7913_WRITE |  ADE7913_CONFIG);
            SPI.transfer(3 << 4 | 1 << 7); //Set ADC_FREQ to 1khz and lower bandwidth to 2khz
            digitalWrite(CS3, HIGH);
            SPI.endTransaction();
      
            Logger::info("ADC chips 2 and 3 have been successfully started!");
            sysioState = SYSSTATE_INITIALIZED;
        }
        break;
    case SYSSTATE_INITIALIZED: //nothing to do, already all set!
        return true;
        break;
    }
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
Get an ADC reading but without any gain or offset
*/
int16_t SystemIO::getRawADC(uint8_t which) {
    int32_t val;

    int32_t valu;
    
    if (!isInitialized()) return 0;
    
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
    val = valu / 2048; //cut reading down to 13 bit signed value +/- 4096 essentially
    
    return val;
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
}

int32_t SystemIO::getAnalogOut(uint8_t which)
{
    if (which >= numAnaOut) return 0;
    CANIODevice *dev;
    dev = extendedAnalogOut[which].device;
    if (dev) return dev->getAnalogOutput(extendedAnalogOut[which].localOffset);    
}


//the new pack voltage and current functions however, being new, don't have legacy problems so they're 24 bit ADC.
int32_t SystemIO::getCurrentReading()
{
    int32_t valu;
    valu = getSPIADCReading(CS1, 0);
    valu -= (adc_comp[6].offset * 32);
    valu = valu >> 3;
    valu = (valu * adc_comp[6].gain) / 1024;
    return valu;
}

int32_t SystemIO::getPackHighReading()
{
    int32_t valu;
    valu = getSPIADCReading(CS3, 1);
    valu -= (adc_comp[4].offset * 32);
    valu = valu >> 3; //divide by 8
    valu = (valu * adc_comp[4].gain) / 1024;
    return valu;
}

int32_t SystemIO::getPackLowReading()
{
    int32_t valu;
    valu = getSPIADCReading(CS3, 2);
    valu -= (adc_comp[5].offset * 32);
    valu = valu >> 3;
    valu = (valu * adc_comp[5].gain) / 1024;
    return valu;
}

//get value of one of the 4 digital inputs
boolean SystemIO::getDigitalIn(uint8_t which) {
    if (which >= numDigIn) return false;
    
    if (which < NUM_DIGITAL) return !(digitalRead(dig[which]));
    else
    {
        CANIODevice *dev;
        dev = extendedDigitalIn[which - NUM_DIGITAL].device;
        if (dev) return dev->getDigitalInput(extendedDigitalIn[which - NUM_DIGITAL].localOffset);
    }
}

//set output high or not
void SystemIO::setDigitalOutput(uint8_t which, boolean active) {
    if (which >= numDigOut) return;
    
    if (which < NUM_OUTPUT)
    {
        if (out[which] == 255) return;
        if (active)
            digitalWrite(out[which], HIGH);
        else digitalWrite(out[which], LOW);
    }
    else
    {
        CANIODevice *dev;
        dev = extendedDigitalOut[which - NUM_OUTPUT].device;
        if (dev) return dev->setDigitalOutput(extendedDigitalOut[which - NUM_OUTPUT].localOffset, active);
    }
}

//get current value of output state (high?)
boolean SystemIO::getDigitalOutput(uint8_t which) {
    if (which >= numDigOut) return false;
    
    if (which < NUM_OUTPUT)
    {
        if (out[which] == 255) return false;
        return digitalRead(out[which]);
    }
    else
    {
        CANIODevice *dev;
        dev = extendedDigitalOut[which - NUM_OUTPUT].device;
        if (dev) return dev->getDigitalOutput(extendedDigitalOut[which - NUM_OUTPUT].localOffset);
    }
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
        watchdogReset();
        delay(4);
    }
    accum /= 500;
    if (adc < 4) accum >>= 11;
    else accum >>= 5;
    //if (accum > 2) accum -= 2;
    if (update) sysPrefs->write(EESYS_ADC0_OFFSET + (4*adc), (uint16_t)(accum));    
    Logger::console("ADC %i offset is now %i", adc, accum);
}

SystemIO systemIO;


