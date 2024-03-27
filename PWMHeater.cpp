/*
 * PWMHeater.cpp - PWM control of a water heater and the water pump. Used to get
   cabin heat in a standard cabin heater
 *
 Copyright (c) 2023 Collin Kidder

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

#include "PWMHeater.h"

typedef struct PWM_TABLE
{
    uint16_t OutputPWM;
    uint16_t inputADC;
    uint8_t powerPercent;
} PWM_TABLE;

//output, in_adc
//table set up to give a good distribution across the available PWM space. The stuff in between doesn't change things much
//pwm really only goes to 66% max but everything is scaled such that 66% PWM is thus 100% power
PWM_TABLE pwmTable[7] = 
{
    {300, 0, 0}, //0% output pwm
    {320, 500, 21}, //14% output pwm
    {340, 550, 36}, //24% output
    {370, 700, 50}, //33% output
    {390, 850, 72}, //48% output
    {430, 1200, 84}, //56% output
    {470, 2300, 100}  //66% output
};

/*
 * Constructor
 */
PWMHeater::PWMHeater() : Device() {
    prefsHandler = new PrefHandler(PWMHEATER);
    commonName = "PWM Water Heater";
    active = false;
    duty = 500;
}

/*
 * Setup the device.
 */
void PWMHeater::setup() {
    tickHandler.detach(this); // unregister from TickHandler first

    Logger::info("add device: PWMHeater (id: %X, %X)", PWMHEATER, this);

    loadConfiguration();

    Device::setup(); //call base class

    tickHandler.attach(this, CFG_TICK_INTERVAL_POT_THROTTLE);

    //setpt = 670.0;
    //pidCtrl.begin(&adcIn, &pwmOut, &setpt, 1.0, 0.0, 0.0);
    //pidCtrl.setOutputLimits(460.0, 650.0);
    //pidCtrl.start();
}

/*
 * Process a timer event. Every tick read the analog temperature input and use it
 to determine whether the PWM must be adjusted up or down. 
 heat adc target 550
 pwm 30% = 0 heat
 pwm 47% = max heat

200 = nothing
300 = very low 1-3%
310 = about 14% like 320
320 = 14%
330 = 22%
340 = 24%
350 = 31%
370 = 33%
380 = 46%
400 = 50%
420 = 52%
440 = 61%
450 = 63%
470 = 66% duty
500 = 60%
600 = 46%
650 = 31%
700 = 12%
750,800,900 = nothing at all                
 */
void PWMHeater::handleTick() 
{
    int power;
    PWMHeaterConfiguration *config = (PWMHeaterConfiguration *) getConfiguration();

    if (config->enablePin == 255) return;

    if (systemIO.getDigitalIn(config->enablePin)) //if enable pin is active
    {
        if (!active)
        {
            Logger::debug("Heat has been requested. Starting PWM");
            //if (config->pumpOutputPin < 255) systemIO.setDigitalOutput(config->pumpOutputPin, true);
            if (config->pwmPin < 255)
            {
                duty = 440; //about 61%. Pretty powerfully on.
                systemIO.setDigitalSlowPWM(config->pwmPin, 50, duty);
            }
            active = true;
        }
        else
        {
            int16_t aInput;
            double temperature;
            if (config->analogTempPin != 255)
            {
                aInput = systemIO.getAnalogIn(config->analogTempPin);
                adcIn = aInput;
                for (int p = 0; p < 7; p++)
                {
                    if (pwmTable[p].inputADC < adcIn)
                    {
                        duty = pwmTable[p].OutputPWM;
                        power = pwmTable[p].powerPercent;
                        //Logger::debug("Potential output pwm: %i", duty);
                    }
                    else break; // no need to keep going then.
                }

                //pidCtrl.compute();
                //duty = (int)pwmOut;
                Logger::debug("ADC Input: %i Duty: %i Power = %i%%", aInput, duty, power);
                if (config->pwmPin != 255) systemIO.updateDigitalSlowPWMDuty(config->pwmPin, duty);
            }
        }
    }
    else
    {
        if (active)
        {
            Logger::debug("Heat request has ceased. Stopping PWM");
            if (config->pumpOutputPin < 255) systemIO.setDigitalOutput(config->pumpOutputPin, false);
            if (config->pwmPin < 255)
            {
                systemIO.updateDigitalSlowPWMDuty(config->pwmPin, 0);
                systemIO.setDigitalOutput(config->pwmPin, false);
            }
            active = false;
        }
    }
}

/*
 * Return the device ID
 */
DeviceId PWMHeater::getId() {
    return (PWMHEATER);
}

/*
 * Load the device configuration.
 * If possible values are read from EEPROM. If not, reasonable default values
 * are chosen and the configuration is overwritten in the EEPROM.
 */
void PWMHeater::loadConfiguration() {
    PWMHeaterConfiguration *config = (PWMHeaterConfiguration *) getConfiguration();

    if (!config) { // as lowest sub-class make sure we have a config object
        config = new PWMHeaterConfiguration();
        setConfiguration(config);
    }

    Device::loadConfiguration(); // call parent

#ifdef USE_HARD_CODED
    if (false) {
#else
    if (prefsHandler->checksumValid()) //checksum is good, read in the values stored in EEPROM
    {
#endif
        Logger::debug(PWMHEATER, (char *)Constants::validChecksum);
        prefsHandler->read(20, &config->desiredWaterTemp);
        prefsHandler->read(21, &config->pwmPin);
        prefsHandler->read(22, &config->pumpOutputPin);
        prefsHandler->read(23, &config->analogTempPin);
        prefsHandler->read(24, &config->enablePin);
    }
    else
    { //checksum invalid. Reinitialize values and store to EEPROM
        Logger::warn(PWMHEATER, (char *)Constants::invalidChecksum);
        config->desiredWaterTemp = 90;
        config->pwmPin = 5; //DOUT5
        config->pumpOutputPin = 6; //DOUT6
        config->analogTempPin = 3; //A3
        config->enablePin = 1; //DIN1
        saveConfiguration();
    }
}

/*
 * Store the current configuration to EEPROM
 */
void PWMHeater::saveConfiguration() {
    PWMHeaterConfiguration *config = (PWMHeaterConfiguration *) getConfiguration();

    Device::saveConfiguration(); // call parent

    prefsHandler->write(20, config->desiredWaterTemp);
    prefsHandler->write(21, config->pwmPin);
    prefsHandler->write(22, config->pumpOutputPin);
    prefsHandler->write(23, config->analogTempPin);
    prefsHandler->write(24, config->enablePin);
    prefsHandler->saveChecksum();
    prefsHandler->forceCacheWrite();
}
