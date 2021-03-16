/*
 * C300MotorController.cpp
 *
 *
Copyright (c) 2021 EVTV

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

#include "C300MotorController.h"

C300MotorController::C300MotorController() : MotorController() {
    prefsHandler = new PrefHandler(C300INV);

    selectedGear = NEUTRAL;
    operationState = DISABLED;
    actualState = DISABLED;
    online = 0;
    activityCount = 0;
//	maxTorque = 2000;
    commonName = "C300 Inverter";
}

void C300MotorController::setup() {
    tickHandler.detach(this);

    Logger::info("add device: C300 (id:%X, %X)", DMOC645, this);

    loadConfiguration();
    MotorController::setup(); // run the parent class version of this function

    //CFF7902, CFF7B02, CFF7A02, CFF7C02 are the addresses. Only the lower nibble of the second byte changes
    //So, set up to filter and let anything in that nibble through but otherwise match exactly
    canHandlerEv.attach(this, 0x0CFF7802, 0x0FFFF0FF, true);

    running = false;
    setPowerMode(modeTorque);
    setSelectedGear(NEUTRAL);
    setOpState(DISABLED );
    ms=millis();

    tickHandler.attach(this, CFG_TICK_INTERVAL_MOTOR_CONTROLLER_C300);
}

void C300MotorController::handleCanFrame(CAN_FRAME *frame) {
    int RotorTemp, invTemp, StatorTemp;
    int temp;
    online = true; //if a frame got to here then it passed the filter and must have been from the DMOC

    Logger::debug("C300 CAN received: %X  %X  %X  %X  %X  %X  %X  %X  %X", frame->id,frame->data.bytes[0] ,frame->data.bytes[1],frame->data.bytes[2],frame->data.bytes[3],frame->data.bytes[4],frame->data.bytes[5],frame->data.bytes[6],frame->data.bytes[70]);

    switch (frame->id) 
    {
    case 0x0CFF7902: //MCU status, mode, output torque, etc
        activityCount++;
        break;
    case 0x0CFF7B02: //torque limits and temperatures
        activityCount++;
        break;
    case 0x0CFF7A02: //input voltage and current
        activityCount++;
        break;
    case 0x0CFF7C02: //fault reporting
        activityCount++;
        break;
    }
}

void C300MotorController::handleTick() {

    MotorController::handleTick(); //kick the ball up to papa

    if (activityCount > 0)
    {
        activityCount--;
        if (activityCount > 60) activityCount = 60;
        if (activityCount > 40) //If we are receiving regular CAN messages from DMOC, this will very quickly get to over 40. We'll limit
            // it to 60 so if we lose communications, within 20 ticks we will decrement below this value.
        {
            Logger::debug("Enable Input Active? %T         Reverse Input Active? %T" ,systemIO.getDigitalIn(getEnableIn()),systemIO.getDigitalIn(getReverseIn()));
            if(getEnableIn()<0)setOpState(ENABLE); //If we HAVE an enableinput 0-3, we'll let that handle opstate. Otherwise set it to ENABLE
            if(getReverseIn()<0)setSelectedGear(DRIVE); //If we HAVE a reverse input, we'll let that determine forward/reverse.  Otherwise set it to DRIVE
        }
    }
    else {
        setSelectedGear(NEUTRAL); //We will stay in NEUTRAL until we get at least 40 frames ahead indicating continous communications.
    }

    if(!online)  //This routine checks to see if we have received any frames from the inverter.  If so, ONLINE would be true and
    {   //we set the RUNNING light on.  If no frames are received for 2 seconds, we set running OFF.
        if ((millis()-ms)>2000)
        {
            running=false; // We haven't received any frames for over 2 seconds.  Otherwise online would be true.
            ms=millis();   //Reset our 2 second timer
        }
    }
    else running=true;
    online=false;//This flag will be set to 1 by received frames.


    sendCmd();  //This actually sets our GEAR and our actualstate cycle
}

//This inverter is controlled by only one ID. We send all commands in here
//byte 0, bit 0 = MCU Enable
//byte 0, bit 1 = Direction (0 = forward, 1 = reverse)
//byte 0, bits 2-3 = Operation command. 0 = standby, 1 = speed mode, 2 = torque mode, 3 = discharge. Only use torque mode
//byte 0, bits 4-7 = heartbeat. Inc constantly, allow wrap
//byte 1, bit 0 = Brake Valid. 0 = No brake, 1 = Brake. Don't know what this does.
//byte 1, bit 1-7 then bytes 2,3 are all reserved. Send 0's
//Byte 4-5 is torque request in 0.25NM with a -5000 offset
//Byte 6-7 is speed request in 1RPM with -12000 offset but speed mode is not likely supported so probably just send a value of 12000 here
void C300MotorController::sendCmd() {
    C300MotorControllerConfiguration *config = (C300MotorControllerConfiguration *)getConfiguration();
    CAN_FRAME output;
    OperationState newstate;
    alive = (alive + 1) & 0x0F;
    output.length = 8;
    output.id = 0x0CFF1401;
    output.extended = 1; //29 bit ID, extended frame
    output.rtr = 0;

    speedRequested = 12000;
    if (selectedGear == NEUTRAL) setOpState(DISABLED);
    output.data.bytes[0] = (operationState == ENABLE)?1:0 | (selectedGear == DRIVE)?0:2 | (2 << 2) | (alive << 4);
    output.data.bytes[1] = 0; //0 at the low bit means brake valid. What brake?
    output.data.bytes[2] = 0; //all reserved bits
    output.data.bytes[3] = 0; //also all reserved bits
    output.data.bytes[4] = (torqueCommand & 0xFF00) >> 8;
    output.data.bytes[5] = (torqueCommand & 0x00FF);
    output.data.bytes[6] = (speedRequested & 0xFF00) >> 8;
    output.data.bytes[7] = (speedRequested & 0x00FF);
 
    Logger::debug("C300 Command tx: %X %X %X %X %X %X %X %X", output.data.bytes[0], output.data.bytes[1], output.data.bytes[2], output.data.bytes[3],
                  output.data.bytes[4], output.data.bytes[5], output.data.bytes[6], output.data.bytes[7]);

    canHandlerEv.sendFrame(output);

    torqueCommand = 30000; //set offset  for zero torque commanded

    Logger::debug("Throttle requested: %i", throttleRequested);

    torqueRequested=0;
    if (actualState == ENABLE) { //don't even try sending torque commands until the DMOC reports it is ready
        if (selectedGear == DRIVE) {
            torqueRequested = (((long) throttleRequested * (long) config->torqueMax) / 1000L);
            //if (speedActual < config->regenTaperUpper && torqueRequested < 0) taperRegen();
        }
        if (selectedGear == REVERSE) {
            torqueRequested = (((long) throttleRequested * -1 *(long) config->torqueMax) / 1000L);//If reversed, regen becomes positive torque and positive pedal becomes regen.  Let's reverse this by reversing the sign.  In this way, we'll have gradually diminishing positive torque (in reverse, regen) followed by gradually increasing regen (positive torque in reverse.)
            //if (speedActual < config->regenTaperUpper && torqueRequested > 0) taperRegen();
        }
    }

    if (powerMode == modeTorque)
    {
        if(speedActual < config->speedMax) {
            torqueCommand+=torqueRequested;   //If actual rpm is less than max rpm, add torque to offset
        }
        else {
            torqueCommand += torqueRequested /1.3;   // else torque is reduced
        }
        output.data.bytes[0] = (torqueCommand & 0xFF00) >> 8;
        output.data.bytes[1] = (torqueCommand & 0x00FF);
        output.data.bytes[2] = output.data.bytes[0];
        output.data.bytes[3] = output.data.bytes[1];
    }
}

void C300MotorController::taperRegen()
{
    C300MotorControllerConfiguration *config = (C300MotorControllerConfiguration *)getConfiguration();
    if (speedActual < config->regenTaperLower) torqueRequested = 0;
    else {        
        int32_t range = config->regenTaperUpper - config->regenTaperLower; //next phase is to not hard code this
        int32_t taper = speedActual - config->regenTaperLower;
        int32_t calc = (torqueRequested * taper) / range;
        torqueRequested = (int16_t)calc;
    }
}

DeviceId C300MotorController::getId() {
    return (C300INV);
}

uint32_t C300MotorController::getTickInterval()
{
    return CFG_TICK_INTERVAL_MOTOR_CONTROLLER_C300;
}

void C300MotorController::loadConfiguration() {
    C300MotorControllerConfiguration *config = (C300MotorControllerConfiguration *)getConfiguration();

    if (!config) {
        config = new C300MotorControllerConfiguration();
        setConfiguration(config);
    }

    MotorController::loadConfiguration(); // call parent
}

void C300MotorController::saveConfiguration() {
    MotorController::saveConfiguration();
}

void C300MotorController::timestamp()
{
    milliseconds = (int) (millis()/1) %1000 ;
    seconds = (int) (millis() / 1000) % 60 ;
    minutes = (int) ((millis() / (1000*60)) % 60);
    hours   = (int) ((millis() / (1000*60*60)) % 24);
    // char buffer[9];
    //sprintf(buffer,"%02d:%02d:%02d.%03d", hours, minutes, seconds, milliseconds);
    // Serial<<buffer<<"\n";
}


