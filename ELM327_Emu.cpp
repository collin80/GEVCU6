/*
 *  ELM327_Emu.cpp
 *
 * Class emulates the serial comm of an ELM327 chip - Used to create an OBDII interface
 *
 * Created: 3/18/2014
 *  Author: Collin Kidder
 */

/*
 Copyright (c) 2013-2014 Collin Kidder, Michael Neuweiler, Charles Galpin

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

#include "ELM327_Emu.h"
#include "adafruitBLE.h"

extern Adafruit_BluefruitLE_SPI ble;

/*
 * Constructor. Assign serial interface to use for comm with bluetooth adapter we're emulating with
 */
ELM327Emu::ELM327Emu() {
    prefsHandler = new PrefHandler(ELM327EMU);

    commonName = "ELM327 Emulator over BLE";
}

/*
 * Initialization of hardware and parameters
 */
void ELM327Emu::setup() {

    Logger::info("add device: ELM327 emulator (id: %X, %X", ELM327EMU, this);

    tickHandler.detach(this);

    tickCounter = 0;
    ibWritePtr = 0;

    elmProc = new ELM327Processor();

    //These apply to both UART and SPI versions
    Logger::debug(ELM327EMU, "Initializing ADAFruit BLE bluetooth device...");    
    ble.begin(true);  //True = verbose mode for debuggin.   

    //this isn't a wifi link but the timer interval can be the same
    //because it serves a similar function and has similar timing requirements
    tickHandler.attach(this, CFG_TICK_INTERVAL_WIFI);
}

/*
 * Send a command to ichip. The "AT+i" part will be added.
 */
void ELM327Emu::sendCmd(String cmd) {
    char buff[50];
    int slen;
    buff[0] = 'A';
    buff[1] = 'T';
    strcat(buff, cmd.c_str());
    slen = strlen(buff);
    buff[slen++] = 13;
    buff[slen++] = 0;
    ble.writeBLEUart((unsigned char *)buff, slen);
    loop(); // parse the response
}

/*

 */
void ELM327Emu::handleTick() {

}

/*
 * Handle a message sent by the DeviceManager.
 */
void ELM327Emu::handleMessage(uint32_t messageType, void* message) {
    Device::handleMessage(messageType, message);

    switch (messageType) {
    case MSG_SET_PARAM: {
        break;
    }
    case MSG_CONFIG_CHANGE:
        break;
    case MSG_COMMAND:
        sendCmd((char *)message);
        break;
    }
}

/*
 * Called in the main loop (hopefully) in order to process serial input waiting for us
 * from the wifi module. It should always terminate its answers with 13 so buffer
 * until we get 13 (CR) and then process it.
 * But, for now just echo stuff to our serial port for debugging
 */

void ELM327Emu::loop() {
    int incoming;
    while (ble.available()) {
        incoming = ble.read();
        if (incoming != -1) { //and there is no reason it should be -1
            if (incoming == 13 || ibWritePtr > 126) { // on CR or full buffer, process the line
                incomingBuffer[ibWritePtr] = 0; //null terminate the string
                ibWritePtr = 0; //reset the write pointer

                if (Logger::isDebug())
                    Logger::debug(ELM327EMU, incomingBuffer);
                processCmd();

            } else { // add more characters
                if (incoming != 10 && incoming != ' ') // don't add a LF character or spaces. Strip them right out
                    incomingBuffer[ibWritePtr++] = (char)tolower(incoming); //force lowercase to make processing easier
            }
        } else
            return;
    }
}

/*
*	There is no need to pass the string in here because it is local to the class so this function can grab it by default
*	But, for reference, this cmd processes the command in incomingBuffer
*/
void ELM327Emu::processCmd() {
    String retString = elmProc->processELMCmd(incomingBuffer);

    char *ret = (char *)retString.c_str();
    ble.writeBLEUart((unsigned char *)ret, strlen(ret));
    if (Logger::isDebug()) {
        Logger::debug(ELM327EMU, ret);
    }
}

DeviceType ELM327Emu::getType() {
    return DEVICE_MISC;
}

DeviceId ELM327Emu::getId() {
    return (ELM327EMU);
}

void ELM327Emu::loadConfiguration() {
    ELM327Configuration *config = (ELM327Configuration *)getConfiguration();

    if (prefsHandler->checksumValid()) { //checksum is good, read in the values stored in EEPROM
        Logger::debug(ELM327EMU, "Valid checksum so using stored elm327 emulator config values");
        //TODO: implement processing of config params for WIFI
//		prefsHandler->read(EESYS_WIFI0_SSID, &config->ssid);
    }
}

void ELM327Emu::saveConfiguration() {
    ELM327Configuration *config = (ELM327Configuration *) getConfiguration();

    //TODO: implement processing of config params for WIFI
//	prefsHandler->write(EESYS_WIFI0_SSID, config->ssid);
//	prefsHandler->saveChecksum();
}



