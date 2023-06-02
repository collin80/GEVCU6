/*
 * CanHandler.h

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

#ifndef CAN_HANDLER_H_
#define CAN_HANDLER_H_

#include <Arduino.h>
#include "evTimer.h"
#include "Logger.h"
#include "can_common.h"

#include "mcp2515_can.h"

#define SPI_CS_PIN 5

enum SDO_COMMAND
{
    SDO_WRITE = 0x20,
    SDO_READ = 0x40,
    SDO_WRITEACK = 0x60,
};

struct SDO_FRAME
{
    uint8_t nodeID;
    SDO_COMMAND cmd;
    uint16_t index;
    uint8_t subIndex;
    uint8_t dataLength;
    uint8_t data[4];
};

enum ISOTP_MODE
{
    SINGLE = 0,
    FIRST = 1,
    CONSEC = 2,
    FLOW = 3
};

class CanObserver
{
public:
    CanObserver();
    virtual void handleCanFrame(CAN_FRAME *frame);
    virtual void handlePDOFrame(CAN_FRAME *frame);
    virtual void handleSDORequest(SDO_FRAME *frame);
    virtual void handleSDOResponse(SDO_FRAME *frame);
    void setCANOpenMode(bool en);
    bool isCANOpen();
    void setNodeID(int id);
    int getNodeID();

private:
    bool canOpenMode; //TODO remove
    int nodeID;
};

class CanHandler
{
public:

    CanHandler( );
    void setup();
    uint32_t getBusSpeed();
    void attach(CanObserver *observer, uint32_t id, uint32_t mask, bool extended);
    void detach(CanObserver *observer, uint32_t id, uint32_t mask);
    void process();
    void prepareOutputFrame(CAN_FRAME *frame, uint32_t id);
    void CANIO(CAN_FRAME& frame);
    void sendFrame(CAN_FRAME& frame);
    void sendISOTP(int id, int length, uint8_t *data);

    //canopen support functions
    void sendNodeStart(int id = 0);
    void sendNodePreop(int id = 0);
    void sendNodeReset(int id = 0);
    void sendNodeStop(int id = 0);
    void sendPDOMessage(int, int, unsigned char *);
    void sendSDORequest(SDO_FRAME *frame);
    void sendSDOResponse(SDO_FRAME *frame);
    void sendHeartbeat();
    void setMasterID(int id);   

protected:

private:
    struct CanObserverData {
        uint32_t id;    // what id to listen to
        uint32_t mask;  // the CAN frame mask to listen to
        bool extended;  // are extended frames expected
        CanObserver *observer;  // the observer object (e.g. a device)
    };

    CanObserverData observerData[CFG_CAN_NUM_OBSERVERS];    // Can observers
    uint32_t busSpeed;

    void logFrame(CAN_FRAME& frame);
    int8_t findFreeObserverData();

    //canopen support functions
    void sendNMTMsg(int, int);
    int masterID; //what is our ID as the master node?      
};

extern CanHandler canHandler;

#endif /* CAN_HANDLER_H_ */