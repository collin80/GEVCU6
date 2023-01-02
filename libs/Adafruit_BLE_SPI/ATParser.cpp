/**************************************************************************/
/*!
    @file     Adafruit_ATParser.cpp
    @author   hathach

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2016, Adafruit Industries (adafruit.com)
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
    1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
    3. Neither the name of the copyright holders nor the
    names of its contributors may be used to endorse or promote products
    derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/**************************************************************************/

#include "ATParser.h"

Adafruit_ATParser *ref;

static inline char digit2ascii(uint8_t digit)
{
  return ( digit + ((digit) < 10 ? '0' : ('A'-10)) );
}

/******************************************************************************/
/*!
    @brief Constructor
*/
/******************************************************************************/
Adafruit_ATParser::Adafruit_ATParser(void)
{
  _mode    = BLUEFRUIT_MODE_COMMAND;
  _verbose = false;
  waitingForReply = false;
  lastCommandOK = true; 
  replyBuffIdx = 0;
  ref = this;
  listener = NULL;
  interruptFlag = false;
}

void gotBLEReply()
{
    Adafruit_ATParser::getRef()->interruptFlag = true;
}

boolean Adafruit_ATParser::attachObj(BLEListener *listener)
{
    this->listener = listener;
    attachInterrupt(digitalPinToInterrupt(m_irq_pin), gotBLEReply, RISING);
    flush();
}

void Adafruit_ATParser::detachObj()
{
    detachInterrupt(digitalPinToInterrupt(m_irq_pin));
    this->listener = 0;
    flush();
}

void Adafruit_ATParser::pollInterruptFlag()
{
    if (interruptFlag) {
        interruptFlag = false;
        bleReply();
    }
}

void Adafruit_ATParser::bleReply()
{
    while(available()) {
        //SerialUSB.write('-');
        char c = read();        
        //SerialUSB.println(c, HEX);
        if (c == 0xFF) {
            replyBuffer[replyBuffIdx] = 0; //just to be sure the string is terminated in case someone tries to use it
            return;
        }

        if (c == '\r') {
            replyBuffer[replyBuffIdx] = 0; //null terminate string
            //see if this is an OK or ERROR line
            if ( strcmp(replyBuffer, "OK") == 0 ) {
                //SerialUSB.println("Got OK");
                waitingForReply = false;
                lastCommandOK = true;
                if (listener) listener->cmdComplete(lastCommandOK);
            }
            else if ( strcmp(replyBuffer, "ERROR") == 0 ) {
                //SerialUSB.println("Got ERR");
                waitingForReply = false;
                lastCommandOK = false;
                if (listener) listener->cmdComplete(lastCommandOK);
            }
            else {
                //send callback
                //SerialUSB.println("Got Line");
                if (listener) listener->gotLine(replyBuffer);
            }
            replyBuffIdx = 0;
            continue;
        }

        if (c == '\n') {
            // the first '\n' is ignored
            /*if (replyBuffIdx == 0)*/ continue;
        }
      
        replyBuffer[replyBuffIdx] = c;
        replyBuffIdx++;

        // Buffer is full
        if (replyBuffIdx >= BLE_BUFSIZE) {
            //if (_verbose) { SerialDebug.println("*overflow*"); }  // for my debuggin' only!
            replyBuffIdx = 0; //zero it out to reset things.
            break;
        }
    }
}

bool Adafruit_ATParser::isWaitingForReply()
{
    return waitingForReply;
}

bool Adafruit_ATParser::isCmdOK()
{
    return lastCommandOK;
}

Adafruit_ATParser *Adafruit_ATParser::getRef()
{
    return ref;
}

/******************************************************************************/
/*!
    @brief
    @param
*/
/******************************************************************************/
bool Adafruit_ATParser::send_arg_get_resp(int32_t* reply, uint8_t argcount, uint16_t argtype[], uint32_t args[])
{
  // Command arguments according to its type
  for(uint8_t i=0; i<argcount; i++)
  {
    // print '=' for WRITE mode
    if (i==0) print('=');

    switch (argtype[i] & 0xFF00)
    {
      case AT_ARGTYPE_STRING:
        this->print( (char const*) args[i] );
      break;

      case AT_ARGTYPE_BYTEARRAY:
      {
        uint8_t count        = lowByte(argtype[i]);
        this->printByteArray( (uint8_t const*) args[i], count );
      }
      break;

      case AT_ARGTYPE_UINT32:
        print( (uint32_t) args[i] );
      break;

      case AT_ARGTYPE_INT32:
        print( (int32_t) args[i] );
      break;

      case AT_ARGTYPE_UINT16:
        print( (uint16_t) args[i] );
      break;

      case AT_ARGTYPE_INT16:
        print( (int16_t) args[i] );
      break;

      case AT_ARGTYPE_UINT8:
        print( (uint8_t) ((uint32_t)args[i]) );
      break;

      case AT_ARGTYPE_INT8:
        print( (int8_t) ((int32_t) args[i]) );
      break;

      default: break;
    }

    if (i != argcount-1) print(',');
  }
  println(); // execute command

  // parse integer response if required
  if (reply)
  {
    if (_verbose) SerialDebug.print( F("\n<- ") );
    //(*reply) = readline_parseInt();
  }

  // check OK or ERROR status
  waitingForReply = true;
  return true;
}

/******************************************************************************/
/*!
    @brief
    @param
*/
/******************************************************************************/
bool Adafruit_ATParser::atcommand_full(const char cmd[], int32_t* reply, uint8_t argcount, uint16_t argtype[], uint32_t args[])
{
  bool result;
  uint8_t current_mode = _mode;

  // switch mode if necessary to execute command
  if ( current_mode == BLUEFRUIT_MODE_DATA ) setMode(BLUEFRUIT_MODE_COMMAND);

  // Execute command with parameter and get response
  print(cmd);
  result = this->send_arg_get_resp(reply, argcount, argtype, args);

  // switch back if necessary
  if ( current_mode == BLUEFRUIT_MODE_DATA ) setMode(BLUEFRUIT_MODE_DATA);

  return result;
}

/******************************************************************************/
/*!
    @brief
    @param
*/
/******************************************************************************/
bool Adafruit_ATParser::atcommand_full(const __FlashStringHelper *cmd, int32_t* reply, uint8_t argcount, uint16_t argtype[], uint32_t args[])
{
  bool result;
  uint8_t current_mode = _mode;

  // switch mode if necessary to execute command
  if ( current_mode == BLUEFRUIT_MODE_DATA ) setMode(BLUEFRUIT_MODE_COMMAND);

  // Execute command with parameter and get response
  print(cmd);
  result = this->send_arg_get_resp(reply, argcount, argtype, args);

  // switch back if necessary
  if ( current_mode == BLUEFRUIT_MODE_DATA ) setMode(BLUEFRUIT_MODE_DATA);

  return result;
}

/******************************************************************************/
/*!
    @brief Print a buffer to BYTE ARRAY format e.g 11-22-33-44-55
    @param bytearray buffer to print
    @param size number of byte
    @return number of printed characters
*/
/******************************************************************************/
int Adafruit_ATParser::printByteArray(uint8_t const bytearray[], int size)
{
  while(size--)
  {
    uint8_t byte = *bytearray++;
    write( digit2ascii((byte & 0xF0) >> 4) );
    write( digit2ascii(byte & 0x0F) );
    if ( size!=0 ) write('-');
  }

  return (size*3) - 1;
}

//Empty versions so compiler doesn't complain and child classes don't need to implement them if they don't need them.
void BLEListener::gotLine(char *txtLine)
{
    
}

void BLEListener::cmdComplete(bool OK)
{
    
}