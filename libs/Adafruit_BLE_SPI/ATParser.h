/**************************************************************************/
/*!
    @file     ATParser.h
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

#ifndef _ADAFRUIT_ATPARSER_H_
#define _ADAFRUIT_ATPARSER_H_

#include <Arduino.h>
#include "utility/sdep.h"

// Class to facilitate sending AT Command and check response

#define BLUEFRUIT_MODE_COMMAND   HIGH
#define BLUEFRUIT_MODE_DATA      LOW
#define BLE_BUFSIZE              4*SDEP_MAX_PACKETSIZE


#if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL)
#define SerialDebug SERIAL_PORT_USBVIRTUAL
#else
#define SerialDebug Serial
#endif

// High byte is type, Low byte is datacount
// datacount is needed when passing ByteArray argument
enum
{
  AT_ARGTYPE_STRING    = 0x0100,
  AT_ARGTYPE_BYTEARRAY = 0x0200,
  AT_ARGTYPE_INT32     = 0x0300,
  AT_ARGTYPE_UINT32    = 0x0400,
  AT_ARGTYPE_INT16     = 0x0500,
  AT_ARGTYPE_UINT16    = 0x0600,
  AT_ARGTYPE_INT8      = 0x0700,
  AT_ARGTYPE_UINT8     = 0x0800,
};

void gotBLEReply();

class BLEListener
{
public:
    virtual void gotLine(char *txtLine);
    virtual void cmdComplete(bool OK);
};

class Adafruit_ATParser : public Stream
{
protected:
  uint8_t _mode;
  bool     _verbose;
  bool waitingForReply;
  bool lastCommandOK;
  BLEListener *listener;
  int8_t          m_irq_pin;

  // internal function
  bool send_arg_get_resp(int32_t* reply, uint8_t argcount, uint16_t argtype[], uint32_t args[]);

public:
  Adafruit_ATParser(void);

  char buffer[BLE_BUFSIZE+1];
  char replyBuffer[BLE_BUFSIZE+1];
  int replyBuffIdx;
  bool interruptFlag;
  
  bool isWaitingForReply();
  bool isCmdOK();
  static Adafruit_ATParser *getRef();
  void bleReply();
  boolean attachObj(BLEListener *listener);
  void detachObj();
  void pollInterruptFlag();

  uint8_t      getMode(void) { return _mode; }
  virtual bool setMode(uint8_t mode) = 0;

  // Auto print out TX & RX data to normal Serial
  void verbose(bool enable) { _verbose = enable; }

  bool atcommand_full(const char cmd[]               , int32_t* reply, uint8_t argcount, uint16_t argtype[], uint32_t args[]);
  bool atcommand_full(const __FlashStringHelper *cmd , int32_t* reply, uint8_t argcount, uint16_t argtype[], uint32_t args[]);

  //--------------------------------------------------------------------+
  // Without Reply
  //--------------------------------------------------------------------+
  bool atcommand(const char cmd[]               ) { return this->atcommand_full(cmd, NULL, 0, NULL, NULL); }
  bool atcommand(const __FlashStringHelper *cmd ) { return this->atcommand_full(cmd, NULL, 0, NULL, NULL); }

  //------------- One integer argument -------------//
  bool atcommand(const char cmd[]              , int32_t para1)
  {
    uint16_t type[] = { AT_ARGTYPE_INT32 };
    uint32_t args[] = { (uint32_t) para1 };
    return this->atcommand_full(cmd, NULL, 1, type, args);
  }

  bool atcommand(const __FlashStringHelper *cmd, int32_t para1)
  {
    uint16_t type[] = { AT_ARGTYPE_INT32 };
    uint32_t args[] = { (uint32_t) para1 };
    return this->atcommand_full(cmd, NULL, 1, type, args);
  }

  //------------- Two integer arguments -------------//
  bool atcommand(const char cmd[]              , int32_t para1, int32_t para2)
  {
    uint16_t type[] = { AT_ARGTYPE_INT32, AT_ARGTYPE_INT32 };
    uint32_t args[] = { (uint32_t) para1, (uint32_t) para2 };
    return this->atcommand_full(cmd, NULL, 2, type, args);
  }

  bool atcommand(const __FlashStringHelper *cmd, int32_t para1, int32_t para2)
  {
    uint16_t type[] = { AT_ARGTYPE_INT32, AT_ARGTYPE_INT32 };
    uint32_t args[] = { (uint32_t) para1, (uint32_t) para2 };
    return this->atcommand_full(cmd, NULL, 2, type, args);
  }

  //------------- One ByteArray arguments -------------//
  bool atcommand(const char cmd[]              , const uint8_t bytearray[], uint16_t count)
  {
    uint16_t type[] = { AT_ARGTYPE_BYTEARRAY+count };
    uint32_t args[] = { (uint32_t) bytearray };
    return this->atcommand_full(cmd, NULL, 1, type, args);
  }

  bool atcommand(const __FlashStringHelper *cmd, const uint8_t bytearray[], uint16_t count)
  {
    uint16_t type[] = { AT_ARGTYPE_BYTEARRAY+count };
    uint32_t args[] = {(uint32_t) bytearray };
    return this->atcommand_full(cmd, NULL, 1, type, args);
  }

  //------------- One String argument -------------//
  bool atcommand(const char cmd[]              , const char* str)
  {
    uint16_t type[] = { AT_ARGTYPE_STRING };
    uint32_t args[] = { (uint32_t) str };
    return this->atcommand_full(cmd, NULL, 1, type, args);
  }

  bool atcommand(const __FlashStringHelper *cmd, const char* str)
  {
    uint16_t type[] = { AT_ARGTYPE_STRING };
    uint32_t args[] = { (uint32_t) str };
    return this->atcommand_full(cmd, NULL, 1, type, args);
  }

  //--------------------------------------------------------------------+
  // With Reply
  //--------------------------------------------------------------------+
  bool atcommandIntReply(const char cmd[], int32_t* reply)               { return this->atcommand_full(cmd, reply, 0, NULL, NULL); }
  bool atcommandIntReply(const __FlashStringHelper *cmd, int32_t* reply) { return this->atcommand_full(cmd, reply, 0, NULL, NULL); }

  //------------- One integer argument -------------//
  bool atcommandIntReply(const char cmd[]              , int32_t* reply, int32_t para1)
  {
    uint16_t type[] = { AT_ARGTYPE_INT32 };
    uint32_t args[] = { (uint32_t) para1 };
    return this->atcommand_full(cmd, reply, 1, type, args);
  }

  bool atcommandIntReply(const __FlashStringHelper *cmd, int32_t* reply, int32_t para1)
  {
    uint16_t type[] = { AT_ARGTYPE_INT32 };
    uint32_t args[] = { (uint32_t) para1 };
    return this->atcommand_full(cmd, reply, 1, type, args);
  }

  //------------- Two integer arguments -------------//
  bool atcommandIntReply(const char cmd[]              , int32_t* reply, int32_t para1, int32_t para2)
  {
    uint16_t type[] = { AT_ARGTYPE_INT32, AT_ARGTYPE_INT32 };
    uint32_t args[] = { (uint32_t) para1, (uint32_t) para2 };
    return this->atcommand_full(cmd, reply, 2, type, args);
  }

  bool atcommandIntReply(const __FlashStringHelper *cmd, int32_t* reply, int32_t para1, int32_t para2)
  {
    uint16_t type[] = { AT_ARGTYPE_INT32, AT_ARGTYPE_INT32 };
    uint32_t args[] = { (uint32_t) para1, (uint32_t) para2 };
    return this->atcommand_full(cmd, reply, 2, type, args);
  }

  //------------- One ByteArray arguments -------------//
  bool atcommandIntReply(const char cmd[]              , int32_t* reply, const uint8_t bytearray[], uint16_t count)
  {
    uint16_t type[] = { AT_ARGTYPE_BYTEARRAY+count };
    uint32_t args[] = { (uint32_t) bytearray };
    return this->atcommand_full(cmd, reply, 1, type, args);
  }

  bool atcommandIntReply(const __FlashStringHelper *cmd, int32_t* reply, const uint8_t bytearray[], uint16_t count)
  {
    uint16_t type[] = { AT_ARGTYPE_BYTEARRAY+count };
    uint32_t args[] = { (uint32_t) bytearray };
    return this->atcommand_full(cmd, reply, 1, type, args);
  }

  //------------- One String argument -------------//
  bool atcommandIntReply(const char cmd[]              , int32_t* reply, const char* str)
  {
    uint16_t type[] = { AT_ARGTYPE_STRING };
    uint32_t args[] = { (uint32_t) str };
    return this->atcommand_full(cmd, reply, 1, type, args);
  }

  bool atcommandIntReply(const __FlashStringHelper *cmd, int32_t* reply, const char* str)
  {
    uint16_t type[] = { AT_ARGTYPE_STRING };
    uint32_t args[] = { (uint32_t) str };
    return this->atcommand_full(cmd, reply, 1, type, args);
  }

  //--------------------------------------------------------------------+
  // HELPER
  //--------------------------------------------------------------------+
  int printByteArray(uint8_t const bytearray[], int size);
};

#endif /* _ADAFRUIT_ATPARSER_H_ */
