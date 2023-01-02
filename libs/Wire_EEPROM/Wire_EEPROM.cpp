#include <Arduino.h>
#include <due_wire.h>
#include "Wire_EEPROM.h"

uint8_t EEPROMCLASS::readByte(uint32_t address) 
{
  uint8_t d,e;
  uint8_t buffer[3];
  uint8_t i2c_id;
  //Wire.begin();
   buffer[0] = ((address & 0xFF00) >> 8);
   buffer[1] = ((uint8_t)(address & 0x00FF));
   i2c_id = 0b01010000 + ((address >> 16) & 0x03); //10100 is the chip ID then the two upper bits of the address
    Wire.beginTransmission(i2c_id);  
    Wire.write(buffer, 2);
    Wire.endTransmission(false); //do NOT generate stop 
    Wire.requestFrom(i2c_id, 1); //this will generate stop though.
     if(Wire.available())    
     { 
        d = Wire.read(); // receive a byte as character
        return d;
     }
     return 255;
}    

void EEPROMCLASS::writeByte(uint32_t address, uint8_t valu) 
{
  uint16_t d;
  uint8_t buffer[3];
  uint8_t i2c_id;

  while (writeTime > millis());

  //Wire.begin();
  buffer[0] = ((address & 0xFF00) >> 8);
  buffer[1] = ((uint8_t)(address & 0x00FF));
  buffer[2] = valu;
  i2c_id = 0b01010000 + ((address >> 16) & 0x03); //10100 is the chip ID then the two upper bits of the address
  Wire.beginTransmission(i2c_id);
  Wire.write(buffer, 3);
  Wire.endTransmission(true);
  writeTime = millis() + 8;
}

void EEPROMCLASS::setWPPin(uint8_t pin) {
	pinMode(pin, OUTPUT);
	digitalWrite(pin, LOW);
}

EEPROMCLASS::EEPROMCLASS()
{
  Wire.begin();
  //digital pin 17 is connected to the write protect function of the EEPROM. It is active high so set it low to enable writes
  pinMode(17, OUTPUT);
  digitalWrite(17, LOW);
}

//Instantiate the class with the proper name to pretend this is still the class from the non-Due arduinos
EEPROMCLASS EEPROM;

