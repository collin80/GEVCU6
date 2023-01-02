#ifndef EEPROM_H_
#define EEPROM_H_

#include "Arduino.h"

class EEPROMCLASS {
public:
	uint8_t readByte(uint32_t address);
	void writeByte(uint32_t address, uint8_t valu);

	void setWPPin(uint8_t pin);

	template <class T> int write(int ee, const T& value)
	{	
		uint8_t buffer[258];
		uint8_t i2c_id;
		const byte* p = (const byte*)(const void*)&value;
		unsigned int i;

		while (writeTime > millis());

		if (ee > 1023) return 0; //NAUGHTY!

		for (i = 0; i < 258; i++) buffer[i] = 0xFF;

		buffer[0] = ee; // each page is 256 bytes but this is the MSB and will be the same as the page #
		buffer[1] = 0; //always start at the start of a page
		i2c_id = 0b01010000 + ((ee >> 8) & 0x03); //10100 is the chip ID then the two upper bits of the address
		for (i = 0; i < sizeof(value); i++) 
		{
			buffer[i + 2] = p[i];
		}
		//Blast it all out at once and immediately return.
		Wire.beginTransmission(i2c_id);
		Wire.write(buffer, 258);
		Wire.endTransmission(true);  

		writeTime = millis() + 30; //wait for transfer over i2c plus eeprom write

		return i;
	}

	template <class T> int read(int ee, T& value)
	{
		uint8_t buffer[3];
		uint8_t i2c_id;
	    byte* p = (byte*)(void*)&value;
		unsigned int i;

		if (ee > 1023) return 0; //NAUGHTY!
			
		buffer[0] = ee; // each page is 256 bytes but this is the MSB and will be the same as the page #
		buffer[1] = 0; //always start at the start of a page
		i2c_id = 0b01010000 + ((ee >> 8) & 0x03); //10100 is the chip ID then the two upper bits of the address
		//send the address to get the chip ready.
		Wire.beginTransmission(i2c_id);  
		Wire.write(buffer, 2);
		Wire.endTransmission(false); //do NOT generate stop
		//Now, tell it we'd like to read the whole input size in one pass.
		Wire.requestFrom(i2c_id, sizeof(value)); //this will generate stop though.

	    for (i = 0; i < sizeof(value); i++) 
		{
			if (Wire.available()) p[i] = Wire.read();
		}

		return i;
	}

	EEPROMCLASS();

private:
	uint32_t writeTime;
};

extern EEPROMCLASS EEPROM;

#endif
