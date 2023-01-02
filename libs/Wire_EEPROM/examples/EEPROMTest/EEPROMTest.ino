#include <Arduino.h>
#include <due_wire.h>
#include <Wire_EEPROM.h>

class TestingClass {
   public:
	uint16_t TestValue1;
	uint16_t TestValue2;
	uint8_t TestValue3;
	int32_t TestValue4;
};

#define Serial SerialUSB //define to use native port

void setup() {

   delay(10000);

  Wire.begin();
  Serial.begin(115200);
  
  TestingClass test;
  
  EEPROM.setWPPin(19);
  
  Serial.println("Setting values");
  
  test.TestValue1 = 0x34BB;
  test.TestValue2 = 0xAAEE;
  test.TestValue3 = 0x8D;
  test.TestValue4 = -2462463;
  
  Serial.println("Writing to EEPROM");
  EEPROM.write(200, test);
  
  Serial.println("Waiting a bit");
  delay(1000); //wait a bit for the EEPROM to actually write
  
  Serial.println("Nulling values stored in class");
  //null out the values in our class
  test.TestValue1 = 0;
  test.TestValue2 = 0;
  test.TestValue3 = 0;
  test.TestValue4 = 0;
  
  Serial.println("Reading back from EEPROM");
  EEPROM.read(200, test);
  
  Serial.print("TestValue1: ");
  Serial.println(test.TestValue1, HEX);
  Serial.print("TestValue2: ");
  Serial.println(test.TestValue2, HEX);
  Serial.print("TestValue3: ");
  Serial.println(test.TestValue3, HEX);
  Serial.print("TestValue4: ");
  Serial.println(test.TestValue4);
}


//Off to lala land where nothing happens.... ever...
void loop() {  
}
