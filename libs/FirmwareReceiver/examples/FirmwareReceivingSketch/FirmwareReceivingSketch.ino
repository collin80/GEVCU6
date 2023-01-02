#include "due_can.h"
#include "FirmwareReceiver.h"

void setup() {
  Serial.begin(115200);
  Serial.println("Starting up in FLASH0 sketch");
  Can0.begin(500000);
  Can0.watchFor();
  pinMode(13, OUTPUT);
}

void loop() {
  CAN_FRAME frame;
  if (Can0.available())
  {
     Can0.read(frame);
     fwGotFrame(&frame);
  }
  digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(500);              // wait for a second
  digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
  delay(500);   
  Serial.print("G");
}
