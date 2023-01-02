#include <Arduino.h>
#include "due_can.h"
#include "DueFlashStorage.h"

#define CANBASE		0x100
#define DEVICETOK  0xCAFEFACE

uint32_t flashWritePosition;
DueFlashStorage dueFlashStorage;
uint8_t pageBuffer[IFLASH1_PAGE_SIZE];

#ifdef __cplusplus
extern "C" {
#endif

__attribute__ ((long_call, section (".ramfunc")))
void setupForReboot()
{
	__disable_irq();

//Adapted from code found in Reset.cpp in Arduino core files for Due
//GPNVM bits are as follows:
//0 = lock bit (1 = flash memory is security locked)
//1 = Booting mode (0 = boot from ROM, 1 = boot from FLASH)
//2 = Flash ordering (0 = normal ordering FLASH0 then FLASH1, 1 = Reverse so FLASH1 is mapped first)
		
	const int EEFC_FCMD_CGPB = 0x0C;
	const int EEFC_FCMD_SGPB = 0x0B;
	const int EEFC_KEY = 0x5A;
	while ((EFC0->EEFC_FSR & EEFC_FSR_FRDY) == 0);
	// Set bootflag to run from FLASH instead of ROM
	EFC0->EEFC_FCR =
		EEFC_FCR_FCMD(EEFC_FCMD_SGPB) |
		EEFC_FCR_FARG(1) |
		EEFC_FCR_FKEY(EEFC_KEY);
	while ((EFC0->EEFC_FSR & EEFC_FSR_FRDY) == 0);	
	// Set bootflag to run from FLASH0
	EFC0->EEFC_FCR =
		EEFC_FCR_FCMD(EEFC_FCMD_CGPB) |
		EEFC_FCR_FARG(2) |
		EEFC_FCR_FKEY(EEFC_KEY);
	while ((EFC0->EEFC_FSR & EEFC_FSR_FRDY) == 0);	

	// Force a hard reset
	const int RSTC_KEY = 0xA5;
	RSTC->RSTC_CR =
		RSTC_CR_KEY(RSTC_KEY) |
		RSTC_CR_PROCRST |
		RSTC_CR_PERRST;

	while (true); //bye cruel world!
}


void setup()
{
	flashWritePosition = 0;
	Serial.begin(115200);
	Can0.begin(500000);
	Can0.setRXFilter(CANBASE, 0x700, false);
  Serial.println("Started CAN Bootloader.");
}

void loop()
{
	CAN_FRAME inFrame;
	CAN_FRAME outFrame;
	int location, bufferWritePtr;
	if (Can0.available() > 0) {
		Can0.read(inFrame);
		switch (inFrame.id)
		{
      case CANBASE: //just in case we're already in the bootloader but someone sends a "go to bootloader" message to get started
        if (inFrame.data.low == (uint32_t)0xDEADBEEF)
        {
          Serial.print("@");
          if (inFrame.data.high == DEVICETOK)
          {
            Serial.println("Starting firmware upload process");
            outFrame.id = CANBASE + 0x10;
            outFrame.extended = false;
            outFrame.length = 8;
            outFrame.data.low = (uint32_t)0xDEAFDEAD;
            outFrame.data.high = DEVICETOK;
            Can0.sendFrame(outFrame);
          }
        }
        break;
        
			case CANBASE + 0x16:
				Serial.print("-");
				location = inFrame.data.byte[0] + (256 * inFrame.data.byte[1]);
				bufferWritePtr = (location * 4) % IFLASH_PAGE_SIZE;
				pageBuffer[bufferWritePtr++] = inFrame.data.byte[2];
				pageBuffer[bufferWritePtr++] = inFrame.data.byte[3];
				pageBuffer[bufferWritePtr++] = inFrame.data.byte[4];
				pageBuffer[bufferWritePtr++] = inFrame.data.byte[5];
				if (bufferWritePtr == (IFLASH1_PAGE_SIZE))
				{					
					Serial.print("Writing flash at ");
					Serial.println(flashWritePosition);
					dueFlashStorage.write(flashWritePosition, pageBuffer, IFLASH1_PAGE_SIZE, 0);
					flashWritePosition += IFLASH1_PAGE_SIZE;
				}		
				outFrame.id = CANBASE + 0x20;
				outFrame.extended = false;
				outFrame.length = 2;
				outFrame.data.byte[0] = inFrame.data.byte[0];
				outFrame.data.byte[1] = inFrame.data.byte[1];
				Can0.sendFrame(outFrame);				
				break;
			case CANBASE + 0x30:
				Serial.print("#");
				if (inFrame.data.low == 0xC0DEFADE)
				{				
					//write out the any last little bits that were received but didn't add up to a full page
					Serial.println(bufferWritePtr);
					Serial.print("Writing flash at ");
					Serial.println(flashWritePosition);
					dueFlashStorage.write(flashWritePosition, pageBuffer, IFLASH1_PAGE_SIZE, 0);
					Serial.println("About to set boot mode and reboot");
					delay(50);
					//switch boot section								
					setupForReboot(); //this will reboot automatically to FLASH0
				}
				break;
		}
	}
}

#ifdef __cplusplus
}
#endif
		

	



		





		

