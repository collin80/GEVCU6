#include "FirmwareReceiver.h"

//#define SER_DEBUG

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
	// Set bootflag to run from FLASH1
	EFC0->EEFC_FCR =
		EEFC_FCR_FCMD(EEFC_FCMD_SGPB) |
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

#ifdef __cplusplus
}
#endif

void fwGotFrame(CAN_FRAME *frame)
{	
	int location, bufferWritePtr;
	CAN_FRAME outFrame;

#ifdef SER_DEBUG
	Serial.print("*");
#endif

	if (frame->id == CANBASE)
	{
#ifdef SER_DEBUG
		Serial.print("!");
#endif
		if (frame->data.low == (uint32_t)0xDEADBEEF)
		{
#ifdef SER_DEBUG

			Serial.print("@");
#endif
			if (frame->data.high == DEVICETOK)
			{
#ifdef SER_DEBUG
				Serial.println("Starting firmware upload process");
#endif
				outFrame.id = CANBASE + 0x10;
				outFrame.extended = false;
				outFrame.length = 8;
				outFrame.data.low = (uint32_t)0xDEAFDEAD;
				outFrame.data.high = DEVICETOK;
				Can0.sendFrame(outFrame);
				delay(100);
				setupForReboot();
			}
		}

	}			
}

