/*
The in-sketch library portion of Firmware Receiver is now very simple. It scans for a sequence that should be globally unique in order to stop the current
sketch and jump to the bootloader sketch that resides up high in FLASH1. Then that sketch takes over and handles the firmware update. After it is done
it resets back to FLASH0 so our new sketch can run.
*/

#include <due_can.h>

#define DEVICETOK	0xCAFEFACE
#define CANBASE	0x100

void fwGotFrame(CAN_FRAME *frame);

