# FirmwareReceiver
Interacts with due_can to allow for a remote unit to upgrade the current sketch. 

### Dependencies
- due_can (I'm the origin of this library)
- DueFlashStorage (custom version from my repos)

### Features
- CANBus bootloader that runs outside the user sketch. Very little modification has to be done to sketches to
  allow them to work with this bootloader

Note: The flash storage is reset every time you upload a new sketch to your Arduino. So, do not upload
sketches via the IDE except the very first time! Thereafter use the special tools. But, otherwise
sketches run without modification.

### Using FirmwareReceiver

Using the bootloader:
Bootloader.bin is included in this project. It is pre-built and ready to go. It can be directly flashed to a Due with bossac:

bossac -e -w -v -b Bootloader.bin

If you're using windows and a recent version of Arduino IDE 1.6.x you'll find Bossac in a directory like this:

C:\users\<YourUserName>\appdata\local\Arduino15\packages\arduino\tools\bossac\1.6.1-arduino

Load the example sketch FirmwareReceiverSketch to see how to modify your sketch. There are really only two things 
to do:

- call fwGotFrame with a pointer to each CAN frame your sketch receives.
- Serial.begin(115200); - This line should be in your setup() function

### Flashing over CAN
So, the next most obvious question is, how do I use this to flash firmware?!? Well, SavvyCAN can do this. By default the binaries
for SavvyCAN have not included this functionality but it exists in all branches, it just is not exposed. In the near future this will
be properly exposed for use. In the meantime, you can download the source and add it to the menu. Once it does exist then
connect SavvyCAN to a CAN source and under the sending menu select Firmware update. This screen then allows one to load
a .bin file (output from Arduino compile), a device token, and the bus to use. The device token by default is 0xCAFEFACE. Yes,
really. Now you can start the firmware upgrade process.

This is all obviously a work in progress. However, it has been tested to be working properly so if you got this far congrats.

