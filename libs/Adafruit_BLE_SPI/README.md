This library is for all nRF51 based Adafruit Bluefruit LE modules that use SPI.

If you were looking to do general work with the Adafruit BLE modules you do not
want this repo! Very heavy modification has been done and code meant for the official
library will not work with this version. This version has been modified to use interrupts and
callbacks for all calls. There are no more delays. This complicates your code but allows for
use of the BLE module in time critical code that cannot sit around and wait.

# AT Commands

The Bluefruit LE modules this library talks to use AT-style commands and responses.

If your are using an SPI board, the AT commands are wrapped in a thin **[SDEP](SDEP.md)** (Simple Data Exchange Protocol) wrapper to transmit and received text data over the binary SPI transport.  Details of this SPI transport layer are detailed in [SDEP.md](SDEP.md) in this same folder.

# Hardware Setup

There are two variants of the nRF51 Bluefruit LE modules.  One uses SPI to communicate, the other uses UART with flow control (TXD, RXD, CTS, RTS).  The wiring you use will depend on the module you are trying to connect.

## SPI Pinout

If you are using an SPI Bluefruit LE board, your Arduino should be connected to the Bluefruit LE SPI module using the following pinout:

Bluefruit LE SPI | Arduino Uno
-----------------|------------
SCLK             | 13
MISO             | 12
MOSI             | 11
CS               | 8
IRQ              | 7

Optional Pins (enable these in the sample sketches)

Bluefruit LE SPI | Arduino Uno
-----------------|------------
RESET            | 6
