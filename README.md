GEVCU
=====

Generalized Electric Vehicle Control Unit

Our website can be found at : http://www.evtv.me

A project to create a fairly Arduino compatible ECU firmware
to interface with various electric vehicle hardware over CANbus
(and possibly other comm channels)

The project now builds in the Arduino IDE. So, use it to compile, send the firmware to the GEVCU hardware, and monitor serial. It all works very nicely.

You will need the following to have any hope of compiling and running the firmware:
- A GEVCU board. This version supports hardware 6.23c and later.
- Arduino IDE 1.5.4 - or later even 1.8.13 or such newer IDEs should work fine.
-All libraries reside in the lib folder

All libraries belong in %USERPROFILE%\Documents\Arduino\libraries (Windows) or ~/Arduino/libraries (Linux/Mac).
You will need to remove -master or any other postfixes. Your library folders should be named as above.

The canbus is supposed to be terminated on both ends of the bus. If you are testing with a DMOC and GEVCU then you've got two devices, each on opposing ends of the bus. GEVCU hardware is already terminated internally.


This software is MIT licensed:

Copyright (c) 2014 Collin Kidder, Michael Neuweiler, Charles Galpin, Jack Rickard

Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice shall be included
in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

