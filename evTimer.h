#ifndef evTimer_h
#define evTimer_h

#include "Arduino.h"

#include <inttypes.h>
#include "SimpleTimer.h"

class evTimer
{
protected:

	// Represents the timer id (index for the array of Timer structs)
	int index;

	// Stores the object timer frequency
	// (allows to access current timer period and frequency):
	static double _frequency[9];


public:

	static evTimer getAvailable();

	// Needs to be public, because the handlers are outside class:
	static void (*callbacks[9])();

	evTimer(int _timer);
	evTimer setInterval(long millis, void (*isr)());
    void loop();


	double getFrequency();
	long getPeriod();
};

// Just to call Timer.getAvailable instead of Timer::getAvailable() :
extern evTimer Timer;

extern evTimer Timer0;
extern evTimer Timer1;
extern evTimer Timer2;
extern evTimer Timer3;
extern evTimer Timer4;
extern evTimer Timer5;
extern evTimer Timer6;
extern evTimer Timer7;
extern evTimer Timer8;

#endif