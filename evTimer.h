#ifndef evTimer_h
#define evTimer_h

#include "Arduino.h"

#include <inttypes.h>

class evTimer
{
protected:

	// Represents the timer id (index for the array of Timer structs)
	int timer;

	// Stores the object timer frequency
	// (allows to access current timer period and frequency):
	static double _frequency[9];

	// Picks the best clock to lower the error
	static uint8_t bestClock(double frequency, uint32_t& retRC);

public:
	struct Timer
	{
		Tc *tc;
		uint32_t channel;
		IRQn_Type irq;
	};

	static evTimer getAvailable();

	// Store timer configuration (static, as it's fix for every object)
	static const Timer Timers[9];

	// Needs to be public, because the handlers are outside class:
	static void (*callbacks[9])();

	evTimer(int _timer);
	evTimer attachInterrupt(void (*isr)());
	evTimer detachInterrupt();
	evTimer start(long microseconds = -1);
	evTimer stop();
	evTimer setFrequency(double frequency);
	evTimer setPeriod(long microseconds);


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

