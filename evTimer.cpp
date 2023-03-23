#include "evTimer.h"

SimpleTimer timer;

void (*evTimer::callbacks[9])() = {};
double evTimer::_frequency[9] = {-1, -1, -1, -1, -1, -1, -1, -1, -1};

/*
	Initializing all timers, so you can use them like this: Timer0.start();
*/
evTimer Timer(0);

evTimer Timer0(0);
evTimer Timer1(1);
evTimer Timer2(2);
evTimer Timer3(3);
evTimer Timer4(4);
evTimer Timer5(5);
evTimer Timer6(6);
evTimer Timer7(7);
evTimer Timer8(8);

evTimer::evTimer(int _index)
{
	/*
		The constructor of the class evTimer
	*/
	index = _index;
}

evTimer evTimer::getAvailable()
{
	/*
		Return the first timer with no callback set
	*/

	for (int i = 0; i < 9; i++)
	{
		if (!callbacks[i])
			return evTimer(i);
	}
	// Default, return Timer0;
	return evTimer(0);
}

void evTimer::loop()
{
	timer.run();
}

evTimer evTimer::setInterval(long millis, void (*isr)())
{
	/*
		Set the timer frequency (in milliseconds)
	*/

	// Prevent negative frequencies
	if (millis <= 0)
	{
		millis = 1;
	}

	timer.setInterval(millis, isr);

	return *this;
}

double evTimer::getFrequency()
{
	/*
		Get current time frequency
	*/

	return _frequency[index];
}

long evTimer::getPeriod()
{
	/*
		Get current time period
	*/

	return 1.0 / getFrequency() * 1000000;
}
