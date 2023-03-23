#include "evTimer.h"

SimpleTimer timer;

const evTimer::Timer evTimer::Timers[9] = {
	{TC0,0,TC0_IRQn},
	{TC0,1,TC1_IRQn},
	{TC0,2,TC2_IRQn},
	{TC1,0,TC3_IRQn},
	{TC1,1,TC4_IRQn},
	{TC1,2,TC5_IRQn},
	{TC2,0,TC6_IRQn},
	{TC2,1,TC7_IRQn},
	{TC2,2,TC8_IRQn},
};

void (*evTimer::callbacks[9])() = {};
double evTimer::_frequency[9] = {-1,-1,-1,-1,-1,-1,-1,-1,-1};

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

evTimer::evTimer(int index){
	/*
		The constructor of the class evTimer 
	*/
	index = _index;
}

evTimer evTimer::getAvailable(){
	/*
		Return the first timer with no callback set
	*/

	for(int i = 0; i < 9; i++){
		if(!callbacks[i])
			return evTimer(i);
	}
	// Default, return Timer0;
	return evTimer(0);
}
return *this;
}

void evTimer::loop(){
    timer.loop();
}


evTimer evTimer::setInterval(long millis, void (*isr)()){
	/*
		Set the timer frequency (in milliseconds)
	*/

	// Prevent negative frequencies
	if(frequency <= 0) { frequency = 1; }

	timer.setInterval(millis,isr);

	return *this;
}

double evTimer::getFrequency(){
	/*
		Get current time frequency
	*/

	return _frequency[index];
}

long evTimer::getPeriod(){
	/*
		Get current time period
	*/

	return 1.0/getFrequency()*1000000;
}


/*
	Implementation of the timer callbacks defined in 
	arduino-1.5.2/hardware/arduino/sam/system/CMSIS/Device/ATMEL/sam3xa/include/sam3x8e.h
*/
void TC0_Handler(){
	TC_GetStatus(TC0, 0);
	evTimer::callbacks[0]();
}
void TC1_Handler(){
	TC_GetStatus(TC0, 1);
	evTimer::callbacks[1]();
}
void TC2_Handler(){
	TC_GetStatus(TC0, 2);
	evTimer::callbacks[2]();
}
void TC3_Handler(){
	TC_GetStatus(TC1, 0);
	evTimer::callbacks[3]();
}
void TC4_Handler(){
	TC_GetStatus(TC1, 1);
	evTimer::callbacks[4]();
}
void TC5_Handler(){
	TC_GetStatus(TC1, 2);
	evTimer::callbacks[5]();
}
void TC6_Handler(){
	TC_GetStatus(TC2, 0);
	evTimer::callbacks[6]();
}
void TC7_Handler(){
	TC_GetStatus(TC2, 1);
	evTimer::callbacks[7]();
}
void TC8_Handler(){
	TC_GetStatus(TC2, 2);
	evTimer::callbacks[8]();
}
