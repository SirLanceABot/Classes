/*
 * LED.cpp
 *
 *  Created on: Jan 30, 2016
 *      Author: 19shwong
 */

#include "WPILib.h"
#include "LED.h"

LED::LED(int port)
{

	mLED = new DigitalOutput(port);
	mState = false;
	Off();
}

LED::~LED()
{
	delete mLED;
}

//Turns on LEDs
void LED::On()
{
	mLED->Set(1);
	mState = true;
}

//Turns off LEDs
void LED::Off()
{
	mLED->Set(0);
	mState = false;
}

//If mState is false, turn on LED
//If mState is true, turn off LED
void LED::Toggle()
{
	if(mState == false)
		On();
	if(mState == true)
		Off();
}

void LED::Test()
{
	On();
	printf("Led on\n");
	Wait(1.0);

	Off();
	printf("Led off\n");
	Wait(1.0);

	On();
	printf("Led off\n");
	Wait(1.0);

	Off();
	printf("Led off\n");
}


