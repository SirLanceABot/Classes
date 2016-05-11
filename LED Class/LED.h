/*
 * LED.h
 *
 *  Created on: Jan 30, 2016
 *      Author: 19shwong
 */

#ifndef LED_H_
#define LED_H_

class DigitalOutput;

class LED
{
public:
	LED(int);
	~LED();
	void On();
	void Off();
	void Toggle();
	void Test();
private:
	DigitalOutput* mLED;
	bool mState;
};

#endif /* LED_H_ */
