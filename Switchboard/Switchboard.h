/*
 * Switchboard.h
 *
 *  Created on: Feb 15, 2016
 *      Author: Andrew
 */

#ifndef SWITCHBOARD_H_
#define SWITCHBOARD_H_

#include <memory>

class Joystick;
class DigitalOutput;
class AnalogPotentiometer;

class Switchboard
{
public:
	Switchboard(int);
	~Switchboard();
	bool GetSwitch0();
	bool GetSwitch1();
	bool GetSwitch2();
	bool GetSwitch3();
	bool GetSwitch4();
	bool GetSmallButton();
	bool GetBigButton();
	bool GetJoystickUp();
	bool GetJoystickDown();

	void SetBigButtonLED(bool);
	void SetSmallButtonLED(bool);
	void SetJoystickUpLED(bool);
	void SetJoystickDownLED(bool);

	float GetPot0();
	float GetPot1();
	float GetPot2();

	int GetCounter();

	void SwitchTest();
	void ButtonTest();
	void POTTest();
	void CounterTest();

private:
//	Joystick* mSwitchboard;
	std::shared_ptr<Joystick> mSwitchboard;
};



#endif /* SRC_SWITCHBOARD_H_ */
