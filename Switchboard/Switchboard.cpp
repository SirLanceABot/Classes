/*
 * Switchboard.cpp
 *
 *  Created on: Feb 15, 2016
 *      Author: Andrew
 */

#include "Switchboard.h"
#include "Constants.h"
#include "WPILib.h"

Switchboard::Switchboard(int port):
mSwitchboard(std::make_shared<Joystick>(port))
{
	printf("File %18s Date %s Time %s Object %p\n",__FILE__,__DATE__, __TIME__, this);
//	mSwitchboard = new Joystick(port);

}

Switchboard::~Switchboard()
{
//	delete mSwitchboard;
}

// Finding out whether or not the button are pressed
bool Switchboard::GetSwitch0()
{
	return mSwitchboard->GetRawButton(SWITCHBOARD::SWITCH_0);
}

bool Switchboard::GetSwitch1()
{
	return mSwitchboard->GetRawButton(SWITCHBOARD::SWITCH_1);
}

bool Switchboard::GetSwitch2()
{
	return mSwitchboard->GetRawButton(SWITCHBOARD::SWITCH_2);
}

bool Switchboard::GetSwitch3()
{
	return mSwitchboard->GetRawButton(SWITCHBOARD::SWITCH_3);
}

bool Switchboard::GetSwitch4()
{
	return mSwitchboard->GetRawButton(SWITCHBOARD::SWITCH_4);
}

bool Switchboard::GetSmallButton()
{
	return mSwitchboard->GetRawButton(SWITCHBOARD::SMALL_BUTTON);
}

bool Switchboard::GetBigButton()
{
	return mSwitchboard->GetRawButton(SWITCHBOARD::BIG_BUTTON);
}

bool Switchboard::GetJoystickUp()
{
	return mSwitchboard->GetRawButton(SWITCHBOARD::JOYSTICK_UP);
}

bool Switchboard::GetJoystickDown()
{
	return mSwitchboard->GetRawButton(SWITCHBOARD::JOYSTICK_DOWN);
}

//Turn the LEDs on or off if the buttons/joysticks are pressed
void Switchboard::SetBigButtonLED(bool state)
{
	mSwitchboard->SetOutput(SWITCHBOARD::BIG_BUTTON_LED, state);
}

void Switchboard::SetSmallButtonLED(bool state)
{
	mSwitchboard->SetOutput(SWITCHBOARD::SMALL_BUTTON_LED, state);
}

void Switchboard::SetJoystickUpLED(bool state)
{
	mSwitchboard->SetOutput(SWITCHBOARD::JOYSTICK_UP_LED, state);
}

void Switchboard::SetJoystickDownLED(bool state)
{
	mSwitchboard->SetOutput(SWITCHBOARD::JOYSTICK_DOWN_LED, state);
}

//Gets the value of the potentiometer and multiplies it by one to make the range -1 to 1
float Switchboard::GetPot0()
{
	return mSwitchboard->GetRawAxis(SWITCHBOARD::POT_0)*9;
}

float Switchboard::GetPot1()
{
	return mSwitchboard->GetRawAxis(SWITCHBOARD::POT_1)*9;
}

float Switchboard::GetPot2()
{
	return mSwitchboard->GetRawAxis(SWITCHBOARD::POT_2)*9;
}

int Switchboard::GetCounter()
{
	return mSwitchboard->GetRawButton(SWITCHBOARD::COUNTER_PORT_8) + mSwitchboard->GetRawButton(SWITCHBOARD::COUNTER_PORT_9) * 2 + mSwitchboard->GetRawButton(SWITCHBOARD::COUNTER_PORT_10) * 4;

}

//Tells whether or not the switches are flipped on or off
void Switchboard::SwitchTest()
{
	if(GetSwitch0() == true || GetSwitch1() == true || GetSwitch2() == true || GetSwitch3() == true || GetSwitch4() == true)
	{
		printf("Switch 0: %d,  Switch 1: %d,  Switch 2: %d,  Switch 3: %d,  Switch 4: %d  \n", GetSwitch0(), GetSwitch1(), GetSwitch2(), GetSwitch3(), GetSwitch4());
	}
}

//Tells whether or not the buttons are currently being pressed
void Switchboard::ButtonTest()
{
	if(GetBigButton() == true)
	{
		SetBigButtonLED(true);
		printf("Big Button  \n");
	}
	else
	{
		SetBigButtonLED(false);
	}

	if(GetSmallButton() == true)
	{
		SetSmallButtonLED(true);
		printf("Small Button  \n");
	}
	else
	{
		SetSmallButtonLED(false);
	}

	if(GetJoystickUp() == true)
	{
		SetJoystickUpLED(true);
		printf("Joystick Up  \n");
	}
	else
	{
		SetJoystickUpLED(false);
	}

	if(GetJoystickDown() == true)
	{
		SetJoystickDownLED(true);
		printf("Joystick Down  \n");
	}
	else
	{
		SetJoystickDownLED(false);
	}
}

//Displays the value of the Potentiometers
void Switchboard::POTTest()
{
	printf("Pot 0: %f  Pot 1: %f  Pot 2: %f  \n", GetPot0(), GetPot1(), GetPot2());
}

//Displays the value of the counter
void Switchboard::CounterTest()
{
	printf("%d\n", GetCounter());
}

