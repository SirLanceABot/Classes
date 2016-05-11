/*
 * XboxJoystick.cpp
 *
 *  Created on: Jan 23, 2016
 *      Author: 18elafrenz
 */
#include "XboxJoystick.h"
#include "WPILib.h"
#include "Constants.h"

XboxJoystick::XboxJoystick(int port):		// constructor
	mJoystick(std::make_shared<Joystick>(port))
{
	printf("File %18s Date %s Time %s Object %p\n",__FILE__,__DATE__, __TIME__, this);
	fflush(NULL);
//	mJoystick = new Joystick(port);
}

XboxJoystick::~XboxJoystick()				// deconstructor
{
//	delete mJoystick;
}

bool XboxJoystick::GetRawButton(int button)		// to allow to get raw buttons
{
	return mJoystick->GetRawButton(button);
}

// getting buttons from the xbox joystick
bool XboxJoystick::GetA()
{
	return GetRawButton(XBOX::A_BUTTON);
}

bool XboxJoystick::GetB()
{
	return GetRawButton(XBOX::B_BUTTON);
}

bool XboxJoystick::GetX()
{
	return GetRawButton(XBOX::X_BUTTON);
}

bool XboxJoystick::GetY()
{
	return GetRawButton(XBOX::Y_BUTTON);
}

bool XboxJoystick::GetStart()
{
	return GetRawButton(XBOX::START_BUTTON);
}

bool XboxJoystick::GetBack()
{
	return GetRawButton(XBOX::BACK_BUTTON);
}

bool XboxJoystick::GetLeftStickPress()
{
	return GetRawButton(XBOX::LEFT_STICK_PRESS_BUTTON);
}

bool XboxJoystick::GetRightStickPress()
{
	return GetRawButton(XBOX::RIGHT_STICK_PRESS_BUTTON);
}

bool XboxJoystick::GetLeftBumper()
{
	return GetRawButton(XBOX::LEFT_BUMPER_BUTTON);
}

bool XboxJoystick::GetRightBumper()
{
	return GetRawButton(XBOX::RIGHT_BUMPER_BUTTON);
}

// get raw axis for the following axis functions
float XboxJoystick::GetRawAxis(int axis)
{
	float value = mJoystick->GetRawAxis(axis);
	if (fabs(value) <= 0.1)
		value = 0.0;
	else if (value > 1.0)
		value = 1.0;
	else if(value < -1.0)
		value = -1.0;
	if(axis == XBOX::LEFT_Y_AXIS || axis == XBOX::RIGHT_Y_AXIS)
		value = -value;
	return value;
}

float XboxJoystick::GetModifiedAxis(int axis, ModificationType modification)
{
	float value;

	value = GetRawAxis(axis);

	switch(modification)
	{
	case kLinear:
		break;
	case kSquared:
		value = value * fabs(value);
		break;
	case kCubic:
		value = pow(value, 3);
		break;
	case kCrawlWalkRun:
		if(GetB() == true)
			value = value * .25;
		else if(GetA() == false)
			value = value* .5;
		break;
	}
	return value;
}

// axis values from the xbox joystick
float XboxJoystick::GetLeftTrigger()
{
	return GetRawAxis(XBOX::LEFT_TRIGGER_AXIS);
}

float XboxJoystick::GetRightTrigger()
{
	return GetRawAxis(XBOX::RIGHT_TRIGGER_AXIS);
}

float XboxJoystick::GetLeftXAxis()
{
	return GetRawAxis(XBOX::LEFT_X_AXIS);
}

float XboxJoystick::GetRightXAxis()
{
	return GetRawAxis(XBOX::RIGHT_X_AXIS);
}

float XboxJoystick::GetLeftYAxis()
{
	return GetRawAxis(XBOX::LEFT_Y_AXIS);
}

float XboxJoystick::GetRightYAxis()
{
	return GetRawAxis(XBOX::RIGHT_Y_AXIS);
}

// recieving values from the directional pad on the xbox joystick
int XboxJoystick::GetDPad()
{
	return mJoystick->GetPOV();
}
int XboxJoystick::GetPOV()
{
	return mJoystick->GetPOV();
}

// setting the values for the rumble of the xbox
void XboxJoystick::SetLeftRumble(float intensity)
{
	// intensity is from 0-1
	if(intensity > 1)				// loop to make sure no invalid value is entered
		intensity = 1;				// so the program does not crash
	if(intensity < 0)
		intensity = 0;

	mJoystick->SetRumble(Joystick::kLeftRumble, intensity);		// set the intensity of the rumble
}

void XboxJoystick::SetRightRumble(float intensity)
{
	// intensity is from 0-1
	if(intensity > 1)			// loop to make sure no invalid value is entered
		intensity = 1;			// so the program does not crash
	if(intensity < 0)
		intensity = 0;

	mJoystick->SetRumble(Joystick::kRightRumble, intensity);	// set the intensity of the rumble
}


void XboxJoystick::Test()
{
//	printf("Press A for left rumble and press B for right rumble; Press any button or axis to check if it is working");
	if (GetA() == 1)
	{
		SetLeftRumble(1);
	}
	else
	{
		SetLeftRumble(0);
	}
	if (GetB() == 1)
	{
		SetRightRumble(.8);
	}
	else
	{
		SetRightRumble(0);
	}

	if (GetA() == 1 || GetB() == 1 || GetX() == 1 || GetY() == 1)
	{
		printf("\nA button: %d, \nB button: %d, \nX button: %d, \nY button: %d", GetA(),GetB(),GetX(), GetY());
	}
	if (GetStart() == 1 || GetBack() == 1)
	{
		printf("\nStart button: %d, \nBack button: %d", GetStart(), GetBack());
	}
	if (GetLeftStickPress() == 1 || GetRightStickPress() == 1)
	{
		printf("\nLeft Stick Press: %d, \nRight Stick Press: %d", GetLeftStickPress(), GetRightStickPress());
	}
	if (GetLeftBumper() == 1 || GetRightBumper() == 1)
	{
		printf("\nLeft Bumper button: %d, \nRight Bumper button: %d", GetLeftBumper(), GetRightBumper());
	}
	if ((GetLeftTrigger() != 0) || (GetRightTrigger() != 0))
	{
		printf("\nLeft trigger axis: %f, \nRight trigger axis: %f", GetLeftTrigger(), GetRightTrigger());
	}
	if ((GetLeftXAxis() != 0) || (GetLeftYAxis() != 0) || (GetRightXAxis() != 0) || (GetRightYAxis() != 0))
	{
		printf("\nLeft X axis: %f, \nLeft Y axis: %f, \nRight X axis: %f, \nRight Y axis: %f", GetLeftXAxis(), GetLeftYAxis(), GetRightXAxis(), GetRightYAxis());
	}
	if (GetDPad() == 0 || GetDPad() == 90 || GetDPad() == 180 || GetDPad() == 270)
	{
		printf("\nDPad: %d", GetDPad());
	}

}
