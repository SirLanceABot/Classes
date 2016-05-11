
/*
 * Logitech.cpp
 *
 *  Created on: Jan 23, 2016
 *      Author: 18elafrenz
 */

#include "Logitech.h"
#include "WPILib.h"
#include "Constants.h"



Logitech::Logitech(int port):			// constructor
mLogitech(std::make_shared<Joystick>(port))
{
	printf("File %18s Date %s Time %s Object %p\n",__FILE__,__DATE__, __TIME__, this);
//	mLogitech = new Joystick(port);
}

Logitech::~Logitech()
{
//	delete mLogitech;
}

// Get buttons and store them into the corresponding variables
bool Logitech::GetRawButton(int button)
{
	return mLogitech->GetRawButton(button);
}

bool Logitech::GetTrigger()
{
	return GetRawButton(LOGITECH::TRIGGER);
}

bool Logitech::GetButton2()
{
	return GetRawButton(LOGITECH::BUTTON_2);
}


bool Logitech::GetButton3()
{
	return GetRawButton(LOGITECH::BUTTON_3);
}


bool Logitech::GetButton4()
{
	return GetRawButton(LOGITECH::BUTTON_4);
}


bool Logitech::GetButton5()
{
	return GetRawButton(LOGITECH::BUTTON_5);
}


bool Logitech::GetButton6()
{
	return GetRawButton(LOGITECH::BUTTON_6);
}


bool Logitech::GetButton7()
{
	return GetRawButton(LOGITECH::BUTTON_7);
}


bool Logitech::GetButton8()
{
	return GetRawButton(LOGITECH::BUTTON_8);
}


bool Logitech::GetButton9()
{
	return GetRawButton(LOGITECH::BUTTON_9);
}


bool Logitech::GetButton10()
{
	return GetRawButton(LOGITECH::BUTTON_10);
}


bool Logitech::GetButton11()
{
	return GetRawButton(LOGITECH::BUTTON_11);
}


bool Logitech::GetButton12()
{
	return GetRawButton(LOGITECH::BUTTON_12);
}

// functions to receive values from the axes and store them into the corresponding variables
float Logitech::GetRawAxis(int axis)
{

	float value = mLogitech->GetRawAxis(axis);
	if (fabs(value) <= 0.1)
		value = 0.0;
	else if(value > 1.0)
		value = 1.0;
	else if(value < -1.0)
		value = -1.0;
	if(axis == LOGITECH::Y_AXIS)
		value = -value;
	return value;
}

float Logitech::GetXAxis()
{
	return GetRawAxis(LOGITECH::X_AXIS);
}

float Logitech::GetYAxis()
{
	return GetRawAxis(LOGITECH::Y_AXIS);
}

float Logitech::GetZAxis()
{
	return GetRawAxis(LOGITECH::Z_AXIS);
}

float Logitech::GetThrottleAxis()
{
	return GetRawAxis(LOGITECH::THROTTLE_AXIS);
}

int Logitech::GetPOV()
{
	return mLogitech->GetPOV();
}

void Logitech::Test()
{
	if(GetTrigger() == 1 || GetButton2() == 1 || GetButton3() == 1 || GetButton4() == 1 || GetButton5() == 1 || GetButton6() == 1)
	{
		printf("\nTrigger: %d, \nButton 2: %d, \nButton 3: %d, \nButton 4: %d, \nButton 5: %d, \nButton 6: %d ", GetTrigger(), GetButton2(), GetButton3(),GetButton4(), GetButton5(), GetButton6());
	}
	if(GetButton7() == 1 || GetButton8() == 1 || GetButton9() == 1 || GetButton10() == 1 || GetButton11() == 1 || GetButton12() == 1)
	{
		printf("\nButton 7: %d, \nButton 8: %d, \nButton 9: %d, \nButton 10: %d, \nButton 11: %d, \nButton 12: %d", GetButton7(), GetButton8(), GetButton9(), GetButton10(), GetButton11(), GetButton12());
	}
	if((GetXAxis() != 0) || (GetYAxis() != 0) || (GetZAxis() != 0))
	{
		printf("\nX Axis: %f, \nY Axis: %f, \nZ Axis: %f, \nThrottle Axis: %f \nPOV: %d", GetXAxis(), GetYAxis(), GetZAxis(), GetThrottleAxis(), GetPOV());
	}

}



