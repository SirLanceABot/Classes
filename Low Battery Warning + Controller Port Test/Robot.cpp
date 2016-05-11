#include "WPILib.h"

class Robot: public SampleRobot
{
public:
	Robot();
	~Robot();
	void OperatorControl();
	void LowBattery();
	void ControllerCheck();
private:
	Joystick* mXbox;
};
Robot::Robot()
{
	mXbox = new Joystick(0);
}

Robot::~Robot()
{
	delete mXbox;
}
void Robot::OperatorControl()
{
	while(IsOperatorControl() && IsEnabled())
	{
		ControllerCheck();
		LowBattery();
		Wait(0.02);
	}
}
void Robot::LowBattery()
{
	double batteryvoltage;
	batteryvoltage = DriverStation::GetInstance().GetBatteryVoltage();
	if(batteryvoltage < 12)
	{
		printf("Low Battery\n");
	}
}
void Robot::ControllerCheck()
{
	int joystick0;
	int joystick1;
	int joystick2;
	joystick0 = DriverStation::GetInstance().GetJoystickType(0);
	joystick1 = DriverStation::GetInstance().GetJoystickType(1);
	joystick2 = DriverStation::GetInstance().GetJoystickType(2);
	if(joystick0 != 1)
	{
		printf("Port 0 Not XBOX\n");
	}
	if(joystick1 != 20)
	{
		printf("Port 1 Not Logitech\n");
	}
	if(joystick2 != 24)
	{
		printf("Port 2 Not Switch Board\n");
	}
}

START_ROBOT_CLASS(Robot)
