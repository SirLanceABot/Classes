/*  L I M I T   S W I T C H   S A M P L E   U S A G E  */

#include "WPILib.h"
#include LimitSwitch.h"

class SimpleRobotDemo : public SimpleRobot
{
public:
	LimitSwitch* mTopLimitSwitch;
	LimitSwitch* mBottomLimitSwitch;

SimpleRobotDemo()
	{
		mTopLimitSwitch = new LimitSwitch(11);
		mBottomLimitSwitch = new LimitSwitch(12);
	}

void OperatorControl()
	{
		while (IsOperatorControl() && IsEnabled())
			{
				printf("Top: %s;    Bottom: %s\n",
					mTopLimitSwitch->IsOn()? "Closed":"Open", mBottomLimitSwitch->IsOff()? "Open":"Closed");
				Wait(0.005);
			}
	}
};

START_ROBOT_CLASS(SimpleRobotDemo);
