#include "WPILib.h"
#include "PortDefine.h"

class Robot : public SampleRobot
{
public:
	Robot();
	~Robot();
	void OperatorControl();
private:
	DigitalInput* mDialOnes;
	DigitalInput* mDialTwos;
	DigitalInput* mDialFours;
	DigitalInput* mDialEights;
};

Robot::Robot()
{
	mDialOnes = new DigitalInput(SIXTEEN_POINT_SWITCH_ONES);
	mDialTwos = new DigitalInput(SIXTEEN_POINT_SWITCH_TWOS);
	mDialFours = new DigitalInput(SIXTEEN_POINT_SWITCH_FOURS);
	mDialEights = new DigitalInput(SIXTEEN_POINT_SWITCH_EIGHTS);
}
Robot::~Robot()
{
	delete mDialEights;
	mDialEights = NULL;
	delete mDialFours;
	mDialFours = NULL;
	delete mDialTwos;
	mDialTwos = NULL;
	delete mDialOnes;
	mDialOnes = NULL;
}
void Robot::OperatorControl()
{
	while(IsOperatorControl() && IsEnabled())
	{
		int dial = 0;
		if(mDialOnes->Get())
		{
			dial = dial + 1;
		}
		if(mDialTwos->Get())
		{
			dial = dial + 2;
		}
		if(mDialFours->Get())
		{
			dial = dial + 4;
		}
		if(mDialEights->Get())
		{
			dial = dial + 8;
		}
		dial = (19 - dial) % 16;
		printf("Dial:  %d\n", dial);
	}
}
START_ROBOT_CLASS(Robot);
