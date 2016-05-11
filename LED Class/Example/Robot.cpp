#include "WPILib.h"
#include "LED.h"

class Robot: public SampleRobot
{
public:
	Robot();
	~Robot();
	void OperatorControl();
	void LEDTest();
private:
	LED* mLED0;
	LED* mLED1;
	LED* mLED2;
	LED* mLED3;
};

Robot::Robot()
{

	mLED0 = new LED(0);
	mLED1 = new LED(1);
	mLED2 = new LED(2);
	mLED3 = new LED(3);
}

Robot::~Robot()
{
	delete mLED3;
	delete mLED2;
	delete mLED1;
	delete mLED0;
}

void Robot::OperatorControl()
{
	while(IsOperatorControl() && IsEnabled())
	{
		mLED0->Test();
		mLED1->Test();
		mLED2->Test();
		mLED3->Test();
		Wait(0.02);
	}
}

START_ROBOT_CLASS(Robot);

