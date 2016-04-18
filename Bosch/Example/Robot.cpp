// Position of BOSCH AHC-2 12V  6004.RA3.194-06 174.9:1 gear w/ encoder 1 tick per motor revolution on roboRIO analog 5 volt bus
// FRC Team 4237 Lakeshore High School
// Sample program merely rotates 1 revolution then reverses for 1 revolution and does so forever.

#include "WPILib.h"
#include "Bosch.h"

class Robot: public SampleRobot
{
public:
	Robot();
	void OperatorControl();

private:
	Bosch* mBosch;
};

Robot::Robot():
		mBosch(new Bosch(0, 0))
{
}

void Robot::OperatorControl()
{
	bool blockForward, blockReverse; // soft limit switches for this example
	int mPos=0;
	float speed = 1.0; // initial speed for this example
	mBosch->mCounter->Reset();

// example back and forth nearly 1 revolution (174.9)

	while(IsEnabled() && IsOperatorControl())
	{
		mPos = mBosch->GetPosition();
		printf("Position %d, Speed %f\n", mPos, speed);

		if (mPos >= 175) blockForward = true; // example check for at limit switch
		else blockForward = false;

		if (mPos <= 0) blockReverse = true; // example check for at limit switch
		else blockReverse = false;

		if (blockForward) speed = -1.; // example if at a limit switch go back the other way
		if (blockReverse) speed = +1.;

		// call CheckDirectionChange with same speed as Set() with (or before or after) every motor Set() to update position if reversing direction
		mBosch->mCANTalon->Set(mBosch->CheckDirectionChange(speed)); // refresh or change speed, update position if changing direction
		Wait(0.01); // ticks won't be lost but wait less to see them all here and respond faster
	}
}

START_ROBOT_CLASS(Robot)
