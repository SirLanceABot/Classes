// start of example of SmartDashBoard and LiveWindow

#include "WPILib.h"
//#include "NetworkTables/NetworkTable.h"

class MyRobot : public SimpleRobot
{
public:
	MyRobot(void);
	~MyRobot(void);
	void OperatorControl(void);
	void Test(void);
	void MotorTest(void);

private:
	Talon* mLeftDrive;
	Talon* mRightDrive;
	Joystick* mStick;
	LiveWindow* mLW;

};


MyRobot::MyRobot(void)
{
	mLeftDrive = new Talon(1); //1 not swerve drive
	mRightDrive = new Talon (2); //2
	mStick = new Joystick (1);
}

MyRobot::~MyRobot(void)
{
	delete mLeftDrive;
	delete mRightDrive;
	delete mStick;

}

void MyRobot::OperatorControl(void)
{
	int counter = 0;



	while (IsOperatorControl())
	{
		SmartDashboard::PutNumber("Left X Axis", mStick->GetRawAxis(1));
		SmartDashboard::PutNumber("Left Y Axis", mStick->GetRawAxis(2));
		SmartDashboard::PutNumber("Counter", counter++);




		if(mStick->GetRawButton(1)) 		//a button
		{
			mLeftDrive->Set(.5);
		}
		else
		{
			mLeftDrive->Disable();
		}

		Wait(0.10);

	}
}

void MyRobot::Test(void)
{
	mLW = LiveWindow::GetInstance();
	mLW->AddActuator("MySubsystem", "LeftDrive", mLeftDrive);	
	mLW->AddActuator("MySubsystem", "RightDrive", mRightDrive);	
	while(IsTest())
	{


		Wait(0.001);
	}
}

void MyRobot::MotorTest(void)
{
	if(mStick->GetRawButton(1)) 		//a button
	{
		mRightDrive->Set(1);
		mLeftDrive->Set(-1);
	}
	if(mStick->GetRawButton(2))		// b button
	{
		mRightDrive->Set(-.1);
		mLeftDrive->Set(.1);
	}
	mRightDrive->Disable();
	mLeftDrive->Disable();

}

START_ROBOT_CLASS(MyRobot);




