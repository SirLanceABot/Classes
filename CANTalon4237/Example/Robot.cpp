#include "WPILib.h"
#include "CANTalon4237.h"

class Robot: public SampleRobot
{
public:
	Robot();
	~Robot();
	void OperatorControl();
	void EncoderTest();
	void SwitchTest();
	void MagneticEncoder();
private:
	Joystick* mXbox;
	CANTalon4237* mSlaveMotor;
	CANTalon4237* mMasterMotor;
	CANTalon4237* mMagneticMotor;
};

Robot::Robot()
{
	mXbox = new Joystick(0);
	mSlaveMotor = new CANTalon4237(1, CANTalon4237::kQuadEncoder);
	mMasterMotor = new CANTalon4237(0, CANTalon4237::kMagneticEncoder);
	//mSlaveMotor->SetFollower(0);
	//mMasterMotor->SetMaster();
	//mMasterMotor->ReverseEncoderDirection();
	//TODO: QuadEncoder - Value of the master motor encoder is reversed; The blue encoder board and the black encoder board are different, the black is correct

}

Robot::~Robot()
{
	delete mMasterMotor;
	delete mSlaveMotor;
	delete mXbox;
}

void Robot::OperatorControl()
{
	while(IsOperatorControl() && IsEnabled())
	{
		//SwitchTest();
		EncoderTest();

		Wait(0.02);
	}
}

void Robot::SwitchTest()
{
	float MotorSpeed;

	MotorSpeed = -mXbox->GetRawAxis(1);
	if(fabs(MotorSpeed) >= 0.1)
	{
		mMasterMotor->Set(MotorSpeed);
		//mSlaveMotor->Set(mXbox->GetRawAxis(1));
	}
	else
	{
		mMasterMotor->Set(0);
		//mSlaveMotor->Set(0);
	}

	printf("Motor Speed: %f \n", MotorSpeed);
}

void Robot::EncoderTest()
{
	float MotorSpeed;

	MotorSpeed = -mXbox->GetRawAxis(1);
	if(fabs(MotorSpeed) >= 0.1)
	{
		mMasterMotor->Set(MotorSpeed);
		//mSlaveMotor->Set(MotorSpeed);
		printf ("Master Encoder: %d   ", mMasterMotor->GetEncoder());
		printf ("Slave Encoder: %d \n", mSlaveMotor->GetEncoder());
	}
	else
	{
		mMasterMotor->Set(0);
		//		mSlaveMotor->Set(0);
	}

	if(mXbox->GetRawButton(2))
	{
		printf ("Encoders Reset\n");
		mMasterMotor->ResetEncoder();
		mSlaveMotor->ResetEncoder();
	}

}
START_ROBOT_CLASS(Robot);
