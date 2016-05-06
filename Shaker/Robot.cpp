#include "WPILib.h"
#include "stats_running.h"
#include "ITG-3200.h"
#include "BrownLinearExpo.h"

namespace VIRTUAL_PORT //set in SilverLight
{
const int POWER_DISTRIBUTION_PANEL = 0;

const int LEFT_FRONT_DRIVE = 2;
const int RIGHT_FRONT_DRIVE = 3;
const int LEFT_REAR_DRIVE = 4;
const int RIGHT_REAR_DRIVE = 5;

const int SHOOTER_ANGLE = 6;
const int SHOOTER_LEFT_WHEEL = 7;
const int SHOOTER_RIGHT_WHEEL = 8;
}

class Robot: public SampleRobot
{
#define ITG3200_I2C_ADDR 0x68

	std::unique_ptr<ITG3200> mITG3200_1;
	std::unique_ptr<BuiltInAccelerometer> mAcc;
	std::unique_ptr<RunningStats> ShakerWindowX, ShakerWindowY, ShakerWindowZ, ShakerWindowPitchX, ShakerWindowPitchY;
	BrownLinearExpo pitchX_BLE, pitchY_BLE;
	RobotDrive* mRobot;

	CANTalon* mFrontLeft;
	CANTalon* mFrontRight;
	CANTalon* mRearLeft;
	CANTalon* mRearRight;

	Joystick* mXbox;

public:
	Robot() :
		mITG3200_1(std::make_unique<ITG3200>(I2C::Port::kOnboard, ITG3200_I2C_ADDR)),
		mAcc(std::make_unique<BuiltInAccelerometer>()),
		ShakerWindowX(std::make_unique<RunningStats>()),
		ShakerWindowY(std::make_unique<RunningStats>()),
		ShakerWindowZ(std::make_unique<RunningStats>()),
		ShakerWindowPitchX(std::make_unique<RunningStats>()),
		ShakerWindowPitchY(std::make_unique<RunningStats>()),
		pitchX_BLE(.1),
		pitchY_BLE(.1),
		mFrontLeft(new CANTalon(VIRTUAL_PORT::LEFT_FRONT_DRIVE)),
		mFrontRight(new CANTalon(VIRTUAL_PORT::RIGHT_FRONT_DRIVE)),
		mRearLeft(new CANTalon(VIRTUAL_PORT::LEFT_REAR_DRIVE)),
		mRearRight(new CANTalon(VIRTUAL_PORT::RIGHT_REAR_DRIVE)),
		mXbox(new Joystick(0))
{

		mRobot = new RobotDrive(mFrontLeft, mRearLeft, mFrontRight, mRearRight);
		mRobot->SetSafetyEnabled(false);
}

	void RobotInit()
	{
	}

	void Autonomous()
	{
		std::cout << "Auto selected" << std::endl;
	}

	void OperatorControl()
	{
		std::cout << "OperatorControl" << std::endl;
		Timer t;

		double Angle[3]={0.,0.,0.};
		double Angle_Rate[3];
		float Temperature;

		if (mITG3200_1->IsWorking())
		{
			mITG3200_1->SetSampleRate(5);
			Wait(.7);
			mITG3200_1->Calibrate();
			mITG3200_1->SetAngle(Angle);
		}

		double x, y, z;
		double StdDevX, StdDevY, StdDevZ;
		double MeanX, MeanY, MeanZ;
		double pitchX, StdDevPitchX, MeanPitchX;
		double pitchY, StdDevPitchY, MeanPitchY;
		double pitchXsmooth, pitchYsmooth;

		t.Start();

		while (IsOperatorControl() && IsEnabled())
		{
//			if(t.Get() <= 3.5)
//			{
//				//mRobot->ArcadeDrive(.75, 0.0);
//				printf("Timer: %f ", t.Get());
//			}
//			else
//			{
//				mRobot->ArcadeDrive(0.0,0.0);
//				t.Stop();
//				break;
//			}

			mRobot->ArcadeDrive(-mXbox->GetRawAxis(1), -mXbox->GetRawAxis(0));

			if (mITG3200_1->IsWorking())
			{
				mITG3200_1->Get();
				mITG3200_1->GetAngle(Angle);
				mITG3200_1->GetAngleRate(Angle_Rate);
				Temperature = mITG3200_1->GetTemperature();
//				printf("[ITG3200_1] Temperature %7.2f X %9.2f Y %9.2f Z %9.2f Xrate %9.2f Yrate %9.2f Zrate %9.2f\n",
//						Temperature,
//						Angle[0], Angle[1], Angle[2],
//						Angle_Rate[0], Angle_Rate[1], Angle_Rate[2]);
			}

			for (int i = 1; i <=10; i++)
			{
				Wait(0.005);

				x = mAcc->GetX();
				y = mAcc->GetY();
				z = mAcc->GetZ();
				pitchX = atan(x/sqrt(pow(y,2) + pow(z,2))) * (360.0/(2.*3.14159));
				pitchY = atan(y/sqrt(pow(x,2) + pow(z,2))) * (360.0/(2.*3.14159));

				pitchXsmooth = pitchX_BLE.step(pitchX);
				pitchYsmooth = pitchY_BLE.step(pitchY);

				ShakerWindowX->Push(x);
				ShakerWindowY->Push(y);
				ShakerWindowZ->Push(z);
				ShakerWindowPitchX->Push(pitchX);
				ShakerWindowPitchY->Push(pitchY);

				//			ShakerWindowX->Print(stdout, "X");
				//			ShakerWindowY->Print(stdout, "Y");
				//			ShakerWindowZ->Print(stdout, "Z");

				StdDevX = ShakerWindowX->StandardDeviation();
				StdDevY = ShakerWindowY->StandardDeviation();
				StdDevZ = ShakerWindowZ->StandardDeviation();
				StdDevPitchX = ShakerWindowPitchX->StandardDeviation();
				StdDevPitchY = ShakerWindowPitchY->StandardDeviation();

				MeanX = ShakerWindowX->Mean();
				MeanY = ShakerWindowY->Mean();
				MeanZ = ShakerWindowZ->Mean();
				MeanPitchX = ShakerWindowPitchX->Mean();
				MeanPitchY = ShakerWindowPitchY->Mean();
				if (StdDevX + StdDevY + StdDevZ < 0.01) // sensors are fairly quiet so stop - should be past the barrier
				{mRobot->ArcadeDrive(0.0,0.0);
				break;
				}

			}

			printf("XYZPitchXY Std Dev: %6.3f, %6.3f, %6.3f, %6.3f, %6.3f XYZPitchXY Mean: %6.3f, %6.3f, %6.3f, %6.3f, %6.3f PitchRaw/SmoothXY: %6.3f, %6.3f, %6.3f, %6.3f\n",
					StdDevX, StdDevY, StdDevZ, StdDevPitchX, StdDevPitchY, MeanX, MeanY, MeanZ, MeanPitchX, MeanPitchY, pitchX, pitchXsmooth, pitchY, pitchYsmooth);

			ShakerWindowX->Clear();
			ShakerWindowY->Clear();
			ShakerWindowZ->Clear();
			ShakerWindowPitchX->Clear();
			ShakerWindowPitchY->Clear();
			Wait(.007);
		}
	}
};

START_ROBOT_CLASS(Robot)

