#include "WPILib.h"
#include "Constants.h"
#include "AutoTune_Example.h"
#include <math.h>

class Robot: public SampleRobot
{
	CANTalon* mFrontLeftMotor;
	CANTalon* mRearLeftMotor;
	CANTalon* mFrontRightMotor;
	CANTalon* mRearRightMotor;
	Timer * mTime;
	PowerDistributionPanel *mPDP;
public:
	Robot()
	{
		mFrontLeftMotor = new CANTalon(CAN_PORT::FRONT_LEFT);
		mRearLeftMotor = new CANTalon(CAN_PORT::REAR_LEFT);
		mFrontRightMotor = new CANTalon(CAN_PORT::FRONT_RIGHT);
		mRearRightMotor = new CANTalon(CAN_PORT::REAR_RIGHT);
		mTime = new Timer();
		mPDP = new  PowerDistributionPanel();
	}

	~Robot()
	{
	}

	void RobotInit()
	{
		mPDP->ClearStickyFaults();

		mFrontLeftMotor->ClearError();
		mFrontLeftMotor->ClearStickyFaults();
		mFrontRightMotor->ClearError();
		mFrontRightMotor->ClearStickyFaults();
		mRearLeftMotor->ClearError();
		mRearLeftMotor->ClearStickyFaults();
		mRearRightMotor->ClearError();
		mRearRightMotor->ClearStickyFaults();
		Wait(.5);

// CAN Talon control frame rate default = 10 ms; set by optional argument to the object constructor
// but changing it slightly to 8 was a disaster - garbage motor control

// CAN Talon status frame rates settings:
	 	mFrontLeftMotor->SetStatusFrameRateMs(CANTalon::StatusFrameRate::StatusFrameRateFeedback, 8);// default 20; GetPosition
		mFrontRightMotor->SetStatusFrameRateMs(CANTalon::StatusFrameRate::StatusFrameRateFeedback, 8);
//		mFrontLeftMotor->SetStatusFrameRateMs(CANTalon::StatusFrameRate::StatusFrameRateGeneral, 10);
//		mFrontRightMotor->SetStatusFrameRateMs(CANTalon::StatusFrameRate::StatusFrameRateGeneral, 10);
//		mFrontLeftMotor->SetStatusFrameRateMs(CANTalon::StatusFrameRate::StatusFrameRateQuadEncoder, 100);//GetEncPosition
//		mFrontRightMotor->SetStatusFrameRateMs(CANTalon::StatusFrameRate::StatusFrameRateQuadEncoder, 100);
//		mFrontLeftMotor->SetStatusFrameRateMs(CANTalon::StatusFrameRate::StatusFrameRateAnalogTempVbat, 100);
//		mFrontRightMotor->SetStatusFrameRateMs(CANTalon::StatusFrameRate::StatusFrameRateAnalogTempVbat, 100);
		Wait(.2);

		std::cout << "Left ControlMode " << mFrontLeftMotor->GetControlMode() << std::endl;
		std::cout << "Right ControlMode " << mFrontLeftMotor->GetControlMode() << std::endl;

		std::cout << "Left IsControlEnabled " << mFrontLeftMotor->IsControlEnabled() << std::endl;
		std::cout << "Right IsControlEnabled " << mFrontRightMotor->IsControlEnabled() << std::endl;

// fixme: Limit Voltage change?
		mFrontLeftMotor->SetVoltageRampRate(50.);
		mFrontRightMotor->SetVoltageRampRate(50.);

		mFrontLeftMotor->SetFeedbackDevice(CANTalon::QuadEncoder);
		mFrontRightMotor->SetFeedbackDevice(CANTalon::QuadEncoder);

		mFrontLeftMotor->SetEncPosition(0);
		mFrontRightMotor->SetEncPosition(0);

		mFrontLeftMotor->SetSensorDirection(false); // true reverses GetPosition & GetSpeed but not GetEncPosition nor GetEncVel
		mFrontRightMotor->SetSensorDirection(true); // invert to match left; true reverses GetPosition & GetSpeed but not GetEncPosition nor GetEncVel

		Wait(.2);

#ifdef TABLE_TOP_PG45775126000_26_9KET
// 7 encoder pulses/motor rev x 26.9:1 motor revs/gear box output shaft rev = 188; times 4x quadrature encoder = 753.2 edges/motor rev/output shaft rev
		mFrontLeftMotor->ConfigEncoderCodesPerRev((uint16_t)(7L*26.9L + 0.5L));
		mFrontRightMotor->ConfigEncoderCodesPerRev(188);
#endif

#ifdef ROBOT
// no ConfigEncoderCodesPerRev()
// GetPosition() == GetEncPosition, GetSpeed == GetEncVel(); but updated faster than Enc versions
#endif

// fixme: PID Tuning parameters go here
		mFrontLeftMotor->SetPID(4., 0.02, 150.);
		mFrontLeftMotor->SetControlMode(mFrontLeftMotor->ControlMode::kSpeed);
		mFrontLeftMotor->Set(0.);
		Wait(.2);

		mFrontRightMotor->SetPID(4., 0.02, 150.);
		mFrontRightMotor->SetControlMode(mFrontRightMotor->ControlMode::kSpeed);
		mFrontRightMotor->Set(0.);
		Wait(.2);

		mRearLeftMotor->SetControlMode(CANSpeedController::kFollower);
		mRearLeftMotor->Set(CAN_PORT::FRONT_LEFT);

		mRearRightMotor->SetControlMode(CANSpeedController::kFollower);
		mRearRightMotor->Set(CAN_PORT::FRONT_RIGHT);
		Wait(.2);
	}

	void Test()
	{
	std::cout << "\nEntered Test mode\n";

	mFrontLeftMotor->ConfigForwardSoftLimitEnable(false);
	mFrontRightMotor->ConfigForwardSoftLimitEnable(false);
	mFrontLeftMotor->ConfigReverseSoftLimitEnable(false);
	mFrontRightMotor->ConfigReverseSoftLimitEnable(false);

	std::cout << "Front Left ControlMode " << mFrontLeftMotor->GetControlMode() << std::endl;
	std::cout << "Front Right ControlMode " << mFrontLeftMotor->GetControlMode() << std::endl;
	std::cout << "Resetting Talons\n";

	mFrontLeftMotor->Reset();
	mFrontRightMotor->Reset();
	Wait(.5);

	mFrontLeftMotor->Enable();
	mFrontRightMotor->Enable();
	Wait(.5);

	std::cout << "Front Left ControlMode " << mFrontLeftMotor->GetControlMode() << std::endl;
	std::cout << "Front Right ControlMode " << mFrontRightMotor->GetControlMode() << std::endl;
	std::cout << "Rear Left ControlMode " << mRearLeftMotor->GetControlMode() << std::endl;
	std::cout << "Rear Right ControlMode " << mRearRightMotor->GetControlMode() << std::endl;

	while(IsTest())
	{
		if(IsEnabled())
		{
			std::cout << "\nPID Auto Tuner Starting.\n";

			TuneMain(mFrontLeftMotor, mFrontRightMotor);

			std::cout << "\nPID Auto Tuner Completed.\n";
			break;
		}
		else
		{
			std::cout << "Test waiting for ENABLE to run PID Auto Tuner";
			Wait(1.);
		}
	}
	std::cout << "\nLeaving Test mode\n";
	}

};

START_ROBOT_CLASS(Robot)
