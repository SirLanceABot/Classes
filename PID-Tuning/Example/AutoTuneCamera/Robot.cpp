#include "WPILib.h"
#include "Constants.h"
#include "AutoTuneCamera.h"
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

//  ____            _               _   
// |  _ \    ___   | |__     ___   | |_ 
// | |_) |  / _ \  | '_ \   / _ \  | __|
// |  _ <  | (_) | | |_) | | (_) | | |_ 
// |_| \_\  \___/  |_.__/   \___/   \__|

	Robot()
	{
		mFrontLeftMotor = new CANTalon(CAN_PORT::FRONT_LEFT);
		mRearLeftMotor = new CANTalon(CAN_PORT::REAR_LEFT);
		mFrontRightMotor = new CANTalon(CAN_PORT::FRONT_RIGHT);
		mRearRightMotor = new CANTalon(CAN_PORT::REAR_RIGHT);
		mTime = new Timer();
		mPDP = new  PowerDistributionPanel();
	}

//  /\/|  ____            _               _   
// |/\/  |  _ \    ___   | |__     ___   | |_ 
//       | |_) |  / _ \  | '_ \   / _ \  | __|
//       |  _ <  | (_) | | |_) | | (_) | | |_ 
//       |_| \_\  \___/  |_.__/   \___/   \__|

	~Robot()
	{
		delete mPDP;
		delete mTime;
		delete mRearRightMotor;
		delete mFrontRightMotor;
		delete mRearLeftMotor;
		delete mFrontLeftMotor;
	}

//  ____            _               _     ___           _   _   
// |  _ \    ___   | |__     ___   | |_  |_ _|  _ __   (_) | |_ 
// | |_) |  / _ \  | '_ \   / _ \  | __|  | |  | '_ \  | | | __|
// |  _ <  | (_) | | |_) | | (_) | | |_   | |  | | | | | | | |_ 
// |_| \_\  \___/  |_.__/   \___/   \__| |___| |_| |_| |_|  \__|                                                          

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
		
		mFrontLeftMotor->ConfigForwardSoftLimitEnable(false);
		mFrontRightMotor->ConfigForwardSoftLimitEnable(false);
		mFrontLeftMotor->ConfigReverseSoftLimitEnable(false);
		mFrontRightMotor->ConfigReverseSoftLimitEnable(false);

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

		mFrontLeftMotor->SetVoltageRampRate(30.);  // good for desktop board with small plugin power supply
		mFrontRightMotor->SetVoltageRampRate(30.); // otherwise low-power faults and D-Link crashes; still crashes from high power to 0 though

		mFrontLeftMotor->SetCloseLoopRampRate(30.);
		mFrontRightMotor->SetCloseLoopRampRate(30.);

		mFrontLeftMotor->SetFeedbackDevice(CANTalon::QuadEncoder);
		mFrontRightMotor->SetFeedbackDevice(CANTalon::QuadEncoder);

		mFrontLeftMotor->SetEncPosition(0);
		mFrontRightMotor->SetEncPosition(0);

		mFrontLeftMotor->ConfigEncoderCodesPerRev(TALON_DRIVE::ENCODER_CODES_PER_REV);
		mFrontRightMotor->ConfigEncoderCodesPerRev(TALON_DRIVE::ENCODER_CODES_PER_REV);
		Wait(.2);
	}

//      _              _
//     / \     _   _  | |_    ___    _ __     ___    _ __ ___     ___    _   _   ___ 
//    / _ \   | | | | | __|  / _ \  | '_ \   / _ \  | '_ ` _ \   / _ \  | | | | / __|
//   / ___ \  | |_| | | |_  | (_) | | | | | | (_) | | | | | | | | (_) | | |_| | \__ \  .
//  /_/   \_\  \__,_|  \__|  \___/  |_| |_|  \___/  |_| |_| |_|  \___/   \__,_| |___/                                                                               

	void Autonomous()
	{
	}

//    ___                                  _                     ____                   _                    _ 
//   / _ \   _ __     ___   _ __    __ _  | |_    ___    _ __   / ___|   ___    _ __   | |_   _ __    ___   | |
//  | | | | | '_ \   / _ \ | '__|  / _` | | __|  / _ \  | '__| | |      / _ \  | '_ \  | __| | '__|  / _ \  | |
//  | |_| | | |_) | |  __/ | |    | (_| | | |_  | (_) | | |    | |___  | (_) | | | | | | |_  | |    | (_) | | |
//   \___/  | .__/   \___| |_|     \__,_|  \__|  \___/  |_|     \____|  \___/  |_| |_|  \__| |_|     \___/  |_|
//          |_|                                                                                                

	void OperatorControl()
	{
	}
	
//  _____                _   
// |_   _|   ___   ___  | |_ 
//   | |    / _ \ / __| | __|
//   | |   |  __/ \__ \ | |_ 
//   |_|    \___| |___/  \__|

	void Test()
	{
	std::cout << "\nEntered Test mode to tune Camera targeting\n";

//	std::cout << "Resetting Talons\n";
//	mFrontLeftMotor->Reset();
//	mFrontRightMotor->Reset();
//	Wait(.5);
//	mFrontLeftMotor->Enable();
//	mFrontRightMotor->Enable();
//	Wait(.5);

	mFrontLeftMotor->SetEncPosition(0);
	mFrontRightMotor->SetEncPosition(0);

	while(IsTest())
	{
		if(IsEnabled())
		{
			std::cout << "\nPID Auto Tuner Starting.\n";

			TuneMain(mFrontLeftMotor, mFrontRightMotor, mRearLeftMotor, mRearRightMotor);

			std::cout << "\nPID Auto Tuner Completed.\n";
			break;
		}
		else
		{
			std::cout << "Test waiting for ENABLE";
			Wait(1.);
		}
	}
	std::cout << "\nLeaving Test mode\n";
	}

};

//   _ __ ___     __ _  (_)  _ __  
//  | '_ ` _ \   / _` | | | | '_ \  .
//  | | | | | | | (_| | | | | | | |
//  |_| |_| |_|  \__,_| |_| |_| |_|

START_ROBOT_CLASS(Robot)
