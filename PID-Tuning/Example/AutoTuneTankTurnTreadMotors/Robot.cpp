#include "WPILib.h"
#include "Constants.h"
#include "AutoTune-tank-turn.h"
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

		mFrontLeftMotor->SetVoltageRampRate(30.);  // good for desktop board with small plugin power supply
		mFrontRightMotor->SetVoltageRampRate(30.); // otherwise low-power faults and D-Link crashes; still crashes from high power to 0 though

		mFrontLeftMotor->SetCloseLoopRampRate(30.);
		mFrontRightMotor->SetCloseLoopRampRate(30.);

		mFrontLeftMotor->SetFeedbackDevice(CANTalon::QuadEncoder);
		mFrontRightMotor->SetFeedbackDevice(CANTalon::QuadEncoder);
		
		Wait(.2);

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
		std::cout << "\nEntered OperatorControl to drive straight using PID control mode & followers\n";
		
		int positionEncLeft, positionEncRight, velocityEncLeft, velocityEncRight, errorLeft, errorRight;
		double positionLeft, positionRight, velocityLeft, velocityRight;

		std::cout << "\n\nResetting Talons\n\n";
		mFrontLeftMotor->Reset();
		mFrontRightMotor->Reset();
		Wait(.5);
		mFrontLeftMotor->Enable();
		mFrontRightMotor->Enable();
		Wait(.5);

		// right motor is mirror of left so invert its actions since it runs backwards of the left motor
		mFrontRightMotor->SetInverted(true);  // invert power so + power goes forward for both sides; not used for PID control of mirrored motors; only for %VBus
		mFrontRightMotor->SetSensorDirection(true); // invert encoder to match left; true reverses GetPosition & GetSpeed but not GetEncPosition nor GetEncVel
		mFrontRightMotor->SetClosedLoopOutputDirection(true); // reverses the power to the motor in PID controller mode - compensate for the other reversals done above
		Wait(.2);

		// PID Tuning parameters go here
		mFrontLeftMotor->SetPID(TALON_DRIVE::TankTurnKp, TALON_DRIVE::TankTurnKi, TALON_DRIVE::TankTurnKd);
		mFrontRightMotor->SetPID(TALON_DRIVE::TankTurnKp, TALON_DRIVE::TankTurnKi, TALON_DRIVE::TankTurnKd);

		mFrontLeftMotor->SetControlMode(mFrontLeftMotor->ControlMode::kSpeed);
		mFrontLeftMotor->Set(0.);
		
		mFrontRightMotor->SetControlMode(mFrontRightMotor->ControlMode::kSpeed);
		mFrontRightMotor->Set(0.);
		Wait(.2);
		
		mRearLeftMotor->SetControlMode(CANSpeedController::kFollower);
		mRearLeftMotor->Set(CAN_PORT::FRONT_LEFT);

		mRearRightMotor->SetControlMode(CANSpeedController::kFollower);
		mRearRightMotor->Set(CAN_PORT::FRONT_RIGHT);
		Wait(.2);
		
		std::cout << "\n\nPID and follower modes set\n\n";
	
		std::cout << "Front Left ControlMode " << mFrontLeftMotor->GetControlMode() << std::endl;
		std::cout << "Front Right ControlMode " << mFrontRightMotor->GetControlMode() << std::endl;
		std::cout << "Rear Left ControlMode " << mRearLeftMotor->GetControlMode() << std::endl;
		std::cout << "Rear Right ControlMode " << mRearRightMotor->GetControlMode() << std::endl;

		std::cout << "Front Left IsControlEnabled " << mFrontLeftMotor->IsControlEnabled() << std::endl;
		std::cout << "Front Right IsControlEnabled " << mFrontRightMotor->IsControlEnabled() << std::endl;
		std::cout << "Rear Left IsControlEnabled " << mRearLeftMotor->IsControlEnabled() << std::endl;
		std::cout << "Rear Right IsControlEnabled " << mRearRightMotor->IsControlEnabled() << std::endl;

		mFrontLeftMotor->SetEncPosition(0);
		mFrontRightMotor->SetEncPosition(0);
		
		mTime->Reset();
		mTime->Start();

		while (IsOperatorControl() && IsEnabled())
		{
			mFrontLeftMotor->Set(270.0);
			mFrontRightMotor->Set(270.0);

//			float motor = 25.f * sin(mTime->Get()*30.);
//			mFrontLeftMotor->Set(motor);
//			mFrontRightMotor->Set(motor);
//			mFrontLeftMotor->ClearIaccum();
//			mFrontRightMotor->ClearIaccum();

			positionEncLeft = mFrontLeftMotor->GetEncPosition();
			positionLeft = mFrontLeftMotor->GetPosition();
			positionEncRight = mFrontRightMotor->GetEncPosition();
			positionRight = mFrontRightMotor->GetPosition();

			velocityEncLeft = mFrontLeftMotor->GetEncVel();
			velocityLeft = mFrontLeftMotor->GetSpeed();
			velocityEncRight = mFrontRightMotor->GetEncVel();
			velocityRight = mFrontRightMotor->GetSpeed();

			errorLeft = mFrontLeftMotor->GetClosedLoopError();
			errorRight = mFrontRightMotor->GetClosedLoopError();

			printf("Time %f; Encoders: %d, %d, %d, %d; EgrUnits: %f, %f, %f, %f, %d, %d; Volts: %f, %f\n",
				mTime->Get(),
				positionEncLeft, positionEncRight, velocityEncLeft, velocityEncRight,
				positionLeft, positionRight,
				velocityLeft, velocityRight, errorLeft, errorRight,
				mFrontLeftMotor->GetOutputVoltage(), mFrontRightMotor->GetOutputVoltage());

			printf("v, v, a, t FL: %f, %f, %f,%f;  FR: %f, %f, %f,%f;  RL: %f, %f, %f,%f;  RR: %f, %f, %f,%f\n",
				mFrontLeftMotor->GetBusVoltage(), mFrontLeftMotor->GetOutputVoltage(), mFrontLeftMotor->GetOutputCurrent(), mFrontLeftMotor->Get(),
				mFrontRightMotor->GetBusVoltage(), mFrontRightMotor->GetOutputVoltage(), mFrontRightMotor->GetOutputCurrent(), mFrontRightMotor->Get(),
				mRearLeftMotor->GetBusVoltage(), mRearLeftMotor->GetOutputVoltage(), mRearLeftMotor->GetOutputCurrent(), mRearLeftMotor->Get(),
				mRearRightMotor->GetBusVoltage(), mRearRightMotor->GetOutputVoltage(), mRearRightMotor->GetOutputCurrent(), mRearRightMotor->Get());
		 
			// positionEncLeft, positionEncRight, velocityEncLeft, velocityEncRight updated 0.1 seconds
			// positionLeft, positionRight, velocityLeft, velocityRight updated 0.02 seconds
			// errorLeft, errorRight updated 0.01 seconds (maybe, maybe faster)
			// set updated 0.01 seconds

//			if(positionEncLeft <= -753) // gear 26.9 x 7 pulses/motor revolution x 4 edges per quad encoder pulse(s) = 753.2
//				{
//				 printf("\nOne revolution\n");
//				 break;
//				}
			Wait(0.02);
		}
		mFrontLeftMotor->Set(0.);
		mFrontRightMotor->Set(0.);
	}

//  _____                _   
// |_   _|   ___   ___  | |_ 
//   | |    / _ \ / __| | __|
//   | |   |  __/ \__ \ | |_ 
//   |_|    \___| |___/  \__|

	void Test()
	{
	std::cout << "\nEntered Test to tune drive motors using %VBus control mode and followers\n";

	std::cout << "Resetting Talons\n";
	mFrontLeftMotor->Reset();
	mFrontRightMotor->Reset();
	Wait(.5);
	mFrontLeftMotor->Enable();
	mFrontRightMotor->Enable();
	Wait(.5);

	mFrontLeftMotor->SetControlMode(mFrontLeftMotor->ControlMode::kPercentVbus);
	mFrontLeftMotor->Set(0.);

	mFrontRightMotor->SetControlMode(mFrontRightMotor->ControlMode::kPercentVbus);
	mFrontRightMotor->Set(0.);

	Wait(.2);

	mRearLeftMotor->SetControlMode(CANSpeedController::kFollower);
	mRearLeftMotor->Set(CAN_PORT::FRONT_LEFT);

	mRearRightMotor->SetControlMode(CANSpeedController::kFollower);
	mRearRightMotor->Set(CAN_PORT::FRONT_RIGHT);
	Wait(.2);

	std::cout << "Front Left ControlMode " << mFrontLeftMotor->GetControlMode() << std::endl;
	std::cout << "Front Right ControlMode " << mFrontRightMotor->GetControlMode() << std::endl;
	std::cout << "Rear Left ControlMode " << mRearLeftMotor->GetControlMode() << std::endl;
	std::cout << "Rear Right ControlMode " << mRearRightMotor->GetControlMode() << std::endl;

	std::cout << "Front Left IsControlEnabled " << mFrontLeftMotor->IsControlEnabled() << std::endl;
	std::cout << "Front Right IsControlEnabled " << mFrontRightMotor->IsControlEnabled() << std::endl;
	std::cout << "Rear Left IsControlEnabled " << mRearLeftMotor->IsControlEnabled() << std::endl;
	std::cout << "Rear Right IsControlEnabled " << mRearRightMotor->IsControlEnabled() << std::endl;

	// right motor is mirror of left so invert its actions since it runs backwards of the left motor
	mFrontRightMotor->SetInverted(true);  // invert power so + power goes forward for both sides; not used for PID control of mirrored motors; only for %VBus
	mFrontRightMotor->SetSensorDirection(true); // invert encoder to match left; true reverses GetPosition & GetSpeed but not GetEncPosition nor GetEncVel
	Wait(.2);

	mFrontLeftMotor->SetEncPosition(0);
	mFrontRightMotor->SetEncPosition(0);

	while(IsTest())
	{
		if(IsEnabled())
		{
			std::cout << "\nPID Auto Tuner Starting for tank tread motors.\n";

// First motor in the argument list is tuned; second motor goes along for the ride.
// The other 2 motors are assumed to follow.
// The last parameter specifies if the second motor moves in the same direction as the first motor (drive straight -> false)
// or moves in the opposite direction as the first motor (tank turn -> true).
			TuneMain(mFrontLeftMotor, mFrontRightMotor, mRearLeftMotor, mRearRightMotor, true);
			Wait(2.);

// tune the other motor
			TuneMain(mFrontRightMotor, mFrontLeftMotor, mRearRightMotor, mRearLeftMotor, true);

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
