#include "CANTalon4237.h"
#include "WPILib.h"

CANTalon4237::CANTalon4237(int port, SensorType type, int softFwdLimit, int softRevLimit)
{
	mCANTalon = new CANTalon(port);
	mSensorType = type;

	//The PG27 Planetary Gearbox with RSbb775 Motor and Encoder is reversed
	switch(type)
	{
	case kNoSensor:
		mCANTalon->ConfigLimitMode(CANSpeedController::kLimitMode_SrxDisableSwitchInputs);
		break;

	case kSwitchNormallyOpen:
		//~751 ticks per revolution
		mCANTalon->ConfigLimitMode(CANSpeedController::kLimitMode_SwitchInputsOnly);
		mCANTalon->ConfigFwdLimitSwitchNormallyOpen(true);
		mCANTalon->ConfigRevLimitSwitchNormallyOpen(true);
		break;

	case kQuadEncoder:
		mCANTalon->SetFeedbackDevice(CANTalon::QuadEncoder);
		mCANTalon->SetEncPosition(0);
		break;

	case kMagneticEncoder:
		mCANTalon->SetFeedbackDevice(CANTalon::CtreMagEncoder_Absolute);
		mCANTalon->SetEncPosition(0);
		break;

	case kSwitchAndQuadEncoder:
		mCANTalon->ConfigLimitMode(CANSpeedController::kLimitMode_SwitchInputsOnly);
		mCANTalon->ConfigFwdLimitSwitchNormallyOpen(true);
		mCANTalon->ConfigRevLimitSwitchNormallyOpen(true);
		mCANTalon->SetFeedbackDevice(CANTalon::QuadEncoder);
		mCANTalon->SetEncPosition(0);
		break;

	case kSoftLimitAndQuadEncoder:
		mCANTalon->ConfigLimitMode(CANSpeedController::kLimitMode_SoftPositionLimits);

		mCANTalon->ConfigForwardLimit(softFwdLimit);
		mCANTalon->ConfigReverseLimit(softRevLimit);

		mCANTalon->SetFeedbackDevice(CANTalon::QuadEncoder);
		mCANTalon->SetEncPosition(0);	//~751 ticks per revolution

	}
}

CANTalon4237::~CANTalon4237()
{
	delete mCANTalon;
}

void CANTalon4237::Set(float value)
{
	if(value > 1.0)
	{
		mCANTalon->Set(1.0);
	}
	else if(value < -1.0)
	{
		mCANTalon->Set(-1.0);
	}
	else
	{
		mCANTalon->Set(value);
	}
}

void CANTalon4237::ResetEncoder()
{
	mCANTalon->SetEncPosition(0);
}

void CANTalon4237::SetFollower(int masterDeviceNumber)
{
	mCANTalon->SetControlMode(CANSpeedController::kFollower);
	mCANTalon->Set(masterDeviceNumber);
}

void CANTalon4237::SetMaster()
{
	// TODO: Fix for kSpeed and kPosition
	mCANTalon->SetControlMode(CANSpeedController::kPercentVbus);
}

int CANTalon4237::GetEncoder()
{
	return mCANTalon->GetEncPosition();

}

void CANTalon4237::ReverseEncoderDirection()
{
	mCANTalon->SetSensorDirection(true);
}
