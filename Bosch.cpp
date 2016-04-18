// Position of BOSCH AHC-2 12V  6004.RA3.194-06 174.9:1 gear w/ encoder 1 tick per motor revolution on roboRIO analog 5 volt bus

#include "Bosch.h"
#include "WPILib.h"

Bosch::Bosch(int CANTalonDeviceID, int32_t encoderPort) :
	mAnalogTrigger(encoderPort)
{
	mCANTalon = new CANTalon(CANTalonDeviceID);
	mAnalogTrigger.SetLimitsVoltage(3.5, 3.8); // values higher than the highest minimum (pulse floor), lower than the lowest maximum (pulse ceiling)
	mCounter = new Counter(&mAnalogTrigger);
	mSpeedPrevious = 0.;
	mPosition = 0;
}

float Bosch::CheckDirectionChange(float NewSpeed)
{
	// update position accumulator if changing direction
	// encoder doesn't know the direction so we have to remember the direction for it
	if ((mSpeedPrevious < 0 && NewSpeed >= 0) || (mSpeedPrevious >= 0 && NewSpeed < 0))
	{
		mPosition = GetPosition(); // changing directions so save what we have
		mCounter->Reset(); // and start counting in the new direction
		mSpeedPrevious = NewSpeed; // return input speed for ease of use (may include it in the Set() argument => Set(CheckDirectionChange(speed)))
	}
		return NewSpeed;
}

int Bosch::GetPosition()
{
	// position from previous direction change plus what's been accumulated so far in this direction
	if (mSpeedPrevious >= 0)
		return mPosition + mCounter->Get(); // been going forward so add counter

	return mPosition - mCounter->Get(); // been going backward so subtract counter
}
