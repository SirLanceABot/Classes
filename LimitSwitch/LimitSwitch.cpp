#include "LimitSwitch.h"

LimitSwitch::LimitSwitch(int channel)
{
	mSwitch = new DigitalInput(channel);
	
	mSwitchCounter = new Counter(mSwitch);
	
	// count the transition from open (high voltage) to closed (low voltage) - the falling signal only
	mSwitchCounter->SetUpSourceEdge(/*rising edge=*/false, /*falling edge=*/true);
	
	mSwitchCounter->Start();  // start counting transitions
	mSwitchCounter->Reset();  // initially assure 0
}

LimitSwitch::~LimitSwitch()
{
	delete mSwitchCounter;
	mSwitchCounter = NULL;
	
	delete mSwitch;
	mSwitch = NULL;
}

bool LimitSwitch::IsOn()
{
	if( mSwitchCounter->Get() > 0 ) // check counter for any transitions since last check
	{
		mSwitchCounter->Reset();  // transition processed so reset back to 0
		return LimitSwitch::kIsOn;
	}
	
	if( mSwitch->Get() == 0 ) // counter was reset after any transition but switch could still be closed
	{
		return LimitSwitch::kIsOn;
	}

	return LimitSwitch::kIsNotOn;
}

bool LimitSwitch::IsOff()
{
	return !IsOn();
}
