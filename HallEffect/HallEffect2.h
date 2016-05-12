#ifndef HALLEFFECT_H
#define HALLEFFECT_H

class HallEffect
{
public:
	HallEffect();
	~HallEffect();
	void TestHallEffect();

private:
	Counter* mHallEffectOutCounter;
	Counter* mHallEffectInCounter;
	DigitalInput* mHallEffectOut;
	DigitalInput* mHallEffectIn;
};

HallEffect::HallEffect()
{
	mHallEffectOutCounter = new Counter(mHallEffectOut);
	mHallEffectInCounter = new Counter(mHallEffectIn);
	mHallEffectOut = new DigitalInput(HALL_EFFECT_OUT);
	mHallEffectIn = new DigitalInput(HALL_EFFECT_IN);
}

HallEffect::~HallEffect()
{
	delete mHallEffectIn;
	mHallEffectIn = NULL;
	delete mHallEffectOut;
	mHallEffectOut = NULL;
	delete mHallEffectInCounter;
	mHallEffectInCounter = NULL;
	delete mHallEffectOutCounter;
	mHallEffectOutCounter = NULL;
}

void HallEffect::TestHallEffect()
{
	mHallEffectInCounter->Reset();
	mHallEffectOutCounter->Reset();
	mHallEffectInCounter->SetUpSourceEdge(false, true);
	mHallEffectOutCounter->SetUpSourceEdge(false, true);
	{
		printf("Hall Effect In: %d          Hall Effect Out: %d\n", mHallEffectIn->Get(), mHallEffectOut->Get());
		Wait(0.2);
	}
}

#endif
