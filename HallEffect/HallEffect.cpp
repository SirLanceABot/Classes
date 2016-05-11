/*
 * HallEffect.cpp
 *
 *  Created on: Feb 13, 2016
 *      Author: Proramming
 */
#include "HallEffect.h"
#include "WPIlib.h"


HallEffect::HallEffect()
{
	mHallEffect = new DigitalInput(0);
}

HallEffect::~HallEffect()
{
	delete mHallEffect;
}

bool HallEffect::Get()
{
	return !mHallEffect->Get();
}

void HallEffect::Test()
{
	bool value;
	value = Get();
	if (value == true)
		printf("Can see magnet\n");
	else
		printf("Can't see magnet\n");

}



