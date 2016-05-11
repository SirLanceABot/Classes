/*
 * HallEffect.h
 *
 *  Created on: Feb 13, 2016
 *      Author: Proramming
 */
#ifndef HALLEFFECT_H_
#define HALLEFFECT_H_

class DigitalInput;

class HallEffect
{
public:
	HallEffect();
	~HallEffect();
	bool Get();
	void Test();
private:
	DigitalInput* mHallEffect;
};


#endif /* HALLEFFECT_H_ */
