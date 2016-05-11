/*
 * Logitech.h
 *
 *  Created on: Jan 23, 2016
 *      Author: 18elafrenz
 */

#ifndef LOGITECH_H_
#define LOGITECH_H_

#include <memory>

class Joystick;

class Logitech
{
public:
	Logitech(int);		//constructor
	~Logitech();		//deconstructor
// declaring functions to be used to receive values from the logitech buttons
	bool GetRawButton(int);
	bool GetTrigger();
	bool GetButton2();
	bool GetButton3();
	bool GetButton4();
	bool GetButton5();
	bool GetButton6();
	bool GetButton7();
	bool GetButton8();
	bool GetButton9();
	bool GetButton10();
	bool GetButton11();
	bool GetButton12();
// declaring functions to be used to receive values from the logitech axes
	float GetRawAxis(int);
	float GetXAxis();
	float GetYAxis();
	float GetZAxis();
	float GetThrottleAxis();
// declaring functions to be used receive values from the pads on the logitech joystick
	int GetPOV();
// test
	void Test();
private:
//	Joystick* mLogitech;
	std::shared_ptr<Joystick> mLogitech;
};



#endif /* SRC_LOGITECH_H_ */
