/*
 * XboxJoystick.h
 *
 *  Created on: Jan 23, 2016
 *      Author: 18elafrenz
 */

#ifndef XBOXJOYSTICK_H_
#define XBOXJOYSTICK_H_

#include <memory>

class Joystick;

class XboxJoystick
{
public:
	enum ModificationType {kLinear, kSquared, kCubic, kCrawlWalkRun};
// declaring functions to get values from corresponding joystick
// making the constructor and the deconstructor
	XboxJoystick(int);
	~XboxJoystick();
// xbox Buttons
	bool GetRawButton(int);
	bool GetA();
	bool GetB();
	bool GetX();
	bool GetY();
	bool GetStart();
	bool GetBack();
	bool GetLeftStickPress();
	bool GetRightStickPress();
	bool GetLeftBumper();
	bool GetRightBumper();
// xbox Axes
	float GetRawAxis(int axis);
	float GetModifiedAxis(int, ModificationType);
	float GetLeftTrigger();
	float GetRightTrigger();
	float GetLeftXAxis();
	float GetRightXAxis();
	float GetLeftYAxis();
	float GetRightYAxis();
// xbox directional pad
	int GetDPad();
	int GetPOV();
// xbox setting rumbles of joystick
	void SetLeftRumble(float kLeftRumble);
	void SetRightRumble(float kRightRumble);
// test
	void Test();


private:						// declare variable to receive values
//	Joystick* mJoystick;
	std::shared_ptr<Joystick> mJoystick;
};
#endif /* SRC_XBOXJOYSTICK_H_ */
