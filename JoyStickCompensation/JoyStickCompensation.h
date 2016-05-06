#ifndef JOYSTICKCOMPENSATION_H_
#define JOYSTICKCOMPENSATION_H_

float JoyStickCompensation(float x, int curve=1, float scale=1.0f);
/*
 * Change the response of the JoyStick
 * Select a response curve to change the feel of the stick
 * Reverse axis by using a negative scale factor
 * Limit or expand action with scale factor but output clamped to curve range
 * JoyStick input any value (nominally -1 to 1)
 * Compensation curve number 1 to 3; Output is 0 if bad curve number
*/
#endif
