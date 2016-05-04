#ifndef CONSTANTS_H_
#define CONSTANTS_H_

#include "WPIlib.h"
#include "ITG-3200.h"

namespace XBOX
{
// declaring buttons as constants from the xbox joystick
const int DRIVER_PORT = 0;
const int A_BUTTON = 1;
const int B_BUTTON = 2;
const int X_BUTTON = 3;
const int Y_BUTTON = 4;
const int LEFT_BUMPER_BUTTON = 5;
const int RIGHT_BUMPER_BUTTON = 6;
const int BACK_BUTTON = 7;
const int START_BUTTON = 8;
const int LEFT_STICK_PRESS_BUTTON = 9;
const int RIGHT_STICK_PRESS_BUTTON = 10;
const int NODE_ID = 1;

// declaring axes as constants from joystick
const int LEFT_X_AXIS = 0;
const int LEFT_Y_AXIS = 1;
const int LEFT_TRIGGER_AXIS = 2;
const int RIGHT_TRIGGER_AXIS = 3;
const int RIGHT_X_AXIS = 4;
const int RIGHT_Y_AXIS = 5;

const int POV_UP = 0;
const int POV_RIGHT = 90;
const int POV_DOWN = 180;
const int POV_LEFT = 270;
}

namespace I2C_PORT
{
const I2C::Port SIX_DOF = I2C::Port::kMXP;
//const I2C::Port SIX_DOF = I2C::Port::kOnboard;
}

namespace GYRO
{
const int ADDRESS = ITG3200::ITG3200_I2C_ADDR;
const int SAMPLE_RATE_FILTER = ITG3200::k1000Hz_10Hz;
const float SENSITIVITY = 0.99 * ITG3200::SENSITIVITY_SCALE_FACTOR; // todo: sensitivity needs to be checked/calibrated; typically multiple default by a number near 1.0 to adjust accurately
}

#endif /* SRC_CONSTANTS_H_ */
