#ifndef CONSTANTS_H_
#define CONSTANTS_H_

#include "WPIlib.h"
#include "ITG-3200.h"

namespace I2C_PORT
{
//const I2C::Port SIX_DOF = I2C::Port::kMXP;
const I2C::Port SIX_DOF = I2C::Port::kOnboard;
}

namespace GYRO
{
const int ADDRESS = ITG3200::ITG3200_I2C_ADDR;
const int SAMPLE_RATE_FILTER = ITG3200::k1000Hz_10Hz;
const float SENSITIVITY = 0.98 * ITG3200::SENSITIVITY_SCALE_FACTOR; // todo: sensitivity needs to be checked/calibrated; typically multiply default by a number near 1.0 to adjust accurately
}

#endif /* SRC_CONSTANTS_H_ */
