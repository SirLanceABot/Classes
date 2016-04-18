#ifndef SRC_BOSCH_H_
#define SRC_BOSCH_H_

// Position of BOSCH AHC-2 12V  6004.RA3.194-06 174.9:1 gear w/ encoder 1 tick per motor revolution on roboRIO analog 5 volt bus

#include "WPILib.h"
class CANTalon;
class Counter;

class Bosch
{
public:
	Bosch(int CANTalonDeviceID, int encoderPort);
	~Bosch();
	float CheckDirectionChange(float);
	int GetPosition();
	CANTalon* mCANTalon; // motor
	Counter* mCounter; // count the encoder pulse triggers in current direction

private:
	AnalogTrigger mAnalogTrigger; // create an encoder pulse trigger
	float mSpeedPrevious; // to remember previous direction
	int mPosition; // position accumulator to remember previous position before last direction change
};

#endif /* SRC_BOSCH_H_ */
