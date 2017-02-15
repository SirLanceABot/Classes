#include "wpilib.h"
#include <memory>
#include "ITG-3200.h"
#include "constants.h"

/**
 * This is a sample program to demonstrate how to use a gyro sensor to make a robot drive
 * straight. This program uses a joystick to drive forwards and backwards while the gyro
 * is used for direction keeping.
 */
class Robot: public frc::SampleRobot {
	double initial_angle[3] = {0.0,0.0,0.0};
public:
	std::shared_ptr<ITG3200> mGyro;

	Robot()
	:
	mGyro(std::make_shared<ITG3200>(I2C_PORT::SIX_DOF, GYRO::ADDRESS, GYRO::SAMPLE_RATE_FILTER, initial_angle, GYRO::SENSITIVITY))
	{
}

void RobotInit()
{
}

void OperatorControl()
{
	std::cout << "Selected Gyro Test" << std::endl;

	while (IsOperatorControl() && IsEnabled())
		{
			if(mGyro->IsWorking()) mGyro->Test();

			frc::Wait(.01);
		}
	std::cout << "Exiting Gyro Test" << std::endl;
}

};

START_ROBOT_CLASS(Robot)
