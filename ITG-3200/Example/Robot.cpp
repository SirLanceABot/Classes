#include "WPILib.h"
#include "ITG-3200.h"
#include "XboxJoystick.h"
#include "Constants.h"

class	Robot: public SampleRobot
{
public:
	Robot();
	~Robot();
	void OperatorControl();
private:
	std::shared_ptr<ITG3200> mGyro;  //gyro
	std::shared_ptr<XboxJoystick> mStick;  //Xbox controller
};

Robot::Robot()
{
	printf("Robot Constructor\n");
	double initial_angle[] = {0.0,0.0,0.0};
	mStick = std::make_shared<XboxJoystick>(XBOX::DRIVER_PORT);
	mGyro = std::make_shared<ITG3200>(I2C_PORT::SIX_DOF, GYRO::ADDRESS, GYRO::SAMPLE_RATE_FILTER, initial_angle, GYRO::SENSITIVITY);
}

Robot::~Robot(){}

void Robot::OperatorControl()
{
	printf("OperatorControl\n");

	while (IsOperatorControl() && IsEnabled())
	{
		std::cout << "Selected Gyro Test" << std::endl;
		if(mGyro->IsWorking()) mGyro->Test(mStick);
		std::cout << "Exiting Gyro Test" << std::endl;
		Wait(2.);
	}
}

START_ROBOT_CLASS(Robot)
