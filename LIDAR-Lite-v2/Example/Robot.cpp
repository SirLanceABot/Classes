/*
 * Written by Sir Lance-A-Bot / Team FRC 4237
 *
 * FRC 2016 Example program to obtain distance from robot to an object with a PulsedLight LIDAR Lite v2 connected to an I2C bus
 */
#include "WPILib.h"
#include "LIDAR.h"

class Robot: public SampleRobot
{
	std::unique_ptr<Lidar> mLidar1;
	std::unique_ptr<Lidar> mLidar2;

public:
	Robot()
	:
	 mLidar1(std::make_unique<Lidar>(I2C::Port::kOnboard, Lidar::ADDRESS_DEFAULT, 0.02)), // 0.015 seconds fastest LIDAR-Lite v2 will work reasonably well on roboRIO; 0.5 very reliable but rather slow
	 mLidar2(std::make_unique<Lidar>(I2C::Port::kMXP, Lidar::ADDRESS_DEFAULT, 0.02))
	{
		printf("[Robot for Lidar] Compiled file:%s, date:%s, time:%s, function:%s, object address:%p, function:%s, printed at line:%d\n",
				__FILE__, __DATE__, __TIME__, __FUNCTION__, this, __PRETTY_FUNCTION__, __LINE__);
	}

	void OperatorControl()
	{
		while (IsOperatorControl() && IsEnabled())
		{
			if ( mLidar1->IsWorking() ) printf("GetDistance Lidar onboard %d\n", mLidar1->GetDistance());
			if ( mLidar2->IsWorking() ) printf("GetDistance Lidar MXP %d\n", mLidar2->GetDistance());
			Wait(0.02);
		}
	}
};

START_ROBOT_CLASS(Robot);
