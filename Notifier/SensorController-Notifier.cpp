// Example usage of Notifier class to schedule periodically a "background" function
// Template of usage of Notifier plus a trivial example of one variable whose value is updated and used

//  The example sensor.h file

#include "WPILib.h"

class SensorBackgroundController
{
public:
	SensorBackgroundController(double period = 0.005);
	~SensorBackgroundController();
	int Get();
	void Set(int);
protected:
	void Acquire();
private:
	mutable priority_mutex m_mutex; // mutex to protect read/write access to process variable
	std::unique_ptr<Notifier> m_backgroundLoop;

	int m_result; // example process variable

};
//  End example sensor.h file
/***************************************************************/
//  The example sensor.cpp file

#include "WPILib.h"

SensorBackgroundController::SensorBackgroundController(double period) :
	m_backgroundLoop (std::make_unique<Notifier>(&SensorBackgroundController::Acquire, this)),
	m_result (0)
{
	printf("Starting Notifier background loop with period = %f seconds\n", period);
	m_backgroundLoop->StartPeriodic(period); // background function starts running immediately
}

SensorBackgroundController::~SensorBackgroundController()
{
	m_backgroundLoop->Stop(); // Notifier Stop doesn't return here until last loop has completed
}

void SensorBackgroundController::Acquire()
{
	std::lock_guard<priority_mutex> ProtectMe(m_mutex); // protect the shared memory from others' access while it's being updated here
	m_result++;
}

void SensorBackgroundController::Set(int result)
{
	std::lock_guard<priority_mutex> ProtectMe(m_mutex); // protect the shared memory from others' access while it's being updated here
	m_result = result;
}

int SensorBackgroundController::Get()
{
	std::lock_guard<priority_mutex> ProtectMe(m_mutex); // protect the shared memory from others' access while it's being read here
	return m_result;
}
//  End example sensor.cpp file

#include "WPILib.h"

class Robot: public SampleRobot
{
public:
	std::unique_ptr<SensorBackgroundController> mSensor;

	Robot() :
		mSensor(std::make_unique<SensorBackgroundController>())
	{
	}

	void OperatorControl()
	{
		printf("%d\n", mSensor->Get());  // example of getting process variable

		mSensor->Set(9999); // example of "resetting" process variable

		while (IsOperatorControl() && IsEnabled())
		{
			printf("%d\n", mSensor->Get());  // example of getting process variable
			Wait(0.005);
		}
	}
};

START_ROBOT_CLASS(Robot)
