#include "WPILib.h"
#include "crc-16.h"
#include <stdlib.h>

class SimpleRobotDemo: public SimpleRobot {

public:
		AnalogChannel *m_analog;
		Timer *m_timer;

	SimpleRobotDemo() {
		m_timer = new Timer();
		m_analog = new AnalogChannel(1);
	}

	void OperatorControl()
	{
			long count=0;
			char outString[1000];
			int outStringLen;

			m_timer->Start();
			m_timer->Reset();

			while (IsOperatorControl() /*&& IsEnabled()*/)
			{
				outStringLen = sprintf(outString, "%ld, %f, %f, %d", count++, m_timer->Get(), m_analog->GetVoltage(), m_analog->GetValue());
				printf("#, %04x, %s\n", CRC16(outString, outStringLen), outString);
				}
			}
};

/*
 * This macro invocation tells WPILib that the named class is your "main" robot class,
 * providing an entry point to your robot code.
 */
START_ROBOT_CLASS(SimpleRobotDemo);
