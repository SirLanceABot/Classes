#include "WPILib.h"
#include "crc-16.h"
#include <stdlib.h>
#include <stdint.h>
#include <iostream>

class SimpleRobotDemo: public SimpleRobot {

public:
		AnalogChannel *m_analog;
		AnalogChannel* mGyroTemp;
		Timer *m_timer;
		static constexpr float kSamplesPerSecond = 50.0;
		static const uint32_t kOversampleBits = 10;
		static const uint32_t kAverageBits = 0;

	SimpleRobotDemo() {
		m_timer = new Timer();
		m_analog = new AnalogChannel(1);
		mGyroTemp = new AnalogChannel(2);

		m_analog->SetAverageBits(kAverageBits);
		m_analog->SetOversampleBits(kOversampleBits);
		float sampleRate = kSamplesPerSecond * (1 << (kAverageBits + kOversampleBits));
		m_analog->GetModule()->SetSampleRate(sampleRate);
		Wait(1.0);
		printf("Ready to enable\n");
	}

	void Autonomous() {
		while (IsAutonomous() && IsEnabled()) {
			// Update actuators based on sensors, elapsed time, etc here....

			/*
			 * Wait a short time before reiterating.  The wait period can be
			 * changed, but some time in a wait state is necessary to allow
			 * the other tasks, such as the Driver Station communication task,
			 * running on your cRIO to have some processor time. This also
			 * gives time for new sensor inputs, etc. to collect prior to
			 * further updating actuators on the subsequent iteration.
			 */
			Wait(0.005);
		}
	}

	void OperatorControl()
	{
			INT64 value, value_prev;
			uint32_t count, count_prev;

			char outString[1000];
			int outStringLen;

//			m_analog->ResetAccumulator();
			m_timer->Start();
			m_timer->Reset();
			m_analog->InitAccumulator();
			Wait(1./kSamplesPerSecond*1.01); // takes some time to init and get first sample
			m_analog->GetAccumulatorOutput(&value_prev, &count_prev);
			outStringLen = sprintf(outString, "%f, %lld, %lu, %lld, %lu, %f, %d",
					m_timer->Get(), value_prev, count_prev, value_prev, count_prev, mGyroTemp->GetVoltage(), mGyroTemp->GetValue());
			printf("#, %04x, %s\n", CRC16(outString, outStringLen), outString);

			while (IsOperatorControl() /*&& IsEnabled()*/)
			{
			m_analog->GetAccumulatorOutput(&value, &count);

			if(count != count_prev)
				{
				outStringLen = sprintf(outString, "%f, %lld, %lu, %lld, %lu, %f, %d",
						m_timer->Get(), value, count, value-value_prev, count-count_prev, mGyroTemp->GetVoltage(), mGyroTemp->GetValue());
				printf("#, %04x, %s\n", CRC16(outString, outStringLen), outString);
				value_prev = value;
				count_prev = count;
				}
			Wait(0.0001);
			}
	}

	void Test() {}
};

/*
 * This macro invocation tells WPILib that the named class is your "main" robot class,
 * providing an entry point to your robot code.
 */
START_ROBOT_CLASS(SimpleRobotDemo);
